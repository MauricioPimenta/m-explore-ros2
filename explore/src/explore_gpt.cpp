#include <explore/explore_gpt.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace explore
{

Explore::Explore(const rclcpp::NodeOptions& options)
  : Node("explore_node", options),
	tf_buffer_(this->get_clock()),
	costmap_client_(*this, &tf_buffer_),
	prev_goal_(),
	last_progress_(this->now()),
	last_markers_count_(0)
{
  // original params
  visualize_ = this->declare_parameter("visualize", true);
  planner_frequency_ = this->declare_parameter("planner_frequency", 0.5);
  progress_timeout_ = this->declare_parameter("progress_timeout", 30.0);
  potential_scale_ = this->declare_parameter("potential_scale", 1e-3);
  orientation_scale_ = this->declare_parameter("orientation_scale", 0.0);
  gain_scale_ = this->declare_parameter("gain_scale", 1.0);
  transform_tolerance_ = this->declare_parameter("transform_tolerance", 0.3);
  min_frontier_size_ = this->declare_parameter("min_frontier_size", 0.5);

  // new params
  max_retries_per_frontier_ = this->declare_parameter("max_retries_per_frontier", 3);
  frontier_key_resolution_  = this->declare_parameter("frontier_key_resolution", 0.25);  // meters
  min_travel_distance_for_abort_ = this->declare_parameter("min_travel_distance_for_abort", 0.30); // meters

  if (visualize_) {
	marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"frontiers", rclcpp::SystemDefaultsQoS());
  }

  move_base_client_ = rclcpp_action::create_client<NavigateToPose>(
	  this->get_node_base_interface(),
	  this->get_node_graph_interface(),
	  this->get_node_logging_interface(),
	  this->get_node_waitables_interface(),
	  "navigate_to_pose");

  RCLCPP_INFO(get_logger(), "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(get_logger(), "Connected to move_base nav2 server");

  timer_ = this->create_wall_timer(
	  std::chrono::duration<double>(1.0 / std::max(0.001, planner_frequency_)),
	  std::bind(&Explore::timer_callback, this));
}

Explore::~Explore() {}

geometry_msgs::msg::Pose Explore::getCurrentRobotPoseSafe(bool* ok) const
{
  geometry_msgs::msg::Pose pose;
  try {
	pose = costmap_client_.getRobotPose();
	if (ok) *ok = true;
	return pose;
  } catch (...) {
	if (ok) *ok = false;
	return geometry_msgs::msg::Pose();
  }
}

std::string Explore::frontierKey(const geometry_msgs::msg::Point& p) const
{
  // Quantize frontier positions to avoid "new key every cycle" due to tiny differences
  const double r = std::max(0.01, frontier_key_resolution_);
  const int xi = static_cast<int>(std::llround(p.x / r));
  const int yi = static_cast<int>(std::llround(p.y / r));
  return std::to_string(xi) + "_" + std::to_string(yi);
}

void Explore::registerAttemptStart(const std::string& key)
{
  auto& a = attempts_[key];
  a.last_attempt_time = this->now();

  bool ok = false;
  a.start_pose = getCurrentRobotPoseSafe(&ok);
  a.start_time = this->now();

  // keep tracking which frontier is active
  current_frontier_key_ = key;
}

bool Explore::shouldBlacklist(const std::string& key) const
{
  auto it = attempts_.find(key);
  if (it == attempts_.end()) return false;
  return it->second.failures >= max_retries_per_frontier_;
}

void Explore::addToBlacklist(const std::string& key, const geometry_msgs::msg::Point& goal)
{
  (void)key;
  blacklist_.push_back(goal);
}

void Explore::clearAttemptState(const std::string& key)
{
  attempts_.erase(key);
}

void Explore::timer_callback()
{
  // If we have an active goal, check progress before replanning
  if (!current_frontier_key_.empty()) {
	bool pose_ok = false;
	auto pose = getCurrentRobotPoseSafe(&pose_ok);

	// if TF is flaky, don't punish frontier; just skip progress logic this cycle
	if (pose_ok) {
	  const double dx = (prev_goal_.x - pose.position.x);
	  const double dy = (prev_goal_.y - pose.position.y);
	  const double dist_to_goal = std::hypot(dx, dy);

	  // "progress" = distance to goal decreasing compared to last time
	  // Here we just implement a simple timeout: if we didn't reduce the distance for too long.
	  // But we avoid instantly blacklisting.
	  if ((this->now() - last_progress_).seconds() > progress_timeout_) {
		auto& a = attempts_[current_frontier_key_];
		a.timeouts++;
		a.failures++;

		RCLCPP_WARN(get_logger(),
					"Progress timeout for frontier key=%s (timeouts=%d, failures=%d/%d).",
					current_frontier_key_.c_str(), a.timeouts, a.failures, max_retries_per_frontier_);

		// Cancel goal to force a fresh plan; only blacklist after repeated failures
		if (current_goal_handle_) {
		  (void)move_base_client_->async_cancel_goal(current_goal_handle_);
		}
		current_goal_handle_.reset();
		goal_handle_future_ = {};
		// If exceeded retries, blacklist permanently
		if (shouldBlacklist(current_frontier_key_)) {
		  RCLCPP_WARN(get_logger(), "Blacklisting frontier after repeated timeouts: key=%s",
					  current_frontier_key_.c_str());
		  addToBlacklist(current_frontier_key_, prev_goal_);
		}
		current_frontier_key_.clear();
		// Continue to select a new frontier below
	  } else {
		(void)dist_to_goal; // keep for debugging if you want logs
	  }
	}
  }

	// Find new frontiers / pick a goal
	auto pose = costmap_client_.getRobotPose();
	auto frontiers = 
		frontier_exploration::FrontierSearch(costmap_client_.getCostmap(), 
											potential_scale_, 
											gain_scale_, 
											min_frontier_size_)
											.searchFrom(pose.position);

  if (frontiers.empty()) {
	RCLCPP_WARN(get_logger(), "No frontiers found.");
	return;
  }

  RCLCPP_DEBUG(get_logger(), "found %zu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); i++) {
	RCLCPP_DEBUG(get_logger(), "frontier %zu cost: %f", i, frontiers[i].cost);
  }

  if (visualize_) {
	visualizeFrontiers(frontiers);
  }

  // Select best frontier that is not blacklisted AND not over-retry limit
  geometry_msgs::msg::Point selected_goal;
  bool found_goal = false;
  std::string selected_key;

  for (const auto& f : frontiers) {
	const auto key = frontierKey(f.centroid);

	// permanent blacklist check (original behavior)
	bool is_blacklisted = false;
	for (const auto& b : blacklist_) {
	  if (std::hypot(b.x - f.centroid.x, b.y - f.centroid.y) < frontier_key_resolution_) {
		is_blacklisted = true;
		break;
	  }
	}
	if (is_blacklisted) continue;

	// retry-based blacklist check
	if (shouldBlacklist(key)) {
	  // promote to permanent blacklist to avoid revisiting forever
	  RCLCPP_WARN(get_logger(), "Frontier exceeded retry budget; blacklisting: key=%s", key.c_str());
	  blacklist_.push_back(f.centroid);
	  continue;
	}

	selected_goal = f.centroid;
	selected_key = key;
	found_goal = true;
	break;
  }

  if (!found_goal) {
	RCLCPP_WARN(get_logger(), "All frontiers are blacklisted (or exceeded retry budget), stopping exploration");
	stopMoving();
	return;
  }

  // Avoid spamming new goals if the goal hasn't meaningfully changed
  const double goal_change = std::hypot(selected_goal.x - prev_goal_.x, selected_goal.y - prev_goal_.y);
  if (current_goal_handle_ && goal_change < 0.15) {
	// keep current goal, do nothing
	return;
  }

  // Send goal
  goal_pose_.header.frame_id = costmap_client_.getGlobalFrameID();
  goal_pose_.header.stamp = this->now();
  goal_pose_.pose.position = selected_goal;
  goal_pose_.pose.orientation.w = 1.0;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose_;

  registerAttemptStart(selected_key);

  RCLCPP_DEBUG(get_logger(), "Sending goal to move base nav2 (key=%s)", selected_key.c_str());

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
	  std::bind(&Explore::reachedGoal, this, std::placeholders::_1);

  goal_handle_future_ = move_base_client_->async_send_goal(goal_msg, send_goal_options);

  // best effort: store handle when ready
  if (goal_handle_future_.valid()) {
	// don't block; but we can try a short wait
	auto status = goal_handle_future_.wait_for(50ms);
	if (status == std::future_status::ready) {
	  current_goal_handle_ = goal_handle_future_.get();
	} else {
	  current_goal_handle_.reset();
	}
  }

  prev_goal_ = selected_goal;
  last_progress_ = this->now();
  startMoving();
}

void Explore::reachedGoal(const GoalHandleNavigate::WrappedResult& result)
{
  // Determine what frontier this was for
  const std::string key = current_frontier_key_;
  current_frontier_key_.clear();

  if (key.empty()) {
	// we don't know which frontier; just return
	return;
  }

  auto& a = attempts_[key];

  // Compute travel distance since attempt started
  bool ok = false;
  auto now_pose = getCurrentRobotPoseSafe(&ok);
  double traveled = 0.0;
  if (ok) {
	const double dx = now_pose.position.x - a.start_pose.position.x;
	const double dy = now_pose.position.y - a.start_pose.position.y;
	traveled = std::hypot(dx, dy);
  }

  switch (result.code) {
	case rclcpp_action::ResultCode::SUCCEEDED:
	  RCLCPP_INFO(get_logger(), "Goal succeeded. Clearing retry state for key=%s", key.c_str());
	  clearAttemptState(key);
	  current_goal_handle_.reset();
	  last_progress_ = this->now();
	  return;

	case rclcpp_action::ResultCode::ABORTED:
	{
	  a.aborts++;

	  // If robot barely moved, treat as transient abort and DO NOT blacklist immediately
	  if (ok && traveled < min_travel_distance_for_abort_) {
		RCLCPP_WARN(get_logger(),
					"Goal ABORTED but robot barely moved (%.2fm < %.2fm). "
					"Not blacklisting yet. key=%s aborts=%d failures=%d/%d",
					traveled, min_travel_distance_for_abort_, key.c_str(),
					a.aborts, a.failures, max_retries_per_frontier_);
		// Count it as a failure, but don't punish too hard
		a.failures++;
	  } else {
		a.failures++;
		RCLCPP_WARN(get_logger(),
					"Goal ABORTED (traveled=%.2fm). key=%s aborts=%d failures=%d/%d",
					traveled, key.c_str(), a.aborts, a.failures, max_retries_per_frontier_);
	  }

	  if (shouldBlacklist(key)) {
		RCLCPP_WARN(get_logger(), "Blacklisting frontier after repeated ABORTED/failures: key=%s", key.c_str());
		blacklist_.push_back(prev_goal_);
	  }

	  current_goal_handle_.reset();
	  return;
	}

	case rclcpp_action::ResultCode::CANCELED:
	  // Common when goals get preempted. Don't blacklist.
	  RCLCPP_INFO(get_logger(), "Goal canceled/preempted. Not blacklisting. key=%s", key.c_str());
	  current_goal_handle_.reset();
	  return;

	default:
	  RCLCPP_WARN(get_logger(), "Unknown result code. Not blacklisting. key=%s", key.c_str());
	  current_goal_handle_.reset();
	  return;
  }
}

void Explore::startMoving() {}
void Explore::stopMoving() {}

void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker points;
  points.header.frame_id = costmap_client_.getGlobalFrameID();
  points.header.stamp = this->now();
  points.ns = "frontiers";
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::POINTS;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;
  points.color.a = 1.0;

  for (const auto& f : frontiers) {
	points.points.push_back(f.centroid);
  }

  markers.markers.push_back(points);

  // delete extra markers if frontier count shrank
  if (last_markers_count_ > markers.markers.size()) {
	for (size_t i = markers.markers.size(); i < last_markers_count_; i++) {
	  visualization_msgs::msg::Marker m;
	  m.header.frame_id = costmap_client_.getGlobalFrameID();
	  m.header.stamp = this->now();
	  m.ns = "frontiers";
	  m.id = static_cast<int>(i);
	  m.action = visualization_msgs::msg::Marker::DELETE;
	  markers.markers.push_back(m);
	}
  }

  last_markers_count_ = markers.markers.size();
  marker_array_publisher_->publish(markers);
}

}  // namespace explore

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(explore::Explore)

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// ROS1 code
	/*
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
																		 ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	} */
	rclcpp::spin(
			std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
	rclcpp::shutdown();
	return 0;
}
