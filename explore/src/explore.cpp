/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include <string>

namespace explore
{
Explore::Explore()
	: Node("explore_node")
	, tf_buffer_(this->get_clock())
	, tf_listener_(tf_buffer_)
	, costmap_client_(*this, &tf_buffer_)
	, prev_distance_(0)
	, last_markers_count_(0)
{
	planner_frequency_ = this->declare_parameter<float>("planner_frequency", 1.0);
	progress_timeout_ = this->declare_parameter<float>("progress_timeout", 30.0);
	visualize_ = this->declare_parameter<bool>("visualize", false);
	potential_scale_ = this->declare_parameter<float>("potential_scale", 1e-3);
	orientation_scale_ = this->declare_parameter<float>("orientation_scale", 0.0);
	gain_scale_ = this->declare_parameter<float>("gain_scale", 1.0);
	min_frontier_size_ = this->declare_parameter<float>("min_frontier_size", 0.5);
	
	// new params
	max_retries_per_frontier_ = this->declare_parameter("max_retries_per_frontier", 3);
	frontier_key_resolution_  = this->declare_parameter("frontier_key_resolution", 0.25);  // meters
	min_travel_distance_for_abort_ = this->declare_parameter("min_travel_distance_for_abort", 0.30); // meters

	// Log Parameter values as info
	std::stringstream param_message;
	param_message << 
		"======= EXPLORE_LITE ======="
		"\n" <<
		"Initializing explore_lite with the following parameters:\n" <<
		"\t - planner_frequency: " << planner_frequency_ << "\n" <<
		"\t - progress_timeout: " << progress_timeout_ << "\n" <<
		"\t - visualize: " << (visualize_ ? "true" : "false") << "\n" <<
		"\t - potential_scale: " << potential_scale_ << "\n" <<
		"\t - orientation_scale: " << orientation_scale_ << "\n" <<
		"\t - gain_scale: " << gain_scale_ << "\n" <<
		"\t - min_frontier_size: " << min_frontier_size_ << "\n" <<
		"\t - max_retries_per_frontier: " << max_retries_per_frontier_ << "\n" <<
		"\t - frontier_key_resolution: " << frontier_key_resolution_ << "\n" <<
		"\t - min_travel_distance_for_abort: " << min_travel_distance_for_abort_ << "\n" <<
		"============================";

	RCLCPP_INFO_STREAM(logger_, param_message.str());


	move_base_client_ =
			rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,ACTION_NAME);

	search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(), potential_scale_, gain_scale_, min_frontier_size_);

	if (visualize_)
	{
		marker_array_publisher_ =
			this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);
	}

	RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");

	move_base_client_->wait_for_action_server();

	RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

	exploring_timer_ = this->create_wall_timer(
			std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
			[this]() 
			{
				{
					std::lock_guard<std::mutex> lock(goal_in_progress_mutex_);
					if (goal_in_progress_) {
						RCLCPP_DEBUG(logger_, "Goal in progress, skipping this cycle");
						return;
					}
				}  // Lock is released here
				makePlan();
			});
}

Explore::~Explore()
{
	stop();
}

void Explore::visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers)
{
	std_msgs::msg::ColorRGBA blue;
	blue.r = 0;
	blue.g = 0;
	blue.b = 1.0;
	blue.a = 1.0;
	std_msgs::msg::ColorRGBA red;
	red.r = 1.0;
	red.g = 0;
	red.b = 0;
	red.a = 1.0;
	std_msgs::msg::ColorRGBA green;
	green.r = 0;
	green.g = 1.0;
	green.b = 0;
	green.a = 1.0;

	RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
	visualization_msgs::msg::MarkerArray markers_msg;
	std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
	visualization_msgs::msg::Marker m;

	m.header.frame_id = costmap_client_.getGlobalFrameID();
	m.header.stamp = this->now();
	m.ns = "frontiers";
	m.scale.x = 1.0;
	m.scale.y = 1.0;
	m.scale.z = 1.0;
	m.color.r = 0;
	m.color.g = 0;
	m.color.b = 255;
	m.color.a = 255;

	// lives forever
	#ifdef ELOQUENT
		m.lifetime = rclcpp::Duration(0); // deprecated in galactic warning
	#elif DASHING
		m.lifetime = rclcpp::Duration(0); // deprecated in galactic warning
	#else
		m.lifetime = rclcpp::Duration::from_seconds(0); // foxy onwards
	#endif
	// m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in galactic
	m.frame_locked = true;

	// weighted frontiers are always sorted
	double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

	m.action = visualization_msgs::msg::Marker::ADD;
	size_t id = 0;
	for (auto& frontier : frontiers) {
		m.type = visualization_msgs::msg::Marker::POINTS;
		m.id = int(id);
		m.pose.position = geometry_msgs::msg::Point();
		m.scale.x = 0.1;
		m.scale.y = 0.1;
		m.scale.z = 0.1;
		m.points = frontier.points;
		if (goalOnBlacklist(frontier.centroid)) {
			m.color = red;
		} else {
			m.color = blue;
		}
		markers.push_back(m);
		++id;
		m.type = visualization_msgs::msg::Marker::SPHERE;
		m.id = int(id);
		m.pose.position = frontier.initial;
		// scale frontier according to its cost (costier frontiers will be smaller)
		double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
		m.scale.x = scale;
		m.scale.y = scale;
		m.scale.z = scale;
		m.points = {};
		m.color = green;
		markers.push_back(m);
		++id;
	}
	size_t current_markers_count = markers.size();

	// delete previous markers, which are now unused
	m.action = visualization_msgs::msg::Marker::DELETE;
	for (; id < last_markers_count_; ++id) {
		m.id = int(id);
		markers.push_back(m);
	}

	last_markers_count_ = current_markers_count;
	marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
	// find frontiers
	RCLCPP_INFO(this->get_logger(), "Getting robot pose from costmap...");
	auto robot_pose = costmap_client_.getRobotPose();

	// get frontiers sorted according to cost
	RCLCPP_INFO(this->get_logger(), "Getting frontiers from costmap...");
	auto frontiers = search_.searchFrom(robot_pose.position);

	RCLCPP_INFO(logger_, "found %lu frontiers", frontiers.size());
	for (size_t i = 0; i < frontiers.size(); ++i) {
		RCLCPP_INFO(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
	}

	if (frontiers.empty()) {
		RCLCPP_WARN(logger_, "No frontiers found, stopping exploration???");
		stop();
		return;
	}

	// publish frontiers as visualization markers
	if (visualize_) {
		visualizeFrontiers(frontiers);
	}

	// find non blacklisted frontier
	// find the first frontier that is not in the blacklist
	auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
		[this](const frontier_exploration::Frontier& f)
		{
			return goalBlacklisted(f.centroid);
		});

	if (frontier == frontiers.end()) {
		RCLCPP_WARN(logger_, "All frontiers are blacklisted, stopping exploration");
		stop();
		return;
	}

	// get the frontier centroid as the target position to navigate
	geometry_msgs::msg::Point target_position = frontier->centroid;

	// time out if we are not making any progress
	bool same_goal = prev_goal_ == target_position;

	prev_goal_ = target_position;
	if (!same_goal || prev_distance_ > frontier->min_distance) {
		// we have different goal or we made some progress
		last_progress_ = this->now();
		prev_distance_ = frontier->min_distance;
	}

	// blacklist if we've made no progress for a long time
	if (this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) {  // progress_timeout_ in seconds
		addFrontierToBlacklist(target_position);
		RCLCPP_WARN(logger_, "PROGRESS TIMEOUT: Adding current goal to blacklist");
		makePlan();
		return;
	}

	// we don't need to do anything if we still pursuing the same goal
	if (same_goal) {
		return;
	}

	RCLCPP_INFO(logger_, "Defining goal to send to move_base nav2");
	// send goal to move_base if we have something new to pursue
	auto goal = nav2_msgs::action::NavigateToPose::Goal();
	goal.pose.pose.position = target_position;
	goal.pose.pose.orientation.w = 1.;
	goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
	goal.pose.header.stamp = rclcpp::Time(0);


	auto send_goal_options =
			rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
	// send_goal_options.goal_response_callback =
	// std::bind(&Explore::goal_response_callback, this, _1);
	// send_goal_options.feedback_callback =
	//   std::bind(&Explore::navigationFeedback, this, _1, _2);

	RCLCPP_INFO(this->get_logger(), "defining response callback");
	send_goal_options.goal_response_callback =
    [this](const NavigationGoalHandle::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(logger_, "Goal was rejected by move_base");
			{
				std::lock_guard<std::mutex> lock(goal_in_progress_mutex_);
				goal_in_progress_ = false;
			}
            return;
        }
        RCLCPP_INFO(logger_, "Goal accepted by move_base");
	};
	RCLCPP_INFO(this->get_logger(), "Response Callback defined");


	RCLCPP_INFO(this->get_logger(), "defining result callback");
	send_goal_options.result_callback =
		[this, target_position](const NavigationGoalHandle::WrappedResult& result)
		{
			reachedGoal(result, target_position);
		};
	RCLCPP_INFO(this->get_logger(), "Result Callback defined");


	// Track when we started pursuing this goal
	goal_start_time_ = this->now();
	goal_start_distance_ = frontier->min_distance;
	current_goal_ = target_position;


	RCLCPP_INFO(this->get_logger(), "Locking Mutex and sending goal to move base nav2");
	{
		std::lock_guard<std::mutex> lock(goal_in_progress_mutex_);
		goal_in_progress_ = true;

		RCLCPP_INFO(this->get_logger(), "Mutex Locked!");
	} // Scope to release mutex lock before waiting for result callback


	auto future = move_base_client_->async_send_goal(goal, send_goal_options);

	RCLCPP_INFO(this->get_logger(), "async_send_goal called");


	// if (future.get() == nullptr) 
	// {
	// 	RCLCPP_ERROR(logger_, "Failed to send goal to move_base nav2");
	// 	return;
	// }
	RCLCPP_INFO(logger_, "MakePlan finished sending goal to move_base nav2");
}

bool Explore::goalBlacklisted(const geometry_msgs::msg::Point& goal)
{
	nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

	RCLCPP_INFO(this->get_logger(), "Checking if goal (%.2f, %.2f) is on blacklist", goal.x, goal.y);

	// check if a goal is on the blacklist for goals that we're pursuing
	for (auto& frontier_blacklist : frontier_blacklist_) 
	{
			double x_diff = fabs(goal.x - frontier_blacklist.point.x);
			double y_diff = fabs(goal.y - frontier_blacklist.point.y);

			if (x_diff < tolerance_for_position_matching_ * costmap2d->getResolution() &&
				y_diff < tolerance_for_position_matching_ * costmap2d->getResolution())
			{
				if (frontier_blacklist.tries <= max_retries_per_frontier_) 
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Goal matches Blacklist Frontier at x: "
						<< frontier_blacklist.point.x << ", y: " << frontier_blacklist.point.y);
					RCLCPP_INFO(this->get_logger(), "Max Tries not achieved yet.. Frontier not yet blacklisted..");
					RCLCPP_INFO_STREAM(this->get_logger(), "Current tries for this frontier: " << frontier_blacklist.tries << "\n");

					return false;
				}
				else
				{
					RCLCPP_WARN_STREAM(this->get_logger(), "Frontier at x: "
						<< frontier_blacklist.point.x << ", y: " << frontier_blacklist.point.y
						<< " is blacklisted after " << frontier_blacklist.tries << " tries.");
					return true;
				}
			}
	}
	return false;
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
	nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
	for (const auto& frontier_blacklist : frontier_blacklist_) 
	{
		double x_diff = fabs(goal.x - frontier_blacklist.point.x);
		double y_diff = fabs(goal.y - frontier_blacklist.point.y);

		if (x_diff < tolerance_for_position_matching_ * costmap2d->getResolution() &&
				y_diff < tolerance_for_position_matching_ * costmap2d->getResolution())
		{
			return true;
		}
	}
	return false;
}

// void Explore::navigationFeedback(
// 	rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
// 	const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
// {
// 	if (!feedback || !goal_handle) {
// 		return;
// 	}

// 	double current_distance = feedback->distance_remaining;
// 	rclcpp::Time now = this->now();
// 	double elapsed_time = (now - goal_start_time_).seconds();
// 	double progress_made = goal_start_distance_ - current_distance;

// 	RCLCPP_DEBUG(logger_, "Navigation feedback: distance_remaining=%.2f m, elapsed_time=%.1f s", 
// 		current_distance, elapsed_time);

// 	// Check if robot is making progress
// 	if (elapsed_time > 5.0) {  // After 5 seconds, check progress
// 		double progress_rate = progress_made / elapsed_time;

// 		RCLCPP_INFO_STREAM(logger_, "Progress: made " << progress_made << " m in " 
// 			<< elapsed_time << " s (rate: " << progress_rate << " m/s)");

// 		// If robot hasn't moved significantly in the time window, consider it stuck
// 		if (progress_made < min_travel_distance_for_abort_) {
// 			RCLCPP_WARN(logger_, 
// 				"Robot making insufficient progress (%.3f m in %.1f s). Aborting goal and adding to blacklist.",
// 				progress_made, elapsed_time);
			
// 			// Cancel the goal and add it to blacklist
// 			move_base_client_->async_cancel_goal(goal_handle);
// 			addFrontierToBlacklist(current_goal_);
// 			return;
// 		}
// 	}

// 	// If we've made no progress for progress_timeout_ seconds, abort
// 	if (progress_made > 0.01) {  // Made some progress recently
// 		last_progress_ = now;
// 	}
	
// 	if (now - last_progress_ > tf2::durationFromSec(progress_timeout_)) {
// 		RCLCPP_WARN(logger_, "No progress for %.1f seconds. Aborting goal.", progress_timeout_);
// 		move_base_client_->async_cancel_goal(goal_handle);
// 		addFrontierToBlacklist(current_goal_);
// 		return;
// 	}
// }

void Explore::reachedGoal(	const NavigationGoalHandle::WrappedResult& result,
							const geometry_msgs::msg::Point& frontier_goal)
{
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
		{
			std::lock_guard<std::mutex> lock(goal_in_progress_mutex_);
			goal_in_progress_ = false;

			RCLCPP_INFO(logger_, "Goal was successful");
			return;
		}
		case rclcpp_action::ResultCode::ABORTED:
		{
			RCLCPP_INFO(logger_, "Goal was aborted");
			addFrontierToBlacklist(frontier_goal);
			return;
		}
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_INFO(logger_, "Goal was canceled");
			return;
		default:
			RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
			return;
	}
	// find new goal immediately regardless of planning frequency.
	// execute via timer to prevent dead lock in move_base_client (this is
	// callback for sendGoal, which is called in makePlan). the timer must live
	// until callback is executed.
	// oneshot_ = relative_nh_.createTimer(
	//     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
	//     true);

	// TODO: Implement this with ros2 timers?
	// Because of the async nature of ros2 calls I think this is not needed.
	// makePlan();
}

void Explore::start()
{
	RCLCPP_INFO(logger_, "Exploration started.");
}

void Explore::stop()
{
	move_base_client_->async_cancel_all_goals();
	exploring_timer_->cancel();
	RCLCPP_INFO(logger_, "Exploration stopped.");
}

// Add the frontier to the blacklist.. if the frontier is already on the blacklist, increment its tries
void Explore::addFrontierToBlacklist(const geometry_msgs::msg::Point& frontier_goal)
{
	if (frontier_blacklist_.empty())
	{
		RCLCPP_WARN(logger_, "Blacklist is empty, adding frontier to blacklist without checking for duplicates..");
		FrontierBlacklist blacklisted_goal;
		blacklisted_goal.point = frontier_goal;
		++blacklisted_goal.tries;
		frontier_blacklist_.push_back(blacklisted_goal);
		return;
	}
	// check if goal corresponds to any frontier in the frontier_blacklist_.
	auto blacklisted_item = std::find_if(frontier_blacklist_.begin(), frontier_blacklist_.end(),
		[this](const explore::FrontierBlacklist& goal)
		{
			return goalOnBlacklist(goal.point);
		});
	// If not, add this goal to the blacklist
	if (blacklisted_item == frontier_blacklist_.end())
	{
		RCLCPP_WARN(logger_, "First time trying to add this frontier to the blacklist, not blacklisting yet..");
		FrontierBlacklist blacklisted_goal;
		blacklisted_goal.point = frontier_goal;
		++blacklisted_goal.tries;
		frontier_blacklist_.push_back(blacklisted_goal);
		return;
	}
	// If it matches, increment the 'tries' of the frontier in the blacklist
		RCLCPP_WARN_STREAM(logger_, "Incrementing tries for this frontier in the blacklist from "
			<< blacklisted_item->tries);
		blacklisted_item->tries++;
		RCLCPP_WARN_STREAM(logger_, "Incrementing tries for this frontier in the blacklist to "
			<< blacklisted_item->tries);
}

}  // namespace explore

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// ROS1 code
	/*
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	} */
	rclcpp::spin(std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
	rclcpp::shutdown();
	return 0;
}
