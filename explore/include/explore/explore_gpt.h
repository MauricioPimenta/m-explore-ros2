#pragma once

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>

#include <unordered_map>
#include <string>
#include <vector>

namespace explore
{

class Explore : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit Explore(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Explore();

private:
  void timer_callback();
  void visualizeFrontiers(const std::vector<frontier_exploration::Frontier>& frontiers);

  void reachedGoal(const GoalHandleNavigate::WrappedResult& result);
  void startMoving();
  void stopMoving();

  geometry_msgs::msg::Pose getCurrentRobotPoseSafe(bool* ok = nullptr) const;

  // Frontier retry bookkeeping
  struct FrontierAttempt
  {
    int failures = 0;           // total failures (timeouts + aborts)
    int aborts = 0;             // aborted results
    int timeouts = 0;           // progress timeouts
    rclcpp::Time last_attempt_time{0, 0, RCL_ROS_TIME};

    geometry_msgs::msg::Pose start_pose;
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  };

  std::string frontierKey(const geometry_msgs::msg::Point& p) const;
  void registerAttemptStart(const std::string& key);
  bool shouldBlacklist(const std::string& key) const;
  void addToBlacklist(const std::string& key, const geometry_msgs::msg::Point& goal);
  void clearAttemptState(const std::string& key);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  Costmap2DClient costmap_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr move_base_client_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  std::vector<geometry_msgs::msg::Point> blacklist_;

  geometry_msgs::msg::Point prev_goal_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  std::shared_ptr<GoalHandleNavigate> current_goal_handle_;
  std::shared_future<std::shared_ptr<GoalHandleNavigate>> goal_handle_future_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  // Retry state
  std::unordered_map<std::string, FrontierAttempt> attempts_;
  std::string current_frontier_key_;

  // Params
  bool visualize_;
  double planner_frequency_;
  double progress_timeout_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double transform_tolerance_;
  double min_frontier_size_;

  // New params
  int max_retries_per_frontier_;
  double frontier_key_resolution_;
  double min_travel_distance_for_abort_;
};

}  // namespace explore
