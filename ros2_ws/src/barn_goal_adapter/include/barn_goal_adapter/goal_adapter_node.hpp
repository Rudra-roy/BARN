// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Compatibility action server that accepts the BARN evaluator's
// nav2_msgs/action/NavigateToPose goal and republishes it as a latched
// internal PoseStamped (/barn/goal) for whichever navigation core is active.
//
// IMPORTANT: the BARN evaluator judges success by physical goal distance, not
// by this action's result. We implement proper ACCEPT/EXECUTING/SUCCEEDED
// semantics for cleanliness and portability, but never gate motion on it.

#ifndef BARN_GOAL_ADAPTER__GOAL_ADAPTER_NODE_HPP_
#define BARN_GOAL_ADAPTER__GOAL_ADAPTER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace barn_goal_adapter
{

class GoalAdapterNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit GoalAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action-server callbacks.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Transform an incoming goal pose into the planning frame; returns false if
  // no transform is available (caller falls back to using it verbatim).
  bool transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped & in, geometry_msgs::msg::PoseStamped & out);

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  double distance_to_goal(const geometry_msgs::msg::PoseStamped & goal) const;

  // Parameters.
  std::string planning_frame_;   ///< frame the internal goal is expressed in (odom)
  std::string goal_topic_;       ///< latched internal goal topic (/barn/goal)
  std::string pose_topic_;       ///< current robot pose (/barn/pose)
  double success_distance_;      ///< [m] mark SUCCEEDED within this of the goal
  double feedback_period_s_;     ///< feedback publication period

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool have_pose_{false};
};

}  // namespace barn_goal_adapter

#endif  // BARN_GOAL_ADAPTER__GOAL_ADAPTER_NODE_HPP_
