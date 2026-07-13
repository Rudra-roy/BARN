// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_goal_adapter/goal_adapter_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace barn_goal_adapter
{

using namespace std::chrono_literals;

GoalAdapterNode::GoalAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("goal_adapter_node", options)
{
  planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/barn/goal");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/barn/pose");
  success_distance_ = declare_parameter<double>("success_distance", 1.0);
  feedback_period_s_ = declare_parameter<double>("feedback_period_s", 0.1);

  // Latched (transient_local) so a late-joining navigation core still gets the
  // goal that was published once at the start of the run.
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(1).transient_local());

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::QoS(10),
    std::bind(&GoalAdapterNode::pose_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this, "/navigate_to_pose",
    std::bind(&GoalAdapterNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GoalAdapterNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&GoalAdapterNode::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), action_cb_group_);

  RCLCPP_INFO(
    get_logger(), "goal_adapter_node ready: /navigate_to_pose -> %s (planning frame '%s')",
    goal_topic_.c_str(), planning_frame_.c_str());
}

rclcpp_action::GoalResponse GoalAdapterNode::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(
    get_logger(), "Received goal in frame '%s': (%.2f, %.2f)", goal->pose.header.frame_id.c_str(),
    goal->pose.pose.position.x, goal->pose.pose.position.y);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoalAdapterNode::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(get_logger(), "Goal cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoalAdapterNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Run the goal loop on a detached thread so the action server stays
  // responsive to feedback/cancel on the executor thread.
  std::thread{std::bind(&GoalAdapterNode::execute, this, std::placeholders::_1), goal_handle}
    .detach();
}

bool GoalAdapterNode::transform_to_planning_frame(
  const geometry_msgs::msg::PoseStamped & in, geometry_msgs::msg::PoseStamped & out)
{
  if (in.header.frame_id.empty() || in.header.frame_id == planning_frame_) {
    out = in;
    out.header.frame_id = planning_frame_;
    return true;
  }
  try {
    out = tf_buffer_->transform(in, planning_frame_, tf2::durationFromSec(0.2));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "Could not transform goal from '%s' to '%s': %s; using verbatim",
      in.header.frame_id.c_str(), planning_frame_.c_str(), ex.what());
    out = in;
    return false;
  }
}

void GoalAdapterNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  geometry_msgs::msg::PoseStamped goal_pose;
  transform_to_planning_frame(goal->pose, goal_pose);
  goal_pose.header.stamp = now();

  // Publish the latched internal goal exactly once.
  goal_pub_->publish(goal_pose);
  RCLCPP_INFO(
    get_logger(), "Published internal goal (%.2f, %.2f) in '%s'", goal_pose.pose.position.x,
    goal_pose.pose.position.y, planning_frame_.c_str());

  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();
  const auto start = now();
  rclcpp::Rate loop(1.0 / feedback_period_s_);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      // The evaluator tears down its CLI action client and this server in the
      // same launch shutdown wave. The goal can disappear between
      // is_canceling() and publishing the terminal result; that is a benign
      // shutdown race and must not abort the adapter process.
      try {
        if (goal_handle->is_active()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(get_logger(), "Goal canceled");
        }
      } catch (const std::runtime_error & error) {
        RCLCPP_DEBUG(get_logger(), "Goal vanished during cancellation: %s", error.what());
      }
      return;
    }

    const double d = distance_to_goal(goal_pose);
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      feedback->current_pose = latest_pose_;
    }
    feedback->distance_remaining = static_cast<float>(d);
    feedback->navigation_time = now() - start;
    try {
      goal_handle->publish_feedback(feedback);
    } catch (const std::runtime_error & error) {
      RCLCPP_DEBUG(get_logger(), "Goal vanished while publishing feedback: %s", error.what());
      return;
    }

    if (d <= success_distance_) {
      try {
        if (goal_handle->is_active()) {
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal reached (%.2f m <= %.2f m)", d, success_distance_);
        }
      } catch (const std::runtime_error & error) {
        RCLCPP_DEBUG(get_logger(), "Goal vanished while publishing success: %s", error.what());
      }
      return;
    }
    loop.sleep();
  }
}

void GoalAdapterNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(pose_mutex_);
  latest_pose_ = *msg;
  have_pose_ = true;
}

double GoalAdapterNode::distance_to_goal(const geometry_msgs::msg::PoseStamped & goal) const
{
  std::lock_guard<std::mutex> lk(pose_mutex_);
  if (!have_pose_) {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = goal.pose.position.x - latest_pose_.pose.position.x;
  const double dy = goal.pose.position.y - latest_pose_.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace barn_goal_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<barn_goal_adapter::GoalAdapterNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
