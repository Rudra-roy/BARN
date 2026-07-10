// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// The runnable vertical-slice controller (M3). Subscribes the latched goal, the
// robot pose, and the relayed scan; runs the pure GoalSeeker control law on a
// fixed-rate timer; publishes a desired velocity command for barn_safety.
//
// It commands motion as soon as it has a pose and a goal (no wait for a map or
// a plan), which is what trips the BARN evaluator's >0.1 m motion clock.

#ifndef BARN_CLASSICAL__GOAL_SEEKER_NODE_HPP_
#define BARN_CLASSICAL__GOAL_SEEKER_NODE_HPP_

#include <memory>
#include <string>

#include "barn_classical/goal_seeker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace barn_classical
{

class GoalSeekerNode : public rclcpp::Node
{
public:
  explicit GoalSeekerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_step();

  double front_clearance() const;

  // Parameters / config.
  std::string goal_topic_;
  std::string pose_topic_;
  std::string scan_topic_;
  std::string cmd_topic_;
  std::string cmd_frame_;
  double control_rate_hz_;
  double front_sector_rad_;  ///< half-width of the forward clearance sector

  GoalSeeker seeker_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  barn_core::Goal2D goal_;
  barn_core::Pose2D pose_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  bool have_goal_{false};
  bool have_pose_{false};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__GOAL_SEEKER_NODE_HPP_
