// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// The single ROS <-> robot boundary shared by all three tracks.
//
// Ingress:  /front/scan            -> /barn/scan   (relay)
//           platform/odom/filtered -> /barn/pose   (base_link in odom, via TF)
// Egress:   /barn/cmd_safe         -> /cmd_vel      (Twist or TwistStamped)
//
// The egress message type is a PARAMETER (`cmd_vel_type`) because the BARN 2026
// baseline routes commands through a velocity smoother whose input type may be
// geometry_msgs/TwistStamped or geometry_msgs/Twist. This node is the ONE place
// the true wire type is confirmed against the live graph; flip the param, no
// recompile. See docs/decisions/0003-configurable-cmd-vel-type.md.

#ifndef BARN_ROBOT_ADAPTER__ROBOT_ADAPTER_NODE_HPP_
#define BARN_ROBOT_ADAPTER__ROBOT_ADAPTER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace barn_robot_adapter
{

class RobotAdapterNode : public rclcpp::Node
{
public:
  explicit RobotAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void safe_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // Parameters.
  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string internal_scan_topic_;
  std::string internal_pose_topic_;
  std::string safe_cmd_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_vel_type_;   ///< "twist_stamped" (default) or "twist"
  std::string cmd_vel_frame_;

  // Ingress.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Egress (exactly one of these is created, per cmd_vel_type).
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_unstamped_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace barn_robot_adapter

#endif  // BARN_ROBOT_ADAPTER__ROBOT_ADAPTER_NODE_HPP_
