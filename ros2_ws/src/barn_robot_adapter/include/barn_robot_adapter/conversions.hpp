// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure ROS-message <-> barn_core conversions. Split out from the node so the
// message-boundary logic is unit-testable without a ROS graph. This is the
// only place ROS message types meet barn_core types.

#ifndef BARN_ROBOT_ADAPTER__CONVERSIONS_HPP_
#define BARN_ROBOT_ADAPTER__CONVERSIONS_HPP_

#include "barn_core/scan.hpp"
#include "barn_core/types.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace barn_robot_adapter
{

/// Wrap a LaserScan in a non-owning ScanView. The view borrows the message's
/// range buffer, so it must not outlive `scan`.
barn_core::ScanView to_view(const sensor_msgs::msg::LaserScan & scan);

/// Odometry -> planar pose (position + yaw from the orientation quaternion).
barn_core::Pose2D to_pose2d(const nav_msgs::msg::Odometry & odom);

/// Extract the planar velocity from a Twist.
barn_core::VelocityCommand from_twist(const geometry_msgs::msg::Twist & twist);

/// VelocityCommand -> unstamped Twist.
geometry_msgs::msg::Twist to_twist(const barn_core::VelocityCommand & cmd);

/// VelocityCommand -> TwistStamped with the given frame and stamp.
geometry_msgs::msg::TwistStamped to_twist_stamped(
  const barn_core::VelocityCommand & cmd, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace barn_robot_adapter

#endif  // BARN_ROBOT_ADAPTER__CONVERSIONS_HPP_
