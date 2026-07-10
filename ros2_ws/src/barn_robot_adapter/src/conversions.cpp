// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_robot_adapter/conversions.hpp"

#include "barn_core/geometry.hpp"

namespace barn_robot_adapter
{

barn_core::ScanView to_view(const sensor_msgs::msg::LaserScan & scan)
{
  barn_core::ScanView view;
  view.ranges = scan.ranges.data();
  view.count = scan.ranges.size();
  view.angle_min = scan.angle_min;
  view.angle_increment = scan.angle_increment;
  view.range_min = scan.range_min;
  view.range_max = scan.range_max;
  return view;
}

barn_core::Pose2D to_pose2d(const nav_msgs::msg::Odometry & odom)
{
  barn_core::Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  const auto & q = odom.pose.pose.orientation;
  pose.yaw = barn_core::yaw_from_quat(q.x, q.y, q.z, q.w);
  return pose;
}

barn_core::VelocityCommand from_twist(const geometry_msgs::msg::Twist & twist)
{
  return barn_core::VelocityCommand{twist.linear.x, twist.angular.z};
}

geometry_msgs::msg::Twist to_twist(const barn_core::VelocityCommand & cmd)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.v;
  twist.angular.z = cmd.w;
  return twist;
}

geometry_msgs::msg::TwistStamped to_twist_stamped(
  const barn_core::VelocityCommand & cmd, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.twist = to_twist(cmd);
  return msg;
}

}  // namespace barn_robot_adapter
