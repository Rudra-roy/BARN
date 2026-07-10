// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure ROS-message -> barn_core conversion for the navigation goal. Split out
// so it can be unit-tested without spinning a ROS graph.

#ifndef BARN_GOAL_ADAPTER__GOAL_CONVERSIONS_HPP_
#define BARN_GOAL_ADAPTER__GOAL_CONVERSIONS_HPP_

#include "barn_core/geometry.hpp"
#include "barn_core/types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace barn_goal_adapter
{

/// Convert a PoseStamped goal into a planar Goal2D with acceptance tolerance.
inline barn_core::Goal2D to_goal2d(const geometry_msgs::msg::PoseStamped & p, double tol)
{
  barn_core::Goal2D g;
  g.x = p.pose.position.x;
  g.y = p.pose.position.y;
  g.yaw = barn_core::yaw_from_quat(
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
  g.tol = tol;
  return g;
}

}  // namespace barn_goal_adapter

#endif  // BARN_GOAL_ADAPTER__GOAL_CONVERSIONS_HPP_
