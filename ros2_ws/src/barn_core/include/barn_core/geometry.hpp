// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure planar-geometry helpers. No ROS, no middleware.

#ifndef BARN_CORE__GEOMETRY_HPP_
#define BARN_CORE__GEOMETRY_HPP_

#include "barn_core/types.hpp"

namespace barn_core
{

/// Wrap an angle to the half-open interval (-pi, pi].
double wrap_angle(double angle);

/// Clamp `value` into [lo, hi]. Assumes lo <= hi.
double clamp(double value, double lo, double hi);

/// Euclidean distance between a pose and a goal (ignores orientation).
double dist2d(const Pose2D & from, const Goal2D & to);

/// Signed heading error from `from` toward `to`, in (-pi, pi].
/// Positive means the goal is to the robot's left.
double heading_to(const Pose2D & from, const Goal2D & to);

/// Extract a planar yaw (radians) from a quaternion given as raw components.
/// Kept quaternion-as-doubles so this header stays free of ROS/tf2 types.
double yaw_from_quat(double x, double y, double z, double w);

}  // namespace barn_core

#endif  // BARN_CORE__GEOMETRY_HPP_
