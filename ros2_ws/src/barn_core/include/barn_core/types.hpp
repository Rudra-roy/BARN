// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure, dependency-free value types shared across every navigation track.
// This header must NOT include rclcpp, ROS messages, Gazebo, or any middleware
// type. Keeping it clean is what lets the same algorithms run on the physical
// Jackal after swapping only the ROS adapter (barn_robot_adapter).

#ifndef BARN_CORE__TYPES_HPP_
#define BARN_CORE__TYPES_HPP_

namespace barn_core
{

/// A 2-D pose in a planar frame (e.g. base_link expressed in odom).
struct Pose2D
{
  double x{0.0};    ///< metres
  double y{0.0};    ///< metres
  double yaw{0.0};  ///< radians, in (-pi, pi]
};

/// A 2-D navigation goal with an acceptance tolerance.
struct Goal2D
{
  double x{0.0};    ///< metres, in the planning frame
  double y{0.0};    ///< metres
  double yaw{0.0};  ///< radians (optional final heading; unused by the goal-seeker)
  double tol{0.8};  ///< metres; internal acceptance radius (see note below)
};

/// A differential-drive velocity command.
///
/// NOTE: the BARN evaluator scores physical goal distance, not any internal
/// tolerance. Keep `Goal2D::tol` smaller than the evaluator's 1 m success
/// radius so the robot physically clears the goal band before we stop.
struct VelocityCommand
{
  double v{0.0};  ///< linear velocity, m/s (forward positive)
  double w{0.0};  ///< angular velocity, rad/s (counter-clockwise positive)
};

/// Planar robot state used by predictive controllers.
struct State2D
{
  Pose2D pose;
  double v{0.0};  ///< measured forward velocity, m/s
  double w{0.0};  ///< measured yaw rate, rad/s
};

/// A timestamp-free point on a locally planned trajectory.
struct TrajectoryPoint
{
  Pose2D pose;
  double v_ref{0.0};       ///< desired forward speed, m/s
  double clearance{0.0};   ///< nearest obstacle distance, m
  bool in_unknown{false};  ///< true if this point crosses unobserved map space
};

/// Kinematic / dynamic limits enforced by barn_safety.
struct Limits
{
  double v_max{2.0};      ///< m/s   (BARN maximum robot speed)
  double w_max{1.5};      ///< rad/s
  double a_lin{2.5};      ///< m/s^2 linear acceleration magnitude
  double a_ang{3.0};      ///< rad/s^2 angular acceleration magnitude
};

}  // namespace barn_core

#endif  // BARN_CORE__TYPES_HPP_
