// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_SAFETY__SWEPT_FOOTPRINT_SHIELD_HPP_
#define BARN_SAFETY__SWEPT_FOOTPRINT_SHIELD_HPP_

#include <string>
#include <vector>

#include "barn_core/types.hpp"

namespace barn_safety
{

struct ObstaclePoint
{
  double x{0.0};
  double y{0.0};
};

struct ShieldParams
{
  double half_length{0.254};
  double half_width{0.2159};
  // Retained for config compatibility but NO LONGER part of the hard-veto box:
  // a static footprint_margin buffer hard-froze the robot on obstacles in the
  // margin shell (see swept_footprint_shield.cpp). Anticipatory clearance now
  // comes solely from sweeping the emergency-margin box along the stopping
  // envelope. emergency_margin is the final hard buffer.
  double footprint_margin{0.04};
  // Reduced from 0.10 m: the MPC's obstacle slack already enforces clearance;
  // a 10 cm extra margin was vetoing commands inside valid narrow BARN corridors.
  double emergency_margin{0.02};
  // Reduced from 1.0 s: still covers full braking distance at max speed plus
  // latency, but no longer sweeps through an entire corridor before moving.
  double horizon_s{0.6};
  double integration_dt{0.05};
  double braking_decel{2.5};
  double latency_s{0.15};
  double scale_step{0.05};

  // --- Escape from an already-occupied veto box --------------------------
  //
  // safe_at_scale() checks the box at the CURRENT pose first, so if an
  // obstacle is already inside it, every scale fails -- including a reverse or
  // an in-place rotation that would move away. The scale search then returns a
  // hard veto and the robot is commanded to zero forever: it cannot move, so
  // the obstacle never leaves the box, so it cannot move. Observed as a robot
  // frozen mid-corridor until the 100 s trial timeout, with the planner's own
  // reverse-to-clearance recovery running but producing no motion.
  //
  // The trap is narrow but reachable: safety_node discards returns inside
  // half_extent + 1 cm as self-hits, so only returns in the shell between that
  // and half_extent + emergency_margin can occupy the box.
  //
  // When trapped, search a small set of slow escape motions and accept one only
  // if it strictly increases the minimum clearance and never decreases it along
  // the way. This is deliberately a relaxation of a hard veto, justified
  // because the state it applies to -- an obstacle within a centimetre of the
  // body -- is one where holding still is not safer than crawling away from it.
  bool escape_enable{true};
  double escape_speed{0.15};
  double escape_yaw_rate{0.5};
  double escape_horizon_s{0.5};
};

struct ShieldResult
{
  barn_core::VelocityCommand command{};
  double scale{0.0};
  double minimum_clearance{0.0};
  std::string reason{"no_scan"};
  std::vector<barn_core::Pose2D> envelope;
};

/// Independent point-cloud swept-footprint veto. It has no planner state.
class SweptFootprintShield
{
public:
  explicit SweptFootprintShield(const ShieldParams & params = {}) : params_(params) {}

  /// `allow_escape` gates the escape search. It must stay false until the robot
  /// has been at a dead stop for a sustained period: the escape injects motion
  /// the planner never asked for, and firing it on a transient veto turns the
  /// shield from a filter into an actuator. Doing that unconditionally pushed
  /// the robot off its start pose during the evaluator's reset, tripping the
  /// "has it moved" check ~30 s early on every trial. The freeze this exists to
  /// break lasts 75+ s, so waiting a second costs nothing.
  ShieldResult apply(
    const barn_core::VelocityCommand & desired,
    const std::vector<ObstaclePoint> & obstacles,
    bool allow_escape = true) const;

private:
  bool safe_at_scale(
    const barn_core::VelocityCommand & desired, double scale,
    const std::vector<ObstaclePoint> & obstacles,
    double & minimum_clearance, std::vector<barn_core::Pose2D> * envelope) const;

  /// Signed clearance of the veto box at `pose`: >= 0 outside (distance to the
  /// nearest obstacle), < 0 inside (negative penetration depth). Unlike the
  /// clamped clearance used by safe_at_scale this keeps varying once an
  /// obstacle is inside the box, which is what makes "getting less trapped"
  /// measurable.
  double signed_clearance(
    const barn_core::Pose2D & pose,
    const std::vector<ObstaclePoint> & obstacles) const;

  /// Last resort when every scale is vetoed: the slowest motion that strictly
  /// increases signed clearance, or a hard veto if nothing does.
  ShieldResult find_escape(
    const barn_core::VelocityCommand & desired,
    const std::vector<ObstaclePoint> & obstacles) const;

  ShieldParams params_;
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__SWEPT_FOOTPRINT_SHIELD_HPP_
