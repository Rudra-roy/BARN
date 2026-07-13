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

  ShieldResult apply(
    const barn_core::VelocityCommand & desired,
    const std::vector<ObstaclePoint> & obstacles) const;

private:
  bool safe_at_scale(
    const barn_core::VelocityCommand & desired, double scale,
    const std::vector<ObstaclePoint> & obstacles,
    double & minimum_clearance, std::vector<barn_core::Pose2D> * envelope) const;

  ShieldParams params_;
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__SWEPT_FOOTPRINT_SHIELD_HPP_
