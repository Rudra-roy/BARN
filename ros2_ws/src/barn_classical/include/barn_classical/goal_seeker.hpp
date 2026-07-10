// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure reactive goal-seeking control law used by the M0-M3 vertical slice.
// Header-only so it unit-tests against barn_core alone. This is intentionally
// trivial: turn toward the goal, creep while poorly aligned, ramp speed with
// forward clearance, stop at the goal or when an obstacle is close. Track A
// replaces this with the map -> A* -> local planner -> controller pipeline; the
// stub interfaces for those live alongside this file.

#ifndef BARN_CLASSICAL__GOAL_SEEKER_HPP_
#define BARN_CLASSICAL__GOAL_SEEKER_HPP_

#include <cmath>

#include "barn_core/geometry.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

struct GoalSeekerParams
{
  double v_nominal{0.5};      ///< cruise speed when aligned and clear, m/s
  double v_max{2.0};          ///< hard cap, m/s
  double w_max{1.2};          ///< angular cap, rad/s
  double k_ang{1.5};          ///< proportional heading gain
  double heading_tol{0.35};   ///< rad; above this we creep instead of cruise
  double stop_distance{0.45}; ///< m; forward clearance below this -> v = 0
  double slow_distance{1.2};  ///< m; clearance below this ramps speed down
  double goal_tolerance{0.8}; ///< m; internal stop radius (< evaluator's 1 m)
  double creep_fraction{0.15};///< fraction of v_nominal while turning in place
};

class GoalSeeker
{
public:
  explicit GoalSeeker(const GoalSeekerParams & p = {}) : p_(p) {}
  void set_params(const GoalSeekerParams & p) { p_ = p; }
  const GoalSeekerParams & params() const { return p_; }

  /// Compute a velocity command from the current pose, the goal, and the
  /// minimum forward clearance (metres) within the front sector.
  barn_core::VelocityCommand compute(
    const barn_core::Pose2D & pose, const barn_core::Goal2D & goal, double front_clearance) const
  {
    barn_core::VelocityCommand cmd{0.0, 0.0};

    if (barn_core::dist2d(pose, goal) <= p_.goal_tolerance) {
      return cmd;  // arrived
    }

    const double he = barn_core::heading_to(pose, goal);
    cmd.w = barn_core::clamp(p_.k_ang * he, -p_.w_max, p_.w_max);

    if (front_clearance < p_.stop_distance) {
      cmd.v = 0.0;  // obstacle too close; rotate only
    } else if (std::abs(he) > p_.heading_tol) {
      cmd.v = p_.creep_fraction * p_.v_nominal;  // realign before committing
    } else {
      const double span = p_.slow_distance - p_.stop_distance;
      const double scale =
        span > 1e-6 ? barn_core::clamp((front_clearance - p_.stop_distance) / span, 0.0, 1.0) : 1.0;
      cmd.v = barn_core::clamp(p_.v_nominal * scale, 0.0, p_.v_max);
    }
    return cmd;
  }

private:
  GoalSeekerParams p_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__GOAL_SEEKER_HPP_
