// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// B-Spline + TOPP-RA local planner — standalone header for A/B comparison
// with the original elastic-band LocalPlanner.

#ifndef BARN_CLASSICAL__LOCAL_PLANNER_BSPLINE_HPP_
#define BARN_CLASSICAL__LOCAL_PLANNER_BSPLINE_HPP_

#include <vector>

#include "barn_classical/collision_checker.hpp"
#include "barn_classical/global_planner_astar.hpp"
#include "barn_classical/local_planner.hpp"   // for LocalTrajectory, ProfileDebug
#include "barn_core/types.hpp"
#include "barn_core/distance_field.hpp"

namespace barn_classical
{

struct LocalPlannerBsplineParams
{
  // ---- Path extraction (unchanged) ----
  double horizon_m{4.0};

  // ---- B-Spline optimisation ----
  // Number of control points for the cubic B-spline.
  // EASILY CHANGEABLE: increase for finer obstacle avoidance in dense worlds,
  // decrease for faster planning in open space.  ~20 is a good default for
  // an 8 m horizon (one control point every ~0.4 m).
  int n_control_points{20};

  int spline_opt_iterations{30};       // gradient descent iterations
  double spline_smooth_weight{0.40};   // second-derivative / smoothness
  double spline_anchor_weight{0.15};   // pull toward initial placement
  double spline_obstacle_weight{0.30}; // distance-field repulsion
  double spline_curvature_weight{0.5}; // penalty when kappa > kappa_max

  // ---- Differential-drive constraints ----
  // Track width W [m] — distance between left and right wheel contact patches.
  // kappa_max = 2/W prevents turning-in-place singularities.
  double track_width{0.4318};          // 2 * half_width for Jackal

  // Individual wheel velocity limit [m/s].
  double wheel_v_max{3.5};

  // Individual wheel acceleration limit [m/s^2].
  double wheel_a_max{2.5};

  // ---- TOPP-RA ----
  // Number of arc-length grid points for the backward-forward sweep.
  int topp_ra_samples{100};

  // ---- Speed limits (carried forward for post-processing) ----
  double max_speed{2.0};
  double unknown_speed{0.4};
  double max_yaw_rate{1.5};
  double max_lateral_accel{1.5};
  double braking_decel{2.0};

  // ---- Clearance / obstacle avoidance ----
  double desired_clearance{0.55};
  double stop_margin{0.08};
  double open_clearance{0.80};
  double tight_lateral_accel{0.50};
  double crawl_speed{0.35};

  // ---- Entry-heading gate (unchanged) ----
  double heading_align_distance{0.35};

  // ---- Spline sampling resolution ----
  // Dense evaluation spacing along the B-spline arc [m].
  double spline_sample_step{0.05};

  Footprint footprint{};
};

/// Fits a cubic B-spline to the global path, optimises control points with
/// obstacle / smoothness / curvature penalties, then runs TOPP-RA to produce
/// a time-optimal velocity profile respecting differential-drive wheel limits.
class LocalPlannerBspline
{
public:
  explicit LocalPlannerBspline(const LocalPlannerBsplineParams & params = {}) : params_(params) {}

  LocalTrajectory plan(
    const Path2D & global_path, const barn_core::Pose2D & pose,
    const barn_core::OccupancyGrid2D & grid,
    const barn_core::DistanceField2D * precomputed_distance_field = nullptr) const;

  /// Terms behind the last computed v_ref for the first trajectory point.
  const ProfileDebug & profile_debug() const {return debug_;}

private:
  LocalPlannerBsplineParams params_;
  mutable ProfileDebug debug_{};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__LOCAL_PLANNER_BSPLINE_HPP_
