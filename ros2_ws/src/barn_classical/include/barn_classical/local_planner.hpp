// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__LOCAL_PLANNER_HPP_
#define BARN_CLASSICAL__LOCAL_PLANNER_HPP_

#include <vector>

#include "barn_classical/collision_checker.hpp"
#include "barn_classical/global_planner_astar.hpp"
#include "barn_core/types.hpp"
#include "barn_core/distance_field.hpp"

namespace barn_classical
{

using LocalTrajectory = std::vector<barn_core::TrajectoryPoint>;

struct LocalPlannerParams
{
  double horizon_m{4.0};
  int elastic_iterations{8};
  double smooth_weight{0.35};
  double anchor_weight{0.20};
  double obstacle_weight{0.25};
  double desired_clearance{0.55};
  double max_speed{2.0};
  double unknown_speed{0.4};
  // Maximum yaw rate of the robot (rad/s). Used to compute the differential-
  // drive kinematic curvature speed limit: v_max = max_yaw_rate / curvature.
  double max_yaw_rate{1.5};
  double max_lateral_accel{1.5};
  double braking_decel{2.0};
  double stop_margin{0.08};
  // Distance over which the entry-heading speed gate fades to full speed.
  double heading_align_distance{1.0};
  // How far ahead the speed profile scans for the WORST curvature, applying it
  // to the current point so the robot pre-decelerates into corners instead of
  // braking once already in them.
  //
  // It is a max over a window, so it is one-sided: a single sharp corner pins
  // the speed for the whole window behind it. In a dense BARN field there is
  // almost always a corner within 3 m, so the profile rarely releases. Measured
  // against the worlds' own reference paths, mean v_ref by window length:
  //
  //     world      0 m    1.5 m    3.0 m
  //        66     3.10     1.86     1.49
  //       294     3.05     1.25     1.25
  //       210     2.92     1.16     1.16
  //
  // and world 294's live mean desired_v is 1.137 m/s against the 1.25 the model
  // predicts for 3.0 m -- so this term, not the shield and not the MPC, is what
  // sets the cruise speed on stall-free worlds. Was raised 1.5 -> 3.0 for
  // smoother deceleration, so shortening it may trade reliability for speed:
  // measure both, do not assume.
  double curvature_lookahead_m{3.0};
  // Clearance at which the surroundings count as fully open, and the multiplier
  // applied to max_lateral_accel there. The curvature speed limit interpolates
  // between the tight-space budget (at min_clearance) and gain x that budget (at
  // open_clearance), because the reason to slow for a curve in BARN is tracking
  // error near a wall, not vehicle dynamics. gain = 1.0 restores the old
  // constant-budget behaviour exactly.
  double open_clearance{1.20};
  double open_lateral_accel_gain{3.0};
  Footprint footprint{};
};

/// Per-term breakdown of how v_ref was arrived at for the FIRST trajectory
/// point -- the one the controller is about to track.
///
/// Measured need: across three slow trials on world 66, 32.5 s of stopped time
/// had the MPC reporting "solved", the planner reporting success and the shield
/// reporting clear, while the commanded speed was zero. Something in this
/// profile is zeroing v_ref and the aggregate telemetry cannot say which term.
struct ProfileDebug
{
  double curvature{0.0};        ///< lookahead (max-over-window) curvature, 1/m
  double curvature_speed{0.0};  ///< limit from that curvature alone
  double clearance_scale{1.0};  ///< side-clearance multiplier
  double heading_scale{1.0};    ///< entry-heading gate multiplier
  double v_ref{0.0};            ///< final, after every term
};

/// Extracts and clearance-refines a short trajectory from the global path.
class LocalPlanner
{
public:
  explicit LocalPlanner(const LocalPlannerParams & params = {}) : params_(params) {}

  LocalTrajectory plan(
    const Path2D & global_path, const barn_core::Pose2D & pose,
    const barn_core::OccupancyGrid2D & grid,
    const barn_core::DistanceField2D * precomputed_distance_field = nullptr) const;

  /// Terms behind the last computed v_ref for the first trajectory point.
  const ProfileDebug & profile_debug() const {return debug_;}

private:
  LocalPlannerParams params_;
  mutable ProfileDebug debug_{};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__LOCAL_PLANNER_HPP_
