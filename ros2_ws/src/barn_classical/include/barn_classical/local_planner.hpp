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
  Footprint footprint{};
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

private:
  LocalPlannerParams params_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__LOCAL_PLANNER_HPP_
