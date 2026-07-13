// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_
#define BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_

#include <vector>

#include "barn_classical/collision_checker.hpp"
#include "barn_core/occupancy.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

using Path2D = std::vector<barn_core::Pose2D>;

struct AStarParams
{
  int yaw_bins{16};
  double step_size{0.20};
  double goal_tolerance{0.20};
  double unknown_cost_multiplier{1.8};
  // Must exceed the default unknown traversal multiplier so the initial
  // mostly-unknown map is searched toward the goal instead of expanding a
  // broad equal-cost wavefront. The lattice is deliberately weighted: BARN
  // needs a safe feasible path inside the 100 ms deadline, not an optimal one.
  double heuristic_weight{3.0};
  double clearance_weight{0.08};
  double turn_weight{0.12};
  double rotate_weight{0.20};
  double timeout_ms{100.0};
  bool allow_unknown{true};
  Footprint footprint{};
};

struct PlannerStats
{
  double elapsed_ms{0.0};
  std::size_t expanded{0};
  bool timed_out{false};
};

/// Footprint-aware differential-drive state-lattice A*.
class GlobalPlannerAStar
{
public:
  explicit GlobalPlannerAStar(const AStarParams & params = {}) : params_(params) {}

  Path2D plan(
    const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & start,
    const barn_core::Goal2D & goal);

  const PlannerStats & stats() const { return stats_; }
  const AStarParams & params() const { return params_; }

private:
  AStarParams params_;
  PlannerStats stats_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_
