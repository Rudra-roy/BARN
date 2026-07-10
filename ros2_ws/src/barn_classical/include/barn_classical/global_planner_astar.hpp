// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M5). Global path planning over the online occupancy grid.
// The slice does not use this; Track A grows into it. Plans through FREE and
// UNKNOWN cells (UNKNOWN at higher cost) so the robot can move into partially
// observed space and replan as geometry is revealed.

#ifndef BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_
#define BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_

#include <vector>

#include "barn_core/occupancy.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

using Path2D = std::vector<barn_core::Pose2D>;

struct AStarParams
{
  double unknown_cost_multiplier{1.5};  ///< penalty for traversing UNKNOWN cells
  bool allow_unknown{true};
};

class GlobalPlannerAStar
{
public:
  explicit GlobalPlannerAStar(const AStarParams & p = {}) : p_(p) {}

  /// Plan a path from `start` to `goal` over `grid`. Returns an empty path if
  /// no route is found.
  ///
  /// STUB: currently returns a two-point straight line start->goal so the
  /// pipeline type-checks and can be wired end-to-end before A* exists.
  Path2D plan(
    const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & start,
    const barn_core::Goal2D & goal) const;

private:
  AStarParams p_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__GLOBAL_PLANNER_ASTAR_HPP_
