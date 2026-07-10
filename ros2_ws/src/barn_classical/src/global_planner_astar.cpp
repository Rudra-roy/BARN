// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/global_planner_astar.hpp"

namespace barn_classical
{

Path2D GlobalPlannerAStar::plan(
  const barn_core::OccupancyGrid2D &, const barn_core::Pose2D & start,
  const barn_core::Goal2D & goal) const
{
  // STUB (M5): straight line so downstream stages can be exercised. Replace
  // with a real A* expansion over the grid's FREE/UNKNOWN cells.
  Path2D path;
  path.push_back(start);
  barn_core::Pose2D end;
  end.x = goal.x;
  end.y = goal.y;
  end.yaw = goal.yaw;
  path.push_back(end);
  return path;
}

}  // namespace barn_classical
