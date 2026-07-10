// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M7). Local passage planner: given the global path and the
// latest scan/grid, produce a short-horizon, kinematically feasible trajectory
// (e.g. DWA / lattice rollout) that threads the nearest narrow passage.

#ifndef BARN_CLASSICAL__LOCAL_PLANNER_HPP_
#define BARN_CLASSICAL__LOCAL_PLANNER_HPP_

#include "barn_classical/global_planner_astar.hpp"
#include "barn_core/scan.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

class LocalPlanner
{
public:
  /// Choose a velocity command that follows `path` while respecting `scan`.
  /// STUB: returns a zero command; the slice uses GoalSeeker instead.
  barn_core::VelocityCommand plan(
    const Path2D & path, const barn_core::Pose2D & pose, const barn_core::ScanView & scan) const;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__LOCAL_PLANNER_HPP_
