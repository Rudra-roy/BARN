// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/local_planner.hpp"

namespace barn_classical
{

barn_core::VelocityCommand LocalPlanner::plan(
  const Path2D &, const barn_core::Pose2D &, const barn_core::ScanView &) const
{
  // STUB (M7): DWA / lattice rollout goes here.
  return barn_core::VelocityCommand{0.0, 0.0};
}

}  // namespace barn_classical
