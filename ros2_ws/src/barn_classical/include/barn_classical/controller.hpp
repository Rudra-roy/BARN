// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M8). Curvature/clearance-aware trajectory tracking controller
// (e.g. pure pursuit + speed scheduling) that turns the local trajectory into a
// smooth velocity command.

#ifndef BARN_CLASSICAL__CONTROLLER_HPP_
#define BARN_CLASSICAL__CONTROLLER_HPP_

#include "barn_classical/global_planner_astar.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

class Controller
{
public:
  /// Track `path` from `pose`. STUB: returns a zero command.
  barn_core::VelocityCommand control(const Path2D & path, const barn_core::Pose2D & pose) const;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__CONTROLLER_HPP_
