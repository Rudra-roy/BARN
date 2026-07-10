// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/controller.hpp"

namespace barn_classical
{

barn_core::VelocityCommand Controller::control(const Path2D &, const barn_core::Pose2D &) const
{
  // STUB (M8): pure-pursuit + curvature-aware speed schedule goes here.
  return barn_core::VelocityCommand{0.0, 0.0};
}

}  // namespace barn_classical
