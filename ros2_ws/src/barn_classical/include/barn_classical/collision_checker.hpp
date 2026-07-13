// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__COLLISION_CHECKER_HPP_
#define BARN_CLASSICAL__COLLISION_CHECKER_HPP_

#include "barn_core/occupancy.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

struct Footprint
{
  double half_length{0.254};
  double half_width{0.2159};
  double margin{0.04};
};

/// Exact grid test of a filled, oriented rectangular footprint.
bool footprint_is_clear(
  const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & pose,
  const Footprint & footprint = {}, bool unknown_is_obstacle = false);

/// Check a segment by interpolating position and shortest-angle yaw.
bool swept_segment_is_clear(
  const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & from,
  const barn_core::Pose2D & to, const Footprint & footprint = {},
  bool unknown_is_obstacle = false, double sample_step = 0.025);

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__COLLISION_CHECKER_HPP_
