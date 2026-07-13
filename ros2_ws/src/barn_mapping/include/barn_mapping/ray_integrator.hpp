// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_MAPPING__RAY_INTEGRATOR_HPP_
#define BARN_MAPPING__RAY_INTEGRATOR_HPP_

#include "barn_core/logodds.hpp"
#include "barn_core/occupancy.hpp"

namespace barn_mapping
{

/// Integrate one grid-space LiDAR ray with Bresenham traversal.
/// The endpoint is occupied only when endpoint_is_hit is true.
void integrate_ray(
  barn_core::OccupancyGrid2D & grid, const barn_core::GridIndex & start,
  const barn_core::GridIndex & end, bool endpoint_is_hit,
  const barn_core::InverseSensorModel & model);

}  // namespace barn_mapping

#endif  // BARN_MAPPING__RAY_INTEGRATOR_HPP_
