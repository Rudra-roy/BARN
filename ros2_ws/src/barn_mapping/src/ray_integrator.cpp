// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_mapping/ray_integrator.hpp"

#include <cstdlib>

namespace barn_mapping
{

void integrate_ray(
  barn_core::OccupancyGrid2D & grid, const barn_core::GridIndex & start,
  const barn_core::GridIndex & end, bool endpoint_is_hit,
  const barn_core::InverseSensorModel & model)
{
  int x = start.col;
  int y = start.row;
  const int dx = std::abs(end.col - start.col);
  const int sx = start.col < end.col ? 1 : -1;
  const int dy = -std::abs(end.row - start.row);
  const int sy = start.row < end.row ? 1 : -1;
  int error = dx + dy;

  while (true) {
    const bool endpoint = x == end.col && y == end.row;
    const barn_core::GridIndex cell{x, y};
    if (endpoint) {
      if (endpoint_is_hit) {
        grid.add_log_odds(cell, model.hit, model.clamp_min, model.clamp_max);
      } else {
        grid.add_log_odds(cell, model.miss, model.clamp_min, model.clamp_max);
      }
      break;
    }

    // Do not repeatedly mark the LiDAR origin. It lies inside the robot and
    // contributes no useful evidence while receiving one update per beam.
    if (x != start.col || y != start.row) {
      grid.add_log_odds(cell, model.miss, model.clamp_min, model.clamp_max);
    }
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y += sy;
    }
  }
}

}  // namespace barn_mapping
