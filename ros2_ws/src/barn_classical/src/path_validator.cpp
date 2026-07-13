// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/path_validator.hpp"

namespace barn_classical
{

bool PathValidator::is_path_clear(
  const Path2D & path, const barn_core::OccupancyGrid2D & grid,
  bool unknown_is_obstacle) const
{
  if (path.empty()) {
    return false;
  }
  if (!footprint_is_clear(grid, path.front(), footprint_, unknown_is_obstacle)) {
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!swept_segment_is_clear(
        grid, path[i - 1], path[i], footprint_, unknown_is_obstacle,
        grid.resolution()))
    {
      return false;
    }
  }
  return true;
}

}  // namespace barn_classical
