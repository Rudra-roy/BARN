#include "barn_classical/path_validator.hpp"
#include <algorithm>
#include <cmath>

namespace barn_classical
{

bool PathValidator::is_path_clear(
  const Path2D & path, const barn_core::OccupancyGrid2D & grid,
  bool unknown_is_obstacle) const
{
  if (path.empty()) {
    return false;
  }

  // Validate that the path centerline does not pass through occupied obstacle cells.
  // We sample segments finely (half grid resolution) from step 1 onwards so narrow
  // corridors are accepted while genuine wall intersections are rejected.
  const double res = std::max(0.01, grid.resolution() * 0.5);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & p0 = path[i - 1];
    const auto & p1 = path[i];
    const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / res)));
    for (int step = 1; step <= steps; ++step) {
      const double frac = static_cast<double>(step) / steps;
      const double x = p0.x + frac * (p1.x - p0.x);
      const double y = p0.y + frac * (p1.y - p0.y);
      const auto cell = grid.world_to_cell(x, y);
      if (!grid.in_bounds(cell)) {
        return false;
      }
      const auto state = grid.classify(cell);
      if (state == barn_core::CellState::kOccupied ||
          (unknown_is_obstacle && state == barn_core::CellState::kUnknown))
      {
        return false;
      }
    }
  }
  return true;
}

}  // namespace barn_classical
