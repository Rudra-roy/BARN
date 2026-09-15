#include "barn_classical/path_validator.hpp"
#include <algorithm>
#include <cmath>

namespace barn_classical
{

bool PathValidator::is_path_clear(
  const Path2D & path, const barn_core::OccupancyGrid2D & grid,
  bool unknown_is_obstacle,
  const barn_core::DistanceField2D * distance_field,
  double min_clearance,
  double max_clearance_check_distance) const
{
  if (path.empty()) {
    return false;
  }

  // Validate that the path centerline does not pass through occupied obstacle cells
  // and has sufficient physical clearance for the robot.
  // Occupied cells are strictly checked along the entire path length.
  // Distance field clearance is evaluated within max_clearance_check_distance to avoid
  // premature replanning or rejection from noisy distant pinches.
  const double res = std::max(0.01, grid.resolution() * 0.5);
  double accumulated_dist = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & p0 = path[i - 1];
    const auto & p1 = path[i];
    const double seg_dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_dist / res)));
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
      const double current_dist = accumulated_dist + frac * seg_dist;
      if (distance_field != nullptr && min_clearance > 0.0 &&
          current_dist <= max_clearance_check_distance)
      {
        const double c = distance_field->distance_world(x, y);
        if (std::isfinite(c) && c < min_clearance) {
          return false;
        }
      }
    }
    accumulated_dist += seg_dist;
  }
  return true;
}

}  // namespace barn_classical
