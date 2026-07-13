// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/collision_checker.hpp"

#include <algorithm>
#include <cmath>

#include "barn_core/geometry.hpp"

namespace barn_classical
{

bool footprint_is_clear(
  const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & pose,
  const Footprint & footprint, bool unknown_is_obstacle)
{
  const double half_length = footprint.half_length + footprint.margin;
  const double half_width = footprint.half_width + footprint.margin;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double extent_x = std::abs(c) * half_length + std::abs(s) * half_width;
  const double extent_y = std::abs(s) * half_length + std::abs(c) * half_width;
  const auto minimum = grid.world_to_cell(pose.x - extent_x, pose.y - extent_y);
  const auto maximum = grid.world_to_cell(pose.x + extent_x, pose.y + extent_y);
  if (!grid.in_bounds(minimum) || !grid.in_bounds(maximum)) {
    return false;
  }

  const double cell_half = 0.5 * grid.resolution();
  for (int row = minimum.row; row <= maximum.row; ++row) {
    for (int col = minimum.col; col <= maximum.col; ++col) {
      const barn_core::GridIndex cell{col, row};
      const auto state = grid.classify(cell);
      if (state != barn_core::CellState::kOccupied &&
        !(unknown_is_obstacle && state == barn_core::CellState::kUnknown))
      {
        continue;
      }
      double cell_x = 0.0;
      double cell_y = 0.0;
      grid.cell_to_world(cell, cell_x, cell_y);
      const double dx = cell_x - pose.x;
      const double dy = cell_y - pose.y;
      // Exact separating-axis test between the oriented robot rectangle and
      // the axis-aligned occupancy cell. This avoids hundreds of duplicate
      // point samples per lattice primitive without weakening collision tests.
      const double robot_x = std::abs(c * dx + s * dy);
      const double robot_y = std::abs(-s * dx + c * dy);
      if (robot_x > half_length + cell_half * (std::abs(c) + std::abs(s)) ||
        robot_y > half_width + cell_half * (std::abs(s) + std::abs(c)) ||
        std::abs(dx) > cell_half + half_length * std::abs(c) + half_width * std::abs(s) ||
        std::abs(dy) > cell_half + half_length * std::abs(s) + half_width * std::abs(c))
      {
        continue;
      }
      return false;
    }
  }
  return true;
}

bool swept_segment_is_clear(
  const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & from,
  const barn_core::Pose2D & to, const Footprint & footprint,
  bool unknown_is_obstacle, double sample_step)
{
  const double distance = std::hypot(to.x - from.x, to.y - from.y);
  const double yaw_delta = barn_core::wrap_angle(to.yaw - from.yaw);
  const int samples = std::max(
    1, static_cast<int>(std::ceil(std::max(distance / sample_step,
    std::abs(yaw_delta) / (5.0 * M_PI / 180.0)))));
  for (int i = 0; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / samples;
    barn_core::Pose2D sample_pose;
    sample_pose.x = from.x + ratio * (to.x - from.x);
    sample_pose.y = from.y + ratio * (to.y - from.y);
    sample_pose.yaw = barn_core::wrap_angle(from.yaw + ratio * yaw_delta);
    if (!footprint_is_clear(grid, sample_pose, footprint, unknown_is_obstacle)) {
      return false;
    }
  }
  return true;
}

}  // namespace barn_classical
