// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// A minimal, ROS-free 2-D occupancy grid stored as log-odds. barn_mapping
// builds one online from LiDAR; barn_classical plans over it. Publishing it as
// a nav_msgs/OccupancyGrid is the adapter's job, not this type's.

#ifndef BARN_CORE__OCCUPANCY_HPP_
#define BARN_CORE__OCCUPANCY_HPP_

#include <cstddef>
#include <vector>

namespace barn_core
{

/// Integer cell coordinate within a grid.
struct GridIndex
{
  int col{0};  ///< x direction
  int row{0};  ///< y direction
};

/// Classification returned by OccupancyGrid2D::classify().
enum class CellState
{
  kFree,
  kUnknown,
  kOccupied
};

/// Dense log-odds occupancy grid in a fixed planar frame (e.g. odom).
class OccupancyGrid2D
{
public:
  OccupancyGrid2D() = default;

  /// Allocate a grid of `width` x `height` cells at `resolution` metres/cell,
  /// with the grid origin (cell 0,0 corner) at world (origin_x, origin_y).
  OccupancyGrid2D(
    std::size_t width, std::size_t height, double resolution, double origin_x, double origin_y);

  std::size_t width() const { return width_; }
  std::size_t height() const { return height_; }
  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }

  bool in_bounds(const GridIndex & idx) const;

  /// World point -> cell index (no bounds check; test with in_bounds()).
  GridIndex world_to_cell(double wx, double wy) const;

  /// Cell centre -> world point.
  void cell_to_world(const GridIndex & idx, double & wx, double & wy) const;

  /// Raw log-odds accessors. Out-of-bounds reads return 0 (unknown); writes
  /// are ignored.
  double log_odds(const GridIndex & idx) const;
  void set_log_odds(const GridIndex & idx, double value);

  /// Classify a cell against the free/occupied thresholds.
  CellState classify(
    const GridIndex & idx, double free_threshold = -0.4, double occupied_threshold = 0.85) const;

  /// Reset all cells to unknown (log-odds 0).
  void clear();

private:
  std::size_t index_of(const GridIndex & idx) const;

  std::size_t width_{0};
  std::size_t height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<double> data_;  ///< log-odds, row-major (row * width + col)
};

}  // namespace barn_core

#endif  // BARN_CORE__OCCUPANCY_HPP_
