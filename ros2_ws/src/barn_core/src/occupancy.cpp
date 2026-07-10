// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_core/occupancy.hpp"

#include <algorithm>
#include <cmath>

namespace barn_core
{

OccupancyGrid2D::OccupancyGrid2D(
  std::size_t width, std::size_t height, double resolution, double origin_x, double origin_y)
: width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  data_(width * height, 0.0)
{
}

bool OccupancyGrid2D::in_bounds(const GridIndex & idx) const
{
  return idx.col >= 0 && idx.row >= 0 && static_cast<std::size_t>(idx.col) < width_ &&
         static_cast<std::size_t>(idx.row) < height_;
}

GridIndex OccupancyGrid2D::world_to_cell(double wx, double wy) const
{
  GridIndex idx;
  idx.col = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  idx.row = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return idx;
}

void OccupancyGrid2D::cell_to_world(const GridIndex & idx, double & wx, double & wy) const
{
  wx = origin_x_ + (static_cast<double>(idx.col) + 0.5) * resolution_;
  wy = origin_y_ + (static_cast<double>(idx.row) + 0.5) * resolution_;
}

std::size_t OccupancyGrid2D::index_of(const GridIndex & idx) const
{
  return static_cast<std::size_t>(idx.row) * width_ + static_cast<std::size_t>(idx.col);
}

double OccupancyGrid2D::log_odds(const GridIndex & idx) const
{
  if (!in_bounds(idx)) {
    return 0.0;
  }
  return data_[index_of(idx)];
}

void OccupancyGrid2D::set_log_odds(const GridIndex & idx, double value)
{
  if (!in_bounds(idx)) {
    return;
  }
  data_[index_of(idx)] = value;
}

CellState OccupancyGrid2D::classify(
  const GridIndex & idx, double free_threshold, double occupied_threshold) const
{
  const double l = log_odds(idx);
  if (l >= occupied_threshold) {
    return CellState::kOccupied;
  }
  if (l <= free_threshold) {
    return CellState::kFree;
  }
  return CellState::kUnknown;
}

void OccupancyGrid2D::clear()
{
  std::fill(data_.begin(), data_.end(), 0.0);
}

}  // namespace barn_core
