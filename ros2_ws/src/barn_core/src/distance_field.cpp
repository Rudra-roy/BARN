// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_core/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace barn_core
{
namespace
{

constexpr double kInfinity = 1.0e20;

// Felzenszwalb-Huttenlocher exact squared Euclidean distance transform in 1-D.
void edt_1d(
  const std::vector<double> & f, std::size_t length,
  std::vector<double> & d, std::vector<int> & v, std::vector<double> & z)
{
  const int n = static_cast<int>(length);
  std::fill_n(d.begin(), length, kInfinity);
  int first_finite = -1;
  for (int i = 0; i < n; ++i) {
    if (f[static_cast<std::size_t>(i)] < kInfinity / 2.0) {
      first_finite = i;
      break;
    }
  }
  if (first_finite < 0) {
    return;
  }

  int k = 0;
  v[0] = first_finite;
  z[0] = -kInfinity;
  z[1] = kInfinity;

  for (int q = first_finite + 1; q < n; ++q) {
    if (f[static_cast<std::size_t>(q)] >= kInfinity / 2.0) {
      continue;
    }
    double s = 0.0;
    while (true) {
      const int vk = v[static_cast<std::size_t>(k)];
      s = ((f[static_cast<std::size_t>(q)] + q * q) -
        (f[static_cast<std::size_t>(vk)] + vk * vk)) / (2.0 * (q - vk));
      if (s > z[static_cast<std::size_t>(k)] || k == 0) {
        break;
      }
      --k;
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = kInfinity;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k + 1)] < q) {
      ++k;
    }
    const double delta = q - v[static_cast<std::size_t>(k)];
    d[static_cast<std::size_t>(q)] = delta * delta +
      f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])];
  }
}

}  // namespace

void DistanceField2D::rebuild(const OccupancyGrid2D & grid, bool unknown_is_obstacle)
{
  width_ = grid.width();
  height_ = grid.height();
  resolution_ = grid.resolution();
  origin_x_ = grid.origin_x();
  origin_y_ = grid.origin_y();
  distances_.assign(width_ * height_, std::numeric_limits<double>::infinity());
  if (width_ == 0 || height_ == 0) {
    return;
  }

  // Reuse one set of 1-D work buffers for every row and column. The original
  // implementation allocated four vectors for each line (thousands of heap
  // allocations per 5 Hz map update), which needlessly competed with Gazebo.
  const std::size_t max_dimension = std::max(width_, height_);
  std::vector<double> line(max_dimension, kInfinity);
  std::vector<double> transformed(max_dimension, kInfinity);
  std::vector<int> envelope_indices(max_dimension);
  std::vector<double> envelope_breakpoints(max_dimension + 1U);
  std::vector<double> vertical(width_ * height_, kInfinity);
  for (std::size_t col = 0; col < width_; ++col) {
    for (std::size_t row = 0; row < height_; ++row) {
      const auto state = grid.classify(
        GridIndex{static_cast<int>(col), static_cast<int>(row)});
      const bool obstacle = state == CellState::kOccupied ||
        (unknown_is_obstacle && state == CellState::kUnknown);
      line[row] = obstacle ? 0.0 : kInfinity;
    }
    edt_1d(
      line, height_, transformed, envelope_indices, envelope_breakpoints);
    for (std::size_t row = 0; row < height_; ++row) {
      vertical[row * width_ + col] = transformed[row];
    }
  }

  for (std::size_t row = 0; row < height_; ++row) {
    std::copy_n(vertical.begin() + static_cast<std::ptrdiff_t>(row * width_), width_,
      line.begin());
    edt_1d(
      line, width_, transformed, envelope_indices, envelope_breakpoints);
    for (std::size_t col = 0; col < width_; ++col) {
      const double squared_cells = transformed[col];
      distances_[row * width_ + col] = squared_cells >= kInfinity / 2.0 ?
        std::numeric_limits<double>::infinity() : std::sqrt(squared_cells) * resolution_;
    }
  }
}

std::size_t DistanceField2D::flat_index(const GridIndex & idx) const
{
  return static_cast<std::size_t>(idx.row) * width_ + static_cast<std::size_t>(idx.col);
}

double DistanceField2D::distance(const GridIndex & idx) const
{
  if (idx.col < 0 || idx.row < 0 || static_cast<std::size_t>(idx.col) >= width_ ||
    static_cast<std::size_t>(idx.row) >= height_)
  {
    return 0.0;
  }
  return distances_[flat_index(idx)];
}

double DistanceField2D::distance_world(double wx, double wy) const
{
  if (!valid()) {
    return 0.0;
  }
  const GridIndex idx{
    static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
    static_cast<int>(std::floor((wy - origin_y_) / resolution_))};
  return distance(idx);
}

bool DistanceField2D::gradient_world(double wx, double wy, double & gx, double & gy) const
{
  if (!valid()) {
    gx = 0.0;
    gy = 0.0;
    return false;
  }
  const GridIndex idx{
    static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
    static_cast<int>(std::floor((wy - origin_y_) / resolution_))};
  if (idx.col <= 0 || idx.row <= 0 || static_cast<std::size_t>(idx.col + 1) >= width_ ||
    static_cast<std::size_t>(idx.row + 1) >= height_)
  {
    gx = 0.0;
    gy = 0.0;
    return false;
  }
  const double left = distance(GridIndex{idx.col - 1, idx.row});
  const double right = distance(GridIndex{idx.col + 1, idx.row});
  const double down = distance(GridIndex{idx.col, idx.row - 1});
  const double up = distance(GridIndex{idx.col, idx.row + 1});
  if (!std::isfinite(left) || !std::isfinite(right) || !std::isfinite(down) ||
    !std::isfinite(up))
  {
    gx = 0.0;
    gy = 0.0;
    return false;
  }
  gx = (right - left) / (2.0 * resolution_);
  gy = (up - down) / (2.0 * resolution_);
  return true;
}

}  // namespace barn_core
