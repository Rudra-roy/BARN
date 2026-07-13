// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Exact Euclidean distance transform over an OccupancyGrid2D. Distances are
// expressed in metres and gradients in metres/metre. This class is ROS-free so
// mapping, planning, MPC, and safety tests share one implementation.

#ifndef BARN_CORE__DISTANCE_FIELD_HPP_
#define BARN_CORE__DISTANCE_FIELD_HPP_

#include <cstddef>
#include <vector>

#include "barn_core/occupancy.hpp"

namespace barn_core
{

class DistanceField2D
{
public:
  DistanceField2D() = default;

  /// Recompute the field. UNKNOWN is free unless unknown_is_obstacle is true.
  void rebuild(const OccupancyGrid2D & grid, bool unknown_is_obstacle = false);

  bool valid() const { return width_ > 0 && height_ > 0 && !distances_.empty(); }
  double distance(const GridIndex & idx) const;
  double distance_world(double wx, double wy) const;

  /// Central-difference world gradient. Returns false outside a valid field.
  bool gradient_world(double wx, double wy, double & gx, double & gy) const;

private:
  std::size_t flat_index(const GridIndex & idx) const;

  std::size_t width_{0};
  std::size_t height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<double> distances_;
};

}  // namespace barn_core

#endif  // BARN_CORE__DISTANCE_FIELD_HPP_
