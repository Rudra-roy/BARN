// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__PATH_VALIDATOR_HPP_
#define BARN_CLASSICAL__PATH_VALIDATOR_HPP_

#include "barn_classical/collision_checker.hpp"
#include "barn_classical/global_planner_astar.hpp"
#include "barn_core/distance_field.hpp"
#include "barn_core/occupancy.hpp"

#include <limits>

namespace barn_classical
{

class PathValidator
{
public:
  explicit PathValidator(const Footprint & footprint = {}) : footprint_(footprint) {}

  bool is_path_clear(
    const Path2D & path, const barn_core::OccupancyGrid2D & grid,
    bool unknown_is_obstacle = false,
    const barn_core::DistanceField2D * distance_field = nullptr,
    double min_clearance = 0.0,
    double max_clearance_check_distance = std::numeric_limits<double>::infinity()) const;

private:
  Footprint footprint_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__PATH_VALIDATOR_HPP_
