// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M6). Checks whether the current global path is still clear on
// the latest occupancy grid, triggering an immediate replan when it is not.

#ifndef BARN_CLASSICAL__PATH_VALIDATOR_HPP_
#define BARN_CLASSICAL__PATH_VALIDATOR_HPP_

#include "barn_classical/global_planner_astar.hpp"
#include "barn_core/occupancy.hpp"

namespace barn_classical
{

class PathValidator
{
public:
  /// True if every cell along `path` is non-occupied on `grid`.
  /// STUB: currently always returns true.
  bool is_path_clear(const Path2D & path, const barn_core::OccupancyGrid2D & grid) const;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__PATH_VALIDATOR_HPP_
