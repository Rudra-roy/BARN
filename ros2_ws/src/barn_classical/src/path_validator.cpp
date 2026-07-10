// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/path_validator.hpp"

namespace barn_classical
{

bool PathValidator::is_path_clear(const Path2D &, const barn_core::OccupancyGrid2D &) const
{
  // STUB (M6): sample the path against grid cells and return false on the first
  // OCCUPIED cell so the node replans immediately.
  return true;
}

}  // namespace barn_classical
