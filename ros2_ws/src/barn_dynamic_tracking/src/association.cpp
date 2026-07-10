// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/association.hpp"

namespace barn_dynamic_tracking
{

std::vector<int> associate(const std::vector<Cluster> & clusters, std::size_t, double)
{
  // STUB (M18): nearest-neighbour / Hungarian gating goes here.
  return std::vector<int>(clusters.size(), -1);
}

}  // namespace barn_dynamic_tracking
