// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M18). Associate the current frame's clusters with existing
// tracks (nearest-neighbour / Hungarian gating) so each track updates its KF.

#ifndef BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_
#define BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_

#include <cstddef>
#include <vector>

#include "barn_dynamic_tracking/clustering.hpp"

namespace barn_dynamic_tracking
{

/// For each cluster, the index of the track it associates with, or -1 for a new
/// track. STUB: returns all -1 (every cluster starts a new track).
std::vector<int> associate(
  const std::vector<Cluster> & clusters, std::size_t num_tracks, double gate_distance = 0.5);

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_
