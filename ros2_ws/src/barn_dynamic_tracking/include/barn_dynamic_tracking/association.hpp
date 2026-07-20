// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M18). Associate the current frame's clusters with existing
// tracks (nearest-neighbour / Hungarian gating) so each track updates its KF.

#ifndef BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_
#define BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_

#include <utility>
#include <vector>

#include "barn_dynamic_tracking/clustering.hpp"

namespace barn_dynamic_tracking
{

/// Greedy nearest-neighbour data association. For each cluster, returns the
/// index (into `track_positions`) of the nearest unclaimed track whose centre
/// is within `gate_distance`, or -1 if none qualifies (i.e. a new track). Each
/// track is claimed by at most one cluster.
std::vector<int> associate(
  const std::vector<Cluster> & clusters,
  const std::vector<std::pair<double, double>> & track_positions,
  double gate_distance = 0.5);

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__ASSOCIATION_HPP_
