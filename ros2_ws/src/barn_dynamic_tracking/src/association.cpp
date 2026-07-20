// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/association.hpp"

#include <cmath>
#include <vector>

namespace barn_dynamic_tracking
{

std::vector<int> associate(
  const std::vector<Cluster> & clusters,
  const std::vector<std::pair<double, double>> & track_positions,
  double gate_distance)
{
  std::vector<int> assignment(clusters.size(), -1);
  std::vector<bool> claimed(track_positions.size(), false);
  const double gate_sq = gate_distance * gate_distance;

  for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
    const double cx = clusters[ci].cx;
    const double cy = clusters[ci].cy;

    int best = -1;
    double best_sq = gate_sq;
    for (std::size_t ti = 0; ti < track_positions.size(); ++ti) {
      if (claimed[ti]) {
        continue;
      }
      const double dx = cx - track_positions[ti].first;
      const double dy = cy - track_positions[ti].second;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq <= best_sq) {
        best_sq = d_sq;
        best = static_cast<int>(ti);
      }
    }

    if (best >= 0) {
      claimed[static_cast<std::size_t>(best)] = true;
      assignment[ci] = best;
    }
  }

  return assignment;
}

}  // namespace barn_dynamic_tracking
