// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M18). Segment a LiDAR scan into obstacle clusters (e.g.
// adaptive-breakpoint / DBSCAN in Cartesian space) and return their centroids.

#ifndef BARN_DYNAMIC_TRACKING__CLUSTERING_HPP_
#define BARN_DYNAMIC_TRACKING__CLUSTERING_HPP_

#include <vector>

#include "barn_core/scan.hpp"

namespace barn_dynamic_tracking
{

struct Cluster
{
  double cx{0.0};   ///< centroid x in the scan frame (m)
  double cy{0.0};   ///< centroid y (m)
  int count{0};     ///< number of beams in the cluster
};

/// Cluster the scan into obstacle centroids. STUB: returns an empty list.
std::vector<Cluster> cluster_scan(const barn_core::ScanView & scan, double distance_threshold = 0.3);

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__CLUSTERING_HPP_
