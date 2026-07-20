// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "barn_core/scan.hpp"
#include "barn_dynamic_tracking/association.hpp"
#include "barn_dynamic_tracking/clustering.hpp"
#include "gtest/gtest.h"

using barn_dynamic_tracking::Cluster;
using barn_dynamic_tracking::associate;
using barn_dynamic_tracking::cluster_scan;

namespace
{

// Build a ScanView over a caller-owned ranges buffer. The buffer must outlive
// the returned view (it points straight into it).
barn_core::ScanView make_view(
  const std::vector<float> & ranges, float angle_min, float angle_increment,
  float range_min, float range_max)
{
  barn_core::ScanView view;
  view.ranges = ranges.data();
  view.count = ranges.size();
  view.angle_min = angle_min;
  view.angle_increment = angle_increment;
  view.range_min = range_min;
  view.range_max = range_max;
  return view;
}

}  // namespace

TEST(Clustering, SeparatesTwoArcs)
{
  // 101 beams over [-0.5, 0.5] rad. Two arcs at range 2 m: indices 15..25 on
  // the robot's right (negative angle) and 75..85 on the left. Everything else
  // is +inf (no return) and must be ignored.
  const std::size_t n = 101;
  const float inff = std::numeric_limits<float>::infinity();
  std::vector<float> ranges(n, inff);
  for (std::size_t i = 15; i <= 25; ++i) {
    ranges[i] = 2.0f;
  }
  for (std::size_t i = 75; i <= 85; ++i) {
    ranges[i] = 2.0f;
  }

  const barn_core::ScanView view = make_view(ranges, -0.5f, 0.01f, 0.05f, 30.0f);
  const std::vector<Cluster> clusters = cluster_scan(view, 0.3);

  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].count, 11);
  EXPECT_EQ(clusters[1].count, 11);

  // First cluster is on the right (cy < 0), second on the left (cy > 0).
  EXPECT_LT(clusters[0].cy, 0.0);
  EXPECT_GT(clusters[1].cy, 0.0);

  // Both centroids sit ~2 m out in front; x ~ 2*cos(0.3) ~ 1.91.
  EXPECT_NEAR(clusters[0].cx, 1.91, 0.1);
  EXPECT_NEAR(clusters[1].cx, 1.91, 0.1);
  EXPECT_NEAR(std::abs(clusters[0].cy), 0.59, 0.1);
  EXPECT_NEAR(std::abs(clusters[1].cy), 0.59, 0.1);

  // Radius is clamped into the sane band.
  for (const auto & c : clusters) {
    EXPECT_GE(c.radius, 0.05);
    EXPECT_LE(c.radius, 1.5);
  }
}

TEST(Clustering, IgnoresInvalidBeams)
{
  // A single valid beam cannot form a cluster (min count is 2).
  std::vector<float> ranges(20, std::numeric_limits<float>::quiet_NaN());
  ranges[5] = 1.0f;
  const barn_core::ScanView view = make_view(ranges, -0.5f, 0.01f, 0.05f, 30.0f);
  const std::vector<Cluster> clusters = cluster_scan(view, 0.3);
  EXPECT_TRUE(clusters.empty());
}

TEST(Association, MatchesWithinGateAndRejectsOutside)
{
  std::vector<Cluster> clusters(2);
  clusters[0].cx = 0.1;
  clusters[0].cy = 0.1;   // close to the track at the origin
  clusters[1].cx = 5.0;
  clusters[1].cy = 5.0;   // far outside the gate

  const std::vector<std::pair<double, double>> tracks = {{0.0, 0.0}};

  const std::vector<int> a = associate(clusters, tracks, 0.5);
  ASSERT_EQ(a.size(), 2u);
  EXPECT_EQ(a[0], 0);   // associates with track 0
  EXPECT_EQ(a[1], -1);  // new track
}

TEST(Association, OneTrackClaimedByOneCluster)
{
  // Two clusters both near a single track: only the nearest may claim it.
  std::vector<Cluster> clusters(2);
  clusters[0].cx = 0.1;
  clusters[0].cy = 0.0;   // nearest
  clusters[1].cx = 0.3;
  clusters[1].cy = 0.0;

  const std::vector<std::pair<double, double>> tracks = {{0.0, 0.0}};
  const std::vector<int> a = associate(clusters, tracks, 0.5);
  ASSERT_EQ(a.size(), 2u);
  EXPECT_EQ(a[0], 0);
  EXPECT_EQ(a[1], -1);
}
