// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/clustering.hpp"

#include <cmath>
#include <vector>

namespace barn_dynamic_tracking
{

namespace
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

// Turn a finished run of Cartesian points into a Cluster (centroid, count,
// radius). Returns false for degenerate runs so the caller can drop them.
bool finalise_cluster(const std::vector<Point> & pts, int min_points, Cluster & out)
{
  const int count = static_cast<int>(pts.size());
  if (count < min_points) {
    return false;
  }

  double sx = 0.0;
  double sy = 0.0;
  for (const auto & p : pts) {
    sx += p.x;
    sy += p.y;
  }
  out.cx = sx / count;
  out.cy = sy / count;
  out.count = count;

  // Radius: half the maximum span of the cluster points. The endpoints of a
  // sequentially ordered LiDAR arc are the farthest apart, so the endpoint
  // chord is a good, cheap estimate of the extent.
  double max_span = 0.0;
  const Point & first = pts.front();
  const Point & last = pts.back();
  const double dx = last.x - first.x;
  const double dy = last.y - first.y;
  max_span = std::sqrt(dx * dx + dy * dy);

  // Also account for radial spread from the centroid (handles curved fronts).
  double max_from_centroid = 0.0;
  for (const auto & p : pts) {
    const double ex = p.x - out.cx;
    const double ey = p.y - out.cy;
    const double d = std::sqrt(ex * ex + ey * ey);
    if (d > max_from_centroid) {
      max_from_centroid = d;
    }
  }

  double radius = 0.5 * max_span;
  if (max_from_centroid > radius) {
    radius = max_from_centroid;
  }

  // Clamp to a sane physical range.
  constexpr double kMinRadius = 0.05;
  constexpr double kMaxRadius = 1.5;
  if (radius < kMinRadius) {
    radius = kMinRadius;
  } else if (radius > kMaxRadius) {
    radius = kMaxRadius;
  }
  out.radius = radius;
  return true;
}

}  // namespace

std::vector<Cluster> cluster_scan(const barn_core::ScanView & scan, double distance_threshold)
{
  std::vector<Cluster> clusters;
  if (!scan.valid()) {
    return clusters;
  }

  constexpr int kMinPoints = 2;

  std::vector<Point> current;
  bool have_prev = false;
  Point prev{};

  for (std::size_t i = 0; i < scan.count; ++i) {
    const float r = scan.ranges[i];
    // Skip NaN, +/-inf, and out-of-band returns.
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }

    const double angle =
      static_cast<double>(scan.angle_min) + static_cast<double>(i) * scan.angle_increment;
    Point p;
    p.x = static_cast<double>(r) * std::cos(angle);
    p.y = static_cast<double>(r) * std::sin(angle);

    if (have_prev) {
      const double dx = p.x - prev.x;
      const double dy = p.y - prev.y;
      const double gap = std::sqrt(dx * dx + dy * dy);

      // Adaptive threshold: allow a slightly larger gap at longer ranges, where
      // the angular beam spacing spreads points apart even on one object.
      const double adaptive =
        distance_threshold + 0.05 * static_cast<double>(r);

      if (gap > adaptive) {
        Cluster c;
        if (finalise_cluster(current, kMinPoints, c)) {
          clusters.push_back(c);
        }
        current.clear();
      }
    }

    current.push_back(p);
    prev = p;
    have_prev = true;
  }

  Cluster c;
  if (finalise_cluster(current, kMinPoints, c)) {
    clusters.push_back(c);
  }

  return clusters;
}

}  // namespace barn_dynamic_tracking
