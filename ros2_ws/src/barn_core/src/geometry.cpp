// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_core/geometry.hpp"

#include <cmath>

namespace barn_core
{

double wrap_angle(double angle)
{
  // Bring into (-pi, pi]. std::remainder returns a value in [-pi, pi] centred
  // on the nearest multiple of 2*pi; we nudge the -pi edge to +pi so the range
  // is half-open and consistent for comparisons.
  double wrapped = std::remainder(angle, 2.0 * M_PI);
  if (wrapped <= -M_PI) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped;
}

double clamp(double value, double lo, double hi)
{
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

double dist2d(const Pose2D & from, const Goal2D & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return std::sqrt(dx * dx + dy * dy);
}

double heading_to(const Pose2D & from, const Goal2D & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return wrap_angle(std::atan2(dy, dx) - from.yaw);
}

double yaw_from_quat(double x, double y, double z, double w)
{
  // Standard ZYX yaw extraction.
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace barn_core
