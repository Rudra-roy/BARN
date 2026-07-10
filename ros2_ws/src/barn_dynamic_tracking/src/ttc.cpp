// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/ttc.hpp"

#include <cmath>

namespace barn_dynamic_tracking
{

double time_to_collision(double rel_x, double rel_y, double rel_vx, double rel_vy, double radius)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();

  // Solve |p + v t| = radius for the smallest positive t.
  const double a = rel_vx * rel_vx + rel_vy * rel_vy;
  const double b = 2.0 * (rel_x * rel_vx + rel_y * rel_vy);
  const double c = rel_x * rel_x + rel_y * rel_y - radius * radius;

  if (c <= 0.0) {
    return 0.0;  // already overlapping
  }
  if (a < 1e-12) {
    return kInf;  // no relative motion
  }
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return kInf;  // never reaches the radius
  }
  const double sqrt_disc = std::sqrt(disc);
  const double t1 = (-b - sqrt_disc) / (2.0 * a);
  const double t2 = (-b + sqrt_disc) / (2.0 * a);
  if (t1 >= 0.0) {
    return t1;
  }
  if (t2 >= 0.0) {
    return t2;
  }
  return kInf;
}

}  // namespace barn_dynamic_tracking
