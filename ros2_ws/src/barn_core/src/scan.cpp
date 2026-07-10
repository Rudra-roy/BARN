// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_core/scan.hpp"

#include <algorithm>
#include <cmath>

namespace barn_core
{

namespace
{
bool beam_is_valid(float r, float range_min, float range_max)
{
  return std::isfinite(r) && r >= range_min && r <= range_max;
}
}  // namespace

float min_range_in_sector(const ScanView & scan, double a_lo, double a_hi)
{
  if (!scan.valid() || scan.angle_increment == 0.0f) {
    return scan.range_max;
  }
  if (a_lo > a_hi) {
    std::swap(a_lo, a_hi);
  }

  // Map the requested sector onto beam indices, clamped to the valid range.
  const double lo_f = (a_lo - scan.angle_min) / scan.angle_increment;
  const double hi_f = (a_hi - scan.angle_min) / scan.angle_increment;
  long i_lo = static_cast<long>(std::ceil(lo_f));
  long i_hi = static_cast<long>(std::floor(hi_f));
  i_lo = std::max<long>(i_lo, 0);
  i_hi = std::min<long>(i_hi, static_cast<long>(scan.count) - 1);

  float best = scan.range_max;
  for (long i = i_lo; i <= i_hi; ++i) {
    const float r = scan.ranges[static_cast<std::size_t>(i)];
    if (beam_is_valid(r, scan.range_min, scan.range_max)) {
      best = std::min(best, r);
    }
  }
  return best;
}

float nearest_obstacle(const ScanView & scan)
{
  if (!scan.valid()) {
    return scan.range_max;
  }
  float best = scan.range_max;
  for (std::size_t i = 0; i < scan.count; ++i) {
    const float r = scan.ranges[i];
    if (beam_is_valid(r, scan.range_min, scan.range_max)) {
      best = std::min(best, r);
    }
  }
  return best;
}

}  // namespace barn_core
