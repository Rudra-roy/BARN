// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure log-odds helpers for the online occupancy mapper (barn_mapping).
// Header-only and inline; no ROS, no allocation.

#ifndef BARN_CORE__LOGODDS_HPP_
#define BARN_CORE__LOGODDS_HPP_

#include <algorithm>
#include <cmath>

namespace barn_core
{

/// Probability -> log-odds. `p` is clamped away from {0,1} to stay finite.
inline double prob_to_logodds(double p)
{
  p = std::min(std::max(p, 1e-6), 1.0 - 1e-6);
  return std::log(p / (1.0 - p));
}

/// Log-odds -> probability.
inline double logodds_to_prob(double l) { return 1.0 - 1.0 / (1.0 + std::exp(l)); }

/// Standard binary inverse-sensor-model increments (tune per sensor).
/// A cell crossed by a ray gets `miss` (evidence of free space); the cell at
/// the ray endpoint gets `hit` (evidence of occupancy).
struct InverseSensorModel
{
  double hit{0.85};    ///< log-odds added at an occupied endpoint
  double miss{-0.40};  ///< log-odds added along a free ray
  double clamp_min{-4.0};
  double clamp_max{4.0};

  /// Integrate one observation into a cell's log-odds and clamp.
  double update(double cell, bool occupied) const
  {
    const double next = cell + (occupied ? hit : miss);
    return std::min(std::max(next, clamp_min), clamp_max);
  }
};

}  // namespace barn_core

#endif  // BARN_CORE__LOGODDS_HPP_
