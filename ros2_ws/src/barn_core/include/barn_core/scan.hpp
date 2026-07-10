// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// A non-owning view over a LiDAR scan plus pure queries on it. The view holds
// a raw pointer into someone else's range buffer (e.g. a sensor_msgs/LaserScan
// owned by the robot adapter), so it must not outlive that buffer.

#ifndef BARN_CORE__SCAN_HPP_
#define BARN_CORE__SCAN_HPP_

#include <cstddef>

namespace barn_core
{

/// Non-owning view over a planar range scan.
struct ScanView
{
  const float * ranges{nullptr};   ///< pointer to `count` range values (metres)
  std::size_t count{0};            ///< number of beams
  float angle_min{0.0f};           ///< angle of ranges[0], radians
  float angle_increment{0.0f};     ///< angular step between beams, radians
  float range_min{0.0f};           ///< minimum valid range, metres
  float range_max{0.0f};           ///< maximum valid range, metres

  /// True once a real buffer has been attached.
  bool valid() const { return ranges != nullptr && count > 0; }
};

/// Minimum valid range within the angular sector [a_lo, a_hi] (radians,
/// measured in the scan frame, typically base_link). The sector is clamped to
/// the scan's angular span; NaN, +/-inf, and out-of-[range_min,range_max]
/// beams are ignored. Returns `range_max` when the sector is empty or clear,
/// which callers treat as "no obstacle in this sector".
float min_range_in_sector(const ScanView & scan, double a_lo, double a_hi);

/// Distance to the single nearest valid beam across the whole scan
/// (used by safety/clearance logic). Returns `range_max` if none are valid.
float nearest_obstacle(const ScanView & scan);

}  // namespace barn_core

#endif  // BARN_CORE__SCAN_HPP_
