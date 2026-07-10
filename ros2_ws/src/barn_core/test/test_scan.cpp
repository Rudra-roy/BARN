// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>
#include <limits>
#include <vector>

#include "barn_core/scan.hpp"
#include "gtest/gtest.h"

using barn_core::ScanView;

namespace
{
// A 181-beam scan spanning [-pi/2, +pi/2] at 1-degree resolution.
ScanView make_scan(std::vector<float> & ranges)
{
  ScanView s;
  s.ranges = ranges.data();
  s.count = ranges.size();
  s.angle_min = -static_cast<float>(M_PI) / 2.0f;
  s.angle_increment = static_cast<float>(M_PI) / 180.0f;
  s.range_min = 0.1f;
  s.range_max = 30.0f;
  return s;
}
}  // namespace

TEST(Scan, EmptyViewReturnsRangeMax)
{
  ScanView s;
  s.range_max = 30.0f;
  EXPECT_FLOAT_EQ(barn_core::min_range_in_sector(s, -0.1, 0.1), 30.0f);
  EXPECT_FLOAT_EQ(barn_core::nearest_obstacle(s), 30.0f);
}

TEST(Scan, MinInFrontSector)
{
  std::vector<float> ranges(181, 5.0f);
  ranges[90] = 1.2f;  // beam at angle 0 (dead ahead)
  ScanView s = make_scan(ranges);
  // Narrow +/-2 degree sector around straight ahead should see 1.2 m.
  const double two_deg = 2.0 * M_PI / 180.0;
  EXPECT_NEAR(barn_core::min_range_in_sector(s, -two_deg, two_deg), 1.2f, 1e-5);
}

TEST(Scan, IgnoresInvalidBeams)
{
  std::vector<float> ranges(181, 5.0f);
  ranges[90] = std::numeric_limits<float>::infinity();
  ranges[91] = 0.05f;  // below range_min -> invalid
  ranges[89] = 2.5f;   // the only valid nearby beam
  ScanView s = make_scan(ranges);
  const double three_deg = 3.0 * M_PI / 180.0;
  EXPECT_NEAR(barn_core::min_range_in_sector(s, -three_deg, three_deg), 2.5f, 1e-5);
}

TEST(Scan, NearestObstacleScansAll)
{
  std::vector<float> ranges(181, 5.0f);
  ranges[10] = 0.7f;
  ScanView s = make_scan(ranges);
  EXPECT_NEAR(barn_core::nearest_obstacle(s), 0.7f, 1e-5);
}
