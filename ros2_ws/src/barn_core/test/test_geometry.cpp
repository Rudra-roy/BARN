// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>

#include "barn_core/geometry.hpp"
#include "gtest/gtest.h"

using barn_core::Goal2D;
using barn_core::Pose2D;

TEST(Geometry, WrapAngleKeepsRange)
{
  EXPECT_NEAR(barn_core::wrap_angle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(barn_core::wrap_angle(M_PI), M_PI, 1e-9);
  EXPECT_NEAR(barn_core::wrap_angle(-M_PI), M_PI, 1e-9);        // -pi maps to +pi
  EXPECT_NEAR(barn_core::wrap_angle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(barn_core::wrap_angle(2.0 * M_PI + 0.5), 0.5, 1e-9);
}

TEST(Geometry, ClampBounds)
{
  EXPECT_DOUBLE_EQ(barn_core::clamp(5.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(barn_core::clamp(-5.0, -1.0, 1.0), -1.0);
  EXPECT_DOUBLE_EQ(barn_core::clamp(0.25, -1.0, 1.0), 0.25);
}

TEST(Geometry, Dist2d)
{
  Pose2D p{0.0, 0.0, 0.0};
  Goal2D g{3.0, 4.0, 0.0, 0.8};
  EXPECT_NEAR(barn_core::dist2d(p, g), 5.0, 1e-9);
}

TEST(Geometry, HeadingToQuadrants)
{
  Pose2D facing_x{0.0, 0.0, 0.0};
  EXPECT_NEAR(barn_core::heading_to(facing_x, Goal2D{1.0, 0.0}), 0.0, 1e-9);
  EXPECT_NEAR(barn_core::heading_to(facing_x, Goal2D{0.0, 1.0}), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(barn_core::heading_to(facing_x, Goal2D{0.0, -1.0}), -M_PI / 2.0, 1e-9);

  // Robot already yawed 90 deg; a goal straight ahead in world +y is dead ahead.
  Pose2D facing_y{0.0, 0.0, M_PI / 2.0};
  EXPECT_NEAR(barn_core::heading_to(facing_y, Goal2D{0.0, 1.0}), 0.0, 1e-9);
}

TEST(Geometry, YawFromQuat)
{
  // 90 degree yaw about z: (x,y,z,w) = (0,0,sin(45),cos(45)).
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  EXPECT_NEAR(barn_core::yaw_from_quat(0.0, 0.0, s, c), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(barn_core::yaw_from_quat(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9);
}
