#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "barn_safety/swept_footprint_shield.hpp"

TEST(SweptFootprintShield, LeavesClearCommandUnchanged)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({1.0, 0.2}, {{5.0, 5.0}});
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.v, 1.0);
  EXPECT_DOUBLE_EQ(result.command.w, 0.2);
}

TEST(SweptFootprintShield, ClampsFrontalStoppingEnvelope)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({1.0, 0.0}, {{0.80, 0.0}});
  EXPECT_LT(result.scale, 1.0);
  EXPECT_GE(result.scale, 0.0);
  EXPECT_LT(result.command.v, 1.0);
}

TEST(SweptFootprintShield, ChecksReverseMotion)
{
  barn_safety::SweptFootprintShield shield;
  // Obstacle close enough behind that the reverse stopping envelope reaches it.
  const auto result = shield.apply({-0.5, 0.0}, {{-0.50, 0.0}});
  EXPECT_LT(result.scale, 1.0);
  EXPECT_GE(result.command.v, -0.5);
}

TEST(SweptFootprintShield, ChecksRotationSweep)
{
  barn_safety::SweptFootprintShield shield;
  // Obstacle at the swept front corner so the rotation clips it.
  const auto result = shield.apply({0.0, 1.5}, {{0.34, 0.0}});
  EXPECT_LT(result.scale, 1.0);
}

TEST(SweptFootprintShield, AllowsEscapeAwayFromShellObstacle)
{
  // An obstacle 3.6 cm behind the body sits in the old footprint_margin shell.
  // It must NOT block a forward creep away from it (this is the freeze the
  // recovery ran into: commanded to move, clamped to zero).
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.3, 0.0}, {{-0.29, 0.0}});
  EXPECT_GT(result.scale, 0.0);
  EXPECT_GT(result.command.v, 0.0);
}

TEST(SweptFootprintShield, StillVetoesImminentCollision)
{
  // Obstacle ~1.6 cm off the front face: genuinely imminent, must hard-veto.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.4, 0.0}, {{0.27, 0.0}});
  EXPECT_DOUBLE_EQ(result.scale, 0.0);
  EXPECT_DOUBLE_EQ(result.command.v, 0.0);
}

TEST(SweptFootprintShield, ValidScanWithNoReturnsCanRemainClear)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.2, 0.0}, {});
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.v, 0.2);
}
