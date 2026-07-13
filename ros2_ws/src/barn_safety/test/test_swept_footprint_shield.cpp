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
  const auto result = shield.apply({-0.5, 0.0}, {{-0.70, 0.0}});
  EXPECT_LT(result.scale, 1.0);
  EXPECT_GE(result.command.v, -0.5);
}

TEST(SweptFootprintShield, ChecksRotationSweep)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.0, 1.0}, {{0.42, 0.0}});
  EXPECT_LT(result.scale, 1.0);
}

TEST(SweptFootprintShield, ValidScanWithNoReturnsCanRemainClear)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.2, 0.0}, {});
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.v, 0.2);
}
