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

TEST(SweptFootprintShield, NeverDrivesForwardIntoAnImminentCollision)
{
  // Obstacle ~1.6 cm off the front face. x = 0.27 is INSIDE the veto box
  // (half_length 0.254 + emergency_margin 0.02 = 0.274), so this is the trapped
  // case: the requested forward motion must never survive, but the shield is
  // now allowed to back out of it rather than freeze.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.4, 0.0}, {{0.27, 0.0}});
  EXPECT_LE(result.command.v, 0.0);
}

TEST(SweptFootprintShield, NeverApproachesAnObstacleJustOutsideTheBox)
{
  // Obstacle a tenth of a millimetre OUTSIDE the box: every forward scale
  // sweeps into it. The invariant that matters is that the robot never advances
  // -- whether it holds still or backs off is policy, and since the scale
  // search cannot change direction, standing still here is what stranded
  // world 216 for 75 s.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.4, 0.0}, {{0.2751, 0.0}});
  EXPECT_LE(result.command.v, 0.0);
  EXPECT_NE(result.reason, "clear");
  EXPECT_NE(result.reason, "braking_distance_clamp");
}

TEST(SweptFootprintShield, OffersARotationWhenOnlyTheDesiredDirectionIsBlocked)
{
  // A wall straight ahead, open to the sides. Scaling the forward command can
  // never help, but turning is free -- the shield must not answer "stop".
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({1.0, 0.0}, {{0.2751, 0.0}});
  EXPECT_EQ(result.reason, "emergency_escape");
  EXPECT_GT(std::abs(result.command.v) + std::abs(result.command.w), 0.0);
  EXPECT_LE(result.command.v, 0.0);
}

TEST(SweptFootprintShield, EscapesWhenAnObstacleIsAlreadyInsideTheVetoBox)
{
  // The freeze this exists to prevent: an obstacle in the shell between the
  // self-hit discard radius (half + 1 cm) and the veto box (half +
  // emergency_margin) makes every scale unsafe, including in-place rotation.
  // Before the escape search the robot sat here until the trial timed out.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({1.0, 0.0}, {{0.268, 0.0}});
  EXPECT_EQ(result.reason, "emergency_escape");
  EXPECT_GT(std::abs(result.command.v) + std::abs(result.command.w), 0.0);
  // Whatever it picks must leave the robot further from the obstacle.
  EXPECT_GT(result.minimum_clearance, -(0.254 + 0.02 - 0.268));
}

TEST(SweptFootprintShield, EscapeWillNotBackIntoASecondObstacle)
{
  // Obstacle in front (trapping) and a wall close behind. Reversing would trade
  // one collision for another, so the escape must not simply back up: either it
  // rotates, or it declines and reports the veto honestly.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({1.0, 0.0}, {{0.268, 0.0}, {-0.30, 0.0}});
  EXPECT_LE(result.command.v, 0.0);
  if (result.reason == "emergency_escape") {
    EXPECT_LT(result.command.v, 0.05);
  }
}

TEST(SweptFootprintShield, EscapeCanBeDisabledToRestoreTheHardVeto)
{
  barn_safety::ShieldParams params;
  params.escape_enable = false;
  barn_safety::SweptFootprintShield shield(params);
  const auto result = shield.apply({1.0, 0.0}, {{0.268, 0.0}});
  EXPECT_EQ(result.reason, "emergency_veto");
  EXPECT_DOUBLE_EQ(result.command.v, 0.0);
  EXPECT_DOUBLE_EQ(result.command.w, 0.0);
}

TEST(SweptFootprintShield, TrappedWithNoWayOutIsReportedDistinctlyFromAPlainStop)
{
  // Boxed in on all four sides: no escape motion can improve clearance, so the
  // shield must still refuse to move -- but report "emergency_trapped", not the
  // routine "emergency_veto" of stopping in front of something. The two mean
  // very different things to whoever reads the log, and only one strands a run.
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply(
    {1.0, 0.0}, {{0.268, 0.0}, {-0.268, 0.0}, {0.0, 0.230}, {0.0, -0.230}});
  EXPECT_EQ(result.reason, "emergency_trapped");
  EXPECT_DOUBLE_EQ(result.command.v, 0.0);
  EXPECT_DOUBLE_EQ(result.command.w, 0.0);
}

TEST(SweptFootprintShield, ValidScanWithNoReturnsCanRemainClear)
{
  barn_safety::SweptFootprintShield shield;
  const auto result = shield.apply({0.2, 0.0}, {});
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_DOUBLE_EQ(result.command.v, 0.2);
}

TEST(SweptFootprintShield, DoesNotInjectMotionWhenEscapeIsNotYetAllowed)
{
  // The shield is a filter first: until the caller says the robot has been
  // stuck long enough, it may only reduce the command, never redirect it.
  // Firing immediately turned it into an actuator and shoved the robot off its
  // start pose during the evaluator's reset, adding ~30 s to every trial.
  barn_safety::SweptFootprintShield shield;
  const auto blocked = shield.apply({1.0, 0.0}, {{0.2751, 0.0}}, /*allow_escape=*/false);
  EXPECT_DOUBLE_EQ(blocked.command.v, 0.0);
  EXPECT_DOUBLE_EQ(blocked.command.w, 0.0);

  const auto trapped = shield.apply({1.0, 0.0}, {{0.268, 0.0}}, /*allow_escape=*/false);
  EXPECT_DOUBLE_EQ(trapped.command.v, 0.0);
  EXPECT_DOUBLE_EQ(trapped.command.w, 0.0);
  EXPECT_EQ(trapped.reason, "emergency_trapped");
}
