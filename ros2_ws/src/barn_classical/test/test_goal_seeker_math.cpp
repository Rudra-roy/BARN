// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>

#include "barn_classical/goal_seeker.hpp"
#include "gtest/gtest.h"

using barn_classical::GoalSeeker;
using barn_classical::GoalSeekerParams;
using barn_core::Goal2D;
using barn_core::Pose2D;

namespace
{
GoalSeekerParams params()
{
  GoalSeekerParams p;
  p.v_nominal = 0.5;
  p.v_max = 2.0;
  p.w_max = 1.2;
  p.k_ang = 1.5;
  p.heading_tol = 0.35;
  p.stop_distance = 0.45;
  p.slow_distance = 1.2;
  p.goal_tolerance = 0.8;
  p.creep_fraction = 0.15;
  return p;
}
}  // namespace

TEST(GoalSeeker, StopsAtGoal)
{
  GoalSeeker s(params());
  Pose2D pose{9.7, 0.0, 0.0};
  Goal2D goal{10.0, 0.0};  // 0.3 m away < goal_tolerance
  const auto cmd = s.compute(pose, goal, 10.0);
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(cmd.w, 0.0);
}

TEST(GoalSeeker, CruisesWhenAlignedAndClear)
{
  GoalSeeker s(params());
  Pose2D pose{0.0, 0.0, 0.0};
  Goal2D goal{10.0, 0.0};
  const auto cmd = s.compute(pose, goal, 10.0);  // wide-open front
  EXPECT_NEAR(cmd.v, 0.5, 1e-9);
  EXPECT_NEAR(cmd.w, 0.0, 1e-9);
}

TEST(GoalSeeker, TurnsTowardGoalOnTheLeft)
{
  GoalSeeker s(params());
  Pose2D pose{0.0, 0.0, 0.0};
  Goal2D goal{0.0, 10.0};  // dead left -> heading error +pi/2
  const auto cmd = s.compute(pose, goal, 10.0);
  EXPECT_GT(cmd.w, 0.0);
  EXPECT_LE(cmd.w, 0.5 * 0.5 + 1.2);  // clamped
  // Poorly aligned -> creep only.
  EXPECT_NEAR(cmd.v, 0.15 * 0.5, 1e-9);
}

TEST(GoalSeeker, StopsForwardWhenObstacleClose)
{
  GoalSeeker s(params());
  Pose2D pose{0.0, 0.0, 0.0};
  Goal2D goal{10.0, 0.0};
  const auto cmd = s.compute(pose, goal, 0.3);  // < stop_distance
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
}

TEST(GoalSeeker, RampsSpeedWithClearance)
{
  GoalSeeker s(params());
  Pose2D pose{0.0, 0.0, 0.0};
  Goal2D goal{10.0, 0.0};
  // Midway between stop (0.45) and slow (1.2): scale ~ 0.5.
  const double mid = 0.5 * (0.45 + 1.2);
  const auto cmd = s.compute(pose, goal, mid);
  EXPECT_GT(cmd.v, 0.0);
  EXPECT_LT(cmd.v, 0.5);
}
