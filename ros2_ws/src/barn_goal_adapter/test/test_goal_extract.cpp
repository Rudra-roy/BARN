// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>

#include "barn_goal_adapter/goal_conversions.hpp"
#include "gtest/gtest.h"

TEST(GoalConversions, ExtractsPositionAndYaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.position.x = 10.0;
  p.pose.position.y = -2.0;
  // 90-degree yaw about z.
  p.pose.orientation.z = std::sin(M_PI / 4.0);
  p.pose.orientation.w = std::cos(M_PI / 4.0);

  const auto g = barn_goal_adapter::to_goal2d(p, 0.8);
  EXPECT_DOUBLE_EQ(g.x, 10.0);
  EXPECT_DOUBLE_EQ(g.y, -2.0);
  EXPECT_NEAR(g.yaw, M_PI / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(g.tol, 0.8);
}

TEST(GoalConversions, IdentityQuaternionIsZeroYaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.orientation.w = 1.0;
  const auto g = barn_goal_adapter::to_goal2d(p, 1.0);
  EXPECT_NEAR(g.yaw, 0.0, 1e-9);
}
