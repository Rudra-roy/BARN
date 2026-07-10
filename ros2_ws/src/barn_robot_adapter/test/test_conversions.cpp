// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>
#include <vector>

#include "barn_robot_adapter/conversions.hpp"
#include "gtest/gtest.h"

TEST(Conversions, OdomToPose2d)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 1.5;
  odom.pose.pose.position.y = -0.5;
  odom.pose.pose.orientation.z = std::sin(M_PI / 4.0);
  odom.pose.pose.orientation.w = std::cos(M_PI / 4.0);

  const auto p = barn_robot_adapter::to_pose2d(odom);
  EXPECT_DOUBLE_EQ(p.x, 1.5);
  EXPECT_DOUBLE_EQ(p.y, -0.5);
  EXPECT_NEAR(p.yaw, M_PI / 2.0, 1e-9);
}

TEST(Conversions, TwistRoundTrip)
{
  barn_core::VelocityCommand cmd{0.8, -0.3};
  const auto twist = barn_robot_adapter::to_twist(cmd);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.8);
  EXPECT_DOUBLE_EQ(twist.angular.z, -0.3);

  const auto back = barn_robot_adapter::from_twist(twist);
  EXPECT_DOUBLE_EQ(back.v, 0.8);
  EXPECT_DOUBLE_EQ(back.w, -0.3);
}

TEST(Conversions, TwistStampedCarriesHeader)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  const auto msg = barn_robot_adapter::to_twist_stamped({1.0, 0.5}, "base_link", stamp);
  EXPECT_EQ(msg.header.frame_id, "base_link");
  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_DOUBLE_EQ(msg.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.twist.angular.z, 0.5);
}

TEST(Conversions, ScanViewBorrowsBuffer)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -1.0f;
  scan.angle_increment = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = std::vector<float>{1.0f, 2.0f, 3.0f};

  const auto view = barn_robot_adapter::to_view(scan);
  ASSERT_TRUE(view.valid());
  EXPECT_EQ(view.count, 3u);
  EXPECT_FLOAT_EQ(view.range_max, 10.0f);
  EXPECT_FLOAT_EQ(view.ranges[1], 2.0f);
}
