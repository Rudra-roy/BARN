// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>

#include "barn_dynamic_tracking/kalman.hpp"
#include "barn_dynamic_tracking/ttc.hpp"
#include "gtest/gtest.h"

using barn_dynamic_tracking::ConstantVelocityKF1D;

TEST(Kalman, ConvergesToConstantVelocity)
{
  ConstantVelocityKF1D kf;
  kf.init(0.0);
  const double dt = 0.1;
  const double true_v = 1.5;  // m/s
  double true_pos = 0.0;
  for (int i = 0; i < 100; ++i) {
    true_pos += true_v * dt;
    kf.predict(dt, /*q=*/0.5);
    kf.update(true_pos, /*r=*/0.02);
  }
  EXPECT_NEAR(kf.velocity(), true_v, 0.15);
  EXPECT_NEAR(kf.position(), true_pos, 0.1);
}

TEST(Kalman, StationaryTargetHasZeroVelocity)
{
  ConstantVelocityKF1D kf;
  kf.init(3.0);
  for (int i = 0; i < 50; ++i) {
    kf.predict(0.1, 0.2);
    kf.update(3.0, 0.02);
  }
  EXPECT_NEAR(kf.velocity(), 0.0, 0.1);
  EXPECT_NEAR(kf.position(), 3.0, 0.05);
}

TEST(Ttc, HeadOnClosing)
{
  // Obstacle 5 m ahead closing at 1 m/s; combined radius 0.5 m -> ~4.5 s.
  const double t = barn_dynamic_tracking::time_to_collision(5.0, 0.0, -1.0, 0.0, 0.5);
  EXPECT_NEAR(t, 4.5, 1e-6);
}

TEST(Ttc, SeparatingIsInfinite)
{
  const double t = barn_dynamic_tracking::time_to_collision(5.0, 0.0, 1.0, 0.0, 0.5);
  EXPECT_TRUE(std::isinf(t));
}
