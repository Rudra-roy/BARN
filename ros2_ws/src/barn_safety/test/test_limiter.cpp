// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_safety/limiter.hpp"
#include "gtest/gtest.h"

using barn_core::Limits;
using barn_core::VelocityCommand;
using barn_safety::Limiter;

namespace
{
Limits test_limits()
{
  Limits l;
  l.v_max = 2.0;
  l.w_max = 1.5;
  l.a_lin = 2.0;  // m/s^2
  l.a_ang = 4.0;  // rad/s^2
  return l;
}
}  // namespace

TEST(Limiter, StaleSensorsForceZero)
{
  Limiter lim(test_limits());
  const auto out = lim.apply({2.0, 1.0}, 0.05, /*sensors_fresh=*/false);
  EXPECT_DOUBLE_EQ(out.v, 0.0);
  EXPECT_DOUBLE_EQ(out.w, 0.0);
}

TEST(Limiter, ClampsMagnitude)
{
  Limiter lim(test_limits());
  // Large dt so the accel rate-limit does not bind; the magnitude clamp should.
  const auto out = lim.apply({10.0, -10.0}, 100.0, true);
  EXPECT_DOUBLE_EQ(out.v, 2.0);
  EXPECT_DOUBLE_EQ(out.w, -1.5);
}

TEST(Limiter, AccelRateLimitsFromRest)
{
  Limiter lim(test_limits());
  // From rest, one 0.1 s step at a_lin=2.0 allows at most +0.2 m/s.
  const auto out = lim.apply({2.0, 0.0}, 0.1, true);
  EXPECT_NEAR(out.v, 0.2, 1e-9);
  EXPECT_NEAR(out.w, 0.0, 1e-9);
}

TEST(Limiter, AccelRampAccumulates)
{
  Limiter lim(test_limits());
  lim.apply({2.0, 0.0}, 0.1, true);          // -> 0.2
  const auto out = lim.apply({2.0, 0.0}, 0.1, true);  // -> 0.4
  EXPECT_NEAR(out.v, 0.4, 1e-9);
}

TEST(Limiter, ResetClearsHistory)
{
  Limiter lim(test_limits());
  lim.apply({2.0, 0.0}, 0.1, true);  // -> 0.2
  lim.reset();
  const auto out = lim.apply({2.0, 0.0}, 0.1, true);  // from rest again -> 0.2
  EXPECT_NEAR(out.v, 0.2, 1e-9);
}
