#include <gtest/gtest.h>

#include "barn_classical/freeze_detector.hpp"

using barn_classical::FreezeDetector;

namespace
{
// Drive the detector for `seconds` at 20 Hz holding one situation, returning the
// detector so the caller can assert on it.
void feed(
  FreezeDetector & det, double & t, double seconds, double commanded_v,
  bool moving, bool shield_passive = true, bool mpc_solved = true,
  double goal_distance = 5.0)
{
  barn_core::Pose2D pose;
  static double x = 0.0;
  for (double elapsed = 0.0; elapsed < seconds; elapsed += 0.05) {
    if (moving) {x += 0.05;}
    pose.x = x;
    det.update(t, pose, commanded_v, goal_distance, shield_passive, mpc_solved);
    t += 0.05;
  }
}
}  // namespace

TEST(FreezeDetector, ReportsAStopWithNothingBlocking)
{
  // The world-66 state: commanded to zero, not moving, shield passive, MPC
  // reporting a solve, goal still far away. Nothing explains the stop.
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 2.0, 0.0, /*moving=*/false);
  EXPECT_TRUE(det.frozen());
  EXPECT_GT(det.duration_s(), 0.0);
}

TEST(FreezeDetector, WaitsForTheProgressWindowPlusTheDwellBeforeReporting)
{
  // A momentary zero is normal -- braking, a replan landing, a tick between
  // trajectories. Only a SUSTAINED stop is the bug.
  //
  // Total latency is progress_window_s + dwell_s (~1.75 s), NOT dwell alone:
  // "has it moved" is answered over a trailing window, and that window still
  // holds the poses from while the robot was moving until it flushes. Worth
  // stating in a test because it is the quantity that decides how much of a
  // freeze is recoverable -- against the 6.8 s+ episodes observed on world 66 it
  // is affordable, but it is a quarter of a short one.
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 1.0, 0.0, false);
  EXPECT_FALSE(det.frozen()) << "fired before window + dwell elapsed";
  feed(det, t, 1.0, 0.0, false);
  EXPECT_TRUE(det.frozen());
}

TEST(FreezeDetector, StaysQuietWhileTheShieldIsVetoing)
{
  // A shield veto is the shield doing its job and has its own escape logic.
  // Reporting it here would double-count a different failure.
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 3.0, 0.0, false, /*shield_passive=*/false);
  EXPECT_FALSE(det.frozen());
}

TEST(FreezeDetector, StaysQuietWhileRecoveryOrStartupOwnsTheRobot)
{
  // Recovery and startup creep are explained stops -- something is already
  // acting. Offline these accounted for 45 s of stalled time across the
  // validation traces and the detector fired on none of it.
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 3.0, 0.0, false, true, /*mpc_solved=*/false);
  EXPECT_FALSE(det.frozen());
}

TEST(FreezeDetector, StaysQuietWhenTheRobotIsActuallyMoving)
{
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 3.0, 1.0, /*moving=*/true);
  EXPECT_FALSE(det.frozen());
}

TEST(FreezeDetector, StaysQuietWhenStoppedAtTheGoal)
{
  // Arriving is not freezing.
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 3.0, 0.0, false, true, true, /*goal_distance=*/0.2);
  EXPECT_FALSE(det.frozen());
}

TEST(FreezeDetector, ClearsOnceTheRobotMovesAgain)
{
  FreezeDetector det;
  double t = 0.0;
  feed(det, t, 2.0, 0.0, false);
  ASSERT_TRUE(det.frozen());
  feed(det, t, 1.0, 1.0, /*moving=*/true);
  EXPECT_FALSE(det.frozen());
  EXPECT_DOUBLE_EQ(det.duration_s(), 0.0);
}

TEST(FreezeDetector, FiresOnceNotEveryCycle)
{
  // just_fired() is the logging edge; it must not spam every control tick.
  FreezeDetector det;
  double t = 0.0;
  barn_core::Pose2D pose;
  int fires = 0;
  for (int i = 0; i < 100; ++i) {
    det.update(t, pose, 0.0, 5.0, true, true);
    if (det.just_fired()) {++fires;}
    t += 0.05;
  }
  EXPECT_EQ(fires, 1);
}
