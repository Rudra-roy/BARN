#include <gtest/gtest.h>

#include "barn_classical/margin_escalator.hpp"

using barn_classical::MarginEscalator;
using barn_classical::MarginEscalatorParams;

namespace
{
// Run the escalator for `seconds` at 20 Hz holding `frozen`, returning the
// margin in force at the end.
double run(MarginEscalator & esc, double & t, double seconds, bool frozen)
{
  double margin = esc.margin();
  for (double elapsed = 0.0; elapsed < seconds; elapsed += 0.05) {
    margin = esc.update(t, frozen);
    t += 0.05;
  }
  return margin;
}
}  // namespace

TEST(MarginEscalator, SitsAtNominalWhileDrivingNormally)
{
  // The whole premise: a world that never freezes must never see a changed
  // margin. Lowering it globally cost world 288 0.4286 -> 0.4128.
  MarginEscalator esc;
  double t = 0.0;
  EXPECT_DOUBLE_EQ(run(esc, t, 30.0, /*frozen=*/false), 0.20);
  EXPECT_FALSE(esc.relaxed());
  EXPECT_FALSE(esc.ever_relaxed());
}

TEST(MarginEscalator, RelaxesInStepsWhileFrozen)
{
  MarginEscalator esc;
  double t = 0.0;
  EXPECT_NEAR(run(esc, t, 0.2, true), 0.15, 1e-9) << "first step is immediate";
  EXPECT_NEAR(run(esc, t, 0.6, true), 0.10, 1e-9) << "second step after the interval";
  EXPECT_TRUE(esc.relaxed());
}

TEST(MarginEscalator, NeverGoesBelowTheFloor)
{
  // 0.10 is the lowest value any measurement here supports. Below it the MPC
  // would plan closer to walls than anything tested.
  MarginEscalator esc;
  double t = 0.0;
  EXPECT_NEAR(run(esc, t, 30.0, true), 0.10, 1e-9);
}

TEST(MarginEscalator, RestoresOnlyAfterSustainedHealthyDriving)
{
  MarginEscalator esc;
  double t = 0.0;
  run(esc, t, 2.0, true);
  ASSERT_NEAR(esc.margin(), 0.10, 1e-9);

  // A brief spell of motion must NOT snap the margin back: that walks the robot
  // straight into the pinch it just escaped and oscillates there.
  EXPECT_NEAR(run(esc, t, 1.0, false), 0.10, 1e-9);
  EXPECT_NEAR(run(esc, t, 1.5, false), 0.15, 1e-9);
  EXPECT_NEAR(run(esc, t, 2.1, false), 0.20, 1e-9);
  EXPECT_FALSE(esc.relaxed());
}

TEST(MarginEscalator, RelaxingIsFastAndRestoringIsSlow)
{
  // Asymmetry is the design: relaxation must be quick to be worth anything
  // against a freeze that otherwise runs out the clock, restoration must be slow
  // so it does not re-enter it.
  MarginEscalator esc;
  double t = 0.0;
  const double t_relax_start = t;
  while (esc.margin() > 0.10 + 1e-9 && t - t_relax_start < 10.0) {esc.update(t, true); t += 0.05;}
  const double relax_time = t - t_relax_start;

  const double t_restore_start = t;
  while (esc.margin() < 0.20 - 1e-9 && t - t_restore_start < 30.0) {esc.update(t, false); t += 0.05;}
  const double restore_time = t - t_restore_start;

  EXPECT_LT(relax_time, 1.5);
  EXPECT_GT(restore_time, relax_time * 3.0);
}

TEST(MarginEscalator, ReFreezingDuringRestoreRelaxesAgain)
{
  MarginEscalator esc;
  double t = 0.0;
  run(esc, t, 2.0, true);
  run(esc, t, 2.1, false);           // one step back up -> 0.15
  ASSERT_NEAR(esc.margin(), 0.15, 1e-9);
  EXPECT_NEAR(run(esc, t, 0.2, true), 0.10, 1e-9);
}

TEST(MarginEscalator, ResetReturnsToNominal)
{
  MarginEscalator esc;
  double t = 0.0;
  run(esc, t, 2.0, true);
  ASSERT_TRUE(esc.relaxed());
  esc.reset();
  EXPECT_DOUBLE_EQ(esc.margin(), 0.20);
  EXPECT_FALSE(esc.ever_relaxed());
}
