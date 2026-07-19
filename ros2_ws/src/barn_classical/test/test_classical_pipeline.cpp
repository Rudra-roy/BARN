#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "barn_classical/collision_checker.hpp"
#include "barn_classical/controller.hpp"
#include "barn_classical/global_planner_astar.hpp"
#include "barn_classical/local_planner.hpp"
#include "barn_classical/path_validator.hpp"
#include "barn_classical/recovery.hpp"
#include "barn_core/distance_field.hpp"
#include "barn_core/occupancy.hpp"

namespace
{

barn_core::OccupancyGrid2D free_grid()
{
  barn_core::OccupancyGrid2D grid(120, 80, 0.10, 0.0, 0.0);
  for (int row = 0; row < 80; ++row) {
    for (int col = 0; col < 120; ++col) {
      grid.set_log_odds({col, row}, -2.0);
    }
  }
  return grid;
}

}  // namespace

TEST(CollisionChecker, RejectsRotatedCornerContact)
{
  auto grid = free_grid();
  grid.set_log_odds(grid.world_to_cell(2.35, 2.0), 4.0);
  EXPECT_TRUE(barn_classical::footprint_is_clear(grid, {2.0, 2.0, 0.0}));
  EXPECT_FALSE(barn_classical::footprint_is_clear(grid, {2.0, 2.0, M_PI / 4.0}));
}

TEST(GlobalPlanner, PlansAndValidatesOpenSpace)
{
  auto grid = free_grid();
  barn_classical::AStarParams params;
  params.timeout_ms = 500.0;
  barn_classical::GlobalPlannerAStar planner(params);
  const auto path = planner.plan(grid, {1.0, 4.0, 0.0}, {8.0, 4.0, 0.0, 0.2});
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(barn_classical::PathValidator().is_path_clear(path, grid, false));
  EXPECT_LT(std::hypot(path.back().x - 8.0, path.back().y - 4.0), 0.25);
}

TEST(GlobalPlanner, RejectsFullyBlockedWall)
{
  auto grid = free_grid();
  for (int row = 0; row < 80; ++row) {
    grid.set_log_odds({50, row}, 4.0);
    grid.set_log_odds({51, row}, 4.0);
  }
  barn_classical::AStarParams params;
  params.timeout_ms = 500.0;
  barn_classical::GlobalPlannerAStar planner(params);
  EXPECT_TRUE(planner.plan(grid, {1.0, 4.0, 0.0}, {8.0, 4.0, 0.0, 0.2}).empty());
}

TEST(LocalPlanner, CapsUnknownSpeed)
{
  barn_core::OccupancyGrid2D grid(120, 80, 0.10, 0.0, 0.0);
  barn_classical::Path2D path;
  for (int i = 10; i <= 50; i += 2) {
    path.push_back({i * 0.1, 4.0, 0.0});
  }
  const auto trajectory = barn_classical::LocalPlanner().plan(path, path.front(), grid);
  ASSERT_FALSE(trajectory.empty());
  for (const auto & point : trajectory) {
    EXPECT_TRUE(point.in_unknown);
    EXPECT_LE(point.v_ref, 0.4 + 1e-9);
  }
}

TEST(Controller, TracksStraightPathWithinConstraints)
{
  auto grid = free_grid();
  // Give the distance field finite values without constraining the straight corridor.
  for (int col = 0; col < 120; ++col) {
    grid.set_log_odds({col, 0}, 4.0);
    grid.set_log_odds({col, 79}, 4.0);
  }
  barn_core::DistanceField2D field;
  field.rebuild(grid);
  barn_classical::LocalTrajectory trajectory;
  for (int i = 0; i <= 20; ++i) {
    trajectory.push_back({{1.0 + 0.2 * i, 4.0, 0.0}, 1.0, 3.0, false});
  }
  barn_classical::MpcParams params;
  params.solve_deadline_ms = 200.0;
  params.max_linearization_passes = 1;
  barn_classical::Controller controller(params);
  const auto result = controller.control(trajectory, {{1.0, 4.0, 0.0}, 0.0, 0.0}, field);
  ASSERT_TRUE(result.success) << result.status << " in " << result.solve_ms << " ms";
  EXPECT_GE(result.command.v, 0.0);
  EXPECT_LE(result.command.v, 2.0);
  EXPECT_LE(std::abs(result.command.w), 1.5);
  EXPECT_EQ(result.prediction.size(), 21U);
}

namespace
{
barn_core::ScanView uniform_scan(std::vector<float> & ranges)
{
  ranges.assign(361, 5.0f);
  return barn_core::ScanView{ranges.data(), ranges.size(), -static_cast<float>(M_PI),
    static_cast<float>(2.0 * M_PI / 360.0), 0.05f, 10.0f};
}

barn_classical::RecoveryContext make_ctx(
  const barn_core::ScanView & scan, double clearance,
  const barn_core::Pose2D & pose = {},
  const std::vector<barn_core::Pose2D> * breadcrumb = nullptr)
{
  barn_classical::RecoveryContext ctx;
  ctx.pose = pose;
  ctx.clearance = clearance;
  ctx.rotation_radius = 0.40;
  ctx.scan = scan;
  ctx.breadcrumb = breadcrumb;
  return ctx;
}
}  // namespace

TEST(Recovery, EnforcesAttemptLimit)
{
  std::vector<float> ranges;
  const auto scan = uniform_scan(ranges);
  barn_classical::RecoveryParams params;
  params.max_attempts = 3;
  barn_classical::Recovery recovery(params);
  auto ctx = make_ctx(scan, /*clearance=*/1.0);  // plenty of room; no reverse needed
  for (int attempt = 0; attempt < 3; ++attempt) {
    recovery.trigger(ctx);
    EXPECT_EQ(recovery.attempts(), attempt + 1);
    for (int i = 0; i < 200 && !recovery.request_replan(); ++i) {
      (void)recovery.step(0.1, ctx);
    }
    EXPECT_TRUE(recovery.request_replan());
    recovery.finish_replan();
  }
  recovery.trigger(ctx);
  EXPECT_EQ(recovery.state(), barn_classical::RecoveryState::kFailed);
}

TEST(Recovery, VetoEscapeStopsWhenShieldClears)
{
  std::vector<float> ranges;
  const auto scan = uniform_scan(ranges);
  barn_classical::RecoveryParams params;
  params.veto_clear_min_rotate = 0.2;
  params.blocked_timeout = 5.0;  // keep the blocked-bail out of the way for this test
  barn_classical::Recovery recovery(params);

  auto ctx = make_ctx(scan, /*clearance=*/0.2);  // too tight to rotate -> reverse maneuver
  recovery.trigger_veto_escape(ctx);
  // While the shield keeps vetoing, the escape maneuver runs and does not conclude.
  ctx.veto_active = true;
  for (int i = 0; i < 5; ++i) {
    (void)recovery.step(0.1, ctx);
  }
  EXPECT_FALSE(recovery.request_replan());
  EXPECT_TRUE(recovery.active());

  // Shield clears: the escape concludes promptly (next tick past the minimal
  // maneuver guard), not after the full timeout.
  ctx.veto_active = false;
  const auto cmd = recovery.step(0.1, ctx);
  EXPECT_TRUE(recovery.request_replan());
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(cmd.w, 0.0);
}

TEST(Recovery, ReverseToClearanceThenReplan)
{
  std::vector<float> ranges;
  const auto scan = uniform_scan(ranges);
  barn_classical::Recovery recovery;
  // Robot at the origin, breadcrumb trailing behind along -x (oldest -> newest).
  const std::vector<barn_core::Pose2D> breadcrumb{{-1.0, 0.0, 0.0}, {-0.5, 0.0, 0.0},
    {0.0, 0.0, 0.0}};
  auto ctx = make_ctx(scan, /*clearance=*/0.2, /*pose=*/{0.0, 0.0, 0.0}, &breadcrumb);

  recovery.trigger(ctx);
  EXPECT_EQ(recovery.state(), barn_classical::RecoveryState::kReverseToClearance);
  const auto reversing = recovery.step(0.1, ctx);
  EXPECT_LT(reversing.v, 0.0);  // commanded to back up along the breadcrumb

  // Once enough room opens up, it hands off to a replan.
  ctx.clearance = 0.6;
  (void)recovery.step(0.1, ctx);
  EXPECT_TRUE(recovery.request_replan());
}

TEST(Recovery, ProgressRefundsAttemptBudget)
{
  std::vector<float> ranges;
  const auto scan = uniform_scan(ranges);
  barn_classical::RecoveryParams params;
  params.max_attempts = 3;
  barn_classical::Recovery recovery(params);
  auto ctx = make_ctx(scan, /*clearance=*/1.0);

  // One episode to its replan, then finish it: INACTIVE but attempts_ == 1.
  recovery.trigger(ctx);
  for (int i = 0; i < 200 && !recovery.request_replan(); ++i) {
    (void)recovery.step(0.1, ctx);
  }
  recovery.finish_replan();
  EXPECT_FALSE(recovery.active());
  EXPECT_EQ(recovery.attempts(), 1);

  // Progress refunds the budget; a fresh episode does not immediately fail.
  recovery.notify_progress();
  EXPECT_EQ(recovery.attempts(), 0);
  recovery.trigger(ctx);
  EXPECT_NE(recovery.state(), barn_classical::RecoveryState::kFailed);
}
