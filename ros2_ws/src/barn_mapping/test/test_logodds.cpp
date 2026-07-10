// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_core/logodds.hpp"
#include "barn_core/occupancy.hpp"
#include "gtest/gtest.h"

TEST(LogOdds, ProbabilityRoundTrip)
{
  for (double p : {0.1, 0.3, 0.5, 0.7, 0.9}) {
    EXPECT_NEAR(barn_core::logodds_to_prob(barn_core::prob_to_logodds(p)), p, 1e-9);
  }
}

TEST(LogOdds, HitsRaiseMissesLower)
{
  barn_core::InverseSensorModel model;
  double cell = 0.0;
  cell = model.update(cell, /*occupied=*/true);
  EXPECT_GT(cell, 0.0);
  const double after_hit = cell;
  cell = model.update(cell, /*occupied=*/false);
  EXPECT_LT(cell, after_hit);
}

TEST(LogOdds, ClampsSaturate)
{
  barn_core::InverseSensorModel model;
  double cell = 0.0;
  for (int i = 0; i < 100; ++i) {
    cell = model.update(cell, true);
  }
  EXPECT_LE(cell, model.clamp_max);
  EXPECT_NEAR(cell, model.clamp_max, 1e-9);
}

TEST(OccupancyGrid, WorldCellRoundTrip)
{
  barn_core::OccupancyGrid2D grid(400, 240, 0.05, -10.0, -6.0);
  const auto idx = grid.world_to_cell(1.23, -0.77);
  ASSERT_TRUE(grid.in_bounds(idx));
  double wx = 0.0;
  double wy = 0.0;
  grid.cell_to_world(idx, wx, wy);
  EXPECT_NEAR(wx, 1.23, grid.resolution());
  EXPECT_NEAR(wy, -0.77, grid.resolution());
}

TEST(OccupancyGrid, Classification)
{
  barn_core::OccupancyGrid2D grid(10, 10, 0.1, 0.0, 0.0);
  barn_core::GridIndex idx{5, 5};
  EXPECT_EQ(grid.classify(idx), barn_core::CellState::kUnknown);
  grid.set_log_odds(idx, 2.0);
  EXPECT_EQ(grid.classify(idx), barn_core::CellState::kOccupied);
  grid.set_log_odds(idx, -2.0);
  EXPECT_EQ(grid.classify(idx), barn_core::CellState::kFree);
}
