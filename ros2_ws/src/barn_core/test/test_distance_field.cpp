// Copyright 2026 barn-2027-prep contributors. MIT License.

#include <cmath>

#include "barn_core/distance_field.hpp"
#include "gtest/gtest.h"

TEST(DistanceField, ComputesExactEuclideanDistance)
{
  barn_core::OccupancyGrid2D grid(7, 7, 0.1, 0.0, 0.0);
  grid.set_log_odds({3, 3}, 1.0);
  barn_core::DistanceField2D field;
  field.rebuild(grid);

  EXPECT_NEAR(field.distance({3, 3}), 0.0, 1e-9);
  EXPECT_NEAR(field.distance({4, 3}), 0.1, 1e-9);
  EXPECT_NEAR(field.distance({4, 4}), std::sqrt(2.0) * 0.1, 1e-9);
}

TEST(DistanceField, UnknownCanBeFreeOrOccupied)
{
  barn_core::OccupancyGrid2D grid(3, 3, 0.1, 0.0, 0.0);
  barn_core::DistanceField2D field;
  field.rebuild(grid, false);
  EXPECT_TRUE(std::isinf(field.distance({1, 1})));
  field.rebuild(grid, true);
  EXPECT_DOUBLE_EQ(field.distance({1, 1}), 0.0);
}
