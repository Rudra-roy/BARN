// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Time-to-collision helper for the hybrid risk gate. This is a small, real,
// closed-form estimate for two points approaching within a combined radius.

#ifndef BARN_DYNAMIC_TRACKING__TTC_HPP_
#define BARN_DYNAMIC_TRACKING__TTC_HPP_

#include <limits>

namespace barn_dynamic_tracking
{

/// Time until the relative position (rel_x, rel_y) closes to within `radius`,
/// given constant relative velocity (rel_vx, rel_vy). Returns +infinity if the
/// pair is separating or never comes within `radius`.
double time_to_collision(
  double rel_x, double rel_y, double rel_vx, double rel_vy, double radius);

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__TTC_HPP_
