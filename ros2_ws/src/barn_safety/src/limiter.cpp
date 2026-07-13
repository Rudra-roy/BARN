// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_safety/limiter.hpp"

#include "barn_core/geometry.hpp"

namespace barn_safety
{

double Limiter::rate_limit(double target, double previous, double max_delta)
{
  const double delta = barn_core::clamp(target - previous, -max_delta, max_delta);
  return previous + delta;
}

barn_core::VelocityCommand Limiter::apply(
  const barn_core::VelocityCommand & desired, double dt, bool sensors_fresh)
{
  if (!sensors_fresh) {
    last_ = {0.0, 0.0};
    return last_;
  }

  // 1. Magnitude clamp.
  barn_core::VelocityCommand cmd;
  cmd.v = barn_core::clamp(desired.v, -limits_.v_max, limits_.v_max);
  cmd.w = barn_core::clamp(desired.w, -limits_.w_max, limits_.w_max);

  // 2. Per-axis acceleration rate-limit (skip if dt is non-positive / unknown).
  if (dt > 0.0) {
    cmd.v = rate_limit(cmd.v, last_.v, limits_.a_lin * dt);
    cmd.w = rate_limit(cmd.w, last_.w, limits_.a_ang * dt);
  }

  last_ = cmd;
  return cmd;
}

void Limiter::reset() { last_ = {0.0, 0.0}; }

}  // namespace barn_safety
