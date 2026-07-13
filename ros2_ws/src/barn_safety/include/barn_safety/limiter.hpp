// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure velocity-limiting logic: magnitude clamp, per-axis acceleration
// rate-limit, and a stale-sensor kill. This is unit-tested without ROS and is
// deliberately independent of the swept-footprint shield.

#ifndef BARN_SAFETY__LIMITER_HPP_
#define BARN_SAFETY__LIMITER_HPP_

#include "barn_core/scan.hpp"
#include "barn_core/types.hpp"

namespace barn_safety
{

class Limiter
{
public:
  explicit Limiter(const barn_core::Limits & limits = {}) : limits_(limits) {}

  void set_limits(const barn_core::Limits & limits) { limits_ = limits; }

  /// Integrate one command. `dt` is the time since the previous accepted
  /// command; `sensors_fresh` is false when scan/command inputs are stale.
  /// Returns the safe command and remembers it for the next rate-limit step.
  barn_core::VelocityCommand apply(
    const barn_core::VelocityCommand & desired, double dt, bool sensors_fresh);

  /// Forget the previous command (call on (re)start or after an e-stop).
  void reset();
  /// Synchronize rate-limit state after an independent shield clamps a command.
  void override_last(const barn_core::VelocityCommand & command) { last_ = command; }

  const barn_core::VelocityCommand & last() const { return last_; }

private:
  static double rate_limit(double target, double previous, double max_delta);

  barn_core::Limits limits_;
  barn_core::VelocityCommand last_{0.0, 0.0};
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__LIMITER_HPP_
