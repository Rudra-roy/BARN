// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Pure velocity-limiting logic: magnitude clamp, per-axis acceleration
// rate-limit, and a stale-sensor kill. This is the real safety math and is
// unit-tested without ROS. The obstacle-corridor / clearance methods are
// declared here but currently pass through (STUB) — fill them as Track A
// matures, but they must never widen the limits below.

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

  const barn_core::VelocityCommand & last() const { return last_; }

  // ---- STUB hooks (return `desired` unchanged for now) -------------------
  /// Reduce forward speed based on the nearest obstacle in the travel corridor.
  barn_core::VelocityCommand apply_forward_corridor(
    const barn_core::VelocityCommand & desired, const barn_core::ScanView & scan) const;

private:
  static double rate_limit(double target, double previous, double max_delta);

  barn_core::Limits limits_;
  barn_core::VelocityCommand last_{0.0, 0.0};
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__LIMITER_HPP_
