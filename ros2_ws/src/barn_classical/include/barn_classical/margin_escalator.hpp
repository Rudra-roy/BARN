// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__MARGIN_ESCALATOR_HPP_
#define BARN_CLASSICAL__MARGIN_ESCALATOR_HPP_

#include <algorithm>

namespace barn_classical
{

/// Tuning for MarginEscalator.
struct MarginEscalatorParams
{
  /// Nominal obstacle_margin, restored whenever the robot is driving normally.
  double nominal{0.20};
  /// Floor. 0.10 is the value measured to clear the freeze on world 66
  /// (0.2263 -> 0.2901, AT max 49.9 -> 20.4 s, recovery 9 -> 0). Going lower was
  /// never tested and would let the MPC plan nearer walls than any measurement
  /// here supports.
  double minimum{0.10};
  /// Size of each relaxation and each restoration step.
  double step{0.05};
  /// Minimum time between relaxation steps. The freeze detector already needs
  /// ~1.75 s to confirm, so this only paces what happens AFTER that: at 0.5 s the
  /// margin reaches the floor 1 s after confirmation.
  double step_interval_s{0.5};
  /// Healthy driving required before the margin is stepped back up. Long
  /// relative to step_interval_s ON PURPOSE -- relaxing must be quick to be
  /// useful, restoring must be slow so the robot does not walk straight back
  /// into the freeze it just escaped and oscillate there.
  double restore_after_s{2.0};
};

/// Relaxes the MPC's obstacle margin while the robot is frozen, and restores it
/// once it is driving again.
///
/// WHY THIS EXISTS
/// The MPC's obstacle row demands `half_width + footprint.margin +
/// obstacle_margin` = 0.4559 m of clearance. BARN's reference paths thread
/// 0.225 m gaps. Where the corridor is narrower than the demand, the constraint
/// is violated at the current pose, no action satisfies it, and the QP's
/// cheapest answer is v = 0 -- the robot stops in a corridor it could drive
/// through, until the trial times out.
///
/// WHY NOT JUST LOWER THE PARAMETER
/// Setting obstacle_margin to 0.10 globally fixes world 66 (0.2263 -> 0.2901)
/// but costs world 288 (0.4286 -> 0.4128, AT median 11.9 -> 13.6 s): with less
/// margin the planner routes nearer walls on worlds that were never
/// clearance-starved. Relaxing ONLY while frozen costs exactly nothing on worlds
/// that never freeze, which is most of them -- the minor tier logged 14 recovery
/// episodes across 150 trials.
///
/// Three code attempts to fix this inside the QP were measured and rejected
/// (clamping the demanded margin at the current pose, over the horizon minimum,
/// and per-step from the plan's own clearance). All three fed the constraint a
/// different clearance number and the mechanism is not sensitive to that; the
/// thing that works is lowering the demand unconditionally, which is what this
/// does for the duration of a freeze and no longer.
class MarginEscalator
{
public:
  explicit MarginEscalator(const MarginEscalatorParams & params = {})
  : params_(params), margin_(params.nominal) {}

  /// One control cycle. `frozen` comes from FreezeDetector.
  /// Returns the obstacle margin to use from now on.
  double update(double now_s, bool frozen)
  {
    if (frozen) {
      healthy_since_ = -1.0;
      if (last_step_s_ < 0.0 || (now_s - last_step_s_) >= params_.step_interval_s) {
        const double next = std::max(params_.minimum, margin_ - params_.step);
        stepped_down_ |= next < margin_;
        margin_ = next;
        last_step_s_ = now_s;
      }
      return margin_;
    }

    // Not frozen. Only restore after a sustained stretch of healthy driving.
    if (healthy_since_ < 0.0) {
      healthy_since_ = now_s;
    }
    if (margin_ < params_.nominal &&
      (now_s - healthy_since_) >= params_.restore_after_s)
    {
      margin_ = std::min(params_.nominal, margin_ + params_.step);
      healthy_since_ = now_s;      // one step per restore_after_s, not a ramp
      last_step_s_ = -1.0;
      if (margin_ >= params_.nominal) {
        stepped_down_ = false;
      }
    }
    return margin_;
  }

  double margin() const {return margin_;}
  /// True while the margin sits below nominal -- i.e. an escape is in progress.
  bool relaxed() const {return margin_ < params_.nominal - 1e-9;}
  /// True if this run has ever relaxed, for reporting.
  bool ever_relaxed() const {return stepped_down_;}

  void reset()
  {
    margin_ = params_.nominal;
    last_step_s_ = -1.0;
    healthy_since_ = -1.0;
    stepped_down_ = false;
  }

private:
  MarginEscalatorParams params_;
  double margin_;
  double last_step_s_{-1.0};
  double healthy_since_{-1.0};
  bool stepped_down_{false};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__MARGIN_ESCALATOR_HPP_
