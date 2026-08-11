// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__FREEZE_DETECTOR_HPP_
#define BARN_CLASSICAL__FREEZE_DETECTOR_HPP_

#include <cmath>
#include <deque>

#include "barn_core/types.hpp"

namespace barn_classical
{

/// Tuning for FreezeDetector. Hoisted to namespace scope (as ShieldParams is
/// in barn_safety) so it can be a defaulted {} constructor argument.
struct FreezeDetectorParams
{
  /// Sustained time in the state before it is reported. Cheaper to be wrong
  /// than the shield's escape (which injects motion, and waits 1.5 s): a false
  /// positive here only relaxes a margin. 0.75 s catches 77% of freeze time at
  /// zero false positives over the validation traces; 0.5 s catches 81% but 8
  /// traces is too small a sample to spend the margin on 4%.
  double dwell_s{0.75};
  /// Below this the command counts as "not asking to move".
  double v_eps{0.05};
  /// Displacement over progress_window_s below this counts as "did not move".
  /// Pose publishes slower than the control loop, so per-tick deltas are mostly
  /// zero and useless; only a window can answer "has it moved".
  double move_eps_m{0.03};
  /// NOTE total detection latency is progress_window_s + dwell_s (~1.75 s), not
  /// dwell alone: the trailing window still holds poses from while the robot was
  /// moving until it flushes. Affordable against the 6.8 s+ freezes observed on
  /// world 66; shorten both if short episodes turn out to matter.
  double progress_window_s{1.0};
  /// Do not report a freeze once essentially at the goal.
  double goal_tol_m{1.0};
};

/// Detects "the robot is stopped and nothing is stopping it".
///
/// The MPC's obstacle row requires `distance_field >= obstacle_margin` at
/// footprint boundary points already inflated by footprint.margin, i.e. a
/// demanded centre clearance of 0.2159 + 0.04 + 0.20 = 0.4559 m. BARN's own
/// reference paths thread gaps at 0.225 m. Where the corridor is narrower than
/// the demand the constraint is violated AT THE CURRENT POSE, no control action
/// satisfies it, every candidate pays the same heavy slack, and the cheapest one
/// is v = 0. The robot stops in a corridor it could drive through.
///
/// Traced on world 66: 6.8 s of a 21 s trial stationary at 0.40 m clearance with
/// 0.69 m free ahead, the shield reporting `clear` at scale 1.0 and the MPC
/// reporting `solved`. Nothing is blocking it and nothing reports an error.
///
/// The point of a detector rather than a smaller global margin: lowering
/// obstacle_margin to 0.10 fixes world 66 (+0.064) but costs world 288 (-0.016),
/// because with less margin the planner routes nearer walls on worlds that were
/// never clearance-starved. A detector costs nothing where there is no freeze,
/// which is most worlds -- the minor tier logged 14 recovery episodes in 150
/// trials.
///
/// Validated offline against 8 recorded traces (evaluation/tuning/freeze_detector.py,
/// which is this logic in Python): of 61.0 s of genuine freeze it catches 77% at
/// a 0.75 s dwell with ZERO false positives, and never fires during recovery,
/// startup creep, or a shield veto. It fires 0.1-0.25 s after freeze onset.
///
/// Deliberately a pure state machine over one sample at a time: no lookahead and
/// no whole-run statistics, so the offline validation and the online behaviour
/// are the same code path.
class FreezeDetector
{
public:
  explicit FreezeDetector(const FreezeDetectorParams & params = {}) : params_(params) {}

  /// One control cycle.
  ///
  /// `shield_passive` must be false when the shield is vetoing: that is the
  /// shield doing its job, a different failure with its own escape logic.
  /// `mpc_solved` must be false whenever the planner is in recovery, creeping at
  /// startup, or waiting for inputs -- those stops are explained, and something
  /// is already acting on them.
  void update(
    double now_s, const barn_core::Pose2D & pose, double commanded_v,
    double goal_distance, bool shield_passive, bool mpc_solved)
  {
    history_.push_back({now_s, pose.x, pose.y});
    while (history_.size() > 1 && now_s - history_.front().t > params_.progress_window_s) {
      history_.pop_front();
    }

    double moved = 0.0;
    const bool window_full = history_.size() >= 2 &&
      (history_.back().t - history_.front().t) >= 0.8 * params_.progress_window_s;
    if (window_full) {
      moved = std::hypot(pose.x - history_.front().x, pose.y - history_.front().y);
    }

    const bool frozen_now =
      std::abs(commanded_v) < params_.v_eps &&
      goal_distance > params_.goal_tol_m &&
      shield_passive && mpc_solved &&
      window_full && moved < params_.move_eps_m;

    just_fired_ = false;
    if (frozen_now) {
      if (!candidate_) {
        candidate_ = true;
        candidate_since_ = now_s;
      } else if (!latched_ && (now_s - candidate_since_) >= params_.dwell_s) {
        latched_ = true;
        just_fired_ = true;
      }
    } else {
      candidate_ = false;
      latched_ = false;
    }
    last_seen_ = now_s;
  }

  /// True while the freeze is confirmed (dwell satisfied and still frozen).
  bool frozen() const {return latched_;}
  /// True on the single cycle the freeze is first confirmed -- for logging.
  bool just_fired() const {return just_fired_;}
  /// Seconds spent in the confirmed freeze, 0 when not frozen.
  double duration_s() const {return latched_ ? last_seen_ - candidate_since_ : 0.0;}

  void reset()
  {
    history_.clear();
    candidate_ = false;
    latched_ = false;
    just_fired_ = false;
  }

private:
  struct Sample
  {
    double t;
    double x;
    double y;
  };

  FreezeDetectorParams params_;
  std::deque<Sample> history_;
  bool candidate_{false};
  bool latched_{false};
  bool just_fired_{false};
  double candidate_since_{0.0};
  double last_seen_{0.0};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__FREEZE_DETECTOR_HPP_
