// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__RECOVERY_HPP_
#define BARN_CLASSICAL__RECOVERY_HPP_

#include <vector>

#include "barn_core/scan.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

// Backtracking recovery. The escape strategy is: if the robot is pinched too
// tightly to rotate, REVERSE along the breadcrumb it just drove (which is
// therefore known-clear) until it reaches space wide enough to turn, then
// (optionally) rotate toward the widest gap, then request a fresh plan from the
// open pose. This mirrors how tight-aisle industrial AMRs unstick themselves,
// and it never fights the safety shield by spinning where it cannot.
enum class RecoveryState
{
  kInactive,
  kReverseToClearance,       // reverse along the breadcrumb until rotation fits
  kRotateToGap,              // rotate in place toward the widest gap (space permitting)
  kRequestReplan,            // hand off: clear path and replan from here
  kRequestReplanClearance,   // replan with a boosted clearance weight
  kFailed                    // temporary give-up; self-clears and retries
};

inline const char * to_string(RecoveryState state)
{
  switch (state) {
    case RecoveryState::kInactive: return "Inactive";
    case RecoveryState::kReverseToClearance: return "ReverseToClearance";
    case RecoveryState::kRotateToGap: return "RotateToGap";
    case RecoveryState::kRequestReplan: return "RequestReplan";
    case RecoveryState::kRequestReplanClearance: return "RequestReplanClearance";
    case RecoveryState::kFailed: return "Failed";
    default: return "Unknown";
  }
}

struct RecoveryParams
{
  // Rotation (kRotateToGap).
  double rotate_speed{0.90};
  double rotate_timeout{2.5};
  double heading_tolerance{0.10};

  // Reverse-to-clearance.
  double reverse_speed{0.35};
  double reverse_lookahead{0.50};    // pure-pursuit look-back distance along the breadcrumb
  double max_reverse_distance{1.50}; // give up reversing past this displacement
  double reverse_timeout{6.0};

  // Episode bookkeeping.
  int max_attempts{5};
  double replan_timeout{1.0};        // wait for the planner before falling back to inactive
  double failed_reset_timeout{3.0};  // kFailed self-clears and retries after this
  double veto_clear_min_rotate{0.20};
  // If the shield fully (emergency) vetoes an escape command this long, the
  // maneuver is frozen — bail to a replan instead of grinding out the timeout.
  double blocked_timeout{0.70};
};

// Per-tick inputs the node feeds recovery. Recovery stays free of ROS and of
// the occupancy/distance-field types: the node resolves clearance and supplies
// the traversed breadcrumb.
struct RecoveryContext
{
  barn_core::Pose2D pose{};                                  // current pose, planning frame
  double clearance{0.0};                                     // distance-field clearance at pose [m]
  double rotation_radius{0.40};                              // clearance needed to rotate in place [m]
  bool veto_active{false};                                   // shield emergency veto this tick
  barn_core::ScanView scan{};                                // for widest-gap selection
  const std::vector<barn_core::Pose2D> * breadcrumb{nullptr};// traversed poses, oldest -> newest
};

class Recovery
{
public:
  explicit Recovery(const RecoveryParams & params = {}) : params_(params) {}

  void trigger(const RecoveryContext & ctx);
  void trigger_veto_escape(const RecoveryContext & ctx);
  barn_core::VelocityCommand step(double dt, const RecoveryContext & ctx);
  /// Mark the requested replan complete without resetting the bounded-attempt count.
  void finish_replan();
  /// Clear the attempt budget after the robot has made real forward progress.
  void notify_progress();
  void reset();

  RecoveryState state() const { return state_; }
  bool active() const { return state_ != RecoveryState::kInactive; }
  bool request_replan() const
  {
    return state_ == RecoveryState::kRequestReplan ||
           state_ == RecoveryState::kRequestReplanClearance;
  }
  bool is_clearance_replan() const { return state_ == RecoveryState::kRequestReplanClearance; }
  int attempts() const { return attempts_; }
  double target_yaw() const { return target_yaw_; }

private:
  double widest_gap_heading(const barn_core::ScanView & scan) const;
  barn_core::VelocityCommand reverse_command(const RecoveryContext & ctx) const;
  bool breadcrumb_exhausted(const RecoveryContext & ctx) const;
  void begin_episode(const RecoveryContext & ctx);

  RecoveryParams params_;
  RecoveryState state_{RecoveryState::kInactive};
  double state_elapsed_{0.0};
  // Time the shield has continuously emergency-vetoed the current motion state.
  double blocked_elapsed_{0.0};
  double target_yaw_{0.0};
  int attempts_{0};
  // True while the episode was launched by a safety-shield veto.
  bool veto_escape_{false};
  // Escalation flags set per episode from the attempt count.
  bool rotate_after_reverse_{false};
  bool boost_after_{false};
  barn_core::Pose2D reverse_start_pose_{};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__RECOVERY_HPP_
