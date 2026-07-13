// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__RECOVERY_HPP_
#define BARN_CLASSICAL__RECOVERY_HPP_

#include "barn_core/scan.hpp"
#include "barn_core/types.hpp"

namespace barn_classical
{

enum class RecoveryState
{
  kInactive,
  kStop,
  kRotateToGap,
  kBackUp,
  kRequestReplan,
  kFailed
};

struct RecoveryParams
{
  double stop_duration{0.30};
  // Raised from 1.5 s: at 0.9 rad/s a 180° turn needs ~3.5 s; 2.5 s covers
  // most escapes without being so long that the watchdog fires first.
  double rotate_timeout{2.5};
  // Raised from 0.45 rad/s: the Jackal is rated for ~2.0 rad/s; 0.9 rad/s
  // is fast enough to turn out of a blind alley in ~2 s.
  double rotate_speed{0.90};
  double heading_tolerance{0.10};
  double backup_distance{0.30};
  // Raised from 0.15 m/s: faster backup clears the obstacle sooner.
  double backup_speed{0.30};
  double rear_clearance{0.55};
  int max_attempts{3};
};

class Recovery
{
public:
  explicit Recovery(const RecoveryParams & params = {}) : params_(params) {}

  void trigger(const barn_core::ScanView & scan);
  barn_core::VelocityCommand step(
    double dt, double current_yaw, double distance_backed,
    const barn_core::ScanView & scan);
  /// Mark the requested replan complete without resetting the bounded-attempt count.
  void finish_replan();
  void reset();

  RecoveryState state() const { return state_; }
  bool active() const { return state_ != RecoveryState::kInactive; }
  bool request_replan() const { return state_ == RecoveryState::kRequestReplan; }
  int attempts() const { return attempts_; }
  double target_yaw() const { return target_yaw_; }

private:
  double widest_gap_heading(const barn_core::ScanView & scan) const;

  RecoveryParams params_;
  RecoveryState state_{RecoveryState::kInactive};
  double state_elapsed_{0.0};
  double target_yaw_{0.0};
  int attempts_{0};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__RECOVERY_HPP_
