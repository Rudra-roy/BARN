// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M10). Recovery state machine for deadlock / local-minimum /
// oscillation situations (back up, rotate, clear costmap, re-seed the planner).

#ifndef BARN_CLASSICAL__RECOVERY_HPP_
#define BARN_CLASSICAL__RECOVERY_HPP_

#include "barn_core/types.hpp"

namespace barn_classical
{

enum class RecoveryState
{
  kInactive,
  kBackUp,
  kRotateInPlace,
  kClearAndReplan
};

class Recovery
{
public:
  /// Advance the recovery FSM by `dt` seconds; returns the command to execute
  /// while recovering. STUB: stays inactive and commands zero.
  barn_core::VelocityCommand step(double dt);

  RecoveryState state() const { return state_; }
  bool active() const { return state_ != RecoveryState::kInactive; }

private:
  RecoveryState state_{RecoveryState::kInactive};
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__RECOVERY_HPP_
