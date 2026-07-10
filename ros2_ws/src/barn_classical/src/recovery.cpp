// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/recovery.hpp"

namespace barn_classical
{

barn_core::VelocityCommand Recovery::step(double)
{
  // STUB (M10): back-up / rotate / clear-and-replan FSM goes here.
  state_ = RecoveryState::kInactive;
  return barn_core::VelocityCommand{0.0, 0.0};
}

}  // namespace barn_classical
