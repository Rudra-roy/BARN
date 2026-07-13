// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "barn_core/geometry.hpp"

namespace barn_classical
{
namespace
{

double rear_clearance(const barn_core::ScanView & scan)
{
  return std::min(
    static_cast<double>(barn_core::min_range_in_sector(scan, M_PI - 0.45, M_PI)),
    static_cast<double>(barn_core::min_range_in_sector(scan, -M_PI, -M_PI + 0.45)));
}

}  // namespace

double Recovery::widest_gap_heading(const barn_core::ScanView & scan) const
{
  if (!scan.valid()) {
    return 0.0;
  }
  double best_angle = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  constexpr double half_window = 20.0 * M_PI / 180.0;
  for (double angle = -M_PI; angle <= M_PI; angle += 10.0 * M_PI / 180.0) {
    const double clearance = barn_core::min_range_in_sector(
      scan, angle - half_window, angle + half_window);
    // Prefer a large gap, with a mild preference for forward-facing options.
    const double score = clearance - 0.15 * std::abs(angle);
    if (score > best_score) {
      best_score = score;
      best_angle = angle;
    }
  }
  return best_angle;
}

void Recovery::trigger(const barn_core::ScanView & scan)
{
  if (state_ == RecoveryState::kFailed) {
    return;
  }
  if (attempts_ >= params_.max_attempts) {
    state_ = RecoveryState::kFailed;
    return;
  }
  ++attempts_;
  state_ = RecoveryState::kStop;
  state_elapsed_ = 0.0;
  target_yaw_ = widest_gap_heading(scan);
}

barn_core::VelocityCommand Recovery::step(
  double dt, double current_yaw, double distance_backed,
  const barn_core::ScanView & scan)
{
  state_elapsed_ += std::max(0.0, dt);
  switch (state_) {
    case RecoveryState::kInactive:
    case RecoveryState::kFailed:
    case RecoveryState::kRequestReplan:
      return {0.0, 0.0};
    case RecoveryState::kStop:
      if (state_elapsed_ >= params_.stop_duration) {
        target_yaw_ = barn_core::wrap_angle(current_yaw + target_yaw_);
        state_ = RecoveryState::kRotateToGap;
        state_elapsed_ = 0.0;
      }
      return {0.0, 0.0};
    case RecoveryState::kRotateToGap: {
        const double error = barn_core::wrap_angle(target_yaw_ - current_yaw);
        if (std::abs(error) <= params_.heading_tolerance) {
          state_ = RecoveryState::kRequestReplan;
          return {0.0, 0.0};
        }
        if (state_elapsed_ >= params_.rotate_timeout) {
          const double rear = rear_clearance(scan);
          if (rear >= params_.rear_clearance) {
            state_ = RecoveryState::kBackUp;
            state_elapsed_ = 0.0;
          } else {
            state_ = RecoveryState::kRequestReplan;
          }
          return {0.0, 0.0};
        }
        return {0.0, std::copysign(params_.rotate_speed, error)};
      }
    case RecoveryState::kBackUp:
      if (distance_backed >= params_.backup_distance) {
        state_ = RecoveryState::kRequestReplan;
        return {0.0, 0.0};
      }
      if (rear_clearance(scan) < params_.rear_clearance)
      {
        state_ = RecoveryState::kRequestReplan;
        return {0.0, 0.0};
      }
      return {-params_.backup_speed, 0.0};
  }
  return {0.0, 0.0};
}

void Recovery::finish_replan()
{
  if (state_ == RecoveryState::kRequestReplan) {
    state_ = RecoveryState::kInactive;
    state_elapsed_ = 0.0;
  }
}

void Recovery::reset()
{
  state_ = RecoveryState::kInactive;
  state_elapsed_ = 0.0;
  target_yaw_ = 0.0;
  attempts_ = 0;
}

}  // namespace barn_classical
