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
  // Finer 15° half-window and 5° steps (was 20°/10°) for better gap detection
  // at tight corners where the corridor opening may be narrow.
  constexpr double half_window = 15.0 * M_PI / 180.0;
  for (double angle = -M_PI; angle <= M_PI; angle += 5.0 * M_PI / 180.0) {
    const double clearance = barn_core::min_range_in_sector(
      scan, angle - half_window, angle + half_window);
    // Reduced forward bias (0.15) so it doesn't force driving straight into obstacles
    // and can properly pick a clear side gap.
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
  state_elapsed_ = 0.0;
  
  // Find closest obstacle direction for opposite-rotation phases
  double closest_angle = 0.0;
  double min_range = std::numeric_limits<double>::infinity();
  if (scan.valid()) {
    for (std::size_t i = 0; i < scan.count; ++i) {
      if (scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
        if (scan.ranges[i] < min_range) {
          min_range = scan.ranges[i];
          closest_angle = scan.angle_min + i * scan.angle_increment;
        }
      }
    }
  }
  // Rotate AWAY from the obstacle
  target_yaw_ = closest_angle >= 0.0 ? 1.0 : -1.0;

  // Progressive escalation: each successive trigger starts at a later phase
  switch (attempts_) {
    case 1:
      // First attempt: just rotate opposite
      state_ = RecoveryState::kRotateOpposite;
      break;
    case 2:
      // Second attempt: back up first, then rotate opposite
      state_ = RecoveryState::kBackUp;
      break;
    case 3:
      // Third attempt: rotate to widest gap
      state_ = RecoveryState::kRotateToGap;
      target_yaw_ = barn_core::wrap_angle(0.0 + widest_gap_heading(scan));
      break;
    case 4:
      // Fourth attempt: back up further, then try gap
      state_ = RecoveryState::kBackUp2;
      break;
    default:
      // Last resort: 1m backup with clearance boost
      state_ = RecoveryState::kBackUp1m;
      break;
  }
}

void Recovery::trigger_veto_escape(const barn_core::ScanView & scan)
{
  trigger(scan);
}

barn_core::VelocityCommand Recovery::step(
  double dt, double current_yaw, double distance_backed,
  const barn_core::ScanView & scan, bool veto_active)
{
  state_elapsed_ += std::max(0.0, dt);
  switch (state_) {
    case RecoveryState::kInactive:
    case RecoveryState::kFailed:
      return {0.0, 0.0};
      
    // Phase 1: Rotate away from nearest obstacle for the full timeout.
    case RecoveryState::kRotateOpposite:
      if (state_elapsed_ >= params_.rotate_timeout) {
        state_ = RecoveryState::kRequestReplan;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      return {0.0, target_yaw_ * params_.rotate_speed};

    // Phase 2: Back up a bit. If physically blocked behind, skip to phase 3.
    case RecoveryState::kBackUp:
      if (rear_clearance(scan) < params_.rear_clearance && state_elapsed_ > 0.2) {
        state_ = RecoveryState::kRotateOpposite2;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      if (distance_backed >= params_.backup_distance) {
        state_ = RecoveryState::kRotateOpposite2;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      return {-params_.backup_speed, 0.0};

    // Phase 3: Rotate opposite again with the space gained from backing up.
    case RecoveryState::kRotateOpposite2:
      if (state_elapsed_ >= params_.rotate_timeout) {
        state_ = RecoveryState::kRequestReplan;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      return {0.0, target_yaw_ * params_.rotate_speed};

    // Phase 4: Rotate towards the widest LiDAR gap.
    case RecoveryState::kRotateToGap: {
        const double error = barn_core::wrap_angle(target_yaw_ - current_yaw);
        if (std::abs(error) <= params_.heading_tolerance || state_elapsed_ >= params_.rotate_timeout) {
          state_ = RecoveryState::kRequestReplan;
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
        return {0.0, std::copysign(params_.rotate_speed, error)};
      }

    // Phase 5: Back up further. If physically blocked, skip to phase 6.
    case RecoveryState::kBackUp2:
      if (rear_clearance(scan) < params_.rear_clearance && state_elapsed_ > 0.2) {
        state_ = RecoveryState::kBackUp1m;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      if (distance_backed >= params_.backup_distance * 2.0) {
        // Try the gap rotation again with more space
        state_ = RecoveryState::kRotateToGap;
        state_elapsed_ = 0.0;
        target_yaw_ = barn_core::wrap_angle(current_yaw + widest_gap_heading(scan));
        return {0.0, 0.0};
      }
      return {-params_.backup_speed, 0.0};

    // Phase 6: Last resort - back up 1m and replan with increased clearance.
    case RecoveryState::kBackUp1m:
      if (rear_clearance(scan) < params_.rear_clearance && state_elapsed_ > 0.2) {
        state_ = RecoveryState::kRequestReplanClearance;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      if (distance_backed >= 1.0) {
        state_ = RecoveryState::kRequestReplanClearance;
        state_elapsed_ = 0.0;
        return {0.0, 0.0};
      }
      return {-params_.backup_speed, 0.0};

    case RecoveryState::kRequestReplan:
    case RecoveryState::kRequestReplanClearance:
      if (state_elapsed_ >= params_.replan_timeout) {
        state_ = RecoveryState::kInactive;
        // Don't re-trigger here; let control_step detect MPC failure again
      }
      return {0.0, 0.0};
  }
  return {0.0, 0.0};
}

void Recovery::finish_replan()
{
  if (state_ == RecoveryState::kRequestReplan || state_ == RecoveryState::kRequestReplanClearance) {
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
