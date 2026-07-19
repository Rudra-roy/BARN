// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "barn_core/geometry.hpp"

namespace barn_classical
{
namespace
{

std::size_t nearest_breadcrumb(
  const std::vector<barn_core::Pose2D> & bc, const barn_core::Pose2D & pose)
{
  std::size_t best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < bc.size(); ++i) {
    const double d = std::hypot(bc[i].x - pose.x, bc[i].y - pose.y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

}  // namespace

double Recovery::widest_gap_heading(const barn_core::ScanView & scan) const
{
  if (!scan.valid()) {
    return 0.0;
  }
  double best_angle = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();
  // 15° half-window, 5° steps: fine enough to resolve a narrow corridor opening.
  constexpr double half_window = 15.0 * M_PI / 180.0;
  for (double angle = -M_PI; angle <= M_PI; angle += 5.0 * M_PI / 180.0) {
    const double clearance = barn_core::min_range_in_sector(
      scan, angle - half_window, angle + half_window);
    // Small forward bias so ties resolve toward straight-ahead rather than a
    // hard about-face.
    const double score = clearance - 0.15 * std::abs(angle);
    if (score > best_score) {
      best_score = score;
      best_angle = angle;
    }
  }
  return best_angle;
}

barn_core::VelocityCommand Recovery::reverse_command(const RecoveryContext & ctx) const
{
  // No usable breadcrumb: back straight out. The safety shield still guards the
  // motion, and a rear obstacle scales it down; max_reverse_distance/timeout
  // bound how long we try.
  const auto * bc = ctx.breadcrumb;
  if (bc == nullptr || bc->size() < 2) {
    return {-params_.reverse_speed, 0.0};
  }

  // Find the breadcrumb point ~reverse_lookahead behind the robot along the
  // traversed path (walk from the nearest crumb toward older samples).
  const std::size_t nearest = nearest_breadcrumb(*bc, ctx.pose);
  std::size_t target = nearest;
  double accumulated = 0.0;
  while (target > 0 && accumulated < params_.reverse_lookahead) {
    accumulated += std::hypot(
      (*bc)[target].x - (*bc)[target - 1].x, (*bc)[target].y - (*bc)[target - 1].y);
    --target;
  }
  const barn_core::Pose2D & goal = (*bc)[target];

  // Reverse pure pursuit: treat the robot's rear as a virtual forward heading.
  // The yaw rate of that virtual forward robot equals the real robot's yaw rate;
  // the real linear velocity is negative.
  const double virtual_heading = barn_core::wrap_angle(ctx.pose.yaw + M_PI);
  const double bearing = std::atan2(goal.y - ctx.pose.y, goal.x - ctx.pose.x);
  const double alpha = barn_core::wrap_angle(bearing - virtual_heading);
  const double lookahead = std::max(0.2, std::hypot(goal.x - ctx.pose.x, goal.y - ctx.pose.y));
  double w = 2.0 * params_.reverse_speed * std::sin(alpha) / lookahead;
  w = std::clamp(w, -params_.rotate_speed, params_.rotate_speed);
  return {-params_.reverse_speed, w};
}

bool Recovery::breadcrumb_exhausted(const RecoveryContext & ctx) const
{
  const auto * bc = ctx.breadcrumb;
  if (bc == nullptr || bc->size() < 2) {
    return false;  // nothing to consume; the distance/timeout bounds apply instead
  }
  const std::size_t nearest = nearest_breadcrumb(*bc, ctx.pose);
  return nearest == 0 &&
         std::hypot(bc->front().x - ctx.pose.x, bc->front().y - ctx.pose.y) < 0.15;
}

void Recovery::begin_episode(const RecoveryContext & ctx)
{
  state_elapsed_ = 0.0;
  blocked_elapsed_ = 0.0;
  // Escalate with repeated failures: attempt 1 just reverses-then-replans;
  // attempt 2+ also rotates toward the gap; attempt 3+ boosts the planner's
  // clearance weight on the follow-up replan.
  rotate_after_reverse_ = attempts_ >= 2;
  boost_after_ = attempts_ >= 3;

  if (ctx.clearance < ctx.rotation_radius) {
    // Too tight to rotate here — back out along the known-clear breadcrumb.
    reverse_start_pose_ = ctx.pose;
    state_ = RecoveryState::kReverseToClearance;
  } else if (rotate_after_reverse_) {
    target_yaw_ = barn_core::wrap_angle(ctx.pose.yaw + widest_gap_heading(ctx.scan));
    state_ = RecoveryState::kRotateToGap;
  } else {
    state_ = boost_after_ ? RecoveryState::kRequestReplanClearance : RecoveryState::kRequestReplan;
  }
}

void Recovery::trigger(const RecoveryContext & ctx)
{
  // A plain trigger is not a veto escape; trigger_veto_escape re-sets the flag.
  veto_escape_ = false;
  if (state_ == RecoveryState::kFailed) {
    return;
  }
  if (attempts_ >= params_.max_attempts) {
    state_ = RecoveryState::kFailed;
    state_elapsed_ = 0.0;
    return;
  }
  ++attempts_;
  begin_episode(ctx);
}

void Recovery::trigger_veto_escape(const RecoveryContext & ctx)
{
  trigger(ctx);
  veto_escape_ = (state_ != RecoveryState::kFailed && state_ != RecoveryState::kInactive);
}

barn_core::VelocityCommand Recovery::step(double dt, const RecoveryContext & ctx)
{
  state_elapsed_ += std::max(0.0, dt);

  // Veto-escape episodes exist only to clear the safety shield. The moment it
  // stops vetoing (after a minimal maneuver so we do not exit on a flicker), the
  // escape has succeeded: replan from the new pose rather than running on.
  if (veto_escape_ && !ctx.veto_active && state_elapsed_ >= params_.veto_clear_min_rotate &&
    state_ != RecoveryState::kInactive && state_ != RecoveryState::kFailed &&
    state_ != RecoveryState::kRequestReplan && state_ != RecoveryState::kRequestReplanClearance)
  {
    state_ = RecoveryState::kRequestReplan;
    state_elapsed_ = 0.0;
    blocked_elapsed_ = 0.0;
    return {0.0, 0.0};
  }

  // Shield-blocked bail-out: a motion state whose command the shield is fully
  // (emergency) vetoing is making zero progress. Past blocked_timeout, replan
  // instead of burning the whole maneuver timeout frozen in place.
  const bool motion_state = state_ == RecoveryState::kReverseToClearance ||
    state_ == RecoveryState::kRotateToGap;
  if (motion_state && ctx.veto_active) {
    blocked_elapsed_ += std::max(0.0, dt);
    if (blocked_elapsed_ >= params_.blocked_timeout) {
      state_ = RecoveryState::kRequestReplanClearance;
      state_elapsed_ = 0.0;
      blocked_elapsed_ = 0.0;
      return {0.0, 0.0};
    }
  } else {
    blocked_elapsed_ = 0.0;
  }

  switch (state_) {
    case RecoveryState::kInactive:
      return {0.0, 0.0};

    // Latched failure would otherwise stop the robot for the rest of the trial.
    // Pause briefly, then clear the budget and let control re-detect the fault.
    case RecoveryState::kFailed:
      if (state_elapsed_ >= params_.failed_reset_timeout) {
        state_ = RecoveryState::kInactive;
        attempts_ = 0;
        state_elapsed_ = 0.0;
        veto_escape_ = false;
      }
      return {0.0, 0.0};

    // Reverse along the breadcrumb until there is room to turn.
    case RecoveryState::kReverseToClearance: {
        if (ctx.clearance >= ctx.rotation_radius) {
          if (rotate_after_reverse_) {
            target_yaw_ = barn_core::wrap_angle(ctx.pose.yaw + widest_gap_heading(ctx.scan));
            state_ = RecoveryState::kRotateToGap;
          } else {
            state_ = boost_after_ ? RecoveryState::kRequestReplanClearance :
              RecoveryState::kRequestReplan;
          }
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
        const double reversed = std::hypot(
          ctx.pose.x - reverse_start_pose_.x, ctx.pose.y - reverse_start_pose_.y);
        if (reversed >= params_.max_reverse_distance ||
          state_elapsed_ >= params_.reverse_timeout || breadcrumb_exhausted(ctx))
        {
          // Could not open enough room by reversing — replan from here with a
          // clearance boost so the planner routes wider next time.
          state_ = RecoveryState::kRequestReplanClearance;
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
        return reverse_command(ctx);
      }

    // Rotate toward the widest gap; clearance already permits a full sweep.
    case RecoveryState::kRotateToGap: {
        const double error = barn_core::wrap_angle(target_yaw_ - ctx.pose.yaw);
        if (std::abs(error) <= params_.heading_tolerance ||
          state_elapsed_ >= params_.rotate_timeout)
        {
          state_ = boost_after_ ? RecoveryState::kRequestReplanClearance :
            RecoveryState::kRequestReplan;
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
        return {0.0, std::clamp(
            std::copysign(params_.rotate_speed, error), -params_.rotate_speed, params_.rotate_speed)};
      }

    case RecoveryState::kRequestReplan:
    case RecoveryState::kRequestReplanClearance:
      if (state_elapsed_ >= params_.replan_timeout) {
        state_ = RecoveryState::kInactive;
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

void Recovery::notify_progress()
{
  // Only meaningful while not mid-episode; the caller invokes this from the
  // normal-navigation path where recovery is inactive.
  if (state_ == RecoveryState::kInactive) {
    attempts_ = 0;
  }
}

void Recovery::reset()
{
  state_ = RecoveryState::kInactive;
  state_elapsed_ = 0.0;
  blocked_elapsed_ = 0.0;
  target_yaw_ = 0.0;
  attempts_ = 0;
  veto_escape_ = false;
  rotate_after_reverse_ = false;
  boost_after_ = false;
}

}  // namespace barn_classical
