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

  // Pivot-First Alignment:
  // If the robot's rear is misaligned with the target breadcrumb (|alpha| > 0.45 rad ≈ 26°),
  // prioritize in-place pivoting of the rear toward the breadcrumb before translating backward.
  // This prevents the rear bumper from swinging diagonally into obstacles behind or beside the robot.
  double v_cmd = -params_.reverse_speed;
  double w_cmd = 0.0;

  if (std::abs(alpha) > 0.45) {
    // Pure in-place pivot to align rear with trail
    v_cmd = 0.0;
    w_cmd = std::clamp(2.0 * std::sin(alpha), -params_.rotate_speed, params_.rotate_speed);
  } else {
    // Smooth blended reverse pure pursuit with cosine scaling
    v_cmd = -params_.reverse_speed * std::cos(alpha);
    w_cmd = 2.0 * params_.reverse_speed * std::sin(alpha) / lookahead;
    w_cmd = std::clamp(w_cmd, -params_.rotate_speed, params_.rotate_speed);
  }

  return {v_cmd, w_cmd};
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

  // Always back out along the known-clear breadcrumb trail first to escape obstacle pockets.
  if (!breadcrumb_exhausted(ctx)) {
    reverse_start_pose_ = ctx.pose;
    state_ = RecoveryState::kReverseToClearance;
  } else if (ctx.clearance >= ctx.rotation_radius && rotate_after_reverse_) {
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

  // Veto-escape episodes only exit early for rotation states once veto clears.
  // Reverse states must continue reversing to open clearance (kReverseToClearance).
  if (veto_escape_ && !ctx.veto_active && state_elapsed_ >= params_.veto_clear_min_rotate &&
    state_ == RecoveryState::kRotateToGap)
  {
    state_ = RecoveryState::kRequestReplan;
    state_elapsed_ = 0.0;
    blocked_elapsed_ = 0.0;
    return {0.0, 0.0};
  }

  // Shield-blocked bail-out: for rotation states, if the shield is vetoing the spin,
  // bail out to replan instead of burning the whole timeout.
  // (Do NOT abort kReverseToClearance, which is the primary escape motion backing away from front vetoes).
  const bool motion_state = state_ == RecoveryState::kRotateToGap;
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
        const double reversed = std::hypot(
          ctx.pose.x - reverse_start_pose_.x, ctx.pose.y - reverse_start_pose_.y);
        // Bounded Reverse Exit:
        // Enforce at least 0.40m reverse to avoid 5cm twitch loops, but allow exiting
        // to replan once the front is clear (!ctx.veto_active) to avoid exhausting
        // long tunnels. Only rotate if full open clearance is achieved.
        if (reversed >= 0.40 && (ctx.clearance >= ctx.rotation_radius || !ctx.veto_active)) {
          if (rotate_after_reverse_ && ctx.clearance >= ctx.rotation_radius) {
            target_yaw_ = barn_core::wrap_angle(ctx.pose.yaw + widest_gap_heading(ctx.scan));
            state_ = RecoveryState::kRotateToGap;
          } else {
            state_ = boost_after_ ? RecoveryState::kRequestReplanClearance :
              RecoveryState::kRequestReplan;
          }
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
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
        // Strict physical clearance gating: never rotate in-place if clearance is below rotation_radius.
        if (ctx.clearance < ctx.rotation_radius) {
          if (!breadcrumb_exhausted(ctx)) {
            reverse_start_pose_ = ctx.pose;
            state_ = RecoveryState::kReverseToClearance;
          } else {
            state_ = boost_after_ ? RecoveryState::kRequestReplanClearance : RecoveryState::kRequestReplan;
          }
          state_elapsed_ = 0.0;
          return {0.0, 0.0};
        }
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
  // Refund ONE attempt, not the whole budget.
  //
  // This used to zero attempts_. The caller invoked it after 0.18 m of travel,
  // which the robot covers in ~0.2 s the moment recovery hands back, so every
  // episode restarted at attempt 1: rotate_after_reverse_ (attempts >= 2) and
  // boost_after_ (>= 3) almost never engaged, and kFailed (5) never latched.
  // The escalation ladder existed and was unreachable.
  //
  // The consequence is a closed loop, observed directly: recovery finishes ->
  // replans -> A* is deterministic and returns the SAME path through the SAME
  // pinch -> the robot re-enters the same state -> recovery again as "attempt 1".
  // Decrementing makes the second episode genuinely different from the first,
  // which is the only thing that breaks the loop.
  if (state_ == RecoveryState::kInactive && attempts_ > 0) {
    --attempts_;
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
