// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_safety/swept_footprint_shield.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace barn_safety
{

bool SweptFootprintShield::safe_at_scale(
  const barn_core::VelocityCommand & desired, double scale,
  const std::vector<ObstaclePoint> & obstacles, double & minimum_clearance,
  std::vector<barn_core::Pose2D> * envelope) const
{
  const double v = desired.v * scale;
  const double w = desired.w * scale;
  const double stopping_time = std::abs(v) / std::max(0.1, params_.braking_decel) + params_.latency_s;
  const double horizon = std::max(params_.horizon_s, stopping_time);
  // Hard-veto footprint: physical body plus ONLY the small emergency buffer.
  //
  // footprint_margin is deliberately NOT in the hard-collision box. Anticipatory
  // clearance is already provided by sweeping this box along the full stopping
  // envelope (horizon covers braking + latency), so an extra static buffer only
  // adds conservatism at rest. safety_node discards returns inside half+1cm, so
  // an obstacle in the shell between the body and half+footprint_margin+
  // emergency (~5-6 cm, routine in a tight BARN pinch) used to hard-veto EVERY
  // scale down to zero — including an in-place rotation or a creep AWAY from the
  // obstacle. The robot then froze in recovery, commanded to move but clamped to
  // zero. With the tighter box, the scale search can return a motion that
  // escapes the pinch, while a genuinely imminent collision (obstacle within
  // emergency_margin of the body along the swept path) is still vetoed.
  const double hx = params_.half_length + params_.emergency_margin;
  const double hy = params_.half_width + params_.emergency_margin;
  barn_core::Pose2D pose;
  minimum_clearance = std::numeric_limits<double>::infinity();
  if (envelope != nullptr) {
    envelope->clear();
    envelope->push_back(pose);
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / params_.integration_dt)));
  for (int step = 0; step <= steps; ++step) {
    const double ct = std::cos(pose.yaw);
    const double st = std::sin(pose.yaw);
    for (const auto & point : obstacles) {
      const double dx = point.x - pose.x;
      const double dy = point.y - pose.y;
      const double local_x = ct * dx + st * dy;
      const double local_y = -st * dx + ct * dy;
      const double outside_x = std::max(0.0, std::abs(local_x) - hx);
      const double outside_y = std::max(0.0, std::abs(local_y) - hy);
      const double clearance = std::hypot(outside_x, outside_y);
      minimum_clearance = std::min(minimum_clearance, clearance);
      if (std::abs(local_x) <= hx && std::abs(local_y) <= hy) {
        return false;
      }
    }
    if (step == steps) {
      break;
    }
    pose.x += params_.integration_dt * v * std::cos(pose.yaw);
    pose.y += params_.integration_dt * v * std::sin(pose.yaw);
    pose.yaw = std::atan2(
      std::sin(pose.yaw + params_.integration_dt * w),
      std::cos(pose.yaw + params_.integration_dt * w));
    if (envelope != nullptr) {
      envelope->push_back(pose);
    }
  }
  return true;
}

ShieldResult SweptFootprintShield::apply(
  const barn_core::VelocityCommand & desired,
  const std::vector<ObstaclePoint> & obstacles,
  bool allow_escape) const
{
  ShieldResult result;
  if (obstacles.empty()) {
    result.command = desired;
    result.scale = 1.0;
    result.minimum_clearance = std::numeric_limits<double>::infinity();
    result.reason = "clear_no_returns";
    return result;
  }

  if (std::abs(desired.v) < 1e-6 && std::abs(desired.w) < 1e-6) {
    result.command = desired;
    result.scale = 1.0;
    result.reason = "stationary";
    (void)safe_at_scale(desired, 1.0, obstacles, result.minimum_clearance, &result.envelope);
    return result;
  }

  const double step = std::clamp(params_.scale_step, 0.01, 0.25);
  for (double scale = 1.0; scale >= -1e-9; scale -= step) {
    const double candidate_scale = std::max(0.0, scale);
    std::vector<barn_core::Pose2D> envelope;
    double clearance = 0.0;
    if (!safe_at_scale(desired, candidate_scale, obstacles, clearance, &envelope)) {
      continue;
    }
    if (candidate_scale > 1e-6) {
      result.scale = candidate_scale;
      result.command = {desired.v * candidate_scale, desired.w * candidate_scale};
      result.minimum_clearance = clearance;
      result.envelope = std::move(envelope);
      result.reason = candidate_scale > 0.999 ? "clear" : "braking_distance_clamp";
      return result;
    }
    // Only a full stop is safe. Scaling can never rescue this, because scaling
    // does not change DIRECTION: if the desired heading is blocked then every
    // multiple of it is blocked too, even when rotating in place is completely
    // free. Measured on world 216: the robot sat for 75 s with an obstacle 1 mm
    // outside the box, stopped, while a pure rotation was available the whole
    // time. Offer an escape before reporting a stop.
    ShieldResult stop;
    stop.scale = 0.0;
    stop.minimum_clearance = clearance;
    stop.envelope = std::move(envelope);
    stop.reason = "emergency_veto";
    if (!allow_escape) {
      return stop;
    }
    const auto escape = find_escape(desired, obstacles);
    return escape.reason == "emergency_escape" ? escape : stop;
  }
  // Every scale failed. That can only happen when the box at the CURRENT pose
  // is already occupied -- as the scale goes to zero the swept envelope
  // collapses onto that box -- so no scaling of `desired` can ever help.
  // Freezing here is a trap, not a safe state: see find_escape().
  if (!allow_escape) {
    ShieldResult trapped;
    trapped.scale = 0.0;
    trapped.minimum_clearance = signed_clearance(barn_core::Pose2D{}, obstacles);
    trapped.reason = "emergency_trapped";
    return trapped;
  }
  return find_escape(desired, obstacles);
}

double SweptFootprintShield::signed_clearance(
  const barn_core::Pose2D & pose, const std::vector<ObstaclePoint> & obstacles) const
{
  const double hx = params_.half_length + params_.emergency_margin;
  const double hy = params_.half_width + params_.emergency_margin;
  const double ct = std::cos(pose.yaw);
  const double st = std::sin(pose.yaw);
  double worst = std::numeric_limits<double>::infinity();
  for (const auto & point : obstacles) {
    const double dx = point.x - pose.x;
    const double dy = point.y - pose.y;
    const double local_x = ct * dx + st * dy;
    const double local_y = -st * dx + ct * dy;
    const double over_x = std::abs(local_x) - hx;
    const double over_y = std::abs(local_y) - hy;
    // Outside on either axis -> positive Euclidean distance to the box.
    // Inside on both -> negative depth, the smaller intrusion being the one
    // that has to be undone to get out.
    const double clearance = (over_x > 0.0 || over_y > 0.0)
      ? std::hypot(std::max(0.0, over_x), std::max(0.0, over_y))
      : std::max(over_x, over_y);
    worst = std::min(worst, clearance);
  }
  return worst;
}

ShieldResult SweptFootprintShield::find_escape(
  const barn_core::VelocityCommand & desired,
  const std::vector<ObstaclePoint> & obstacles) const
{
  ShieldResult result;
  result.reason = "emergency_veto";
  result.scale = 0.0;
  const barn_core::Pose2D origin;
  const double start = signed_clearance(origin, obstacles);
  result.minimum_clearance = start;

  if (!params_.escape_enable) {
    return result;
  }

  // An obstacle already inside the box is strictly worse than being blocked in
  // front of one, and it is the state that strands a trial, so it gets its own
  // name -- a log filter and a human can then tell them apart.
  if (start < 0.0) {
    result.reason = "emergency_trapped";
  }

  const double v = std::abs(params_.escape_speed);
  const double w = std::abs(params_.escape_yaw_rate);
  // Rotations first: they sweep the least ground. Reverse before forward, since
  // the robot arrived from behind and that space was clear a moment ago.
  const std::vector<barn_core::VelocityCommand> candidates{
    {0.0, w}, {0.0, -w}, {-v, 0.0}, {-v, w}, {-v, -w}, {v, 0.0}, {v, w}, {v, -w}};

  const double dt = std::max(1e-3, params_.integration_dt);
  const int steps = std::max(1, static_cast<int>(std::ceil(params_.escape_horizon_s / dt)));

  double best_final = start;
  for (const auto & candidate : candidates) {
    barn_core::Pose2D pose;
    std::vector<barn_core::Pose2D> envelope{pose};
    double worst_along = std::numeric_limits<double>::infinity();
    double final_clearance = start;
    for (int step = 0; step < steps; ++step) {
      pose.x += dt * candidate.v * std::cos(pose.yaw);
      pose.y += dt * candidate.v * std::sin(pose.yaw);
      pose.yaw = std::atan2(
        std::sin(pose.yaw + dt * candidate.w), std::cos(pose.yaw + dt * candidate.w));
      envelope.push_back(pose);
      final_clearance = signed_clearance(pose, obstacles);
      worst_along = std::min(worst_along, final_clearance);
    }
    // Never allow a motion that digs deeper at any point on the way, even if it
    // ends better -- that is how an "escape" clips a second obstacle. The
    // min-over-obstacles clearance also means turning into anything else is
    // rejected automatically.
    if (worst_along < start - 1e-9 || final_clearance <= best_final + 1e-6) {
      continue;
    }
    best_final = final_clearance;
    result.command = candidate;
    result.minimum_clearance = final_clearance;
    result.envelope = std::move(envelope);
    result.reason = "emergency_escape";
  }
  (void)desired;
  return result;
}

}  // namespace barn_safety
