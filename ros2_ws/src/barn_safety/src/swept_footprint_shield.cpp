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
  const double hx = params_.half_length + params_.footprint_margin + params_.emergency_margin;
  const double hy = params_.half_width + params_.footprint_margin + params_.emergency_margin;
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
  const std::vector<ObstaclePoint> & obstacles) const
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
    if (safe_at_scale(desired, candidate_scale, obstacles, clearance, &envelope)) {
      result.scale = candidate_scale;
      result.command = {desired.v * candidate_scale, desired.w * candidate_scale};
      result.minimum_clearance = clearance;
      result.envelope = std::move(envelope);
      result.reason = candidate_scale > 0.999 ? "clear" :
        (candidate_scale > 1e-6 ? "braking_distance_clamp" : "emergency_veto");
      return result;
    }
  }
  result.reason = "emergency_veto";
  return result;
}

}  // namespace barn_safety
