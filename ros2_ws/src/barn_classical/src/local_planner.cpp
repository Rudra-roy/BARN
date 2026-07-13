// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "barn_classical/path_validator.hpp"
#include "barn_core/distance_field.hpp"
#include "barn_core/geometry.hpp"

namespace barn_classical
{
namespace
{

std::size_t nearest_path_index(const Path2D & path, const barn_core::Pose2D & pose)
{
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

void assign_tangent_yaws(Path2D & path)
{
  if (path.size() < 2) {
    return;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.back().yaw = path[path.size() - 2].yaw;
}

}  // namespace

LocalTrajectory LocalPlanner::plan(
  const Path2D & global_path, const barn_core::Pose2D & pose,
  const barn_core::OccupancyGrid2D & grid,
  const barn_core::DistanceField2D * precomputed_distance_field) const
{
  if (global_path.empty() || grid.width() == 0) {
    return {};
  }

  const std::size_t start = nearest_path_index(global_path, pose);
  Path2D raw;
  raw.reserve(global_path.size() - start + 1);
  raw.push_back(pose);
  double length = 0.0;
  for (std::size_t i = std::max<std::size_t>(start, 1); i < global_path.size(); ++i) {
    const auto & previous = raw.back();
    const double segment = std::hypot(
      global_path[i].x - previous.x, global_path[i].y - previous.y);
    if (segment < 1e-4) {
      continue;
    }
    if (length + segment > params_.horizon_m && raw.size() > 1) {
      break;
    }
    raw.push_back(global_path[i]);
    length += segment;
  }
  if (raw.size() < 2) {
    return {};
  }
  assign_tangent_yaws(raw);

  barn_core::DistanceField2D owned_distance_field;
  if (precomputed_distance_field == nullptr || !precomputed_distance_field->valid()) {
    owned_distance_field.rebuild(grid);
    precomputed_distance_field = &owned_distance_field;
  }
  const auto & distance_field = *precomputed_distance_field;
  Path2D refined = raw;
  const Path2D anchors = raw;
  for (int iteration = 0; iteration < params_.elastic_iterations; ++iteration) {
    Path2D next = refined;
    for (std::size_t i = 1; i + 1 < refined.size(); ++i) {
      double dx = params_.smooth_weight *
        (0.5 * (refined[i - 1].x + refined[i + 1].x) - refined[i].x);
      double dy = params_.smooth_weight *
        (0.5 * (refined[i - 1].y + refined[i + 1].y) - refined[i].y);
      dx += params_.anchor_weight * (anchors[i].x - refined[i].x);
      dy += params_.anchor_weight * (anchors[i].y - refined[i].y);

      const double clearance = distance_field.distance_world(refined[i].x, refined[i].y);
      double gx = 0.0;
      double gy = 0.0;
      if (std::isfinite(clearance) && clearance < params_.desired_clearance &&
        distance_field.gradient_world(refined[i].x, refined[i].y, gx, gy))
      {
        const double norm = std::hypot(gx, gy);
        if (norm > 1e-6) {
          const double push = params_.obstacle_weight *
            (params_.desired_clearance - clearance);
          dx += push * gx / norm;
          dy += push * gy / norm;
        }
      }

      const double displacement = std::hypot(dx, dy);
      if (displacement > 0.10) {
        dx *= 0.10 / displacement;
        dy *= 0.10 / displacement;
      }
      next[i].x += dx;
      next[i].y += dy;
    }
    assign_tangent_yaws(next);
    refined.swap(next);
  }

  PathValidator validator(params_.footprint);
  if (!validator.is_path_clear(refined, grid, false)) {
    refined = raw;
  }
  if (!validator.is_path_clear(refined, grid, false)) {
    return {};
  }

  LocalTrajectory result;
  result.reserve(refined.size());
  for (std::size_t i = 0; i < refined.size(); ++i) {
    barn_core::TrajectoryPoint point;
    point.pose = refined[i];
    point.clearance = distance_field.distance_world(refined[i].x, refined[i].y);
    const auto cell = grid.world_to_cell(refined[i].x, refined[i].y);
    point.in_unknown = !grid.in_bounds(cell) ||
      grid.classify(cell) == barn_core::CellState::kUnknown;

    double curvature = 0.0;
    if (i > 0 && i + 1 < refined.size()) {
      const double ds = std::hypot(
        refined[i + 1].x - refined[i - 1].x,
        refined[i + 1].y - refined[i - 1].y);
      if (ds > 1e-4) {
        curvature = std::abs(barn_core::wrap_angle(
          refined[i + 1].yaw - refined[i - 1].yaw)) / ds;
      }
    }
    const double curvature_speed = curvature > 1e-4 ?
      std::min(
        std::sqrt(params_.max_lateral_accel / curvature),
        params_.max_yaw_rate / curvature)
      : params_.max_speed;

    // Soft side-clearance slowdown: reduce speed in narrow spaces for safety,
    // but never to 0. If the path is actually blocked ahead, the global/local
    // planner will reject it and trigger a replan.
    double clearance_scale = 1.0;
    const double min_clearance = params_.footprint.half_width + params_.footprint.margin;
    if (std::isfinite(point.clearance)) {
      if (point.clearance < params_.desired_clearance) {
        const double t = (point.clearance - min_clearance) / (params_.desired_clearance - min_clearance);
        // Raised minimum scale from 0.25 to 0.85 to maintain high speed near obstacles
        clearance_scale = 0.85 + 0.15 * std::clamp(t, 0.0, 1.0);
      }
    }

    point.v_ref = std::min(params_.max_speed, curvature_speed) * clearance_scale;
    if (point.in_unknown) {
      point.v_ref = std::min(point.v_ref, params_.unknown_speed);
    }
    result.push_back(point);
  }

  // Entry-heading gate: Enforce strict differential-drive behavior. If the
  // robot's current heading is more than ~25 degrees off the first path tangent,
  // drop v_ref to 0. This forces the MPC to completely rotate in place and align
  // before creeping forward, instead of driving in a wide Ackermann-style arc.
  if (!result.empty() && refined.size() >= 2) {
    const double heading_error = std::abs(barn_core::wrap_angle(refined[0].yaw - pose.yaw));
    // Linearly scale from 1.0 (at 0 error) down to 0.0 at 0.45 rad (~25 deg).
    const double heading_scale = std::max(0.0, 1.0 - heading_error / 0.45);
    // Apply the scale over the first ~1.0 m of the trajectory, fading out
    // linearly so the speed ramps up naturally once the robot is aligned.
    double arc = 0.0;
    for (std::size_t i = 0; i < result.size(); ++i) {
      if (i > 0) {
        arc += std::hypot(
          refined[i].x - refined[i - 1].x, refined[i].y - refined[i - 1].y);
      }
      const double fade = std::max(0.0, 1.0 - arc / params_.heading_align_distance);
      const double scale = 1.0 - fade * (1.0 - heading_scale);
      result[i].v_ref *= scale;
    }
  }

  // If the local segment reaches the global goal, create a stopping profile.
  const auto & goal = global_path.back();
  if (std::hypot(refined.back().x - goal.x, refined.back().y - goal.y) < 0.25) {
    double distance_to_goal = 0.0;
    result.back().v_ref = 0.0;
    for (std::size_t i = result.size() - 1; i-- > 0;) {
      distance_to_goal += std::hypot(
        result[i + 1].pose.x - result[i].pose.x,
        result[i + 1].pose.y - result[i].pose.y);
      result[i].v_ref = std::min(
        result[i].v_ref, std::sqrt(2.0 * params_.braking_decel * distance_to_goal));
    }
  }
  return result;
}

}  // namespace barn_classical
