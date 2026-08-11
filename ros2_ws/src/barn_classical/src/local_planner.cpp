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

// Curvature from a finite difference is only meaningful over a physically
// meaningful baseline, and only up to a physically meaningful value.
//
// MEASURED: during a 7.6 s stall with the MPC reporting "solved", the planner
// reporting success and the shield reporting clear, the lookahead curvature was
// 11.678 rad/m -- an 8.6 cm turn radius on a 51 cm robot -- which drove
// curvature_speed to 0.259 m/s and v_ref to 0.132. While moving, the same
// quantity was 1.367. It is an artifact, not geometry.
//
// Cause: kappa = |dyaw| / ds was guarded only by `ds > 1e-4` (0.1 mm). The
// elastic band displaces path points without resampling, so neighbours can end
// up nearly coincident and a real yaw change divided by a near-zero baseline
// explodes. Because the profile takes the MAX over the lookahead window, one
// such sample pins the speed for the whole approach to it.
//
// kMinCurvatureBaseline: below this the sample says nothing about curvature.
// kMaxCurvature: 1/0.30 m. A differential-drive robot cannot usefully arc
// tighter than its own rotation radius; anything sharper is a place to rotate in
// place, which the entry-heading gate already handles -- not a reason to crawl.
constexpr double kMinCurvatureBaseline = 0.05;
constexpr double kMaxCurvature = 1.0 / 0.30;

double curvature_between(
  const barn_core::Pose2D & prev, const barn_core::Pose2D & next)
{
  const double ds = std::hypot(next.x - prev.x, next.y - prev.y);
  if (ds < kMinCurvatureBaseline) {
    return 0.0;
  }
  const double k = std::abs(barn_core::wrap_angle(next.yaw - prev.yaw)) / ds;
  return std::min(k, kMaxCurvature);
}


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
      curvature = curvature_between(refined[i - 1], refined[i + 1]);
    }

    // Lookahead curvature: scan ahead ~1.5 m to find the worst-case curvature.
    // This pre-decelerates the robot BEFORE reaching a corner, instead of only
    // reacting when it's already at the turn (where MPC overshoots and stalls).
    double lookahead_curvature = curvature;
    // Clearance AT THE CORNER THAT BINDS, not at the robot. The speed being set
    // here is for negotiating that corner, so the space available there is what
    // decides how fast it can be taken.
    double limiting_clearance = point.clearance;
    {
      double lookahead_arc = 0.0;
      const double kLookaheadDistance = params_.curvature_lookahead_m;
      for (std::size_t j = i + 1; j + 1 < refined.size(); ++j) {
        lookahead_arc += std::hypot(
          refined[j].x - refined[j - 1].x, refined[j].y - refined[j - 1].y);
        if (lookahead_arc > kLookaheadDistance) {
          break;
        }
        if (j > 0 && j + 1 < refined.size()) {
          const double curv_j = curvature_between(refined[j - 1], refined[j + 1]);
          if (curv_j > lookahead_curvature) {
            lookahead_curvature = curv_j;
            limiting_clearance = distance_field.distance_world(refined[j].x, refined[j].y);
          }
        }
      }
    }
    const double min_clearance = params_.footprint.half_width + params_.footprint.margin;

    // Spend the lateral-acceleration budget according to how much room the corner
    // has. Previously this was a constant, so a curve was taken at exactly the
    // same speed threading a 0.3 m gap as crossing an open field -- and since the
    // curvature is a MAX over the lookahead window, one corner pinned the speed
    // for the whole approach to it. Observed directly: the robot decelerates hard
    // for every curve regardless of size or surroundings.
    //
    // A lateral-accel cap is nominally about tipping or slipping, but a 0.43 m
    // Jackal at ~1 m/s is nowhere near either. The real reason to slow for a BARN
    // curve is that tracking error near a wall becomes a collision -- and that
    // reason scales with clearance, so the budget does too. In a pinch the
    // original conservatism is unchanged; in the open the robot is allowed to
    // corner at up to open_lateral_accel_gain x the budget.
    double lateral_budget = params_.max_lateral_accel;
    if (std::isfinite(limiting_clearance)) {
      const double span = std::max(1e-3, params_.open_clearance - min_clearance);
      const double t = std::clamp((limiting_clearance - min_clearance) / span, 0.0, 1.0);
      lateral_budget *= 1.0 + t * (std::max(1.0, params_.open_lateral_accel_gain) - 1.0);
    }

    const double effective_curvature = lookahead_curvature;
    const double curvature_speed = effective_curvature > 1e-4 ?
      std::min(
        std::sqrt(lateral_budget / effective_curvature),
        params_.max_yaw_rate / effective_curvature)
      : params_.max_speed;

    // Soft side-clearance slowdown: reduce speed in narrow spaces for safety.
    // NOTE the comment below is stale -- the code produces a 0.85..1.0 range, not
    // a 0.45 floor. Left as-is: it is a 15% effect and not what governs speed.
    double clearance_scale = 1.0;
    if (std::isfinite(point.clearance)) {
      if (point.clearance < params_.desired_clearance) {
        const double t = (point.clearance - min_clearance) / (params_.desired_clearance - min_clearance);
        clearance_scale = 0.85 + 0.15 * std::clamp(t, 0.0, 1.0);
      }
    }

    point.v_ref = std::min(params_.max_speed, curvature_speed) * clearance_scale;
    if (result.empty()) {
      // First point == the one the controller tracks next. Recorded so a live
      // trial can say WHICH term zeroed the command, not merely that it was zero.
      debug_.curvature = effective_curvature;
      debug_.curvature_speed = curvature_speed;
      debug_.clearance_scale = clearance_scale;
      debug_.heading_scale = 1.0;
      debug_.v_ref = point.v_ref;
    }
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
    // Linearly scale from 1.0 (at 0 error) down to 0.3 at 1.5 rad (~86 deg).
    const double heading_scale = std::max(0.3, 1.0 - heading_error / 1.5);
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
      if (i == 0) {
        debug_.heading_scale = scale;
        debug_.v_ref = result[0].v_ref;
      }
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
