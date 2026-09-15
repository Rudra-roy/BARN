// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// B-Spline local planner with TOPP-RA time-optimal velocity profiling.
//
// Standalone implementation for A/B comparison with the elastic-band
// LocalPlanner.  The public interface mirrors LocalPlanner: plan() returns a
// LocalTrajectory (vector<TrajectoryPoint>) that the MPC tracks as before.
//
// Pipeline:
//   1. Extract a raw path slice from the global A* path.
//   2. Fit a uniform cubic B-spline (C² continuous).
//   3. Optimise control points via gradient descent (obstacle, anchor,
//      smoothness, curvature penalties with κ_max ≤ 2/W).
//   4. Evaluate the locked spline to dense arc-length samples.
//   5. Run TOPP-RA with differential-drive wheel velocity/acceleration
//      constraints to produce the time-optimal ṡ(s).
//   6. Emit v_ref = ṡ(s) and yaw = atan2(y', x') for each sample.

#include "barn_classical/local_plannerBspline.hpp"

#include <algorithm>
#include <cassert>
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

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::size_t nearest_path_index_bspline(const Path2D & path, const barn_core::Pose2D & pose)
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

void assign_tangent_yaws_bspline(Path2D & path)
{
  if (path.size() < 2) {
    return;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.back().yaw = path[path.size() - 2].yaw;
}

// ---------------------------------------------------------------------------
// Uniform cubic B-spline utilities
//
// A uniform cubic B-spline with n control points P_0..P_{n-1} has (n-3)
// segments.  Segment j (0-indexed) is parameterised by t ∈ [0,1) using
// control points P_j..P_{j+3} and the uniform cubic basis matrix:
//
//   C(t) = (1/6) [1-3t+3t²-t³, 4-6t²+3t³, 1+3t+3t²-3t³, t³] · [Pj..Pj+3]
//
// This guarantees C² continuity at every knot.
// ---------------------------------------------------------------------------

struct Vec2 { double x, y; };

Vec2 bspline_eval(const std::vector<Vec2> & cp, int seg, double t)
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double b0 = (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0;
  const double b1 = (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0;
  const double b2 = (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0;
  const double b3 = t3 / 6.0;
  return {
    b0 * cp[seg].x + b1 * cp[seg + 1].x + b2 * cp[seg + 2].x + b3 * cp[seg + 3].x,
    b0 * cp[seg].y + b1 * cp[seg + 1].y + b2 * cp[seg + 2].y + b3 * cp[seg + 3].y};
}

Vec2 bspline_deriv1(const std::vector<Vec2> & cp, int seg, double t)
{
  const double t2 = t * t;
  const double d0 = (-3.0 + 6.0 * t - 3.0 * t2) / 6.0;
  const double d1 = (-12.0 * t + 9.0 * t2) / 6.0;
  const double d2 = (3.0 + 6.0 * t - 9.0 * t2) / 6.0;
  const double d3 = (3.0 * t2) / 6.0;
  return {
    d0 * cp[seg].x + d1 * cp[seg + 1].x + d2 * cp[seg + 2].x + d3 * cp[seg + 3].x,
    d0 * cp[seg].y + d1 * cp[seg + 1].y + d2 * cp[seg + 2].y + d3 * cp[seg + 3].y};
}

Vec2 bspline_deriv2(const std::vector<Vec2> & cp, int seg, double t)
{
  const double dd0 = (6.0 - 6.0 * t) / 6.0;
  const double dd1 = (-12.0 + 18.0 * t) / 6.0;
  const double dd2 = (6.0 - 18.0 * t) / 6.0;
  const double dd3 = (6.0 * t) / 6.0;
  return {
    dd0 * cp[seg].x + dd1 * cp[seg + 1].x + dd2 * cp[seg + 2].x + dd3 * cp[seg + 3].x,
    dd0 * cp[seg].y + dd1 * cp[seg + 1].y + dd2 * cp[seg + 2].y + dd3 * cp[seg + 3].y};
}

double bspline_curvature(const std::vector<Vec2> & cp, int seg, double t)
{
  const auto d1 = bspline_deriv1(cp, seg, t);
  const auto d2 = bspline_deriv2(cp, seg, t);
  const double speed_sq = d1.x * d1.x + d1.y * d1.y;
  if (speed_sq < 1e-12) {
    return 0.0;
  }
  const double cross = d1.x * d2.y - d1.y * d2.x;
  return std::abs(cross) / std::pow(speed_sq, 1.5);
}

// Number of segments for n control points.
int n_segments(int n_cp) { return std::max(0, n_cp - 3); }

// ---------------------------------------------------------------------------
// Sample the spline at roughly uniform arc-length intervals.
// Returns dense (x,y) points, their curvatures, and cumulative arc lengths.
// ---------------------------------------------------------------------------

struct SplineSample
{
  double x, y;
  double dx, dy;     // first derivative (tangent)
  double kappa;      // curvature
  double arc;        // cumulative arc length from start
};

std::vector<SplineSample> sample_spline(
  const std::vector<Vec2> & cp, double step)
{
  const int nseg = n_segments(static_cast<int>(cp.size()));
  if (nseg <= 0) {
    return {};
  }

  // First pass: fine-step to accumulate arc length, then resample uniformly.
  constexpr int fine_steps = 8;  // sub-steps per segment for length estimation
  std::vector<SplineSample> fine;
  fine.reserve(nseg * fine_steps + 1);
  double arc = 0.0;
  for (int s = 0; s < nseg; ++s) {
    for (int k = 0; k < fine_steps; ++k) {
      const double t = static_cast<double>(k) / fine_steps;
      const auto p = bspline_eval(cp, s, t);
      const auto d = bspline_deriv1(cp, s, t);
      const double kappa = bspline_curvature(cp, s, t);
      if (!fine.empty()) {
        arc += std::hypot(p.x - fine.back().x, p.y - fine.back().y);
      }
      fine.push_back({p.x, p.y, d.x, d.y, kappa, arc});
    }
  }
  // Last point (t=1 of last segment).
  {
    const auto p = bspline_eval(cp, nseg - 1, 1.0);
    const auto d = bspline_deriv1(cp, nseg - 1, 1.0);
    const double kappa = bspline_curvature(cp, nseg - 1, 1.0);
    arc += std::hypot(p.x - fine.back().x, p.y - fine.back().y);
    fine.push_back({p.x, p.y, d.x, d.y, kappa, arc});
  }

  const double total_length = fine.back().arc;
  if (total_length < 1e-4) {
    return fine;
  }

  // Resample at uniform arc-length steps by linearly interpolating fine.
  const int n_out = std::max(2, static_cast<int>(std::ceil(total_length / step)) + 1);
  std::vector<SplineSample> out;
  out.reserve(n_out);
  std::size_t cursor = 0;
  for (int i = 0; i < n_out; ++i) {
    double target = static_cast<double>(i) / (n_out - 1) * total_length;
    while (cursor + 1 < fine.size() && fine[cursor + 1].arc < target) {
      ++cursor;
    }
    if (cursor + 1 >= fine.size()) {
      out.push_back(fine.back());
      out.back().arc = target;
      continue;
    }
    const double span = fine[cursor + 1].arc - fine[cursor].arc;
    const double frac = span > 1e-9 ? (target - fine[cursor].arc) / span : 0.0;
    SplineSample s;
    s.x = fine[cursor].x + frac * (fine[cursor + 1].x - fine[cursor].x);
    s.y = fine[cursor].y + frac * (fine[cursor + 1].y - fine[cursor].y);
    s.dx = fine[cursor].dx + frac * (fine[cursor + 1].dx - fine[cursor].dx);
    s.dy = fine[cursor].dy + frac * (fine[cursor + 1].dy - fine[cursor].dy);
    s.kappa = fine[cursor].kappa + frac * (fine[cursor + 1].kappa - fine[cursor].kappa);
    s.arc = target;
    out.push_back(s);
  }
  return out;
}

// ---------------------------------------------------------------------------
// TOPP-RA  (Time-Optimal Path Parameterization via Reachability Analysis)
//
// Given a geometric path with curvature κ(s) and differential-drive wheel
// constraints, compute the time-optimal squared path velocity x(s) = ṡ².
//
// Constraints (per grid point):
//   velocity:     v_{L,R} = ṡ (1 ∓ W/2 · κ(s)) ≤ v_wheel_max
//   acceleration: a_{L,R} = ẍ (1 ∓ W/2·κ)/2 + x·(∓W/2·dκ/ds) ≤ a_wheel_max
//     where ẍ = dx/ds  and  x = ṡ²
//
// The algorithm:
//   1. Compute x_max(s) from velocity bounds.
//   2. Backward pass: for each s from end→start, find the maximum x(s)
//      reachable while decelerating to x(s+ds) within acceleration limits.
//   3. Forward pass: for each s from start→end, find the maximum x(s)
//      achievable from x(s-ds) within acceleration limits and the backward
//      ceiling.
// ---------------------------------------------------------------------------

std::vector<double> topp_ra(
  const std::vector<SplineSample> & samples,
  double track_width, double v_wheel_max, double a_wheel_max,
  double max_linear_speed, double max_yaw_rate, double max_lateral_accel,
  const barn_core::DistanceField2D * distance_field = nullptr,
  double min_clearance = 0.25, double open_clearance = 0.80, double tight_lateral_accel = 0.50)
{
  const int n = static_cast<int>(samples.size());
  if (n < 2) {
    return std::vector<double>(n, 0.0);
  }

  const double half_w = track_width / 2.0;

  // Step 1: velocity ceiling x_max(s) = ṡ_max²
  std::vector<double> x_max(n, 0.0);
  for (int i = 0; i < n; ++i) {
    const double kappa = samples[i].kappa;
    const double factor_l = std::abs(1.0 - half_w * kappa);
    const double factor_r = std::abs(1.0 + half_w * kappa);
    const double max_factor = std::max(factor_l, factor_r);
    
    // 1. Individual wheel linear velocity limit: ṡ · max_factor <= v_wheel_max
    double sdot_max = max_factor > 1e-9 ? v_wheel_max / max_factor : max_linear_speed;

    // 2. Maximum yaw rate limit: omega = ṡ · kappa <= max_yaw_rate => ṡ <= max_yaw_rate / kappa
    if (kappa > 1e-4 && max_yaw_rate > 1e-4) {
      sdot_max = std::min(sdot_max, max_yaw_rate / kappa);
    }

    // 3. Maximum lateral acceleration limit: a_lat = ṡ² · kappa <= a_lat_max
    // Continuous clearance scaling: open space (c >= open_clearance) gets max_lateral_accel,
    // narrow space scales down smoothly toward tight_lateral_accel.
    double a_lat_max = max_lateral_accel;
    if (distance_field != nullptr) {
      const double c = distance_field->distance_world(samples[i].x, samples[i].y);
      if (std::isfinite(c)) {
        const double span = std::max(1e-3, open_clearance - min_clearance);
        const double ratio = std::clamp((c - min_clearance) / span, 0.0, 1.0);
        a_lat_max = tight_lateral_accel + ratio * (max_lateral_accel - tight_lateral_accel);
      }
    }
    if (kappa > 1e-4 && a_lat_max > 1e-4) {
      sdot_max = std::min(sdot_max, std::sqrt(a_lat_max / kappa));
    }

    // 4. Also cap by the robot's linear speed limit.
    sdot_max = std::min(sdot_max, max_linear_speed);
    x_max[i] = sdot_max * sdot_max;
  }

  // Numerical dκ/ds for acceleration constraints.
  std::vector<double> dkappa_ds(n, 0.0);
  for (int i = 1; i + 1 < n; ++i) {
    const double ds = samples[i + 1].arc - samples[i - 1].arc;
    if (ds > 1e-9) {
      dkappa_ds[i] = (samples[i + 1].kappa - samples[i - 1].kappa) / ds;
    }
  }
  if (n > 1) {
    dkappa_ds[0] = dkappa_ds[1];
    dkappa_ds[n - 1] = dkappa_ds[n - 2];
  }

  // Acceleration bounds: for a given x = ṡ², the acceleration of each wheel is
  //   a_L = (dx/ds) · (1 - W/2·κ)/2  +  x · (-W/2 · dκ/ds)
  //   a_R = (dx/ds) · (1 + W/2·κ)/2  +  x · (+W/2 · dκ/ds)
  // Both must satisfy |a| ≤ a_wheel_max.
  //
  // For a given x, this constrains dx/ds to an interval [u_min, u_max].
  // Returns the feasible dx/ds interval as (u_min, u_max).
  auto accel_bounds = [&](int idx, double x) -> std::pair<double, double> {
      const double kappa = samples[idx].kappa;
      const double dk = dkappa_ds[idx];
      const double fl = (1.0 - half_w * kappa) / 2.0;  // coefficient of dx/ds for left wheel
      const double fr = (1.0 + half_w * kappa) / 2.0;  // for right wheel
      const double cl = -half_w * dk;   // coefficient of x for left wheel
      const double cr = +half_w * dk;   // for right wheel

      double u_min = -1e12, u_max = 1e12;

      // Left wheel: fl · u + cl · x ∈ [-a_max, a_max]
      if (std::abs(fl) > 1e-12) {
        double lo = (-a_wheel_max - cl * x) / fl;
        double hi = ( a_wheel_max - cl * x) / fl;
        if (fl < 0) { std::swap(lo, hi); }
        u_min = std::max(u_min, lo);
        u_max = std::min(u_max, hi);
      } else {
        // fl ≈ 0: check if the constraint is feasible at all.
        if (std::abs(cl * x) > a_wheel_max) {
          return {1.0, -1.0};  // infeasible
        }
      }

      // Right wheel: fr · u + cr · x ∈ [-a_max, a_max]
      if (std::abs(fr) > 1e-12) {
        double lo = (-a_wheel_max - cr * x) / fr;
        double hi = ( a_wheel_max - cr * x) / fr;
        if (fr < 0) { std::swap(lo, hi); }
        u_min = std::max(u_min, lo);
        u_max = std::min(u_max, hi);
      } else {
        if (std::abs(cr * x) > a_wheel_max) {
          return {1.0, -1.0};
        }
      }

      return {u_min, u_max};
    };

  // Step 2: Backward pass — maximum x(s) starting from x(L) = 0.
  std::vector<double> x_bwd(n, 0.0);
  x_bwd[n - 1] = 0.0;  // stop at end
  for (int i = n - 2; i >= 0; --i) {
    const double ds = samples[i + 1].arc - samples[i].arc;
    if (ds < 1e-9) {
      x_bwd[i] = x_bwd[i + 1];
      continue;
    }
    // We want to find the max x[i] such that we can decelerate to x[i+1] over ds.
    // x[i+1] = x[i] + u*ds  =>  u = (x[i+1] - x[i]) / ds
    // We need u ∈ [u_min, u_max] at position i with velocity x[i].
    // Since we're going backward, we want the max x[i] such that
    // (x[i+1] - x[i]) / ds >= u_min(i, x[i])   (can decelerate fast enough)
    // Start from x_max and check feasibility.
    double xi = std::min(x_max[i], x_bwd[i + 1] + 2.0 * a_wheel_max * ds);
    xi = std::max(xi, 0.0);
    // Refine: check if the acceleration interval is feasible.
    const auto [u_min, u_max] = accel_bounds(i, xi);
    const double u_needed = (x_bwd[i + 1] - xi) / ds;
    if (u_needed < u_min) {
      // Need to decelerate harder than possible — reduce x[i].
      // u_min * ds = x[i+1] - x[i]  =>  x[i] = x[i+1] - u_min * ds
      // But u_min depends on x[i], so iterate a few times.
      for (int iter = 0; iter < 5; ++iter) {
        const auto [um, uM] = accel_bounds(i, xi);
        (void)uM;
        xi = x_bwd[i + 1] - um * ds;
        xi = std::clamp(xi, 0.0, x_max[i]);
      }
    }
    x_bwd[i] = std::clamp(xi, 0.0, x_max[i]);
  }

  // Step 3: Forward pass — actual profile, starting from current speed (~0).
  std::vector<double> x_fwd(n, 0.0);
  x_fwd[0] = std::min(x_max[0], x_bwd[0]);
  for (int i = 1; i < n; ++i) {
    const double ds = samples[i].arc - samples[i - 1].arc;
    if (ds < 1e-9) {
      x_fwd[i] = x_fwd[i - 1];
      continue;
    }
    const auto [u_min, u_max] = accel_bounds(i - 1, x_fwd[i - 1]);
    (void)u_min;
    double xi = x_fwd[i - 1] + u_max * ds;
    xi = std::min(xi, x_max[i]);
    xi = std::min(xi, x_bwd[i]);
    x_fwd[i] = std::max(xi, 0.0);
  }

  // Convert x = ṡ² to ṡ (linear velocity along the path).
  std::vector<double> sdot(n);
  for (int i = 0; i < n; ++i) {
    sdot[i] = std::sqrt(std::max(0.0, x_fwd[i]));
  }
  return sdot;
}

}  // namespace

// ===========================================================================
// LocalPlannerBspline::plan()
// ===========================================================================

LocalTrajectory LocalPlannerBspline::plan(
  const Path2D & global_path, const barn_core::Pose2D & pose,
  const barn_core::OccupancyGrid2D & grid,
  const barn_core::DistanceField2D * precomputed_distance_field) const
{
  if (global_path.empty() || grid.width() == 0) {
    return {};
  }

  // -------------------------------------------------------------------
  // Step 1: Extract a raw path slice (unchanged from elastic band).
  // -------------------------------------------------------------------
  const std::size_t start = nearest_path_index_bspline(global_path, pose);
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
  assign_tangent_yaws_bspline(raw);

  // -------------------------------------------------------------------
  // Step 2: Build distance field if not provided.
  // -------------------------------------------------------------------
  barn_core::DistanceField2D owned_distance_field;
  if (precomputed_distance_field == nullptr || !precomputed_distance_field->valid()) {
    owned_distance_field.rebuild(grid);
    precomputed_distance_field = &owned_distance_field;
  }
  const auto & distance_field = *precomputed_distance_field;

  // -------------------------------------------------------------------
  // Step 3: Fit initial control points by uniform arc-length sampling.
  // -------------------------------------------------------------------
  const int n_cp = std::max(4, params_.n_control_points);  // need >= 4 for cubic B-spline
  std::vector<Vec2> control_points(n_cp);

  // Compute cumulative arc lengths of the raw path.
  std::vector<double> raw_arc(raw.size(), 0.0);
  for (std::size_t i = 1; i < raw.size(); ++i) {
    raw_arc[i] = raw_arc[i - 1] +
      std::hypot(raw[i].x - raw[i - 1].x, raw[i].y - raw[i - 1].y);
  }
  const double total_raw_length = raw_arc.back();

  // Sample uniformly along the raw path to initialise control points.
  for (int j = 0; j < n_cp; ++j) {
    const double target = static_cast<double>(j) / (n_cp - 1) * total_raw_length;
    // Find the segment on the raw path.
    std::size_t seg = 0;
    while (seg + 1 < raw.size() && raw_arc[seg + 1] < target) {
      ++seg;
    }
    if (seg + 1 >= raw.size()) {
      control_points[j] = {raw.back().x, raw.back().y};
    } else {
      const double span = raw_arc[seg + 1] - raw_arc[seg];
      const double frac = span > 1e-9 ? (target - raw_arc[seg]) / span : 0.0;
      control_points[j] = {
        raw[seg].x + frac * (raw[seg + 1].x - raw[seg].x),
        raw[seg].y + frac * (raw[seg + 1].y - raw[seg].y)};
    }
  }
  const std::vector<Vec2> anchors = control_points;  // save for anchor penalty

  // -------------------------------------------------------------------
  // Step 4: Gradient descent optimisation of interior control points.
  // -------------------------------------------------------------------
  const double kappa_max = 2.0 / params_.track_width;
  const int nseg = n_segments(n_cp);

  for (int iteration = 0; iteration < params_.spline_opt_iterations; ++iteration) {
    std::vector<Vec2> grad(n_cp, {0.0, 0.0});

    // -- Smoothness: penalise second differences ‖P_{i-1} - 2P_i + P_{i+1}‖² --
    for (int i = 1; i + 1 < n_cp; ++i) {
      const double ddx = control_points[i - 1].x - 2.0 * control_points[i].x + control_points[i + 1].x;
      const double ddy = control_points[i - 1].y - 2.0 * control_points[i].y + control_points[i + 1].y;
      // Gradient w.r.t. P_i of ‖dd‖² = 2·dd·(-2) = -4·dd
      grad[i].x += params_.spline_smooth_weight * (-4.0 * ddx);
      grad[i].y += params_.spline_smooth_weight * (-4.0 * ddy);
    }

    // -- Anchor: pull toward initial positions --
    for (int i = 1; i + 1 < n_cp; ++i) {
      grad[i].x += params_.spline_anchor_weight * 2.0 * (control_points[i].x - anchors[i].x);
      grad[i].y += params_.spline_anchor_weight * 2.0 * (control_points[i].y - anchors[i].y);
    }

    // -- Obstacle: push away from obstacles using distance field --
    // Evaluate at a few points per segment and accumulate gradient to nearby CPs.
    for (int s = 0; s < nseg; ++s) {
      for (int k = 0; k < 4; ++k) {
        const double t = (k + 0.5) / 4.0;
        const auto pt = bspline_eval(control_points, s, t);
        const double clearance = distance_field.distance_world(pt.x, pt.y);
        if (!std::isfinite(clearance) || clearance >= params_.desired_clearance) {
          continue;
        }
        double gx = 0.0, gy = 0.0;
        if (!distance_field.gradient_world(pt.x, pt.y, gx, gy)) {
          continue;
        }
        const double norm = std::hypot(gx, gy);
        if (norm < 1e-6) {
          continue;
        }
        const double push = params_.spline_obstacle_weight *
          (params_.desired_clearance - clearance);
        // Distribute to the 4 control points of this segment proportionally
        // to their basis weight.  Approximate: weight each CP equally.
        for (int ci = s; ci <= s + 3 && ci < n_cp; ++ci) {
          if (ci == 0 || ci == n_cp - 1) continue;  // pin endpoints
          grad[ci].x -= push * gx / norm * 0.25;
          grad[ci].y -= push * gy / norm * 0.25;
        }
      }
    }

    // -- Curvature penalty: when κ > κ_max at eval points, push CPs gentler --
    for (int s = 0; s < nseg; ++s) {
      for (int k = 0; k <= 4; ++k) {
        const double t = static_cast<double>(k) / 4.0;
        const double kappa = bspline_curvature(control_points, s, t);
        if (kappa <= kappa_max) {
          continue;
        }
        // Penalty is (κ - κ_max)². Approximate gradient: push the middle two
        // CPs toward the line between the outer two CPs (straighten the curve).
        const double penalty = params_.spline_curvature_weight * (kappa - kappa_max);
        // Straighten: move P_{s+1} and P_{s+2} toward midpoint of P_s and P_{s+3}.
        const double mx = 0.5 * (control_points[s].x + control_points[s + 3].x);
        const double my = 0.5 * (control_points[s].y + control_points[s + 3].y);
        for (int ci = s + 1; ci <= s + 2; ++ci) {
          if (ci <= 0 || ci >= n_cp - 1) continue;
          const double dx = mx - control_points[ci].x;
          const double dy = my - control_points[ci].y;
          const double d = std::hypot(dx, dy);
          if (d > 1e-6) {
            grad[ci].x -= penalty * dx / d;
            grad[ci].y -= penalty * dy / d;
          }
        }
      }
    }

    // -- Apply gradients to interior CPs (pin endpoints) --
    for (int i = 1; i + 1 < n_cp; ++i) {
      double dx = -grad[i].x;  // gradient descent: step = -grad
      double dy = -grad[i].y;
      const double displacement = std::hypot(dx, dy);
      if (displacement > 0.10) {
        dx *= 0.10 / displacement;
        dy *= 0.10 / displacement;
      }
      control_points[i].x += dx;
      control_points[i].y += dy;
    }
  }

  // -------------------------------------------------------------------
  // Step 5: Sample the optimised spline at dense arc-length intervals.
  // -------------------------------------------------------------------
  auto samples = sample_spline(control_points, params_.spline_sample_step);
  if (samples.size() < 2) {
    return {};
  }

  // Build a Path2D from the samples for collision validation.
  Path2D refined;
  refined.reserve(samples.size());
  for (const auto & s : samples) {
    barn_core::Pose2D p;
    p.x = s.x;
    p.y = s.y;
    p.yaw = std::atan2(s.dy, s.dx);
    refined.push_back(p);
  }
  assign_tangent_yaws_bspline(refined);  // ensure consistent tangent yaws

  // Collision validation fallback.
  PathValidator validator(params_.footprint);
  if (!validator.is_path_clear(refined, grid, false)) {
    // Spline path collides — fall back to raw path.
    refined = raw;
    // Re-create samples from raw path for TOPP-RA.
    samples.clear();
    double arc = 0.0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
      if (i > 0) {
        arc += std::hypot(raw[i].x - raw[i - 1].x, raw[i].y - raw[i - 1].y);
      }
      double dx_t = 0.0, dy_t = 0.0;
      if (i + 1 < raw.size()) {
        dx_t = raw[i + 1].x - raw[i].x;
        dy_t = raw[i + 1].y - raw[i].y;
      } else if (i > 0) {
        dx_t = raw[i].x - raw[i - 1].x;
        dy_t = raw[i].y - raw[i - 1].y;
      }
      double kappa = 0.0;
      if (i > 0 && i + 1 < raw.size()) {
        const double ds = std::hypot(raw[i + 1].x - raw[i - 1].x, raw[i + 1].y - raw[i - 1].y);
        if (ds > 0.05) {
          kappa = std::abs(barn_core::wrap_angle(raw[i + 1].yaw - raw[i - 1].yaw)) / ds;
          kappa = std::min(kappa, 1.0 / 0.30);  // sanity cap
        }
      }
      samples.push_back({raw[i].x, raw[i].y, dx_t, dy_t, kappa, arc});
    }
  }
  if (!validator.is_path_clear(refined, grid, false)) {
    return {};
  }

  // -------------------------------------------------------------------
  // Step 6: TOPP-RA velocity profiling (clearance-aware).
  // -------------------------------------------------------------------
  const double min_clearance = params_.footprint.half_width + params_.stop_margin;
  auto sdot = topp_ra(
    samples, params_.track_width, params_.wheel_v_max, params_.wheel_a_max,
    params_.max_speed, params_.max_yaw_rate, params_.max_lateral_accel,
    &distance_field, min_clearance, params_.open_clearance, params_.tight_lateral_accel);

  // -------------------------------------------------------------------
  // Step 7: Build the LocalTrajectory output with clearance slowdown.
  // -------------------------------------------------------------------
  LocalTrajectory result;
  result.reserve(refined.size());
  for (std::size_t i = 0; i < refined.size(); ++i) {
    barn_core::TrajectoryPoint point;
    point.pose = refined[i];
    point.clearance = distance_field.distance_world(refined[i].x, refined[i].y);
    const auto cell = grid.world_to_cell(refined[i].x, refined[i].y);
    point.in_unknown = !grid.in_bounds(cell) ||
      grid.classify(cell) == barn_core::CellState::kUnknown;

    // v_ref from TOPP-RA.
    double v = (i < sdot.size()) ? sdot[i] : 0.0;
    v = std::min(v, params_.max_speed);

    // In tight pinches with sharp curvature, throttle speed to safe clearance-scaled ceiling:
    const double kappa_i = (i < samples.size()) ? samples[i].kappa : 0.0;
    double clearance_ratio_i = 1.0;
    if (std::isfinite(point.clearance)) {
      const double span = std::max(1e-3, params_.open_clearance - min_clearance);
      clearance_ratio_i = std::clamp((point.clearance - min_clearance) / span, 0.0, 1.0);
    }
    if (clearance_ratio_i < 0.35 && kappa_i > 0.8) {
      const double tight_v_cap = params_.crawl_speed + (clearance_ratio_i / 0.35) * (0.60 - params_.crawl_speed);
      v = std::min(v, tight_v_cap);
      v = std::max(params_.crawl_speed, v);
    }
    point.v_ref = v;

    if (result.empty()) {
      // Record debug info for the first point.
      double max_kappa = 0.0;
      for (const auto & s : samples) {
        max_kappa = std::max(max_kappa, s.kappa);
      }
      debug_.curvature = max_kappa;
      debug_.curvature_speed = (i < sdot.size()) ? sdot[i] : 0.0;
      debug_.clearance_scale = 1.0;
      debug_.heading_scale = 1.0;
      debug_.v_ref = v;
    }

    if (point.in_unknown) {
      point.v_ref = std::min(point.v_ref, params_.unknown_speed);
    }
    result.push_back(point);
  }

  // -------------------------------------------------------------------
  // Step 8: Clearance-Aware Heading Gate (Entering Turns).
  // -------------------------------------------------------------------
  // In OPEN space: takes smooth, high-speed sweeping arcs (85% to 100% speed).
  // In NARROW space: if heading error > 20 deg (0.35 rad), pivots in place first to prevent corner clipping.
  if (!result.empty() && refined.size() >= 2) {
    const double heading_error = std::abs(barn_core::wrap_angle(refined[0].yaw - pose.yaw));
    const double start_clearance = result[0].clearance;
    double start_clearance_ratio = 1.0;
    if (std::isfinite(start_clearance)) {
      const double span = std::max(1e-3, params_.open_clearance - min_clearance);
      start_clearance_ratio = std::clamp((start_clearance - min_clearance) / span, 0.0, 1.0);
    }

    double heading_scale = 1.0;
    if (start_clearance_ratio < 0.25 && heading_error > 0.35) {
      // Really narrow space and sharp turn: pivot in place first
      heading_scale = 0.0;
    } else {
      // Continuous speed floor based on clearance ratio:
      // In open space: floor is 0.85 (fast arc). In tighter corridors: floor scales smoothly.
      const double min_floor = 0.30 + 0.55 * start_clearance_ratio;
      const double threshold = 0.35 + 0.70 * start_clearance_ratio;
      if (heading_error > threshold) {
        heading_scale = min_floor;
      } else {
        heading_scale = 1.0 - (1.0 - min_floor) * (heading_error / threshold);
      }
    }

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

  // -------------------------------------------------------------------
  // Step 9: Braking ramp near goal (unchanged from elastic band).
  // -------------------------------------------------------------------
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
