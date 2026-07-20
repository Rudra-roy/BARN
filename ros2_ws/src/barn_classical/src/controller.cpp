// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Sparse>
#include <osqp.h>

namespace barn_classical
{
namespace
{

constexpr int kStateSize = 5;
constexpr int kInputSize = 2;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double unwrap_near(double angle, double reference)
{
  return reference + normalize_angle(angle - reference);
}

using Sparse = Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int>;
using Triplet = Eigen::Triplet<c_float, c_int>;

struct QpSolution
{
  std::vector<double> x;
  bool solved{false};
  bool timed_out{false};
  std::string status{"setup_failed"};
};

}  // namespace

void Controller::reset()
{
  warm_start_.clear();
}

MpcResult Controller::control(
  const LocalTrajectory & trajectory, const barn_core::State2D & state,
  const barn_core::DistanceField2D & distance_field,
  const std::vector<DynamicObstacle> & obstacles)
{
  MpcResult result;
  const auto started = std::chrono::steady_clock::now();
  if (trajectory.empty() || !distance_field.valid() || params_.horizon < 1 || params_.dt <= 0.0) {
    result.status = "missing_trajectory_or_map";
    return result;
  }

  const int n_steps = params_.horizon;
  const int state_vars = kStateSize * (n_steps + 1);
  const int input_offset = state_vars;
  const int slack_offset = input_offset + kInputSize * n_steps;
  // A second, tighter slack block dedicated to moving obstacles. The static
  // field slack is sized for tight BARN corridors and would fully absorb the
  // dynamic keep-out; a separate, small-capped slack keeps that avoidance firm.
  const int dyn_slack_offset = slack_offset + (n_steps + 1);
  const int n_vars = dyn_slack_offset + (n_steps + 1);
  const auto sx = [](int k, int component) {return kStateSize * k + component;};
  const auto ui = [input_offset](int k, int component) {
      return input_offset + kInputSize * k + component;
    };

  // Select references by arc length, not by callback timing. This keeps the
  // QP deterministic when the control loop jitters.
  std::vector<barn_core::TrajectoryPoint> reference(n_steps + 1, trajectory.back());
  std::size_t nearest = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const double dx = trajectory[i].pose.x - state.pose.x;
    const double dy = trajectory[i].pose.y - state.pose.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest = i;
    }
  }
  reference[0] = trajectory[nearest];
  double target_arc = 0.0;
  double accumulated = 0.0;
  std::size_t cursor = nearest;
  for (int k = 1; k <= n_steps; ++k) {
    target_arc += std::max(0.10, reference[k - 1].v_ref) * params_.dt;
    while (cursor + 1 < trajectory.size() && accumulated < target_arc) {
      accumulated += std::hypot(
        trajectory[cursor + 1].pose.x - trajectory[cursor].pose.x,
        trajectory[cursor + 1].pose.y - trajectory[cursor].pose.y);
      ++cursor;
    }
    reference[k] = trajectory[cursor];
  }
  double previous_yaw = state.pose.yaw;
  for (auto & point : reference) {
    point.pose.yaw = unwrap_near(point.pose.yaw, previous_yaw);
    previous_yaw = point.pose.yaw;
  }

  if (warm_start_.size() == static_cast<std::size_t>(n_vars)) {
    double max_divergence = 0.0;
    for (int k = 0; k <= n_steps; ++k) {
      const double dx = warm_start_[sx(k, 0)] - reference[k].pose.x;
      const double dy = warm_start_[sx(k, 1)] - reference[k].pose.y;
      max_divergence = std::max(max_divergence, std::hypot(dx, dy));
    }
    if (max_divergence > 0.5) {
      warm_start_.clear();
    }
  }

  std::vector<double> linearization(n_vars, 0.0);
  if (warm_start_.size() == static_cast<std::size_t>(n_vars)) {
    for (int k = 0; k < n_steps; ++k) {
      for (int c = 0; c < kStateSize; ++c) {
        linearization[sx(k, c)] = warm_start_[sx(k + 1, c)];
      }
    }
    for (int c = 0; c < kStateSize; ++c) {
      linearization[sx(n_steps, c)] = warm_start_[sx(n_steps, c)];
    }
    for (int k = 0; k < n_steps - 1; ++k) {
      for (int c = 0; c < kInputSize; ++c) {
        linearization[ui(k, c)] = warm_start_[ui(k + 1, c)];
      }
    }
    for (int c = 0; c < kInputSize; ++c) {
      linearization[ui(n_steps - 1, c)] = warm_start_[ui(n_steps - 1, c)];
    }
    linearization[sx(0, 0)] = state.pose.x;
    linearization[sx(0, 1)] = state.pose.y;
    linearization[sx(0, 2)] = state.pose.yaw;
    linearization[sx(0, 3)] = std::clamp(state.v, 0.0, params_.max_speed);
    linearization[sx(0, 4)] = std::clamp(state.w, -params_.max_yaw_rate, params_.max_yaw_rate);
  } else {
    for (int k = 0; k <= n_steps; ++k) {
      linearization[sx(k, 0)] = reference[k].pose.x;
      linearization[sx(k, 1)] = reference[k].pose.y;
      linearization[sx(k, 2)] = reference[k].pose.yaw;
      linearization[sx(k, 3)] = reference[k].v_ref;
      linearization[sx(k, 4)] = 0.0;
    }
  }

  QpSolution latest;
  const std::array<std::array<double, 2>, 8> boundary_points{{
    {{1.0, 1.0}}, {{1.0, -1.0}}, {{-1.0, 1.0}}, {{-1.0, -1.0}},
    {{1.0, 0.0}}, {{-1.0, 0.0}}, {{0.0, 1.0}}, {{0.0, -1.0}}
  }};

  // Pre-select the nearest dynamic obstacles so the constraint count stays
  // bounded regardless of how many the tracker reports.
  std::vector<const DynamicObstacle *> active_obstacles;
  {
    std::vector<std::pair<double, const DynamicObstacle *>> ranked;
    ranked.reserve(obstacles.size());
    for (const auto & o : obstacles) {
      ranked.emplace_back(std::hypot(o.x - state.pose.x, o.y - state.pose.y), &o);
    }
    std::sort(
      ranked.begin(), ranked.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    const std::size_t cap = params_.max_dynamic_obstacles > 0
      ? static_cast<std::size_t>(params_.max_dynamic_obstacles)
      : ranked.size();
    for (std::size_t i = 0; i < ranked.size() && i < cap; ++i) {
      active_obstacles.push_back(ranked[i].second);
    }
  }
  // Circumscribed footprint radius used for the (yaw-free) moving-obstacle
  // keep-out; conservative relative to the exact box used for static obstacles.
  const double robot_radius = std::hypot(
    params_.footprint.half_length + params_.footprint.margin,
    params_.footprint.half_width + params_.footprint.margin);

  for (int pass = 0; pass < std::max(1, params_.max_linearization_passes); ++pass) {
    const double elapsed_before = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    if (elapsed_before >= params_.solve_deadline_ms ||
      (pass > 0 && elapsed_before > 0.55 * params_.solve_deadline_ms))
    {
      break;
    }

    std::vector<Triplet> p_triplets;
    std::vector<Triplet> a_triplets;
    std::vector<c_float> q(n_vars, 0.0);
    const auto add_square = [&](int index, double weight, double target) {
        p_triplets.emplace_back(index, index, static_cast<c_float>(2.0 * weight));
        q[index] += static_cast<c_float>(-2.0 * weight * target);
      };

    for (int k = 0; k <= n_steps; ++k) {
      const double terminal = k == n_steps ? 3.0 : 1.0;
      add_square(sx(k, 0), 14.0 * terminal, reference[k].pose.x);
      add_square(sx(k, 1), 14.0 * terminal, reference[k].pose.y);
      // Yaw tracking raised to 14.0 (was 8.0) — equal to position weight.
      // At corners, turning is as important as following x,y reference.
      add_square(sx(k, 2), 14.0 * terminal, reference[k].pose.yaw);
      // Velocity tracking lowered to 1.5 (was 3.0) so MPC freely slows for turns.
      add_square(sx(k, 3), 1.5, std::clamp(reference[k].v_ref, 0.0, params_.max_speed));
      add_square(sx(k, 4), 0.02, 0.0);
      // Obstacle slack penalty doubled (5000→10000) to prevent corner-cutting.
      add_square(slack_offset + k, 10000.0, 0.0);
      // Dynamic-obstacle slack: penalised even harder so the robot yields to
      // moving obstacles rather than buying its way through with slack.
      add_square(dyn_slack_offset + k, 20000.0, 0.0);
    }
    for (int k = 0; k < n_steps; ++k) {
      add_square(ui(k, 0), 0.10, 0.0);
      add_square(ui(k, 1), 0.08, 0.0);
      if (k > 0) {
        for (int c = 0; c < kInputSize; ++c) {
          const double slew_weight = c == 0 ? 0.20 : 0.15;
          p_triplets.emplace_back(
            ui(k - 1, c), ui(k - 1, c), static_cast<c_float>(2.0 * slew_weight));
          p_triplets.emplace_back(ui(k, c), ui(k, c), static_cast<c_float>(2.0 * slew_weight));
          p_triplets.emplace_back(
            ui(k - 1, c), ui(k, c), static_cast<c_float>(-2.0 * slew_weight));
        }
      }
    }

    std::vector<c_float> lower;
    std::vector<c_float> upper;
    lower.reserve(n_vars + kStateSize * n_steps + 8 * (n_steps + 1));
    upper.reserve(lower.capacity());
    int row = 0;
    const auto add_bound = [&](int variable, double lo, double hi) {
        a_triplets.emplace_back(row, variable, 1.0);
        lower.push_back(static_cast<c_float>(lo));
        upper.push_back(static_cast<c_float>(hi));
        ++row;
      };

    for (int k = 0; k <= n_steps; ++k) {
      for (int c = 0; c < kStateSize; ++c) {
        double lo = -OSQP_INFTY;
        double hi = OSQP_INFTY;
        if (c == 3) {lo = 0.0; hi = params_.max_speed;}
        if (c == 4) {lo = -params_.max_yaw_rate; hi = params_.max_yaw_rate;}
        if (k == 0) {
          const std::array<double, kStateSize> initial{{
            state.pose.x, state.pose.y, state.pose.yaw,
            std::clamp(state.v, 0.0, params_.max_speed),
            std::clamp(state.w, -params_.max_yaw_rate, params_.max_yaw_rate)}};
          lo = initial[c];
          hi = initial[c];
        }
        add_bound(sx(k, c), lo, hi);
      }
    }
    for (int k = 0; k < n_steps; ++k) {
      add_bound(ui(k, 0), -params_.max_accel, params_.max_accel);
      add_bound(ui(k, 1), -params_.max_yaw_accel, params_.max_yaw_accel);
    }
    for (int k = 0; k <= n_steps; ++k) {
      add_bound(slack_offset + k, 0.0, params_.max_obstacle_slack);
      add_bound(dyn_slack_offset + k, 0.0, params_.max_dynamic_slack);
    }

    const auto add_equality = [&](const std::vector<std::pair<int, double>> & terms, double value) {
        for (const auto & term : terms) {
          a_triplets.emplace_back(row, term.first, static_cast<c_float>(term.second));
        }
        lower.push_back(static_cast<c_float>(value));
        upper.push_back(static_cast<c_float>(value));
        ++row;
      };
    for (int k = 0; k < n_steps; ++k) {
      const double theta = linearization[sx(k, 2)];
      const double velocity = std::clamp(linearization[sx(k, 3)], 0.0, params_.max_speed);
      const double ct = std::cos(theta);
      const double st = std::sin(theta);
      add_equality({
        {sx(k + 1, 0), 1.0}, {sx(k, 0), -1.0},
        {sx(k, 3), -params_.dt * ct}, {sx(k, 2), params_.dt * velocity * st}},
        params_.dt * velocity * st * theta);
      add_equality({
        {sx(k + 1, 1), 1.0}, {sx(k, 1), -1.0},
        {sx(k, 3), -params_.dt * st}, {sx(k, 2), -params_.dt * velocity * ct}},
        -params_.dt * velocity * ct * theta);
      add_equality({{sx(k + 1, 2), 1.0}, {sx(k, 2), -1.0}, {sx(k, 4), -params_.dt}}, 0.0);
      add_equality({{sx(k + 1, 3), 1.0}, {sx(k, 3), -1.0}, {ui(k, 0), -params_.dt}}, 0.0);
      add_equality({{sx(k + 1, 4), 1.0}, {sx(k, 4), -1.0}, {ui(k, 1), -params_.dt}}, 0.0);
    }

    const double hx = params_.footprint.half_length + params_.footprint.margin;
    const double hy = params_.footprint.half_width + params_.footprint.margin;
    for (int k = 0; k <= n_steps; ++k) {
      const double xb = linearization[sx(k, 0)];
      const double yb = linearization[sx(k, 1)];
      const double tb = linearization[sx(k, 2)];
      const double ct = std::cos(tb);
      const double st = std::sin(tb);
      for (const auto & unit : boundary_points) {
        const double bx = unit[0] * hx;
        const double by = unit[1] * hy;
        const double px = xb + ct * bx - st * by;
        const double py = yb + st * bx + ct * by;
        const double dtheta_x = -st * bx - ct * by;
        const double dtheta_y = ct * bx - st * by;
        double gx = 0.0;
        double gy = 0.0;
        const double d0 = distance_field.distance_world(px, py);
        if (!std::isfinite(d0) || !distance_field.gradient_world(px, py, gx, gy)) {
          continue;
        }
        const double gt = gx * dtheta_x + gy * dtheta_y;
        a_triplets.emplace_back(row, sx(k, 0), static_cast<c_float>(gx));
        a_triplets.emplace_back(row, sx(k, 1), static_cast<c_float>(gy));
        a_triplets.emplace_back(row, sx(k, 2), static_cast<c_float>(gt));
        a_triplets.emplace_back(row, slack_offset + k, 1.0);
        const double rhs = params_.obstacle_margin - d0 + gx * xb + gy * yb + gt * tb;
        lower.push_back(static_cast<c_float>(rhs));
        upper.push_back(OSQP_INFTY);
        ++row;
      }
    }

    // --- Moving-obstacle keep-out (Family-1 spatiotemporal soft constraints) ---
    // Each tracked obstacle is propagated with a constant-velocity model. Per
    // horizon step k we add a linearized half-plane keeping the robot center at
    // least r_safe from the obstacle's predicted position at time k*dt. These
    // share the per-step slack with the static field constraints, so both stay
    // soft under the same heavy penalty and the robot yields smoothly.
    for (const DynamicObstacle * obs : active_obstacles) {
      const double r_safe = obs->radius + robot_radius + params_.dynamic_margin;
      for (int k = 0; k <= n_steps; ++k) {
        const double t = k * params_.dt;
        const double ox = obs->x + obs->vx * t;
        const double oy = obs->y + obs->vy * t;
        const double xb = linearization[sx(k, 0)];
        const double yb = linearization[sx(k, 1)];
        const double dx = xb - ox;
        const double dy = yb - oy;
        const double dist0 = std::hypot(dx, dy);
        // Skip when comfortably clear (constraint inactive) or when the
        // linearization point coincides with the obstacle (gradient undefined).
        if (dist0 > r_safe + 1.5 || dist0 < 1e-3) {
          continue;
        }
        const double nx = dx / dist0;
        const double ny = dy / dist0;
        a_triplets.emplace_back(row, sx(k, 0), static_cast<c_float>(nx));
        a_triplets.emplace_back(row, sx(k, 1), static_cast<c_float>(ny));
        a_triplets.emplace_back(row, dyn_slack_offset + k, 1.0);
        const double rhs = r_safe - dist0 + nx * xb + ny * yb;
        lower.push_back(static_cast<c_float>(rhs));
        upper.push_back(OSQP_INFTY);
        ++row;
      }
    }

    Sparse p_matrix(n_vars, n_vars);
    Sparse a_matrix(row, n_vars);
    p_matrix.setFromTriplets(p_triplets.begin(), p_triplets.end());
    a_matrix.setFromTriplets(a_triplets.begin(), a_triplets.end());
    p_matrix.makeCompressed();
    a_matrix.makeCompressed();

    csc * p_csc = csc_matrix(
      n_vars, n_vars, static_cast<c_int>(p_matrix.nonZeros()), p_matrix.valuePtr(),
      p_matrix.innerIndexPtr(), p_matrix.outerIndexPtr());
    csc * a_csc = csc_matrix(
      row, n_vars, static_cast<c_int>(a_matrix.nonZeros()), a_matrix.valuePtr(),
      a_matrix.innerIndexPtr(), a_matrix.outerIndexPtr());
    OSQPData data{};
    data.n = n_vars;
    data.m = row;
    data.P = p_csc;
    data.A = a_csc;
    data.q = q.data();
    data.l = lower.data();
    data.u = upper.data();
    OSQPSettings settings{};
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 0;
    settings.warm_start = 1;
    // 600 (was 400): dynamic keep-out constraints can need more iterations to
    // converge; well-conditioned cases still stop early, and the per-pass time
    // check bounds wall-clock regardless.
    settings.max_iter = 600;
    settings.eps_abs = 1e-3;
    settings.eps_rel = 1e-3;
#ifdef PROFILING
    settings.time_limit = std::max(
      0.001, (params_.solve_deadline_ms - elapsed_before) / 1000.0);
#endif

    OSQPWorkspace * workspace = nullptr;
    const c_int setup_status = osqp_setup(&workspace, &data, &settings);
    QpSolution candidate;
    if (setup_status == 0 && workspace != nullptr) {
      if (warm_start_.size() == static_cast<std::size_t>(n_vars)) {
        std::vector<c_float> warm(warm_start_.begin(), warm_start_.end());
        (void)osqp_warm_start_x(workspace, warm.data());
      } else {
        std::vector<c_float> warm(linearization.begin(), linearization.end());
        (void)osqp_warm_start_x(workspace, warm.data());
      }
      (void)osqp_solve(workspace);
      const c_int status = workspace->info->status_val;
      candidate.status = workspace->info->status;
      candidate.timed_out = status == OSQP_TIME_LIMIT_REACHED;
      candidate.solved = status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
      if (candidate.solved && workspace->solution != nullptr && workspace->solution->x != nullptr) {
        candidate.x.assign(workspace->solution->x, workspace->solution->x + n_vars);
      }
      (void)osqp_cleanup(workspace);
    }
    if (p_csc != nullptr) {c_free(p_csc);}
    if (a_csc != nullptr) {c_free(a_csc);}

    if (!candidate.solved) {
      latest = std::move(candidate);
      break;
    }
    latest = candidate;
    linearization = candidate.x;
  }

  result.solve_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.timed_out = latest.timed_out || result.solve_ms > params_.solve_deadline_ms;
  result.status = latest.status;
  if (!latest.solved || latest.x.size() != static_cast<std::size_t>(n_vars) || result.timed_out) {
    warm_start_.clear();
    result.success = false;
    return result;
  }

  warm_start_ = latest.x;
  result.command.v = std::clamp(latest.x[sx(1, 3)], 0.0, params_.max_speed);
  result.command.w = std::clamp(
    latest.x[sx(1, 4)], -params_.max_yaw_rate, params_.max_yaw_rate);
  result.prediction.reserve(n_steps + 1);
  for (int k = 0; k <= n_steps; ++k) {
    result.prediction.push_back({
      latest.x[sx(k, 0)], latest.x[sx(k, 1)], normalize_angle(latest.x[sx(k, 2)])});
  }
  result.success = true;
  return result;
}

}  // namespace barn_classical
