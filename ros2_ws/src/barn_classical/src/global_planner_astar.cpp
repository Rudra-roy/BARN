// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/global_planner_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "barn_core/distance_field.hpp"
#include "barn_core/geometry.hpp"

namespace barn_classical
{
namespace
{

struct SearchNode
{
  barn_core::Pose2D pose;
  double g{std::numeric_limits<double>::infinity()};
  std::uint64_t parent{0};
  bool has_parent{false};
};

struct QueueEntry
{
  double f;
  std::uint64_t key;
  bool operator<(const QueueEntry & other) const { return f > other.f; }
};

int yaw_to_bin(double yaw, int bins)
{
  const double bin_angle = 2.0 * M_PI / bins;
  int bin = static_cast<int>(std::llround(
    (barn_core::wrap_angle(yaw) + M_PI) / bin_angle));
  bin %= bins;
  return bin < 0 ? bin + bins : bin;
}

double bin_to_yaw(int bin, int bins)
{
  return barn_core::wrap_angle(-M_PI + bin * 2.0 * M_PI / bins);
}

std::uint64_t make_key(const barn_core::GridIndex & cell, int yaw_bin, std::size_t width, int bins)
{
  return (static_cast<std::uint64_t>(cell.row) * width +
    static_cast<std::uint64_t>(cell.col)) * static_cast<std::uint64_t>(bins) + yaw_bin;
}

}  // namespace

Path2D GlobalPlannerAStar::plan(
  const barn_core::OccupancyGrid2D & grid, const barn_core::Pose2D & start,
  const barn_core::Goal2D & goal)
{
  stats_ = {};
  const auto begin = std::chrono::steady_clock::now();
  if (params_.yaw_bins < 4 || params_.step_size <= 0.0 || grid.width() == 0 ||
    grid.height() == 0)
  {
    return {};
  }
  const auto start_cell = grid.world_to_cell(start.x, start.y);
  const auto goal_cell = grid.world_to_cell(goal.x, goal.y);
  if (!grid.in_bounds(start_cell) || !grid.in_bounds(goal_cell) ||
    grid.classify(start_cell) == barn_core::CellState::kOccupied ||
    grid.classify(goal_cell) == barn_core::CellState::kOccupied)
  {
    return {};
  }

  barn_core::DistanceField2D distance_field;
  distance_field.rebuild(grid);
  const int start_bin = yaw_to_bin(start.yaw, params_.yaw_bins);
  const auto start_key = make_key(start_cell, start_bin, grid.width(), params_.yaw_bins);

  std::vector<double> h_grid(grid.width() * grid.height(), std::numeric_limits<double>::infinity());
  {
    struct DijkstraEntry {
      double dist;
      barn_core::GridIndex cell;
      bool operator>(const DijkstraEntry & other) const { return dist > other.dist; }
    };
    std::priority_queue<DijkstraEntry, std::vector<DijkstraEntry>, std::greater<DijkstraEntry>> pq;
    h_grid[goal_cell.row * grid.width() + goal_cell.col] = 0.0;
    pq.push({0.0, goal_cell});

    const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};
    const double dc[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    double start_dist = std::numeric_limits<double>::infinity();

    while (!pq.empty()) {
      auto top = pq.top();
      pq.pop();
      const int idx = top.cell.row * grid.width() + top.cell.col;
      if (top.dist > h_grid[idx]) {
        continue;
      }
      
      if (top.cell.row == start_cell.row && top.cell.col == start_cell.col) {
        start_dist = top.dist;
      }
      if (top.dist > start_dist + 5.0) {
        break; // Reached past start cell, no need to evaluate the rest of the map
      }

      for (int i = 0; i < 8; ++i) {
        barn_core::GridIndex n{top.cell.col + dx[i], top.cell.row + dy[i]};
        if (!grid.in_bounds(n)) continue;
        const auto state = grid.classify(n);
        if (state == barn_core::CellState::kOccupied || 
            (!params_.allow_unknown && state == barn_core::CellState::kUnknown)) {
          continue;
        }
        
        double penalty = 0.0;
        const double clearance = distance_field.distance(n);
        if (std::isfinite(clearance) && clearance < params_.clearance_penalty_radius) {
          penalty = params_.clearance_weight *
            (1.0 / std::max(clearance, 0.05) - 1.0 / params_.clearance_penalty_radius) *
            dc[i] * grid.resolution();
        }
        double step_cost = params_.distance_weight * dc[i] * grid.resolution();
        if (state == barn_core::CellState::kUnknown) {
          step_cost *= params_.unknown_cost_multiplier;
        }
        
        const double new_d = top.dist + step_cost + penalty;
        const int nidx = n.row * grid.width() + n.col;
        if (new_d < h_grid[nidx]) {
          h_grid[nidx] = new_d;
          pq.push({new_d, n});
        }
      }
    }
  }

  if (std::isinf(h_grid[start_cell.row * grid.width() + start_cell.col])) {
    return {}; // Goal is completely walled off from start
  }

  auto heuristic = [&](const barn_core::Pose2D & pose) {
    const auto cell = grid.world_to_cell(pose.x, pose.y);
    if (!grid.in_bounds(cell)) return std::numeric_limits<double>::infinity();
    return h_grid[cell.row * grid.width() + cell.col];
  };

  std::unordered_map<std::uint64_t, SearchNode> nodes;
  nodes.reserve(100000);
  SearchNode initial;
  initial.pose = start;
  initial.pose.yaw = bin_to_yaw(start_bin, params_.yaw_bins);
  initial.g = 0.0;
  nodes.emplace(start_key, initial);

  std::priority_queue<QueueEntry> open;
  open.push({params_.heuristic_weight * heuristic(start), start_key});
  std::uint64_t reached_key = 0;
  bool reached = false;

  while (!open.empty()) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double, std::milli>(now - begin).count();
    if (elapsed > params_.timeout_ms) {
      stats_.timed_out = true;
      break;
    }

    const auto current_entry = open.top();
    open.pop();
    const auto current_it = nodes.find(current_entry.key);
    if (current_it == nodes.end()) {
      continue;
    }
    const SearchNode current = current_it->second;
    if (current_entry.f >
      current.g + params_.heuristic_weight * heuristic(current.pose) + 1e-9)
    {
      continue;
    }
    ++stats_.expanded;

    if (std::hypot(current.pose.x - goal.x, current.pose.y - goal.y) <= params_.goal_tolerance) {
      reached_key = current_entry.key;
      reached = true;
      break;
    }

    std::vector<std::pair<barn_core::Pose2D, double>> neighbors;
    neighbors.reserve(5);
    for (int turn : {-1, 0, 1}) {
      const double yaw_delta = turn * (2.0 * M_PI / params_.yaw_bins);
      const double mid_yaw = current.pose.yaw + yaw_delta * 0.5;
      barn_core::Pose2D next{
        current.pose.x + params_.step_size * std::cos(mid_yaw),
        current.pose.y + params_.step_size * std::sin(mid_yaw),
        barn_core::wrap_angle(current.pose.yaw + yaw_delta)};
      neighbors.emplace_back(next, params_.step_size + params_.turn_weight * std::abs(turn));
    }
    for (int turn : {-1, 1}) {
      // In-place rotation requires physical rotation clearance (half-diagonal + margin >= 0.35m)
      // to prevent A* from pinning the robot into sharp corners against walls.
      const auto cur_cell = grid.world_to_cell(current.pose.x, current.pose.y);
      const double c = distance_field.distance(cur_cell);
      if (std::isfinite(c) && c < 0.35) {
        continue;
      }
      barn_core::Pose2D next = current.pose;
      next.yaw = barn_core::wrap_angle(next.yaw + turn * (2.0 * M_PI / params_.yaw_bins));
      neighbors.emplace_back(next, params_.rotate_weight * (2.0 * M_PI / params_.yaw_bins));
    }

    for (auto & candidate : neighbors) {
      auto & next = candidate.first;
      const auto cell = grid.world_to_cell(next.x, next.y);
      if (!grid.in_bounds(cell)) {
        continue;
      }
      const int next_bin = yaw_to_bin(next.yaw, params_.yaw_bins);
      next.yaw = bin_to_yaw(next_bin, params_.yaw_bins);
      
      if (!swept_segment_is_clear(
          grid, current.pose, next, params_.footprint, false, grid.resolution()))
      {
        continue;
      }
      const auto state = grid.classify(cell);
      if (state == barn_core::CellState::kOccupied ||
        (!params_.allow_unknown && state == barn_core::CellState::kUnknown))
      {
        continue;
      }

      double transition = params_.distance_weight * candidate.second;
      if (state == barn_core::CellState::kUnknown) {
        transition *= params_.unknown_cost_multiplier;
      }
      const double clearance = distance_field.distance(cell);
      if (std::isfinite(clearance) && clearance < params_.clearance_penalty_radius) {
        transition += params_.clearance_weight *
          (1.0 / std::max(clearance, 0.05) - 1.0 / params_.clearance_penalty_radius) *
          candidate.second;
      }

      const auto key = make_key(cell, next_bin, grid.width(), params_.yaw_bins);
      const double tentative = current.g + transition;
      auto [it, inserted] = nodes.try_emplace(key);
      if (inserted || tentative + 1e-9 < it->second.g) {
        it->second.pose = next;
        it->second.g = tentative;
        it->second.parent = current_entry.key;
        it->second.has_parent = true;
        open.push({tentative + params_.heuristic_weight * heuristic(next), key});
      }
    }
  }

  stats_.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  if (!reached) {
    return {};
  }

  Path2D reverse_path;
  std::uint64_t key = reached_key;
  while (true) {
    const auto & node = nodes.at(key);
    reverse_path.push_back(node.pose);
    if (!node.has_parent) {
      break;
    }
    key = node.parent;
  }
  std::reverse(reverse_path.begin(), reverse_path.end());
  barn_core::Pose2D goal_pose{goal.x, goal.y, reverse_path.back().yaw};
  if (swept_segment_is_clear(
      grid, reverse_path.back(), goal_pose, params_.footprint, false, grid.resolution()))
  {
    reverse_path.push_back(goal_pose);
  }

  // 1. Loop Elimination:
  // If the path doubles back or visits near an earlier waypoint, cut out the redundant loop.
  Path2D loop_free;
  loop_free.reserve(reverse_path.size());
  std::size_t i = 0;
  while (i < reverse_path.size()) {
    loop_free.push_back(reverse_path[i]);
    std::size_t skip_to = i;
    for (std::size_t j = reverse_path.size() - 1; j > i + 2; --j) {
      const double d = std::hypot(
        reverse_path[j].x - reverse_path[i].x,
        reverse_path[j].y - reverse_path[i].y);
      if (d < 0.35 && swept_segment_is_clear(
          grid, reverse_path[i], reverse_path[j], params_.footprint, false, grid.resolution()))
      {
        skip_to = j;
        break;
      }
    }
    if (skip_to > i) {
      i = skip_to;
    } else {
      ++i;
    }
  }

  // 2. Line-of-Sight Shortcutting:
  // Greedily connect distant visible waypoints to remove jagged lattice turns and Dubins arcs.
  Path2D shortcut_path;
  if (!loop_free.empty()) {
    shortcut_path.push_back(loop_free.front());
    std::size_t curr_idx = 0;
    while (curr_idx < loop_free.size() - 1) {
      std::size_t next_idx = curr_idx + 1;
      for (std::size_t test_idx = loop_free.size() - 1; test_idx > curr_idx + 1; --test_idx) {
        if (swept_segment_is_clear(
            grid, loop_free[curr_idx], loop_free[test_idx], params_.footprint, false, grid.resolution()))
        {
          next_idx = test_idx;
          break;
        }
      }
      shortcut_path.push_back(loop_free[next_idx]);
      curr_idx = next_idx;
    }
  } else {
    shortcut_path = reverse_path;
  }

  // 3. Densification:
  // Interpolate waypoints at regular 0.15m intervals with smooth tangent headings.
  Path2D densified;
  densified.reserve(shortcut_path.size() * 5);
  constexpr double kSpacing = 0.15;
  for (std::size_t k = 0; k + 1 < shortcut_path.size(); ++k) {
    const auto & p0 = shortcut_path[k];
    const auto & p1 = shortcut_path[k + 1];
    const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / kSpacing)));
    const double seg_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    for (int s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      barn_core::Pose2D pt;
      pt.x = p0.x + t * (p1.x - p0.x);
      pt.y = p0.y + t * (p1.y - p0.y);
      pt.yaw = seg_yaw;
      densified.push_back(pt);
    }
  }
  if (!shortcut_path.empty()) {
    densified.push_back(shortcut_path.back());
  }

  // 4. Moving average smoothing on densified path to smooth orientation transitions:
  if (densified.size() > 2) {
    Path2D smoothed_path = densified;
    const int half_window = 2; // 5-point moving average
    for (std::size_t k = 1; k + 1 < densified.size(); ++k) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_sin = 0.0;
      double sum_cos = 0.0;
      int count = 0;
      for (int j = -half_window; j <= half_window; ++j) {
        int idx = std::clamp(static_cast<int>(k) + j, 0, static_cast<int>(densified.size()) - 1);
        sum_x += densified[idx].x;
        sum_y += densified[idx].y;
        sum_sin += std::sin(densified[idx].yaw);
        sum_cos += std::cos(densified[idx].yaw);
        count++;
      }
      smoothed_path[k].x = sum_x / count;
      smoothed_path[k].y = sum_y / count;
      smoothed_path[k].yaw = std::atan2(sum_sin, sum_cos);
    }
    return smoothed_path;
  }
  return densified;
}

}  // namespace barn_classical
