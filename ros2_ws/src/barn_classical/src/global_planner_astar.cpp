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

double heuristic(const barn_core::Pose2D & pose, const barn_core::Goal2D & goal)
{
  return std::hypot(goal.x - pose.x, goal.y - pose.y);
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
    !footprint_is_clear(grid, start, params_.footprint, false))
  {
    return {};
  }

  barn_core::DistanceField2D distance_field;
  distance_field.rebuild(grid);
  const double bin_angle = 2.0 * M_PI / params_.yaw_bins;
  const int start_bin = yaw_to_bin(start.yaw, params_.yaw_bins);
  const auto start_key = make_key(start_cell, start_bin, grid.width(), params_.yaw_bins);

  std::unordered_map<std::uint64_t, SearchNode> nodes;
  nodes.reserve(100000);
  SearchNode initial;
  initial.pose = start;
  initial.pose.yaw = bin_to_yaw(start_bin, params_.yaw_bins);
  initial.g = 0.0;
  nodes.emplace(start_key, initial);

  std::priority_queue<QueueEntry> open;
  open.push({params_.heuristic_weight * heuristic(start, goal), start_key});
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
      current.g + params_.heuristic_weight * heuristic(current.pose, goal) + 1e-9)
    {
      continue;
    }
    ++stats_.expanded;

    if (heuristic(current.pose, goal) <= params_.goal_tolerance) {
      reached_key = current_entry.key;
      reached = true;
      break;
    }

    std::vector<std::pair<barn_core::Pose2D, double>> neighbors;
    neighbors.reserve(5);
    for (int turn : {-1, 0, 1}) {
      const double yaw_delta = turn * bin_angle;
      const double mid_yaw = current.pose.yaw + yaw_delta * 0.5;
      barn_core::Pose2D next{
        current.pose.x + params_.step_size * std::cos(mid_yaw),
        current.pose.y + params_.step_size * std::sin(mid_yaw),
        barn_core::wrap_angle(current.pose.yaw + yaw_delta)};
      neighbors.emplace_back(next, params_.step_size + params_.turn_weight * std::abs(turn));
    }
    for (int turn : {-1, 1}) {
      barn_core::Pose2D next = current.pose;
      next.yaw = barn_core::wrap_angle(next.yaw + turn * bin_angle);
      neighbors.emplace_back(next, params_.rotate_weight * bin_angle);
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

      double transition = candidate.second;
      if (state == barn_core::CellState::kUnknown) {
        transition *= params_.unknown_cost_multiplier;
      }
      const double clearance = distance_field.distance(cell);
      if (std::isfinite(clearance)) {
        transition += params_.clearance_weight / std::max(clearance, 0.05);
      }

      const auto key = make_key(cell, next_bin, grid.width(), params_.yaw_bins);
      const double tentative = current.g + transition;
      auto [it, inserted] = nodes.try_emplace(key);
      if (inserted || tentative + 1e-9 < it->second.g) {
        it->second.pose = next;
        it->second.g = tentative;
        it->second.parent = current_entry.key;
        it->second.has_parent = true;
        open.push({tentative + params_.heuristic_weight * heuristic(next, goal), key});
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
  return reverse_path;
}

}  // namespace barn_classical
