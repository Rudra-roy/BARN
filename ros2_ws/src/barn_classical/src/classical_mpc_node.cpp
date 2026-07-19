// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/classical_mpc_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "barn_core/geometry.hpp"
#include "barn_core/scan.hpp"
#include "rclcpp/create_timer.hpp"
#include "std_msgs/msg/bool.hpp"

namespace barn_classical
{
namespace
{

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

barn_core::ScanView scan_view(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  barn_core::ScanView view;
  if (!scan) {
    return view;
  }
  view.ranges = scan->ranges.data();
  view.count = scan->ranges.size();
  view.angle_min = scan->angle_min;
  view.angle_increment = scan->angle_increment;
  view.range_min = scan->range_min;
  view.range_max = scan->range_max;
  return view;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

barn_core::OccupancyGrid2D planning_grid_at_10cm(const barn_core::OccupancyGrid2D & source)
{
  constexpr double resolution = 0.10;
  const std::size_t width =
    static_cast<std::size_t>(std::ceil(source.width() * source.resolution() / resolution));
  const std::size_t height =
    static_cast<std::size_t>(std::ceil(source.height() * source.resolution() / resolution));
  barn_core::OccupancyGrid2D output(
    width, height, resolution, source.origin_x(), source.origin_y());
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t col = 0; col < width; ++col) {
      const double x0 = source.origin_x() + col * resolution;
      const double y0 = source.origin_y() + row * resolution;
      bool occupied = false;
      bool all_free = true;
      for (double dy : {0.025, 0.075}) {
        for (double dx : {0.025, 0.075}) {
          const auto source_cell = source.world_to_cell(x0 + dx, y0 + dy);
          if (!source.in_bounds(source_cell)) {
            all_free = false;
            continue;
          }
          const auto state = source.classify(source_cell);
          occupied = occupied || state == barn_core::CellState::kOccupied;
          all_free = all_free && state == barn_core::CellState::kFree;
        }
      }
      if (occupied) {
        output.set_log_odds({static_cast<int>(col), static_cast<int>(row)}, 3.5);
      } else if (all_free) {
        output.set_log_odds({static_cast<int>(col), static_cast<int>(row)}, -2.0);
      }
    }
  }
  return output;
}

}  // namespace

ClassicalMpcNode::ClassicalMpcNode(const rclcpp::NodeOptions & options)
: Node("classical_mpc_node", options),
  goal_received_time_(0, 0, RCL_ROS_TIME),
  last_progress_time_(0, 0, RCL_ROS_TIME),
  oscillation_window_start_(0, 0, RCL_ROS_TIME)
{
  const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/barn/goal");
  const std::string pose_topic = declare_parameter<std::string>("pose_topic", "/barn/pose");
  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/barn/odom");
  const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/barn/scan");
  const std::string map_topic = declare_parameter<std::string>("map_topic", "/barn/occupancy");
  const std::string cmd_topic = declare_parameter<std::string>("cmd_topic", "/barn/cmd_desired");
  frame_id_ = declare_parameter<std::string>("planning_frame", "odom");
  cmd_frame_ = declare_parameter<std::string>("cmd_frame", "base_link");
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.70);
  enable_los_shortcut_ = declare_parameter<bool>("enable_los_shortcut", true);
  los_max_range_ = declare_parameter<double>("los_max_range", 4.0);
  startup_creep_delay_s_ = declare_parameter<double>("startup_creep_delay_s", 1.0);
  startup_creep_speed_ = declare_parameter<double>("startup_creep_speed", 0.15);
  no_progress_timeout_s_ = declare_parameter<double>("no_progress_timeout_s", 2.0);
  // Obstacle inflation radius applied to the planning grid before A* search.
  // 10 cm keeps the planned path away from walls so the MPC and safety layer
  // never see the robot on a path that grazes an obstacle.
  inflation_radius_ = declare_parameter<double>("obstacle_inflation_radius_m", 0.00);
  // Number of consecutive safety vetoes before requesting a replan. At 20 Hz
  // this is ~300 ms — fast enough to react before the no-progress watchdog
  // fires, but long enough to ignore transient sensor noise.
  veto_replan_threshold_ = declare_parameter<int>("veto_replan_threshold", 6);

  AStarParams astar;
  astar.timeout_ms = declare_parameter<double>("global_planner_budget_ms", 100.0);
  astar.unknown_cost_multiplier = declare_parameter<double>("unknown_cost_multiplier", 1.8);
  // Lower the heuristic weight to make search less greedy, so A* actually
  // respects clearance and turn penalties, choosing safer wide routes over
  // tight direct bottlenecks.
  astar.heuristic_weight = declare_parameter<double>("heuristic_weight", 1.5);
  astar.distance_weight = declare_parameter<double>("distance_weight", 1.0);
  // Soft clearance weight replaces the old binary inflation layer. A higher
  // value pushes paths to the center of corridors without hard-blocking them.
  astar.clearance_weight = declare_parameter<double>("clearance_weight", 0.3);
  astar.clearance_penalty_radius = declare_parameter<double>("clearance_penalty_radius", 1.0);
  // Reduced turn/rotate penalties: the old values (0.35/0.50) heavily penalized
  // the sequence of turns needed for L-turns and U-turns, causing A* to prefer
  // longer routes or timeout. Lower values let A* freely navigate corners.
  astar.turn_weight = declare_parameter<double>("turn_weight", 0.15);
  astar.rotate_weight = declare_parameter<double>("rotate_weight", 0.20);
  astar.yaw_bins = declare_parameter<int>("yaw_bins", 24);
  global_planner_ = GlobalPlannerAStar(astar);
  base_clearance_weight_ = astar.clearance_weight;

  LocalPlannerParams local;
  local.max_speed = declare_parameter<double>("max_speed", 3.5);
  local.unknown_speed = declare_parameter<double>("unknown_speed", 0.4);
  local.horizon_m = declare_parameter<double>("local_horizon_m", 8.0); // Increased horizon to support high speeds
  local.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 2.5);
  local.max_lateral_accel = declare_parameter<double>("max_lateral_accel", 1.5);
  local_planner_ = LocalPlanner(local);

  MpcParams mpc;
  mpc.horizon = declare_parameter<int>("mpc_horizon", 20);
  mpc.dt = declare_parameter<double>("mpc_dt", 0.1);
  mpc.max_speed = local.max_speed;
  mpc.max_yaw_rate = local.max_yaw_rate;
  mpc.max_accel = declare_parameter<double>("max_accel", 2.5);
  mpc.max_yaw_accel = declare_parameter<double>("max_yaw_accel", 3.0);
  mpc.solve_deadline_ms = declare_parameter<double>("mpc_deadline_ms", 35.0);
  mpc.obstacle_margin = declare_parameter<double>("obstacle_margin", 0.10);
  mpc.max_obstacle_slack = declare_parameter<double>("max_obstacle_slack", 1.20);
  mpc.max_linearization_passes = 4;  // One extra pass for better convergence
  controller_ = Controller(mpc);

  RecoveryParams recovery_params;
  recovery_params.max_attempts = declare_parameter<int>("max_recovery_attempts", 5);
  recovery_params.rotate_speed = declare_parameter<double>("recovery_rotate_speed", 1.2);
  // Backtracking recovery: reverse along the known-clear breadcrumb until the
  // robot reaches space wide enough to rotate, then replan from there.
  recovery_params.reverse_speed = declare_parameter<double>("recovery_reverse_speed", 0.35);
  recovery_params.reverse_lookahead = declare_parameter<double>("recovery_reverse_lookahead", 0.5);
  recovery_params.max_reverse_distance =
    declare_parameter<double>("recovery_max_reverse_distance", 1.5);
  recovery_params.reverse_timeout = declare_parameter<double>("recovery_reverse_timeout", 6.0);
  recovery_ = Recovery(recovery_params);

  // Clearance needed to rotate in place (footprint half-diagonal + margin) and
  // breadcrumb sampling.
  rotation_clearance_m_ = declare_parameter<double>("rotation_clearance_m", 0.40);
  breadcrumb_spacing_m_ = declare_parameter<double>("breadcrumb_spacing_m", 0.10);
  breadcrumb_max_ =
    static_cast<std::size_t>(declare_parameter<int>("breadcrumb_max_points", 160));

  command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);
  global_path_pub_ =
    create_publisher<nav_msgs::msg::Path>("/barn/global_path", rclcpp::QoS(1).transient_local());
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/barn/local_path", 2);
  prediction_pub_ = create_publisher<nav_msgs::msg::Path>("/barn/mpc_prediction", 2);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/barn/planner_markers", 2);
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/barn/navigation_diagnostics", 5);
  // Planning grid: published for RViz so the 10cm downsampled grid and inflation layer
  // is visible alongside the global path.
  planning_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/barn/planning_grid", rclcpp::QoS(1).transient_local());

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, rclcpp::QoS(1).transient_local(),
    std::bind(&ClassicalMpcNode::goal_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, 20, std::bind(&ClassicalMpcNode::pose_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20, std::bind(&ClassicalMpcNode::odom_callback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&ClassicalMpcNode::scan_callback, this, std::placeholders::_1));
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ClassicalMpcNode::map_callback, this, std::placeholders::_1));
  veto_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/barn/safety_veto", rclcpp::QoS(1).best_effort(),
    std::bind(&ClassicalMpcNode::veto_callback, this, std::placeholders::_1));

  // Navigation rates are rates of the simulated robot dynamics. Using wall
  // timers creates hundreds of redundant MPC solves per simulated second when
  // Gazebo's real-time factor drops, causing a positive CPU feedback loop.
  // Raised control rate to 30 Hz to increase red dot trajectory replanning speed.
  control_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(33),
    std::bind(&ClassicalMpcNode::control_step, this));
  local_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(100),
    std::bind(&ClassicalMpcNode::local_plan_step, this));
  replan_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(500), [this]() {
    bool need_replan = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      need_replan = global_path_.empty();
    }
    if (need_replan) {
      request_plan();
    }
  });
  planner_thread_ = std::thread(&ClassicalMpcNode::planner_loop, this);
  RCLCPP_INFO(get_logger(), "classical_mpc_node ready: 20 Hz MPC, 10 Hz local, 2 Hz global");
}

ClassicalMpcNode::~ClassicalMpcNode()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_planner_ = true;
  }
  planner_cv_.notify_all();
  if (planner_thread_.joinable()) {
    planner_thread_.join();
  }
}

void ClassicalMpcNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = barn_core::yaw_from_quat(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);
    goal_.tol = goal_tolerance_;
    have_goal_ = true;
    ++goal_generation_;
    global_path_.clear();
    local_trajectory_.clear();
    goal_received_time_ = now();
    progress_initialized_ = false;
    consecutive_mpc_failures_ = 0;
    consecutive_veto_count_ = 0;
    recovery_.reset();
    controller_.reset();
  }
  breadcrumb_.clear();
  RCLCPP_INFO(
    get_logger(), "Goal received at (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
  request_plan();
}

void ClassicalMpcNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.pose.x = msg->pose.position.x;
  state_.pose.y = msg->pose.position.y;
  state_.pose.yaw = barn_core::yaw_from_quat(
    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
    msg->pose.orientation.w);
  have_pose_ = true;
}

void ClassicalMpcNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.v = msg->twist.twist.linear.x;
  state_.w = msg->twist.twist.angular.z;
  have_odom_ = true;
}

void ClassicalMpcNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scan_ = msg;
}

void ClassicalMpcNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  barn_core::OccupancyGrid2D next(
    msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x,
    msg->info.origin.position.y);
  for (std::size_t row = 0; row < msg->info.height; ++row) {
    for (std::size_t col = 0; col < msg->info.width; ++col) {
      const auto value = msg->data[row * msg->info.width + col];
      const barn_core::GridIndex index{static_cast<int>(col), static_cast<int>(row)};
      if (value >= 65) {
        next.set_log_odds(index, 3.5);
      } else if (value >= 0 && value <= 25) {
        next.set_log_odds(index, -2.0);
      }
    }
  }
  auto next_planning_grid = planning_grid_at_10cm(next);
  barn_core::DistanceField2D next_field;
  // The controller/local planner use metric queries, so a 10 cm EDT preserves
  // their interface while reducing the continuously updated field to one
  // quarter of the 5 cm occupancy map's cells.
  next_field.rebuild(next_planning_grid);

  // NOTE: Binary inflation has been removed from the A* planning path.
  // The old inflated grid hard-blocked corridors at tight turns and U-bends.
  // A* now uses the raw planning grid with soft clearance costs (via the
  // distance field) to stay away from obstacles without closing off passages.
  // The inflated grid is still built for RViz visualization only.
  barn_core::OccupancyGrid2D next_inflated(
    next_planning_grid.width(), next_planning_grid.height(), next_planning_grid.resolution(),
    next_planning_grid.origin_x(), next_planning_grid.origin_y());
  for (std::size_t row = 0; row < next_planning_grid.height(); ++row) {
    for (std::size_t col = 0; col < next_planning_grid.width(); ++col) {
      const barn_core::GridIndex idx{static_cast<int>(col), static_cast<int>(row)};
      const auto state = next_planning_grid.classify(idx);
      if (
        state == barn_core::CellState::kOccupied ||
        (state != barn_core::CellState::kUnknown && next_field.distance(idx) < inflation_radius_)) {
        next_inflated.set_log_odds(idx, 3.5);  // mark inflated cell occupied
      } else if (state == barn_core::CellState::kFree) {
        next_inflated.set_log_odds(idx, -2.0);
      }
    }
  }

  // Build RViz message from the inflated grid while we have it.
  nav_msgs::msg::OccupancyGrid inflated_msg;
  inflated_msg.header.stamp = rclcpp::Clock().now();
  inflated_msg.header.frame_id = frame_id_;
  inflated_msg.info.resolution = static_cast<float>(next_inflated.resolution());
  inflated_msg.info.width = static_cast<uint32_t>(next_inflated.width());
  inflated_msg.info.height = static_cast<uint32_t>(next_inflated.height());
  inflated_msg.info.origin.position.x = next_inflated.origin_x();
  inflated_msg.info.origin.position.y = next_inflated.origin_y();
  inflated_msg.data.resize(next_inflated.width() * next_inflated.height());
  for (std::size_t row = 0; row < next_inflated.height(); ++row) {
    for (std::size_t col = 0; col < next_inflated.width(); ++col) {
      const barn_core::GridIndex idx{static_cast<int>(col), static_cast<int>(row)};
      const auto state = next_inflated.classify(idx);
      int8_t cell_val = -1;  // unknown
      if (state == barn_core::CellState::kOccupied) {
        cell_val = 100;
      } else if (state == barn_core::CellState::kFree) {
        cell_val = 0;
      }
      inflated_msg.data[row * next_inflated.width() + col] = cell_val;
    }
  }

  bool needs_replan = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_ = std::make_shared<const barn_core::OccupancyGrid2D>(std::move(next));
    planning_grid_ =
      std::make_shared<const barn_core::OccupancyGrid2D>(std::move(next_planning_grid));
    inflated_planning_grid_ =
      std::make_shared<const barn_core::OccupancyGrid2D>(std::move(next_inflated));
    distance_field_ = std::make_shared<const barn_core::DistanceField2D>(std::move(next_field));
    have_map_ = true;

    // Validate against the raw planning grid, not the inflated one.
    // Binary inflation was closing off valid corridors at tight corners.
    if (
      global_path_.empty() ||
      !path_validator_.is_path_clear(global_path_, *planning_grid_, false)) {
      needs_replan = true;
    }
  }
  planning_grid_pub_->publish(inflated_msg);
  if (needs_replan) {
    request_plan();
  }
}

void ClassicalMpcNode::request_plan()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_requested_ = true;
  }
  planner_cv_.notify_one();
}

void ClassicalMpcNode::veto_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    veto_active_ = msg->data;
  }

  if (!msg->data) {
    // Shield cleared — reset counter so we don't replan on the next transient.
    consecutive_veto_count_ = 0;
    return;
  }

  std::lock_guard<std::mutex> control_lock(control_mutex_);
  // Only react when actively navigating (have a goal and a path).
  bool navigating = false;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan;
  barn_core::Pose2D current_pose;
  std::shared_ptr<const barn_core::DistanceField2D> field;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    navigating = have_goal_ && !global_path_.empty();
    latest_scan = scan_;
    current_pose = state_.pose;
    field = distance_field_;
  }
  if (!navigating || recovery_.active()) {
    consecutive_veto_count_ = 0;
    return;
  }
  ++consecutive_veto_count_;
  if (consecutive_veto_count_ >= veto_replan_threshold_) {
    RCLCPP_WARN(
      get_logger(),
      "Safety shield has vetoed %d consecutive commands — executing veto escape recovery",
      consecutive_veto_count_);
    consecutive_veto_count_ = 0;
    recovery_.trigger_veto_escape(recovery_context(current_pose, field, latest_scan, true));
    RCLCPP_INFO(get_logger(), "[Recovery] Triggered due to: safety_veto. Action taken: %s", to_string(recovery_.state()));
  }
}

void ClassicalMpcNode::planner_loop()
{
  while (true) {
    std::shared_ptr<const barn_core::OccupancyGrid2D> grid;
    std::shared_ptr<const barn_core::OccupancyGrid2D> planning_grid;
    barn_core::Pose2D pose;
    barn_core::Goal2D goal;
    std::uint64_t generation = 0;
    Path2D path_to_publish;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      planner_cv_.wait(lock, [this]() { return plan_requested_ || shutdown_planner_; });
      if (shutdown_planner_) {
        return;
      }
      plan_requested_ = false;
      if (!have_goal_ || !have_pose_ || !have_map_) {
        planner_status_ = "waiting_for_inputs";
        continue;
      }
      grid = grid_;
      // Use the raw planning grid for A* instead of the inflated one.
      // Soft clearance cost in A* replaces hard binary inflation.
      planning_grid = planning_grid_;
      pose = state_.pose;
      goal = goal_;
      generation = goal_generation_;
    }

    if (!planning_grid) {
      continue;
    }

    Path2D candidate;
    PlannerStats stats;
    
    // Path smoother / shortcut optimization:
    // If there is a direct line of sight to the goal, skip A* and just go straight.
    // The shortcut is restricted to the final approach: a long straight
    // sprint in the believed frame turns any residual frame rotation into a
    // large lateral miss at the goal, with no walls to funnel the robot back.
    barn_core::Pose2D los_goal{goal.x, goal.y, barn_core::wrap_angle(std::atan2(goal.y - pose.y, goal.x - pose.x))};
    if (enable_los_shortcut_ &&
      std::hypot(goal.x - pose.x, goal.y - pose.y) <= los_max_range_ &&
      swept_segment_is_clear(*planning_grid, pose, los_goal, global_planner_.params().footprint, false, planning_grid->resolution())) {
      const double dist = std::hypot(los_goal.x - pose.x, los_goal.y - pose.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.2)));
      for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        barn_core::Pose2D pt;
        pt.x = pose.x + t * (los_goal.x - pose.x);
        pt.y = pose.y + t * (los_goal.y - pose.y);
        pt.yaw = los_goal.yaw;
        candidate.push_back(pt);
      }
      is_los_path_ = true;
    } else {
      candidate = global_planner_.plan(*planning_grid, pose, goal);
      stats = global_planner_.stats();
      is_los_path_ = false;
    }
    bool accepted = !candidate.empty() && path_validator_.is_path_clear(candidate, *grid, false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      planner_ms_ = stats.elapsed_ms;
      planner_expanded_ = stats.expanded;
      if (generation != goal_generation_) {
        planner_status_ = "superseded";
        continue;
      }
      
      // Path stability: enforce a cooldown between path swaps to prevent
      // rapid oscillation near obstacles where small map changes invalidate
      // the first few path points (which the robot is already on top of).
      bool force_accept = global_path_.empty();
      if (!force_accept && last_path_swap_time_.nanoseconds() > 0) {
        const double since_swap = (now() - last_path_swap_time_).seconds();
        if (since_swap < path_cooldown_s_) {
          // Still in cooldown — check if path ahead is truly blocked
          // (skip first 5 points near robot to avoid false positives)
          bool ahead_blocked = false;
          if (global_path_.size() > 5 && grid_) {
            Path2D ahead(global_path_.begin() + 5, global_path_.end());
            ahead_blocked = !path_validator_.is_path_clear(ahead, *grid_, false);
          }
          if (!ahead_blocked) {
            // Path ahead is fine, keep it
            if (accepted) {
              planner_status_ = "retained_cooldown";
              replan_completed_ = true;
            }
            goto done;
          }
          // Path ahead is truly blocked — allow the swap despite cooldown
          force_accept = true;
        }
      }
      
      if (accepted) {
        global_path_ = std::move(candidate);
        path_to_publish = global_path_;
        planner_status_ = "success";
        replan_completed_ = true;
        last_path_swap_time_ = now();
      } else if (!global_path_.empty()) {
        planner_status_ = stats.timed_out ? "timeout_retaining_path" : "failed_retaining_path";
      } else {
        global_path_.clear();
        is_los_path_ = false;
        planner_status_ = stats.timed_out ? "timeout_dead_end" : "failed_dead_end";
      }
    }
    done:
    if (!path_to_publish.empty()) {
      global_path_pub_->publish(make_path_message(path_to_publish, now()));
    }
  }
}

void ClassicalMpcNode::local_plan_step()
{
  Path2D global;
  std::shared_ptr<const barn_core::OccupancyGrid2D> grid;
  std::shared_ptr<const barn_core::DistanceField2D> field;
  barn_core::Pose2D pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_ || !have_map_ || global_path_.empty()) {
      return;
    }
    pose = state_.pose;

    // Prune global_path_ by erasing points that the robot has already passed.
    std::size_t start = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < global_path_.size(); ++i) {
      const double distance = std::hypot(global_path_[i].x - pose.x, global_path_[i].y - pose.y);
      if (distance < best_distance) {
        best_distance = distance;
        start = i;
      }
    }
    if (start > 0) {
      global_path_.erase(global_path_.begin(), global_path_.begin() + start);
    }

    // Dynamic path smoother: continuously check if a direct line to the goal has opened up.
    if (planning_grid_) {
      barn_core::Pose2D los_goal{goal_.x, goal_.y, barn_core::wrap_angle(std::atan2(goal_.y - pose.y, goal_.x - pose.x))};
      if (enable_los_shortcut_ && !is_los_path_ &&
        std::hypot(goal_.x - pose.x, goal_.y - pose.y) <= los_max_range_ &&
        swept_segment_is_clear(*planning_grid_, pose, los_goal, global_planner_.params().footprint, false, planning_grid_->resolution())) {
        global_path_.clear();
        is_los_path_ = true;
        const double dist = std::hypot(los_goal.x - pose.x, los_goal.y - pose.y);
        const int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.2)));
        for (int i = 0; i <= steps; ++i) {
          double t = static_cast<double>(i) / steps;
          barn_core::Pose2D pt;
          pt.x = pose.x + t * (los_goal.x - pose.x);
          pt.y = pose.y + t * (los_goal.y - pose.y);
          pt.yaw = los_goal.yaw;
          global_path_.push_back(pt);
        }
      }
    }

    global = global_path_;
    grid = grid_;
    field = distance_field_;
  }
  auto local = local_planner_.plan(global, pose, *grid, field.get());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_trajectory_ = local;
    if (local.empty()) {
      plan_requested_ = true;
    }
  }
  if (local.empty()) {
    planner_cv_.notify_one();
  } else {
    local_path_pub_->publish(make_path_message(local, now()));
    global_path_pub_->publish(make_path_message(global, now()));
  }
}

barn_core::VelocityCommand ClassicalMpcNode::exploratory_command(
  const barn_core::Pose2D & pose, const barn_core::Goal2D & goal,
  const sensor_msgs::msg::LaserScan & scan) const
{
  const double goal_bearing =
    barn_core::wrap_angle(std::atan2(goal.y - pose.y, goal.x - pose.x) - pose.yaw);
  double selected = goal_bearing;
  double best = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double score =
      std::min(range, 3.0) - 0.9 * std::abs(barn_core::wrap_angle(angle - goal_bearing));
    if (score > best) {
      best = score;
      selected = angle;
    }
  }
  barn_core::VelocityCommand command;
  command.w = std::clamp(1.2 * selected, -0.5, 0.5);
  if (std::abs(selected) < 0.35) {
    command.v = startup_creep_speed_;
  }
  return command;
}

RecoveryContext ClassicalMpcNode::recovery_context(
  const barn_core::Pose2D & pose,
  const std::shared_ptr<const barn_core::DistanceField2D> & field,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan, bool veto_active) const
{
  RecoveryContext ctx;
  ctx.pose = pose;
  ctx.rotation_radius = rotation_clearance_m_;
  ctx.veto_active = veto_active;
  ctx.scan = scan_view(scan);
  ctx.breadcrumb = &breadcrumb_;
  ctx.clearance = field ? field->distance_world(pose.x, pose.y)
                        : std::numeric_limits<double>::infinity();
  return ctx;
}

void ClassicalMpcNode::control_step()
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  const auto stamp = now();
  barn_core::State2D state;
  barn_core::Goal2D goal;
  std::shared_ptr<const barn_core::OccupancyGrid2D> grid;
  std::shared_ptr<const barn_core::DistanceField2D> field;
  LocalTrajectory local;
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  bool have_goal = false;
  bool have_pose = false;
  bool have_odom = false;
  bool have_map = false;
  bool veto_active = false;
  rclcpp::Time goal_time(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = state_;
    goal = goal_;
    grid = grid_;
    field = distance_field_;
    local = local_trajectory_;
    scan = scan_;
    have_goal = have_goal_;
    have_pose = have_pose_;
    have_odom = have_odom_;
    have_map = have_map_;
    veto_active = veto_active_;
    goal_time = goal_received_time_;
    // Bug 5 fix: consume replan_completed_ under the same mutex_ that the
    // planner thread writes it, eliminating the data race with control_mutex_.
    if (replan_completed_) {
      recovery_.finish_replan();
      replan_completed_ = false;
    }
  }

  barn_core::VelocityCommand command{};
  std::string status = "waiting_for_goal_pose_scan_map";
  double mpc_ms = 0.0;
  double clearance = std::numeric_limits<double>::infinity();
  if (have_goal && have_pose && scan) {
    const double goal_distance = std::hypot(goal.x - state.pose.x, goal.y - state.pose.y);
    if (goal_distance <= goal_tolerance_) {
      status = "goal_reached";
      recovery_.reset();
      controller_.reset();
    } else if (recovery_.active()) {
      const auto ctx = recovery_context(state.pose, field, scan, veto_active);
      command = recovery_.step(0.033, ctx);
      // The no-progress watchdog only ticks in the MPC branch below, so its
      // timer goes stale while recovery runs (often several seconds of rotating
      // in place). Force a clean re-initialisation on the first MPC tick after
      // recovery ends; otherwise the stale timer instantly re-triggers recovery
      // and the robot flaps between the two states.
      progress_initialized_ = false;
      status = recovery_.state() == RecoveryState::kFailed ? "recovery_failed" : "recovery";
      // Issue the replan once per episode (recovery holds the RequestReplan
      // state across several ticks while the async planner runs).
      if (recovery_.request_replan() && !recovery_replan_issued_) {
        recovery_replan_issued_ = true;
        if (recovery_.is_clearance_replan()) {
          // Boost clearance for the retry, capped at 3x baseline so repeated
          // clearance-replans cannot ratchet the weight until A* refuses every
          // narrow BARN corridor.
          auto params = global_planner_.params();
          params.clearance_weight =
            std::min(params.clearance_weight * 1.5, base_clearance_weight_ * 3.0);
          global_planner_.set_params(params);
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          global_path_.clear();
          local_trajectory_.clear();
        }
        request_plan();
      }
    } else if (!have_map || !have_odom || local.empty()) {
      status = "waiting_for_initial_path";
      if ((stamp - goal_time).seconds() >= startup_creep_delay_s_) {
        command = exploratory_command(state.pose, goal, *scan);
        status = "startup_safety_creep";
      }
    } else {
      const auto mpc = controller_.control(local, state, *field);
      mpc_ms = mpc.solve_ms;
      status = mpc.status;
      if (mpc.success) {
        command = mpc.command;
        last_mpc_ = mpc;
        consecutive_mpc_failures_ = 0;
        prediction_pub_->publish(make_path_message(mpc.prediction, stamp));
      } else {
        ++consecutive_mpc_failures_;
        bool previous_is_safe = !last_mpc_.prediction.empty() &&
                                path_validator_.is_path_clear(last_mpc_.prediction, *grid, false);
        if (previous_is_safe && consecutive_mpc_failures_ == 1) {
          command = last_command_;
          status = "mpc_one_cycle_reuse";
        } else {
          command = {};
        }
        if (consecutive_mpc_failures_ >= 3) {
          recovery_.trigger(recovery_context(state.pose, field, scan, veto_active));
          status = "mpc_failure_recovery";
          RCLCPP_INFO(get_logger(), "[Recovery] Triggered due to: mpc_failure. Action taken: %s", to_string(recovery_.state()));
        }
      }
      clearance = field->distance_world(state.pose.x, state.pose.y);

      if (!progress_initialized_) {
        progress_pose_ = state.pose;
        last_progress_time_ = stamp;
        progress_initialized_ = true;
      } else if (
        std::hypot(state.pose.x - progress_pose_.x, state.pose.y - progress_pose_.y) > 0.18) {
        progress_pose_ = state.pose;
        last_progress_time_ = stamp;
        // Real ground covered — refund the recovery attempt budget so
        // max_recovery_attempts bounds *consecutive* failed episodes, not the
        // lifetime total. Without this, enough scattered recoveries latch
        // kFailed and the robot stops for the rest of the trial.
        recovery_.notify_progress();
        // Relax any clearance boost back to baseline now that we are moving.
        if (global_planner_.params().clearance_weight > base_clearance_weight_) {
          auto params = global_planner_.params();
          params.clearance_weight = base_clearance_weight_;
          global_planner_.set_params(params);
        }
        // Bug 3 fix: raise the speed guard to 0.25 m/s (matching startup_creep_speed_).
        // A command near 0.10-0.20 m/s is legitimately slow near obstacles; firing
        // recovery at that speed was causing spurious no-progress triggers.
      } else if (
        (command.v > 0.25 || (command.v < 0.08 && std::abs(command.w) < 0.15)) &&
        (stamp - last_progress_time_).seconds() > no_progress_timeout_s_) {
        recovery_.trigger(recovery_context(state.pose, field, scan, veto_active));
        status = "no_progress_recovery";
        RCLCPP_INFO(get_logger(), "[Recovery] Triggered due to: no_progress. Action taken: %s", to_string(recovery_.state()));
      }

      const int turn_sign = command.w > 0.35 ? 1 : (command.w < -0.35 ? -1 : 0);
      if (
        oscillation_window_start_.nanoseconds() == 0 ||
        (stamp - oscillation_window_start_).seconds() > 3.0) {
        oscillation_window_start_ = stamp;
        oscillation_count_ = 0;
      }
      if (turn_sign != 0 && last_turn_sign_ != 0 && turn_sign != last_turn_sign_) {
        ++oscillation_count_;
      }
      if (turn_sign != 0) {
        last_turn_sign_ = turn_sign;
      }
      // Bug 4 fix: only treat oscillation as pathological when the robot is
      // nearly stopped. Legitimate corridor turning at speed generates many
      // yaw-rate sign changes that would otherwise trip this counter.
      if (oscillation_count_ >= 4 && command.v < 0.25) {
        recovery_.trigger(recovery_context(state.pose, field, scan, veto_active));
        oscillation_count_ = 0;
        status = "oscillation_recovery";
      }
    }
  }

  // Allow the next episode to issue its replan once recovery has ended.
  if (!recovery_.active()) {
    recovery_replan_issued_ = false;
  }

  // Record the traversed trail (known-clear) for reverse recovery. Sample only
  // while navigating normally — during recovery we consume it in reverse, and
  // appending our own reverse motion would corrupt the trail.
  if (have_pose && !recovery_.active()) {
    if (breadcrumb_.empty() ||
      std::hypot(state.pose.x - breadcrumb_.back().x, state.pose.y - breadcrumb_.back().y) >=
      breadcrumb_spacing_m_)
    {
      breadcrumb_.push_back(state.pose);
      if (breadcrumb_.size() > breadcrumb_max_) {
        breadcrumb_.erase(breadcrumb_.begin());
      }
    }
  }

  // Reverse is reserved exclusively for the recovery state machine.
  if (!recovery_.active()) {
    command.v = std::max(0.0, command.v);
  }
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = stamp;
  output.header.frame_id = cmd_frame_;
  output.twist.linear.x = command.v;
  output.twist.angular.z = command.w;
  command_pub_->publish(output);
  last_command_ = command;
  publish_debug(stamp, command, status, mpc_ms, clearance);
}

nav_msgs::msg::Path ClassicalMpcNode::make_path_message(
  const Path2D & path, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id_;
  message.poses.reserve(path.size());
  for (const auto & pose : path) {
    geometry_msgs::msg::PoseStamped item;
    item.header = message.header;
    item.pose.position.x = pose.x;
    item.pose.position.y = pose.y;
    item.pose.orientation = yaw_quaternion(pose.yaw);
    message.poses.push_back(item);
  }
  return message;
}

nav_msgs::msg::Path ClassicalMpcNode::make_path_message(
  const LocalTrajectory & path, const rclcpp::Time & stamp) const
{
  Path2D poses;
  poses.reserve(path.size());
  for (const auto & point : path) {
    poses.push_back(point.pose);
  }
  return make_path_message(poses, stamp);
}

void ClassicalMpcNode::publish_debug(
  const rclcpp::Time & stamp, const barn_core::VelocityCommand & command,
  const std::string & control_status, double mpc_ms, double clearance)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "barn/classical_mpc";
  status.hardware_id = "jackal";
  status.level = recovery_.state() == RecoveryState::kFailed
                   ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                   : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = control_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status.values.push_back(key_value("planner_status", planner_status_));
    status.values.push_back(key_value("planner_ms", std::to_string(planner_ms_)));
    status.values.push_back(key_value("planner_expanded", std::to_string(planner_expanded_)));
  }
  status.values.push_back(key_value("mpc_ms", std::to_string(mpc_ms)));
  status.values.push_back(key_value("clearance_m", std::to_string(clearance)));
  status.values.push_back(key_value("selected_speed", std::to_string(command.v)));
  status.values.push_back(key_value("selected_yaw_rate", std::to_string(command.w)));
  status.values.push_back(
    key_value("recovery_state", std::to_string(static_cast<int>(recovery_.state()))));
  status.values.push_back(key_value("recovery_attempts", std::to_string(recovery_.attempts())));
  array.status.push_back(status);
  diagnostics_pub_->publish(array);

  barn_core::Pose2D pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose = state_.pose;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker footprint;
  footprint.header.stamp = stamp;
  footprint.header.frame_id = frame_id_;
  footprint.ns = "footprint";
  footprint.id = 0;
  footprint.type = visualization_msgs::msg::Marker::LINE_STRIP;
  footprint.action = visualization_msgs::msg::Marker::ADD;
  footprint.scale.x = 0.025;
  footprint.color.r = 0.1f;
  footprint.color.g = 0.9f;
  footprint.color.b = 0.2f;
  footprint.color.a = 1.0f;
  constexpr double hx = 0.254 + 0.04;
  constexpr double hy = 0.2159 + 0.04;
  const double ct = std::cos(pose.yaw);
  const double st = std::sin(pose.yaw);
  for (const auto & corner : std::array<std::pair<double, double>, 5>{
         {{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}, {hx, hy}}}) {
    geometry_msgs::msg::Point point;
    point.x = pose.x + ct * corner.first - st * corner.second;
    point.y = pose.y + st * corner.first + ct * corner.second;
    footprint.points.push_back(point);
  }
  markers.markers.push_back(footprint);

  visualization_msgs::msg::Marker obstacle_constraints;
  obstacle_constraints.header = footprint.header;
  obstacle_constraints.ns = "mpc_obstacle_constraints";
  obstacle_constraints.id = 2;
  obstacle_constraints.type = visualization_msgs::msg::Marker::POINTS;
  obstacle_constraints.action = visualization_msgs::msg::Marker::ADD;
  obstacle_constraints.scale.x = 0.035;
  obstacle_constraints.scale.y = 0.035;
  obstacle_constraints.color.r = 1.0f;
  obstacle_constraints.color.g = 0.15f;
  obstacle_constraints.color.b = 0.05f;
  obstacle_constraints.color.a = 0.85f;
  // These are the eight oriented footprint samples constrained at every MPC
  // prediction step (corners plus edge midpoints).
  for (const auto & predicted : last_mpc_.prediction) {
    const double pc = std::cos(predicted.yaw);
    const double ps = std::sin(predicted.yaw);
    for (const auto & sample : std::array<std::pair<double, double>, 8>{
           {{hx, hy},
            {hx, -hy},
            {-hx, hy},
            {-hx, -hy},
            {hx, 0.0},
            {-hx, 0.0},
            {0.0, hy},
            {0.0, -hy}}}) {
      geometry_msgs::msg::Point point;
      point.x = predicted.x + pc * sample.first - ps * sample.second;
      point.y = predicted.y + ps * sample.first + pc * sample.second;
      obstacle_constraints.points.push_back(point);
    }
  }
  markers.markers.push_back(obstacle_constraints);

  visualization_msgs::msg::Marker recovery_target;
  recovery_target.header = footprint.header;
  recovery_target.ns = "recovery_target";
  recovery_target.id = 1;
  recovery_target.type = visualization_msgs::msg::Marker::ARROW;
  recovery_target.action = visualization_msgs::msg::Marker::ADD;
  recovery_target.pose.position.x = pose.x;
  recovery_target.pose.position.y = pose.y;
  recovery_target.pose.orientation = yaw_quaternion(recovery_.target_yaw());
  recovery_target.scale.x = recovery_.active() ? 0.8 : 0.0;
  recovery_target.scale.y = 0.08;
  recovery_target.scale.z = 0.08;
  recovery_target.color.r = 1.0f;
  recovery_target.color.g = 0.4f;
  recovery_target.color.a = 1.0f;
  markers.markers.push_back(recovery_target);

  // Breadcrumb trail (the known-clear path the reverse recovery follows back).
  // Safe to read here: publish_debug runs inside control_step's control_mutex_.
  visualization_msgs::msg::Marker crumbs;
  crumbs.header = footprint.header;
  crumbs.ns = "breadcrumb";
  crumbs.id = 3;
  crumbs.type = visualization_msgs::msg::Marker::LINE_STRIP;
  crumbs.scale.x = 0.03;
  crumbs.color.r = 0.2f;
  crumbs.color.g = 0.5f;
  crumbs.color.b = 1.0f;
  crumbs.color.a = 0.8f;
  if (breadcrumb_.size() >= 2) {
    crumbs.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & crumb : breadcrumb_) {
      geometry_msgs::msg::Point point;
      point.x = crumb.x;
      point.y = crumb.y;
      crumbs.points.push_back(point);
    }
  } else {
    crumbs.action = visualization_msgs::msg::Marker::DELETE;
  }
  markers.markers.push_back(crumbs);

  marker_pub_->publish(markers);
}

}  // namespace barn_classical

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  auto node = std::make_shared<barn_classical::ClassicalMpcNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
