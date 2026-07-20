// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_CLASSICAL__CLASSICAL_MPC_NODE_HPP_
#define BARN_CLASSICAL__CLASSICAL_MPC_NODE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "barn_classical/controller.hpp"
#include "barn_classical/global_planner_astar.hpp"
#include "barn_classical/local_planner.hpp"
#include "barn_classical/path_validator.hpp"
#include "barn_classical/recovery.hpp"
#include "barn_core/distance_field.hpp"
#include "barn_core/occupancy.hpp"
#include "barn_msgs/msg/obstacle_track_array.hpp"

namespace barn_classical
{

class ClassicalMpcNode : public rclcpp::Node
{
public:
  explicit ClassicalMpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ClassicalMpcNode() override;

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void veto_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void tracks_callback(const barn_msgs::msg::ObstacleTrackArray::SharedPtr msg);
  void local_plan_step();
  void control_step();
  void planner_loop();
  void request_plan();

  nav_msgs::msg::Path make_path_message(const Path2D & path, const rclcpp::Time & stamp) const;
  nav_msgs::msg::Path make_path_message(
    const LocalTrajectory & path, const rclcpp::Time & stamp) const;
  void publish_debug(
    const rclcpp::Time & stamp, const barn_core::VelocityCommand & command,
    const std::string & control_status, double mpc_ms, double clearance);
  barn_core::VelocityCommand exploratory_command(
    const barn_core::Pose2D & pose, const barn_core::Goal2D & goal,
    const sensor_msgs::msg::LaserScan & scan) const;
  // Assemble the per-tick recovery inputs (clearance, breadcrumb, gap scan).
  // Call sites hold control_mutex_, which serialises breadcrumb_ access.
  RecoveryContext recovery_context(
    const barn_core::Pose2D & pose,
    const std::shared_ptr<const barn_core::DistanceField2D> & field,
    const sensor_msgs::msg::LaserScan::SharedPtr & scan, bool veto_active) const;

  mutable std::mutex mutex_;
  std::mutex control_mutex_;
  std::condition_variable planner_cv_;
  std::thread planner_thread_;
  bool shutdown_planner_{false};
  bool plan_requested_{false};
  bool replan_completed_{false};
  std::uint64_t goal_generation_{0};

  barn_core::Goal2D goal_;
  barn_core::State2D state_;
  std::shared_ptr<const barn_core::OccupancyGrid2D> grid_;
  std::shared_ptr<const barn_core::OccupancyGrid2D> planning_grid_;
  std::shared_ptr<const barn_core::OccupancyGrid2D> inflated_planning_grid_;
  std::shared_ptr<const barn_core::DistanceField2D> distance_field_;
  Path2D global_path_;
  LocalTrajectory local_trajectory_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  bool have_goal_{false};
  bool have_pose_{false};
  bool have_odom_{false};
  bool have_map_{false};
  double planner_ms_{0.0};
  std::size_t planner_expanded_{0};
  std::string planner_status_{"waiting"};
  bool is_los_path_{false};
  bool enable_los_shortcut_{true};  ///< allow straight-to-goal paths on clear line of sight
  double los_max_range_{4.0};  ///< [m] LOS shortcut only inside this goal distance
  rclcpp::Time last_path_swap_time_;
  double path_cooldown_s_{2.0};

  GlobalPlannerAStar global_planner_;
  LocalPlanner local_planner_;
  Controller controller_;
  PathValidator path_validator_;
  Recovery recovery_;

  rclcpp::Time goal_received_time_;
  rclcpp::Time last_progress_time_;
  barn_core::Pose2D progress_pose_;
  // Known-clear trail of traversed poses (oldest -> newest), consumed in reverse
  // by the recovery reverse-to-clearance maneuver. Touched only under
  // control_mutex_ (control_step and veto_callback).
  std::vector<barn_core::Pose2D> breadcrumb_;
  bool progress_initialized_{false};
  int consecutive_mpc_failures_{0};
  int oscillation_count_{0};
  int last_turn_sign_{0};
  rclcpp::Time oscillation_window_start_;
  MpcResult last_mpc_;
  barn_core::VelocityCommand last_command_;

  double goal_tolerance_{0.70};
  double startup_creep_delay_s_{1.0};
  double startup_creep_speed_{0.15};
  double no_progress_timeout_s_{3.0};
  // Clearance the robot needs to rotate in place; below this, recovery reverses
  // instead of spinning. Breadcrumb sampling spacing and cap.
  double rotation_clearance_m_{0.40};
  double breadcrumb_spacing_m_{0.10};
  std::size_t breadcrumb_max_{160};
  // One-shot guard + baseline for the clearance-replan boost, so it fires once
  // per episode and is capped rather than compounding every control tick.
  bool recovery_replan_issued_{false};
  double base_clearance_weight_{0.0};
  double inflation_radius_{0.10};  // metres — inflated planning grid margin
  std::string frame_id_{"odom"};
  std::string cmd_frame_{"base_link"};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  // Safety-veto feedback: sustained vetoes trigger a replan so the global
  // planner can search for an alternative route around the blocked area.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr veto_sub_;
  // Dynamic-obstacle tracks (DynaBARN moving cylinders) feeding the MPC's
  // spatiotemporal keep-out constraints. Guarded by mutex_.
  rclcpp::Subscription<barn_msgs::msg::ObstacleTrackArray>::SharedPtr tracks_sub_;
  std::vector<DynamicObstacle> dynamic_obstacles_;
  rclcpp::Time tracks_stamp_{0, 0, RCL_ROS_TIME};
  bool enable_dynamic_obstacles_{true};
  double tracks_timeout_s_{0.5};
  int consecutive_veto_count_{0};
  int veto_replan_threshold_{0};  // populated from parameter in constructor
  bool veto_active_{false};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  // Inflated planning grid published for RViz visualisation.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_grid_pub_;
  rclcpp::TimerBase::SharedPtr local_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

}  // namespace barn_classical

#endif  // BARN_CLASSICAL__CLASSICAL_MPC_NODE_HPP_
