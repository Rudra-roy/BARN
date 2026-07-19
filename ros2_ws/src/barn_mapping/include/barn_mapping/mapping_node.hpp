// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Online occupancy mapping from LiDAR. Subscribes the
// relayed scan and the robot pose, maintains a log-odds OccupancyGrid2D in the
// odom frame, and publishes it as nav_msgs/OccupancyGrid for the planner + RViz.
//
// It builds the map ONLY from allowed sensor data — never the ground-truth
// world map.

#ifndef BARN_MAPPING__MAPPING_NODE_HPP_
#define BARN_MAPPING__MAPPING_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "barn_core/distance_field.hpp"
#include "barn_core/logodds.hpp"
#include "barn_core/occupancy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace barn_mapping
{

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_grid();

  std::string scan_topic_;
  std::string pose_topic_;
  std::string grid_topic_;
  std::string frame_id_;
  std::string base_frame_;
  std::string map_frame_;
  double resolution_;
  double width_m_;
  double height_m_;
  double publish_rate_hz_;
  double max_usable_range_;
  double map_decay_rate_{0.0};
  double max_integration_yaw_rate_{0.0};
  double rotation_decay_boost_{1.5};
  barn_core::InverseSensorModel sensor_model_;

  // Scan-to-map yaw alignment. The platform EKF fuses absolute wheel-odometry
  // yaw, which slips during in-place rotation, so the odom frame itself drifts
  // in yaw. Each scan is registered against the map before integration and the
  // accumulated 2-D correction (raw odom -> map anchor frame) is published for
  // the robot adapter to apply to /barn/pose and /barn/odom.
  bool scan_match_enable_{true};
  double scan_match_yaw_window_{0.06};   ///< rad searched each side per scan
  double scan_match_yaw_step_{0.005};    ///< rad per candidate
  double scan_match_xy_window_{0.04};    ///< m searched each side per scan
  double scan_match_min_score_gain_{0.75};  ///< accept only clear improvements
  int scan_match_min_points_{30};        ///< matched hits required to trust
  double corr_cos_{1.0};
  double corr_sin_{0.0};
  double corr_x_{0.0};
  double corr_y_{0.0};
  // Divergence guard: wheel-slip drift is slow, so accepted yaw corrections
  // are budgeted (regen rad/s up to a burst cap); demand beyond the budget
  // indicates the matcher is chasing feature-poor geometry, not real slip.
  double corr_budget_regen_{0.25};
  double corr_budget_cap_{0.50};
  double corr_rate_budget_{0.50};
  rclcpp::Time last_match_stamp_;
  bool have_match_stamp_{false};

  // Yaw of the previous scan's base pose, used to measure the actual rotation
  // rate. Odometry yaw drifts while spinning, so map evidence painted before a
  // rotation ages in proportion to the rotation; publish_grid() decays faster
  // while current_yaw_rate_ is high.
  double last_scan_yaw_{0.0};
  rclcpp::Time last_scan_stamp_;
  bool have_last_scan_yaw_{false};
  std::atomic<double> current_yaw_rate_{0.0};

  barn_core::OccupancyGrid2D grid_;
  barn_core::DistanceField2D distance_field_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool have_pose_{false};
  bool map_initialized_{false};
  std::mutex grid_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr correction_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace barn_mapping

#endif  // BARN_MAPPING__MAPPING_NODE_HPP_
