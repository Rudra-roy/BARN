// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M4). Online occupancy mapping from LiDAR. Subscribes the
// relayed scan and the robot pose, maintains a log-odds OccupancyGrid2D in the
// odom frame, and publishes it as nav_msgs/OccupancyGrid for the planner + RViz.
//
// The current implementation wires the interface and publishes an all-UNKNOWN
// grid of the configured size. The ray-tracing log-odds update (using
// barn_core::InverseSensorModel) is the M4 task. It must build the map ONLY
// from allowed sensor data — never the ground-truth world map.

#ifndef BARN_MAPPING__MAPPING_NODE_HPP_
#define BARN_MAPPING__MAPPING_NODE_HPP_

#include <memory>
#include <string>

#include "barn_core/occupancy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
  double resolution_;
  double width_m_;
  double height_m_;
  double publish_rate_hz_;

  barn_core::OccupancyGrid2D grid_;
  geometry_msgs::msg::PoseStamped last_pose_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace barn_mapping

#endif  // BARN_MAPPING__MAPPING_NODE_HPP_
