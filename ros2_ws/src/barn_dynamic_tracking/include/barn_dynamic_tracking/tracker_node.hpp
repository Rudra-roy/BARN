// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// STUB (milestone M18). Ties clustering -> association -> per-track KF -> TTC
// together and publishes dynamic-obstacle tracks for the hybrid risk gate.
// Track C consumes this; Track B (end-to-end RL baseline) does not need it.

#ifndef BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_
#define BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "barn_dynamic_tracking/kalman.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace barn_dynamic_tracking
{

struct Track
{
  int id{0};
  ConstantVelocityKF1D x;  ///< tracks centroid x
  ConstantVelocityKF1D y;  ///< tracks centroid y
};

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_tracks();

  std::string scan_topic_;
  std::string pose_topic_;
  std::string tracks_topic_;
  double publish_rate_hz_;

  std::vector<Track> tracks_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_
