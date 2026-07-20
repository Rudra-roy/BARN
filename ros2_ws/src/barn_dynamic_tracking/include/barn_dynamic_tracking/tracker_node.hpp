// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Ties clustering -> association -> per-track KF -> TTC together and publishes
// dynamic-obstacle tracks for the hybrid risk gate. Track C consumes this;
// Track B (end-to-end RL baseline) does not need it.

#ifndef BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_
#define BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "barn_dynamic_tracking/kalman.hpp"
#include "barn_msgs/msg/obstacle_track_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace barn_dynamic_tracking
{

struct Track
{
  int id{0};
  int hits{0};             ///< consecutive/total successful updates
  int misses{0};           ///< consecutive frames with no matching cluster
  double radius{0.0};      ///< smoothed obstacle radius (m)
  ConstantVelocityKF1D x;  ///< tracks centroid x in the output frame
  ConstantVelocityKF1D y;  ///< tracks centroid y in the output frame
};

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_timer();
  void publish_tracks();
  void publish_markers();

  // Parameters.
  std::string scan_topic_;
  std::string pose_topic_;
  std::string tracks_topic_;
  std::string markers_topic_;
  std::string output_frame_;
  double publish_rate_hz_{10.0};
  double cluster_distance_threshold_{0.3};
  double association_gate_distance_{0.5};
  double process_noise_q_{1.0};
  double measurement_noise_r_{0.05};
  int min_hits_{3};
  int max_misses_{5};

  // Latest robot pose (in output_frame_).
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  bool have_pose_{false};

  // Latest scan (kept alive so the ScanView stays valid while we cluster it).
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  std::vector<Track> tracks_;
  int next_track_id_{0};
  rclcpp::Time last_update_time_;
  bool have_last_time_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<barn_msgs::msg::ObstacleTrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace barn_dynamic_tracking

#endif  // BARN_DYNAMIC_TRACKING__TRACKER_NODE_HPP_
