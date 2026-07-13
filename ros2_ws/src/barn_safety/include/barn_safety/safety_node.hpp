// Copyright 2026 barn-2027-prep contributors. MIT License.

#ifndef BARN_SAFETY__SAFETY_NODE_HPP_
#define BARN_SAFETY__SAFETY_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "barn_safety/limiter.hpp"
#include "barn_safety/swept_footprint_shield.hpp"

namespace barn_safety
{

class SafetyNode : public rclcpp::Node
{
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void desired_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void watchdog();
  bool sensors_fresh(const rclcpp::Time & stamp) const;
  void publish_status(const ShieldResult & result, const rclcpp::Time & stamp);
  void publish_zero(const rclcpp::Time & stamp, const std::string & reason);

  std::string desired_topic_;
  std::string safe_topic_;
  std::string scan_topic_;
  std::string base_frame_;
  double cmd_timeout_s_{0.4};
  double scan_timeout_s_{0.5};
  double watchdog_period_s_{0.05};

  Limiter limiter_;
  SweptFootprintShield shield_;
  std::vector<ObstaclePoint> obstacle_points_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  // Publishes true when a veto is active so planners can request a replan.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr veto_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_accept_time_;
  bool have_cmd_{false};
  bool have_valid_scan_{false};
  std::string last_reason_{"startup_no_scan"};
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__SAFETY_NODE_HPP_
