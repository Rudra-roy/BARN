// Copyright 2026 barn-2027-prep contributors. MIT License.
//
// Final command authority. Every track's desired command passes through here
// before reaching the robot adapter. Subscribes /barn/cmd_desired and /barn/scan,
// applies the Limiter, and publishes /barn/cmd_safe. A watchdog zeroes the
// output if the desired command goes stale. The RL policy must not bypass this.

#ifndef BARN_SAFETY__SAFETY_NODE_HPP_
#define BARN_SAFETY__SAFETY_NODE_HPP_

#include <memory>
#include <string>

#include "barn_safety/limiter.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

  bool sensors_fresh(const rclcpp::Time & now) const;

  std::string desired_topic_;
  std::string safe_topic_;
  std::string scan_topic_;
  double cmd_timeout_s_;
  double scan_timeout_s_;
  double watchdog_period_s_;

  Limiter limiter_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_accept_time_;
  bool have_cmd_{false};
};

}  // namespace barn_safety

#endif  // BARN_SAFETY__SAFETY_NODE_HPP_
