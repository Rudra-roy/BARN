// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_safety/safety_node.hpp"

#include <chrono>
#include <memory>

namespace barn_safety
{

using namespace std::chrono_literals;

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_node", options),
  last_cmd_time_(0, 0, RCL_ROS_TIME),
  last_scan_time_(0, 0, RCL_ROS_TIME),
  last_accept_time_(0, 0, RCL_ROS_TIME)
{
  desired_topic_ = declare_parameter<std::string>("desired_topic", "/barn/cmd_desired");
  safe_topic_ = declare_parameter<std::string>("safe_topic", "/barn/cmd_safe");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.4);
  scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.5);
  watchdog_period_s_ = declare_parameter<double>("watchdog_period_s", 0.05);

  barn_core::Limits limits;
  limits.v_max = declare_parameter<double>("v_max", 2.0);
  limits.w_max = declare_parameter<double>("w_max", 1.5);
  limits.a_lin = declare_parameter<double>("max_lin_accel", 2.5);
  limits.a_ang = declare_parameter<double>("max_ang_accel", 3.0);
  limiter_.set_limits(limits);

  safe_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(safe_topic_, 10);
  desired_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    desired_topic_, 10,
    std::bind(&SafetyNode::desired_callback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));

  watchdog_timer_ = create_wall_timer(
    std::chrono::duration<double>(watchdog_period_s_), std::bind(&SafetyNode::watchdog, this));

  RCLCPP_INFO(
    get_logger(), "safety_node: %s -> %s (v_max=%.2f w_max=%.2f)", desired_topic_.c_str(),
    safe_topic_.c_str(), limits.v_max, limits.w_max);
}

bool SafetyNode::sensors_fresh(const rclcpp::Time & now) const
{
  const bool cmd_ok = have_cmd_ && (now - last_cmd_time_).seconds() < cmd_timeout_s_;
  // Treat the scan as fresh until one is seen, so the slice can begin moving at
  // t0 before the first relayed scan arrives; once a scan is seen we enforce it.
  const bool scan_ok = (last_scan_time_.nanoseconds() == 0) ||
                       (now - last_scan_time_).seconds() < scan_timeout_s_;
  return cmd_ok && scan_ok;
}

void SafetyNode::desired_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time now = get_clock()->now();
  double dt = 0.0;
  if (have_cmd_) {
    dt = (now - last_accept_time_).seconds();
  }
  last_cmd_time_ = now;
  have_cmd_ = true;

  const barn_core::VelocityCommand desired{msg->twist.linear.x, msg->twist.angular.z};
  const barn_core::VelocityCommand safe = limiter_.apply(desired, dt, sensors_fresh(now));
  last_accept_time_ = now;

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now;
  out.header.frame_id = msg->header.frame_id;
  out.twist.linear.x = safe.v;
  out.twist.angular.z = safe.w;
  safe_pub_->publish(out);
}

void SafetyNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr)
{
  last_scan_time_ = get_clock()->now();
}

void SafetyNode::watchdog()
{
  const rclcpp::Time now = get_clock()->now();
  if (have_cmd_ && (now - last_cmd_time_).seconds() >= cmd_timeout_s_) {
    // Desired command went stale: command a hard zero and reset the limiter so
    // the next accepted command ramps from rest.
    limiter_.reset();
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now;
    safe_pub_->publish(stop);
  }
}

}  // namespace barn_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_safety::SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
