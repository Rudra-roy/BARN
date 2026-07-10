// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_classical/goal_seeker_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "barn_core/scan.hpp"

namespace barn_classical
{

GoalSeekerNode::GoalSeekerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("goal_seeker_node", options)
{
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/barn/goal");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/barn/pose");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/barn/cmd_desired");
  cmd_frame_ = declare_parameter<std::string>("cmd_frame", "base_link");
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  const double front_sector_deg = declare_parameter<double>("front_sector_deg", 60.0);
  front_sector_rad_ = front_sector_deg * M_PI / 180.0;

  GoalSeekerParams p;
  p.v_nominal = declare_parameter<double>("v_nominal", 0.5);
  p.v_max = declare_parameter<double>("v_max", 2.0);
  p.w_max = declare_parameter<double>("w_max", 1.2);
  p.k_ang = declare_parameter<double>("k_ang", 1.5);
  p.heading_tol = declare_parameter<double>("heading_tol", 0.35);
  p.stop_distance = declare_parameter<double>("stop_distance", 0.45);
  p.slow_distance = declare_parameter<double>("slow_distance", 1.2);
  p.goal_tolerance = declare_parameter<double>("goal_tolerance", 0.8);
  p.creep_fraction = declare_parameter<double>("creep_fraction", 0.15);
  seeker_.set_params(p);

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic_, 10);

  // The goal is latched; match with transient_local so we receive the single
  // publication even though we may start after the goal adapter.
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&GoalSeekerNode::goal_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&GoalSeekerNode::pose_callback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GoalSeekerNode::scan_callback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(period, std::bind(&GoalSeekerNode::control_step, this));

  RCLCPP_INFO(get_logger(), "goal_seeker_node ready at %.1f Hz", control_rate_hz_);
}

void GoalSeekerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_.x = msg->pose.position.x;
  goal_.y = msg->pose.position.y;
  goal_.tol = seeker_.params().goal_tolerance;
  have_goal_ = true;
  RCLCPP_INFO(get_logger(), "Goal received: (%.2f, %.2f)", goal_.x, goal_.y);
}

void GoalSeekerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pose_.x = msg->pose.position.x;
  pose_.y = msg->pose.position.y;
  const auto & q = msg->pose.orientation;
  pose_.yaw = barn_core::yaw_from_quat(q.x, q.y, q.z, q.w);
  have_pose_ = true;
}

void GoalSeekerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

double GoalSeekerNode::front_clearance() const
{
  if (!last_scan_) {
    // No scan yet: report clear so motion can begin at t0. barn_safety still
    // guards against real obstacles once scans arrive.
    return std::numeric_limits<double>::infinity();
  }
  barn_core::ScanView view;
  view.ranges = last_scan_->ranges.data();
  view.count = last_scan_->ranges.size();
  view.angle_min = last_scan_->angle_min;
  view.angle_increment = last_scan_->angle_increment;
  view.range_min = last_scan_->range_min;
  view.range_max = last_scan_->range_max;
  return barn_core::min_range_in_sector(view, -front_sector_rad_, front_sector_rad_);
}

void GoalSeekerNode::control_step()
{
  if (!have_goal_ || !have_pose_) {
    return;  // wait for the essentials only
  }
  const barn_core::VelocityCommand cmd = seeker_.compute(pose_, goal_, front_clearance());

  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now();
  out.header.frame_id = cmd_frame_;
  out.twist.linear.x = cmd.v;
  out.twist.angular.z = cmd.w;
  cmd_pub_->publish(out);
}

}  // namespace barn_classical

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_classical::GoalSeekerNode>());
  rclcpp::shutdown();
  return 0;
}
