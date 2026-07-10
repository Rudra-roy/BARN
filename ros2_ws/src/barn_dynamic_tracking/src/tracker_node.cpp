// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/tracker_node.hpp"

#include <chrono>
#include <memory>

namespace barn_dynamic_tracking
{

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tracker_node", options)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/barn/pose");
  tracks_topic_ = declare_parameter<std::string>("tracks_topic", "/barn/tracks");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

  tracks_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(tracks_topic_, 1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TrackerNode::scan_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&TrackerNode::pose_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&TrackerNode::publish_tracks, this));

  RCLCPP_INFO(get_logger(), "tracker_node (STUB) publishing %s", tracks_topic_.c_str());
}

void TrackerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr)
{
  // STUB (M18): cluster_scan -> associate -> per-track KF predict/update.
}

void TrackerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr)
{
  // STUB: transform clusters into a stable frame using the robot pose.
}

void TrackerNode::publish_tracks()
{
  // STUB: publish an empty MarkerArray so the topic exists for downstream nodes.
  visualization_msgs::msg::MarkerArray markers;
  tracks_pub_->publish(markers);
}

}  // namespace barn_dynamic_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_dynamic_tracking::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
