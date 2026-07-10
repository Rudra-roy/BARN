// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_mapping/mapping_node.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

namespace barn_mapping
{

MappingNode::MappingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mapping_node", options)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/barn/pose");
  grid_topic_ = declare_parameter<std::string>("grid_topic", "/barn/occupancy");
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  resolution_ = declare_parameter<double>("resolution", 0.05);
  width_m_ = declare_parameter<double>("width_m", 20.0);
  height_m_ = declare_parameter<double>("height_m", 12.0);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);

  const auto w = static_cast<std::size_t>(width_m_ / resolution_);
  const auto h = static_cast<std::size_t>(height_m_ / resolution_);
  // Origin is set at publish time to keep the robot centred; start at (0,0).
  grid_ = barn_core::OccupancyGrid2D(w, h, resolution_, -width_m_ / 2.0, -height_m_ / 2.0);

  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MappingNode::scan_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&MappingNode::pose_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&MappingNode::publish_grid, this));

  RCLCPP_INFO(
    get_logger(), "mapping_node (STUB): %zux%zu @ %.2fm publishing %s", w, h, resolution_,
    grid_topic_.c_str());
}

void MappingNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // STUB (M4): integrate rays into grid_ here using barn_core::InverseSensorModel.
  last_scan_ = msg;
}

void MappingNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pose_ = *msg;
}

void MappingNode::publish_grid()
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.info.resolution = static_cast<float>(grid_.resolution());
  msg.info.width = static_cast<uint32_t>(grid_.width());
  msg.info.height = static_cast<uint32_t>(grid_.height());
  msg.info.origin.position.x = grid_.origin_x();
  msg.info.origin.position.y = grid_.origin_y();
  msg.info.origin.orientation.w = 1.0;

  // All UNKNOWN (-1) until the M4 log-odds update is implemented.
  msg.data.assign(grid_.width() * grid_.height(), static_cast<int8_t>(-1));
  grid_pub_->publish(msg);
}

}  // namespace barn_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_mapping::MappingNode>());
  rclcpp::shutdown();
  return 0;
}
