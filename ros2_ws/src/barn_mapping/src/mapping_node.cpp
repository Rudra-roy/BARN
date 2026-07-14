// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_mapping/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

#include "barn_mapping/ray_integrator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "tf2/time.h"

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
  width_m_ = declare_parameter<double>("width_m", 50.0);
  height_m_ = declare_parameter<double>("height_m", 30.0);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
  max_usable_range_ = declare_parameter<double>("max_usable_range", 12.0);
  sensor_model_.hit = declare_parameter<double>("log_odds_hit", 0.85);
  sensor_model_.miss = declare_parameter<double>("log_odds_miss", -0.40);
  sensor_model_.clamp_min = declare_parameter<double>("log_odds_min", -4.0);
  sensor_model_.clamp_max = declare_parameter<double>("log_odds_max", 4.0);
  map_decay_rate_ = declare_parameter<double>("map_decay_rate", 0.0);

  if (resolution_ <= 0.0 || width_m_ <= 0.0 || height_m_ <= 0.0 ||
    publish_rate_hz_ <= 0.0 || max_usable_range_ <= 0.0)
  {
    throw std::invalid_argument("mapping dimensions, range, and rates must be positive");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).transient_local());
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MappingNode::scan_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&MappingNode::pose_callback, this, std::placeholders::_1));

  // Follow simulation time. A wall timer can execute the same expensive map
  // publication repeatedly while Gazebo is advancing slowly, which further
  // starves the simulator and makes the evaluator's wall-clock timeout fire.
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&MappingNode::publish_grid, this));

  RCLCPP_INFO(
    get_logger(), "mapping_node: %.1fx%.1fm @ %.2fm from LiDAR only", width_m_, height_m_,
    resolution_);
}

void MappingNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pose_ = *msg;
  have_pose_ = true;
  if (!map_initialized_) {
    const auto width = static_cast<std::size_t>(std::ceil(width_m_ / resolution_));
    const auto height = static_cast<std::size_t>(std::ceil(height_m_ / resolution_));
    grid_ = barn_core::OccupancyGrid2D(
      width, height, resolution_, msg->pose.position.x - width_m_ / 2.0,
      msg->pose.position.y - height_m_ / 2.0);
    map_initialized_ = true;
    RCLCPP_INFO(
      get_logger(), "Map anchored at first pose (%.2f, %.2f), %zux%zu cells",
      msg->pose.position.x, msg->pose.position.y, width, height);
  }
}

void MappingNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!have_pose_ || !map_initialized_ || msg->ranges.empty() || msg->header.frame_id.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped sensor_tf;
  try {
    sensor_tf = tf_buffer_->lookupTransform(
      frame_id_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Dropping scan: %s <- %s unavailable: %s",
      frame_id_.c_str(), msg->header.frame_id.c_str(), error.what());
    return;
  }

  const auto & q = sensor_tf.transform.rotation;
  const double sx = sensor_tf.transform.translation.x;
  const double sy = sensor_tf.transform.translation.y;
  const auto & robot_q = last_pose_.pose.orientation;
  const double robot_yaw = std::atan2(
    2.0 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y),
    1.0 - 2.0 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z));
  const double robot_c = std::cos(robot_yaw);
  const double robot_s = std::sin(robot_yaw);

  std::lock_guard<std::mutex> lock(grid_mutex_);
  const auto start = grid_.world_to_cell(sx, sy);
  if (!grid_.in_bounds(start)) {
    return;
  }

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const float measured = msg->ranges[i];
    if (std::isnan(measured) || measured == -std::numeric_limits<float>::infinity() ||
      (std::isfinite(measured) &&
      (measured < msg->range_min || measured > msg->range_max)))
    {
      continue;
    }
    const bool finite_hit = std::isfinite(measured) && measured >= msg->range_min &&
      measured < msg->range_max && measured <= max_usable_range_;
    const double range = finite_hit ? static_cast<double>(measured) : max_usable_range_;
    if (!std::isfinite(range) || range < msg->range_min) {
      continue;
    }

    const double local_angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double lx = range * std::cos(local_angle);
    const double ly = range * std::sin(local_angle);
    
    // Apply full 3D rotation to handle inverted (roll=PI) LiDAR mounts correctly
    const double ex = sx + (1.0 - 2.0*(q.y*q.y + q.z*q.z))*lx + 2.0*(q.x*q.y - q.w*q.z)*ly;
    const double ey = sy + 2.0*(q.x*q.y + q.w*q.z)*lx + (1.0 - 2.0*(q.x*q.x + q.z*q.z))*ly;
    if (finite_hit) {
      const double dx = ex - last_pose_.pose.position.x;
      const double dy = ey - last_pose_.pose.position.y;
      const double body_x = robot_c * dx + robot_s * dy;
      const double body_y = -robot_s * dx + robot_c * dy;
      if (std::abs(body_x) <= 0.254 + 0.01 && std::abs(body_y) <= 0.2159 + 0.01) {
        continue;
      }
    }
    const auto end = grid_.world_to_cell(ex, ey);
    integrate_ray(grid_, start, end, finite_hit && grid_.in_bounds(end), sensor_model_);
  }
}

void MappingNode::publish_grid()
{
  if (!map_initialized_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = now();
  message.header.frame_id = frame_id_;

  std::lock_guard<std::mutex> lock(grid_mutex_);
  distance_field_.rebuild(grid_);
  message.info.resolution = static_cast<float>(grid_.resolution());
  message.info.width = static_cast<uint32_t>(grid_.width());
  message.info.height = static_cast<uint32_t>(grid_.height());
  message.info.origin.position.x = grid_.origin_x();
  message.info.origin.position.y = grid_.origin_y();
  message.info.origin.orientation.w = 1.0;
  message.data.resize(grid_.width() * grid_.height());

  const double decay_per_tick = map_decay_rate_ / publish_rate_hz_;

  for (std::size_t row = 0; row < grid_.height(); ++row) {
    for (std::size_t col = 0; col < grid_.width(); ++col) {
      const barn_core::GridIndex idx{static_cast<int>(col), static_cast<int>(row)};
      
      if (decay_per_tick > 0.0) {
        double current = grid_.log_odds(idx);
        if (current > 0.0) {
          grid_.set_log_odds(idx, std::max(0.0, current - decay_per_tick));
        } else if (current < 0.0) {
          grid_.set_log_odds(idx, std::min(0.0, current + decay_per_tick));
        }
      }

      const auto state = grid_.classify(idx);
      int8_t value = -1;
      if (state == barn_core::CellState::kFree) {
        value = 0;
      } else if (state == barn_core::CellState::kOccupied) {
        value = 100;
      }
      message.data[row * grid_.width() + col] = value;
    }
  }
  grid_pub_->publish(message);
}

}  // namespace barn_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_mapping::MappingNode>());
  rclcpp::shutdown();
  return 0;
}
