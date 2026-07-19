// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_mapping/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

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
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
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
  max_integration_yaw_rate_ = declare_parameter<double>("max_integration_yaw_rate", 0.0);
  rotation_decay_boost_ = declare_parameter<double>("rotation_decay_boost", 0.0);
  scan_match_enable_ = declare_parameter<bool>("scan_match_enable", true);
  scan_match_yaw_window_ = declare_parameter<double>("scan_match_yaw_window", 0.06);
  scan_match_yaw_step_ = declare_parameter<double>("scan_match_yaw_step", 0.005);
  scan_match_xy_window_ = declare_parameter<double>("scan_match_xy_window", 0.04);
  scan_match_min_score_gain_ = declare_parameter<double>("scan_match_min_score_gain", 0.75);
  scan_match_min_points_ = static_cast<int>(declare_parameter<int>("scan_match_min_points", 30));
  corr_budget_regen_ = declare_parameter<double>("scan_match_max_yaw_rate", 0.25);
  corr_budget_cap_ = declare_parameter<double>("scan_match_yaw_burst", 0.50);
  corr_rate_budget_ = corr_budget_cap_;

  if (resolution_ <= 0.0 || width_m_ <= 0.0 || height_m_ <= 0.0 ||
    publish_rate_hz_ <= 0.0 || max_usable_range_ <= 0.0)
  {
    throw std::invalid_argument("mapping dimensions, range, and rates must be positive");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).transient_local());
  correction_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    declare_parameter<std::string>("correction_topic", "/barn/odom_correction"),
    rclcpp::QoS(1).transient_local());
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

  // Both transforms are looked up at the scan's own stamp. Mixing the scan
  // with any other time base (e.g. the latest /barn/pose) rotates endpoints
  // by yaw_rate * dt and paints ghost obstacles while the robot spins.
  geometry_msgs::msg::TransformStamped sensor_tf;
  geometry_msgs::msg::TransformStamped base_tf;
  try {
    sensor_tf = tf_buffer_->lookupTransform(
      frame_id_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
      tf2::durationFromSec(0.05));
    base_tf = tf_buffer_->lookupTransform(
      frame_id_, base_frame_, rclcpp::Time(msg->header.stamp),
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
  const auto & robot_q = base_tf.transform.rotation;
  const double robot_yaw = std::atan2(
    2.0 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y),
    1.0 - 2.0 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z));
  const double robot_c = std::cos(robot_yaw);
  const double robot_s = std::sin(robot_yaw);
  const double robot_x = base_tf.transform.translation.x;
  const double robot_y = base_tf.transform.translation.y;

  // Measure the rotation rate from the same TF used to project the scan.
  // Odometry yaw drifts while the robot spins in place, so publish_grid()
  // uses this rate to age pre-rotation evidence faster (rotation_decay_boost);
  // the current scan is always painted, keeping map and pose consistent.
  const rclcpp::Time scan_stamp(msg->header.stamp);
  if (have_last_scan_yaw_) {
    const double dt = (scan_stamp - last_scan_stamp_).seconds();
    if (dt > 1e-4) {
      double dyaw = robot_yaw - last_scan_yaw_;
      dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
      current_yaw_rate_.store(std::abs(dyaw / dt));
    }
  }
  last_scan_yaw_ = robot_yaw;
  last_scan_stamp_ = scan_stamp;
  have_last_scan_yaw_ = true;
  // Optional hard gate, off by default: freezing the map while odom keeps
  // drifting makes the stale map visibly rotate against fresh scans.
  if (max_integration_yaw_rate_ > 0.0 &&
    current_yaw_rate_.load() > max_integration_yaw_rate_)
  {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Pausing map integration: yaw rate above %.2f rad/s", max_integration_yaw_rate_);
    return;
  }

  // Pass 1: project every beam in the RAW odom frame. The full 3-D sensor
  // rotation handles inverted (roll=PI) LiDAR mounts; the 2-D drift correction
  // is a rigid transform applied uniformly afterwards, so the body filter can
  // run on raw coordinates (relative geometry is invariant).
  struct RawBeam
  {
    double ex;
    double ey;
    bool hit;
  };
  std::vector<RawBeam> beams;
  beams.reserve(msg->ranges.size());
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
      const double dx = ex - robot_x;
      const double dy = ey - robot_y;
      const double body_x = robot_c * dx + robot_s * dy;
      const double body_y = -robot_s * dx + robot_c * dy;
      if (std::abs(body_x) <= 0.254 + 0.01 && std::abs(body_y) <= 0.2159 + 0.01) {
        continue;
      }
    }
    beams.push_back({ex, ey, finite_hit});
  }

  std::lock_guard<std::mutex> lock(grid_mutex_);

  // Register the scan against the map before integrating it: search a small
  // yaw window (about the robot, so the fix never translates it) for the
  // rotation that best re-aligns this scan's hits with existing occupied
  // evidence, and fold accepted corrections into the accumulated drift
  // correction. This absorbs the wheel-slip yaw drift the platform EKF bakes
  // into the odom frame during in-place rotation.
  if (scan_match_enable_) {
    const double prx = corr_cos_ * robot_x - corr_sin_ * robot_y + corr_x_;
    const double pry = corr_sin_ * robot_x + corr_cos_ * robot_y + corr_y_;

    std::size_t hit_count = 0;
    for (const auto & beam : beams) {
      hit_count += beam.hit ? 1u : 0u;
    }
    const std::size_t stride = std::max<std::size_t>(1, hit_count / 150);
    std::vector<std::pair<double, double>> offsets;
    offsets.reserve(hit_count / stride + 1);
    std::size_t hit_index = 0;
    for (const auto & beam : beams) {
      if (!beam.hit) {
        continue;
      }
      if (hit_index++ % stride != 0) {
        continue;
      }
      const double cx = corr_cos_ * beam.ex - corr_sin_ * beam.ey + corr_x_;
      const double cy = corr_sin_ * beam.ex + corr_cos_ * beam.ey + corr_y_;
      offsets.emplace_back(cx - prx, cy - pry);
    }

    if (static_cast<int>(offsets.size()) >= scan_match_min_points_) {
      // Bilinear occupancy sampling: cell-quantized scoring is blind to
      // sub-cell misalignment, which lets slow drift repaint the walls at the
      // drifted position before it ever becomes visible ("map dragging").
      // Interpolated scores give a gradient the matcher can track each scan.
      const double res = grid_.resolution();
      const auto sample_occupancy = [&](double wx, double wy) {
          const double gx = (wx - grid_.origin_x()) / res - 0.5;
          const double gy = (wy - grid_.origin_y()) / res - 0.5;
          const int c0 = static_cast<int>(std::floor(gx));
          const int r0 = static_cast<int>(std::floor(gy));
          const double fx = gx - c0;
          const double fy = gy - r0;
          double value = 0.0;
          for (int dr = 0; dr <= 1; ++dr) {
            for (int dc = 0; dc <= 1; ++dc) {
              const double v =
                std::max(0.0, grid_.log_odds(barn_core::GridIndex{c0 + dc, r0 + dr}));
              value += (dc ? fx : 1.0 - fx) * (dr ? fy : 1.0 - fy) * v;
            }
          }
          return value;
        };
      const auto score_at = [&](double dyaw, double dx, double dy, int * on_map) {
          const double cd = std::cos(dyaw);
          const double sd = std::sin(dyaw);
          double score = 0.0;
          int known = 0;
          for (const auto & o : offsets) {
            const double px = prx + dx + cd * o.first - sd * o.second;
            const double py = pry + dy + sd * o.first + cd * o.second;
            const double v = sample_occupancy(px, py);
            if (v > 0.05) {
              score += v;
              ++known;
            }
          }
          if (on_map != nullptr) {
            *on_map = known;
          }
          return score;
        };

      const double s0 = score_at(0.0, 0.0, 0.0, nullptr);
      double best = s0;
      double best_dyaw = 0.0;
      double best_dx = 0.0;
      double best_dy = 0.0;
      int best_known = 0;
      const double xy[3] = {-scan_match_xy_window_, 0.0, scan_match_xy_window_};
      for (double d = 0.0; d <= scan_match_yaw_window_ + 1e-9;
        d += scan_match_yaw_step_)
      {
        for (const double dyaw : {d, -d}) {
          for (const double dx : xy) {
            for (const double dy : xy) {
              if (dyaw == 0.0 && dx == 0.0 && dy == 0.0) {
                continue;
              }
              int known = 0;
              const double s = score_at(dyaw, dx, dy, &known);
              if (s > best) {
                best = s;
                best_dyaw = dyaw;
                best_dx = dx;
                best_dy = dy;
                best_known = known;
              }
            }
          }
          if (d == 0.0) {
            break;  // the -0.0 pass would duplicate the d == 0.0 candidates
          }
        }
      }
      if (have_match_stamp_) {
        const double budget_dt = (scan_stamp - last_match_stamp_).seconds();
        corr_rate_budget_ = std::min(
          corr_budget_cap_, corr_rate_budget_ + std::max(0.0, budget_dt) * corr_budget_regen_);
      }
      last_match_stamp_ = scan_stamp;
      have_match_stamp_ = true;

      if ((best_dyaw != 0.0 || best_dx != 0.0 || best_dy != 0.0) &&
        best_known >= scan_match_min_points_ &&
        best >= s0 + scan_match_min_score_gain_ &&
        std::abs(best_dyaw) <= corr_rate_budget_)
      {
        corr_rate_budget_ -= std::abs(best_dyaw);
        const double cd = std::cos(best_dyaw);
        const double sd = std::sin(best_dyaw);
        const double c_new = cd * corr_cos_ - sd * corr_sin_;
        const double s_new = sd * corr_cos_ + cd * corr_sin_;
        const double tx = corr_x_ - prx;
        const double ty = corr_y_ - pry;
        corr_x_ = cd * tx - sd * ty + prx + best_dx;
        corr_y_ = sd * tx + cd * ty + pry + best_dy;
        corr_cos_ = c_new;
        corr_sin_ = s_new;
      }
    }
  }

  // Broadcast the accumulated correction so the robot adapter republishes
  // /barn/pose and /barn/odom in the same drift-corrected frame as this map.
  geometry_msgs::msg::TransformStamped corr_msg;
  corr_msg.header.stamp = msg->header.stamp;
  corr_msg.header.frame_id = map_frame_;
  corr_msg.child_frame_id = frame_id_;
  corr_msg.transform.translation.x = corr_x_;
  corr_msg.transform.translation.y = corr_y_;
  const double half_theta = 0.5 * std::atan2(corr_sin_, corr_cos_);
  corr_msg.transform.rotation.z = std::sin(half_theta);
  corr_msg.transform.rotation.w = std::cos(half_theta);
  correction_pub_->publish(corr_msg);

  // Pass 2: integrate with the (possibly just-updated) correction applied.
  const double sx_c = corr_cos_ * sx - corr_sin_ * sy + corr_x_;
  const double sy_c = corr_sin_ * sx + corr_cos_ * sy + corr_y_;
  const auto start = grid_.world_to_cell(sx_c, sy_c);
  if (!grid_.in_bounds(start)) {
    return;
  }
  for (const auto & beam : beams) {
    const double ex = corr_cos_ * beam.ex - corr_sin_ * beam.ey + corr_x_;
    const double ey = corr_sin_ * beam.ex + corr_cos_ * beam.ey + corr_y_;
    const auto end = grid_.world_to_cell(ex, ey);
    integrate_ray(grid_, start, end, beam.hit && grid_.in_bounds(end), sensor_model_);
  }
}

void MappingNode::publish_grid()
{
  std::lock_guard<std::mutex> lock(grid_mutex_);

  // REP-105: broadcast map -> odom carrying the accumulated drift correction,
  // so TF consumers (RViz fixed frame 'map') render scans, the robot model,
  // and this grid consistently. Sent from startup (identity before the first
  // accepted match) so the frame always exists.
  geometry_msgs::msg::TransformStamped map_tf;
  map_tf.header.stamp = now();
  map_tf.header.frame_id = map_frame_;
  map_tf.child_frame_id = frame_id_;
  map_tf.transform.translation.x = corr_x_;
  map_tf.transform.translation.y = corr_y_;
  const double half_theta = 0.5 * std::atan2(corr_sin_, corr_cos_);
  map_tf.transform.rotation.z = std::sin(half_theta);
  map_tf.transform.rotation.w = std::cos(half_theta);
  tf_broadcaster_->sendTransform(map_tf);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 10000,
    "drift correction: yaw %+.2f deg, xy (%+.2f, %+.2f) m",
    std::atan2(corr_sin_, corr_cos_) * 180.0 / M_PI, corr_x_, corr_y_);

  if (!map_initialized_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = now();
  message.header.frame_id = map_frame_;

  distance_field_.rebuild(grid_);
  message.info.resolution = static_cast<float>(grid_.resolution());
  message.info.width = static_cast<uint32_t>(grid_.width());
  message.info.height = static_cast<uint32_t>(grid_.height());
  message.info.origin.position.x = grid_.origin_x();
  message.info.origin.position.y = grid_.origin_y();
  message.info.origin.orientation.w = 1.0;
  message.data.resize(grid_.width() * grid_.height());

  // Stale evidence goes invalid in proportion to rotation: yaw drift only
  // accumulates while the robot spins. Cells re-observed by the current scan
  // are refreshed at scan rate and easily outpace this decay.
  const double effective_decay =
    map_decay_rate_ + current_yaw_rate_.load() * rotation_decay_boost_;
  const double decay_per_tick = effective_decay / publish_rate_hz_;

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
