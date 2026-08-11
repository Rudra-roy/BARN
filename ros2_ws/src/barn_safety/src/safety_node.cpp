// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_safety/safety_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/create_timer.hpp"
#include "tf2/exceptions.h"
#include "barn_core/geometry.hpp"
#include "std_msgs/msg/bool.hpp"

namespace barn_safety
{

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: Node("safety_node", options),
  last_cmd_time_(0, 0, RCL_ROS_TIME),
  last_scan_time_(0, 0, RCL_ROS_TIME),
  last_accept_time_(0, 0, RCL_ROS_TIME)
{
  desired_topic_ = declare_parameter<std::string>("desired_topic", "/barn/cmd_desired");
  safe_topic_ = declare_parameter<std::string>("safe_topic", "/barn/cmd_safe");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.4);
  scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.5);
  watchdog_period_s_ = declare_parameter<double>("watchdog_period_s", 0.05);

  barn_core::Limits limits;
  limits.v_max = declare_parameter<double>("v_max", 2.0);
  limits.w_max = declare_parameter<double>("w_max", 1.5);
  limits.a_lin = declare_parameter<double>("max_lin_accel", 2.5);
  limits.a_ang = declare_parameter<double>("max_ang_accel", 3.0);
  limiter_.set_limits(limits);

  ShieldParams shield_params;
  shield_params.emergency_margin = declare_parameter<double>("emergency_margin", 0.02);
  shield_params.latency_s = declare_parameter<double>("shield_latency_s", 0.15);
  shield_params.braking_decel = declare_parameter<double>("braking_decel", 2.5);
  shield_params.horizon_s = declare_parameter<double>("shield_horizon_s", 0.6);
  // Escape from a blocked direction (see swept_footprint_shield.hpp).
  //
  // DEFAULT OFF, deliberately. The mechanism is real -- the scale search cannot
  // change direction, so a blocked heading stops the robot even when rotating
  // is free, and that stranded world 216 for 75 s. But 12 trials on a clean
  // machine could not show it helping: 11/12 at score 0.3039 against 11/12 at
  // 0.3154 without it, with 28 escapes fired and 5 still-trapped events. It
  // costs time (each escape crawls at escape_speed) for no measurable
  // reliability gain, and it changes the shield from a filter that can only
  // reduce a command into one that can redirect it.
  //
  // Enable it for a properly powered comparison (30+ trials per arm on a
  // speed-bound world such as 216) before adopting it.
  shield_params.escape_enable = declare_parameter<bool>("shield_escape_enable", false);
  shield_params.escape_speed = declare_parameter<double>("shield_escape_speed", 0.15);
  shield_params.escape_yaw_rate = declare_parameter<double>("shield_escape_yaw_rate", 0.5);
  shield_params.escape_horizon_s = declare_parameter<double>("shield_escape_horizon_s", 0.5);
  // How long the robot must be at a dead stop before the shield is allowed to
  // move it. The freeze this breaks lasts 75+ s, so a wait here is free; firing
  // instantly is not, because startup and reset produce brief vetoes.
  escape_after_s_ = declare_parameter<double>("shield_escape_after_s", 1.5);
  shield_ = SweptFootprintShield(shield_params);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  safe_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(safe_topic_, 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/barn/navigation_diagnostics", 5);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/barn/safety_markers", 2);
  // Veto feedback: planners subscribe to this to trigger replanning when the
  // safety shield is blocking forward motion.
  veto_pub_ = create_publisher<std_msgs::msg::Bool>("/barn/safety_veto", rclcpp::QoS(1).best_effort());
  desired_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    desired_topic_, 10, std::bind(&SafetyNode::desired_callback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));
  watchdog_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(watchdog_period_s_),
    std::bind(&SafetyNode::watchdog, this));

  RCLCPP_INFO(
    get_logger(), "independent safety shield: %s -> %s", desired_topic_.c_str(), safe_topic_.c_str());
}

bool SafetyNode::sensors_fresh(const rclcpp::Time & stamp) const
{
  const bool cmd_ok = have_cmd_ && (stamp - last_cmd_time_).seconds() < cmd_timeout_s_;
  const bool scan_ok = have_valid_scan_ && (stamp - last_scan_time_).seconds() < scan_timeout_s_;
  return cmd_ok && scan_ok;
}

void SafetyNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      base_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.04));
  } catch (const tf2::TransformException & exception) {
    have_valid_scan_ = false;
    obstacle_points_.clear();
    last_reason_ = "scan_transform_unavailable";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", exception.what());
    return;
  }

  const auto & q = transform.transform.rotation;
  const double yaw = barn_core::yaw_from_quat(q.x, q.y, q.z, q.w);
  const double ct = std::cos(yaw);
  const double st = std::sin(yaw);
  obstacle_points_.clear();
  obstacle_points_.reserve(msg->ranges.size());
  for (std::size_t index = 0; index < msg->ranges.size(); ++index) {
    const double range = msg->ranges[index];
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }
    const double angle = msg->angle_min + static_cast<double>(index) * msg->angle_increment;
    const double laser_x = range * std::cos(angle);
    const double laser_y = range * std::sin(angle);
    const double base_x = transform.transform.translation.x + ct * laser_x - st * laser_y;
    const double base_y = transform.transform.translation.y + st * laser_x + ct * laser_y;
    // The official model's fenders are visible to its own LiDAR at ~7 cm.
    // Ignore only returns inside the physical body. The planning and emergency
    // margins remain outside this rectangle and still protect external objects.
    if (std::abs(base_x) <= 0.254 + 0.01 && std::abs(base_y) <= 0.2159 + 0.01) {
      continue;
    }
    obstacle_points_.push_back({base_x, base_y});
  }
  last_scan_time_ = get_clock()->now();
  have_valid_scan_ = true;
}

void SafetyNode::desired_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = get_clock()->now();
  double dt = have_cmd_ ? (stamp - last_accept_time_).seconds() : 0.0;
  last_cmd_time_ = stamp;
  have_cmd_ = true;
  const barn_core::VelocityCommand desired{msg->twist.linear.x, msg->twist.angular.z};
  barn_core::VelocityCommand limited = limiter_.apply(desired, dt, sensors_fresh(stamp));
  ShieldResult result;
  if (!sensors_fresh(stamp)) {
    result.command = {};
    result.reason = have_valid_scan_ ? "stale_scan_or_command" : last_reason_;
  } else {
    // Escape only after a sustained dead stop -- see shield_escape_after_s.
    const bool allow_escape = stopped_latched_ &&
      (stamp - stopped_since_).seconds() >= escape_after_s_;
    result = shield_.apply(limited, obstacle_points_, allow_escape);
  }

  // Latch on the SHIELD'S STATE, not on the magnitude of its output.
  //
  // This used to clear the latch whenever the output was non-zero. The escape's
  // own first command is non-zero, so it cleared the latch that authorised it:
  // next cycle allow_escape was false, the shield vetoed back to zero, and the
  // 1.5 s wait restarted. At 30 Hz the escape therefore emitted ONE 33 ms
  // command every 1.5 s -- about 5 mm of travel -- and could never leave the box.
  //
  // The earlier evaluation of the escape ("11/12 either way, median AT 16.2 ->
  // 18.3 s, 28 escapes fired, 5 still trapped") measured this defect rather than
  // the escape: 28 firings x 1.5 s of enforced waiting is most of the 2 s per
  // trial it appeared to cost. Any judgement about shield_escape_enable made
  // before this fix is void.
  //
  // Latching on the reason keeps the escape authorised for as long as the shield
  // is still blocking, so it runs continuously instead of twitching, and clears
  // as soon as the shield reports clear/braking_distance_clamp.
  const bool wants_motion = std::abs(desired.v) > 1e-6 || std::abs(desired.w) > 1e-6;
  const bool shield_blocking = result.reason == "emergency_veto" ||
    result.reason == "emergency_trapped" || result.reason == "emergency_escape";
  if (wants_motion && shield_blocking) {
    if (!stopped_latched_) {
      stopped_latched_ = true;
      stopped_since_ = stamp;
    }
  } else if (!shield_blocking) {
    // Only a genuinely unblocked shield clears the latch. A momentary zero from
    // the planner (reason "stationary") must not re-arm the 1.5 s wait.
    stopped_latched_ = false;
  }
  limiter_.override_last(result.command);
  last_accept_time_ = stamp;
  last_reason_ = result.reason;

  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = stamp;
  output.header.frame_id = base_frame_;
  output.twist.linear.x = result.command.v;
  output.twist.angular.z = result.command.w;
  safe_pub_->publish(output);

  // A shield that stops the robot dead used to say NOTHING in any log, so a
  // frozen trial looked identical to a planner that had simply given up --
  // 89 s of stationary robot with no diagnosis available afterwards. These two
  // lines are the difference between "reproduce it and instrument" and reading
  // the answer out of the trial log.
  // Deliberately NOT logging plain "emergency_veto": that is the routine stop in
  // front of an obstacle and fires constantly in a tight corridor. Only the two
  // states that can strand a trial are worth a warning.
  if (result.reason == "emergency_trapped") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "SHIELD TRAPPED: an obstacle is inside the veto box (signed clearance "
      "%.3f m, %zu returns) and no escape motion increases it. The robot is "
      "stopped and cannot move until the scene changes.",
      result.minimum_clearance, obstacle_points_.size());
  } else if (result.reason == "emergency_escape") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "shield escape: an obstacle is inside the veto box (signed clearance "
      "%.3f m); overriding the desired command with v=%.2f w=%.2f to back out.",
      result.minimum_clearance, result.command.v, result.command.w);
  }

  // Publish veto state so the planner can react without waiting for the
  // no-progress watchdog to expire. An escape is deliberately NOT a veto: the
  // robot is moving under shield control, and reporting a veto would make the
  // planner trigger recovery on top of the escape and fight it.
  std_msgs::msg::Bool veto_msg;
  veto_msg.data =
    (result.reason == "emergency_veto" || result.reason == "emergency_trapped");
  veto_pub_->publish(veto_msg);

  publish_status(result, stamp);
}

void SafetyNode::publish_zero(const rclcpp::Time & stamp, const std::string & reason)
{
  // Same reasoning as the veto logging in desired_callback: this is the other
  // way the robot stops dead, and it used to be completely silent. A stale scan
  // or a planner that stopped publishing looks exactly like a planner that
  // decided to hold still, and the trial log could not tell them apart.
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "safety stop (%s): publishing zero velocity.", reason.c_str());
  limiter_.reset();
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = stamp;
  stop.header.frame_id = base_frame_;
  safe_pub_->publish(stop);
  ShieldResult status;
  status.reason = reason;
  publish_status(status, stamp);
  last_reason_ = reason;
}

void SafetyNode::watchdog()
{
  const rclcpp::Time stamp = get_clock()->now();
  if (have_cmd_ && (stamp - last_cmd_time_).seconds() >= cmd_timeout_s_) {
    publish_zero(stamp, "command_stale_veto");
    have_cmd_ = false;
  } else if (have_cmd_ && (!have_valid_scan_ ||
    (stamp - last_scan_time_).seconds() >= scan_timeout_s_))
  {
    publish_zero(stamp, "scan_stale_veto");
  }
}

void SafetyNode::publish_status(const ShieldResult & result, const rclcpp::Time & stamp)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "barn/safety_shield";
  status.hardware_id = "jackal";
  status.message = result.reason;
  // "emergency_trapped" carries no "veto" substring but is strictly worse than
  // one, so match it explicitly rather than relying on the name.
  status.level = (result.reason.find("veto") != std::string::npos ||
    result.reason == "emergency_trapped") ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    (result.scale < 0.999 ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  diagnostic_msgs::msg::KeyValue reason;
  reason.key = "safety_veto_reason";
  reason.value = result.reason;
  status.values.push_back(reason);
  diagnostic_msgs::msg::KeyValue scale;
  scale.key = "command_scale";
  scale.value = std::to_string(result.scale);
  status.values.push_back(scale);
  diagnostic_msgs::msg::KeyValue clearance;
  clearance.key = "minimum_clearance";
  clearance.value = std::to_string(result.minimum_clearance);
  status.values.push_back(clearance);
  array.status.push_back(status);
  diagnostics_pub_->publish(array);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker envelope;
  envelope.header.stamp = stamp;
  envelope.header.frame_id = base_frame_;
  envelope.ns = "stopping_envelope";
  envelope.id = 0;
  envelope.type = visualization_msgs::msg::Marker::LINE_STRIP;
  envelope.action = visualization_msgs::msg::Marker::ADD;
  envelope.scale.x = 0.035;
  envelope.color.r = result.scale < 0.999 ? 1.0f : 0.1f;
  envelope.color.g = result.scale < 0.999 ? 0.2f : 0.8f;
  envelope.color.b = 0.1f;
  envelope.color.a = 1.0f;
  for (const auto & pose : result.envelope) {
    geometry_msgs::msg::Point point;
    point.x = pose.x;
    point.y = pose.y;
    envelope.points.push_back(point);
  }
  markers.markers.push_back(envelope);
  marker_pub_->publish(markers);
}

}  // namespace barn_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_safety::SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
