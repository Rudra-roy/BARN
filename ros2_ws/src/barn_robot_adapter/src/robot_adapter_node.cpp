// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_robot_adapter/robot_adapter_node.hpp"

#include <cmath>
#include <memory>

#include "barn_robot_adapter/conversions.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace barn_robot_adapter
{

RobotAdapterNode::RobotAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_adapter_node", options)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/front/scan");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "platform/odom/filtered");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  internal_scan_topic_ = declare_parameter<std::string>("internal_scan_topic", "/barn/scan");
  internal_pose_topic_ = declare_parameter<std::string>("internal_pose_topic", "/barn/pose");
  internal_odom_topic_ = declare_parameter<std::string>("internal_odom_topic", "/barn/odom");
  safe_cmd_topic_ = declare_parameter<std::string>("safe_cmd_topic", "/barn/cmd_safe");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  cmd_vel_type_ = declare_parameter<std::string>("cmd_vel_type", "twist_stamped");
  cmd_vel_frame_ = declare_parameter<std::string>("cmd_vel_frame", "base_link");
  corrected_frame_ = declare_parameter<std::string>("corrected_frame", "map");

  // Sensor QoS for the LiDAR relay; keep-last, best-effort matches drivers.
  const auto sensor_qos = rclcpp::SensorDataQoS();

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(internal_scan_topic_, sensor_qos);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(internal_pose_topic_, 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(internal_odom_topic_, 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, sensor_qos,
    std::bind(&RobotAdapterNode::scan_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 20, std::bind(&RobotAdapterNode::odom_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  correction_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
    declare_parameter<std::string>("correction_topic", "/barn/odom_correction"),
    rclcpp::QoS(1).transient_local(),
    std::bind(&RobotAdapterNode::correction_callback, this, std::placeholders::_1));

  // Egress: create exactly one publisher based on the configured wire type.
  if (cmd_vel_type_ == "twist") {
    cmd_unstamped_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  } else {
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
  }
  safe_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    safe_cmd_topic_, 10,
    std::bind(&RobotAdapterNode::safe_cmd_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "robot_adapter_node: %s->%s, %s(+TF)->%s, %s->%s [%s]", scan_topic_.c_str(),
    internal_scan_topic_.c_str(), odom_topic_.c_str(), internal_pose_topic_.c_str(),
    safe_cmd_topic_.c_str(), cmd_vel_topic_.c_str(), cmd_vel_type_.c_str());
}

void RobotAdapterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Relay verbatim. Navigation cores build their own ScanView from this.
  scan_pub_->publish(*msg);
}

void RobotAdapterNode::correction_callback(
  const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  // Planar correction: extract yaw from the (z, w)-only quaternion.
  const auto & rot = msg->transform.rotation;
  const double yaw = 2.0 * std::atan2(rot.z, rot.w);
  corr_cos_ = std::cos(yaw);
  corr_sin_ = std::sin(yaw);
  corr_x_ = msg->transform.translation.x;
  corr_y_ = msg->transform.translation.y;
}

void RobotAdapterNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  // Coordinates are drift-corrected below, so the label is the map frame.
  pose.header.frame_id = corrected_frame_;
  pose.header.stamp = msg->header.stamp;

  // Prefer the authoritative TF (odom -> base_link); fall back to the odom
  // message pose if the TF tree is not yet populated (race at startup). The
  // fallback keeps the goal-seeker moving at t0, which trips the evaluator's
  // >0.1 m motion clock promptly.
  try {
    const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF %s->%s unavailable (%s); using odom pose",
      odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    pose.pose = msg->pose.pose;
  }
  // Rotate/translate into the mapping node's drift-corrected frame so the
  // planner's pose, the latched goal, and the occupancy grid stay consistent.
  {
    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    pose.pose.position.x = corr_cos_ * px - corr_sin_ * py + corr_x_;
    pose.pose.position.y = corr_sin_ * px + corr_cos_ * py + corr_y_;
    const auto qin = pose.pose.orientation;
    const double half = 0.5 * std::atan2(corr_sin_, corr_cos_);
    const double zc = std::sin(half);
    const double wc = std::cos(half);
    pose.pose.orientation.w = wc * qin.w - zc * qin.z;
    pose.pose.orientation.x = wc * qin.x - zc * qin.y;
    pose.pose.orientation.y = wc * qin.y + zc * qin.x;
    pose.pose.orientation.z = wc * qin.z + zc * qin.w;
  }
  pose_pub_->publish(pose);

  // Preserve measured twist while making the pose agree with the pose relay.
  // Downstream predictive control therefore never subscribes around the robot
  // boundary to the evaluator-owned odometry topic.
  nav_msgs::msg::Odometry odom = *msg;
  odom.header.frame_id = corrected_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = pose.pose;
  odom_pub_->publish(odom);
}

void RobotAdapterNode::safe_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (cmd_unstamped_pub_) {
    cmd_unstamped_pub_->publish(msg->twist);
    return;
  }
  const auto cmd = from_twist(msg->twist);
  cmd_stamped_pub_->publish(to_twist_stamped(cmd, cmd_vel_frame_, now()));
}

}  // namespace barn_robot_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_robot_adapter::RobotAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
