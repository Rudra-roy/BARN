// Copyright 2026 barn-2027-prep contributors. MIT License.

#include "barn_dynamic_tracking/tracker_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "barn_core/geometry.hpp"
#include "barn_core/scan.hpp"
#include "barn_dynamic_tracking/association.hpp"
#include "barn_dynamic_tracking/clustering.hpp"
#include "barn_msgs/msg/obstacle_track.hpp"

namespace barn_dynamic_tracking
{

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tracker_node", options), last_update_time_(0, 0, RCL_ROS_TIME)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/barn/scan");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/barn/pose");
  tracks_topic_ = declare_parameter<std::string>("tracks_topic", "/barn/tracks");
  markers_topic_ = declare_parameter<std::string>("markers_topic", "/barn/track_markers");
  output_frame_ = declare_parameter<std::string>("output_frame", "odom");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  cluster_distance_threshold_ = declare_parameter<double>("cluster_distance_threshold", 0.3);
  association_gate_distance_ = declare_parameter<double>("association_gate_distance", 0.5);
  process_noise_q_ = declare_parameter<double>("process_noise_q", 1.0);
  measurement_noise_r_ = declare_parameter<double>("measurement_noise_r", 0.05);
  min_hits_ = declare_parameter<int>("min_hits", 3);
  max_misses_ = declare_parameter<int>("max_misses", 5);

  tracks_pub_ = create_publisher<barn_msgs::msg::ObstacleTrackArray>(tracks_topic_, 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TrackerNode::scan_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&TrackerNode::pose_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&TrackerNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(), "tracker_node publishing tracks on %s (frame '%s')",
    tracks_topic_.c_str(), output_frame_.c_str());
}

void TrackerNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

void TrackerNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pose_x_ = msg->pose.position.x;
  pose_y_ = msg->pose.position.y;
  pose_yaw_ = barn_core::yaw_from_quat(
    msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w);
  have_pose_ = true;
}

void TrackerNode::on_timer()
{
  // Determine the elapsed time since the last processed frame.
  const rclcpp::Time now = this->now();
  double dt = 1.0 / publish_rate_hz_;
  if (have_last_time_) {
    const double measured = (now - last_update_time_).seconds();
    if (measured > 1e-4 && measured < 5.0) {
      dt = measured;
    }
  }
  last_update_time_ = now;
  have_last_time_ = true;

  // Predict every existing track forward by dt regardless of new data.
  for (auto & t : tracks_) {
    t.x.predict(dt, process_noise_q_);
    t.y.predict(dt, process_noise_q_);
  }

  // Without a fresh scan and a pose we can only coast the existing tracks.
  if (latest_scan_ && have_pose_) {
    // Build a ScanView over a local copy of the ranges so the raw pointer stays
    // valid for the whole update (latest_scan_ also keeps the buffer alive).
    const std::vector<float> & ranges = latest_scan_->ranges;
    barn_core::ScanView view;
    view.ranges = ranges.data();
    view.count = ranges.size();
    view.angle_min = latest_scan_->angle_min;
    view.angle_increment = latest_scan_->angle_increment;
    view.range_min = latest_scan_->range_min;
    view.range_max = latest_scan_->range_max;

    const std::vector<Cluster> clusters =
      cluster_scan(view, cluster_distance_threshold_);

    // Transform each cluster centroid from the scan frame into the stable
    // output frame using the latest robot pose. Carry the world coordinates in
    // Cluster objects so association can gate against track positions.
    const double cos_yaw = std::cos(pose_yaw_);
    const double sin_yaw = std::sin(pose_yaw_);
    std::vector<Cluster> world_clusters = clusters;
    for (std::size_t i = 0; i < world_clusters.size(); ++i) {
      world_clusters[i].cx = pose_x_ + clusters[i].cx * cos_yaw - clusters[i].cy * sin_yaw;
      world_clusters[i].cy = pose_y_ + clusters[i].cx * sin_yaw + clusters[i].cy * cos_yaw;
    }

    // Associate clusters against the (already-predicted) track positions.
    std::vector<std::pair<double, double>> track_positions;
    track_positions.reserve(tracks_.size());
    for (const auto & t : tracks_) {
      track_positions.emplace_back(t.x.position(), t.y.position());
    }

    const std::vector<int> assignment =
      associate(world_clusters, track_positions, association_gate_distance_);

    std::vector<bool> track_updated(tracks_.size(), false);

    for (std::size_t i = 0; i < world_clusters.size(); ++i) {
      const double mx = world_clusters[i].cx;
      const double my = world_clusters[i].cy;
      const double meas_radius = clusters[i].radius;
      const int ti = assignment[i];

      if (ti >= 0) {
        Track & t = tracks_[static_cast<std::size_t>(ti)];
        t.x.update(mx, measurement_noise_r_);
        t.y.update(my, measurement_noise_r_);
        // Exponential moving average of the radius.
        constexpr double kAlpha = 0.3;
        t.radius = (1.0 - kAlpha) * t.radius + kAlpha * meas_radius;
        t.hits += 1;
        t.misses = 0;
        track_updated[static_cast<std::size_t>(ti)] = true;
      } else {
        Track t;
        t.id = next_track_id_++;
        t.x.init(mx);
        t.y.init(my);
        t.radius = meas_radius;
        t.hits = 1;
        t.misses = 0;
        tracks_.push_back(std::move(t));
      }
    }

    // Unmatched existing tracks accumulate a miss.
    for (std::size_t i = 0; i < track_updated.size(); ++i) {
      if (!track_updated[i]) {
        tracks_[i].misses += 1;
      }
    }
  } else {
    // No measurement this cycle: every track counts as missed.
    for (auto & t : tracks_) {
      t.misses += 1;
    }
  }

  // Prune stale tracks.
  std::vector<Track> survivors;
  survivors.reserve(tracks_.size());
  for (auto & t : tracks_) {
    if (t.misses <= max_misses_) {
      survivors.push_back(std::move(t));
    }
  }
  tracks_ = std::move(survivors);

  publish_tracks();
  publish_markers();
}

void TrackerNode::publish_tracks()
{
  barn_msgs::msg::ObstacleTrackArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = output_frame_;

  for (const auto & t : tracks_) {
    if (t.hits < min_hits_) {
      continue;
    }
    barn_msgs::msg::ObstacleTrack track;
    track.id = t.id;
    track.position.x = t.x.position();
    track.position.y = t.y.position();
    track.position.z = 0.0;
    track.velocity.x = t.x.velocity();
    track.velocity.y = t.y.velocity();
    track.velocity.z = 0.0;
    track.radius = t.radius;
    const double denom = static_cast<double>(t.hits + t.misses) + 1.0;
    double conf = static_cast<double>(t.hits) / denom;
    conf = barn_core::clamp(conf, 0.0, 1.0);
    track.confidence = conf;
    msg.tracks.push_back(track);
  }

  tracks_pub_->publish(msg);
}

void TrackerNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray markers;

  // Clear last cycle's markers before re-adding, so stale ids disappear.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = output_frame_;
  clear.header.stamp = this->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (const auto & t : tracks_) {
    if (t.hits < min_hits_) {
      continue;
    }

    visualization_msgs::msg::Marker cyl;
    cyl.header.frame_id = output_frame_;
    cyl.header.stamp = this->now();
    cyl.ns = "tracks";
    cyl.id = t.id;
    cyl.type = visualization_msgs::msg::Marker::CYLINDER;
    cyl.action = visualization_msgs::msg::Marker::ADD;
    cyl.pose.position.x = t.x.position();
    cyl.pose.position.y = t.y.position();
    cyl.pose.position.z = 0.0;
    cyl.pose.orientation.w = 1.0;
    cyl.scale.x = 2.0 * t.radius;
    cyl.scale.y = 2.0 * t.radius;
    cyl.scale.z = 0.3;
    cyl.color.r = 1.0f;
    cyl.color.g = 0.4f;
    cyl.color.b = 0.0f;
    cyl.color.a = 0.6f;
    markers.markers.push_back(cyl);

    // Velocity arrow.
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = output_frame_;
    arrow.header.stamp = this->now();
    arrow.ns = "track_velocity";
    arrow.id = t.id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start;
    start.x = t.x.position();
    start.y = t.y.position();
    start.z = 0.0;
    geometry_msgs::msg::Point end;
    end.x = t.x.position() + t.x.velocity();
    end.y = t.y.position() + t.y.velocity();
    end.z = 0.0;
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.scale.x = 0.05;  // shaft diameter
    arrow.scale.y = 0.1;   // head diameter
    arrow.scale.z = 0.1;   // head length
    arrow.color.r = 0.0f;
    arrow.color.g = 0.8f;
    arrow.color.b = 1.0f;
    arrow.color.a = 0.9f;
    markers.markers.push_back(arrow);
  }

  markers_pub_->publish(markers);
}

}  // namespace barn_dynamic_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<barn_dynamic_tracking::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
