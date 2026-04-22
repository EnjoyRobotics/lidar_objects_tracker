/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#include <open3d/geometry/BoundingVolume.h>

#include <optional>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lidar_objects_tracker/lidar_objects_tracker.hpp"
#include "lidar_objects_tracker/lmb_tracker.hpp"

#include "tf2_eigen/tf2_eigen.hpp"

namespace lidar_objects_tracker
{

ObjectsTracker::ObjectsTracker(const rclcpp::NodeOptions & options)
: rclcpp::Node("objects_tracker", options)
{
  // Get parameters
  declare_parameter<std::string>("target_frame", "odom");
  target_frame_ = get_parameter("target_frame").as_string();

  declare_parameter<double>("cluster.neighbor_radius", 0.2);
  cluster_neighbor_radius_ = get_parameter("cluster.neighbor_radius").as_double();

  declare_parameter<int>("cluster.min_points", 6);
  cluster_min_points_ = get_parameter("cluster.min_points").as_int();

  declare_parameter<bool>("visualize", true);
  visualize_ = get_parameter("visualize").as_bool();

  declare_parameter<bool>("enable_static_bg_subtraction", false);
  enable_static_bg_subtraction_ = get_parameter("enable_static_bg_subtraction").as_bool();

  if (enable_static_bg_subtraction_) {
    static_bg_subtractor_ = std::make_unique<StaticBackgroundSubtractor>(*this);
    RCLCPP_INFO(get_logger(), "Static background subtraction enabled");
  }

  declare_parameter<bool>("publish_filtered_pcl", false);
  publish_filtered_pcl_ = get_parameter("publish_filtered_pcl").as_bool();
  if (publish_filtered_pcl_) {
    filtered_pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_pcl", 10);
  }

  declare_parameter<bool>("enable_radius_outlier_removal", false);
  enable_radius_outlier_removal_ = get_parameter("enable_radius_outlier_removal").as_bool();
  if (enable_radius_outlier_removal_) {
    declare_parameter<int>("radius_outlier_removal.min_points", 5);
    declare_parameter<double>("radius_outlier_removal.radius", 0.3);
    radius_outlier_removal_min_points_ =
      get_parameter("radius_outlier_removal.min_points").as_int();
    radius_outlier_removal_radius_ =
      get_parameter("radius_outlier_removal.radius").as_double();
    RCLCPP_INFO(
      get_logger(),
      "Radius outlier removal enabled (radius=%.3f, min_points=%d)",
      radius_outlier_removal_radius_, radius_outlier_removal_min_points_);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tracker_ = std::make_unique<LMBTracker>(*this);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    std::bind(&ObjectsTracker::scanCallback, this, std::placeholders::_1));

  tracked_objects_pub_ = create_publisher<lidar_objects_tracker_msgs::msg::TrackedObjects>(
    "tracked_objects", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "tracked_objects_markers", 10);

  if (visualize_) {
    declare_parameter<double>("visualization_period", 0.1);
    const double vis_period = get_parameter("visualization_period").as_double();
    visualization_timer_ = create_wall_timer(
      std::chrono::duration<double>(vis_period),
      std::bind(&ObjectsTracker::visualizationTimerCallback, this));
  }
}

void ObjectsTracker::scanCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  // Convert LaserScan to 2D points
  open3d::geometry::PointCloud pc = laserScanToPointCloud(msg);

  if (pc.points_.size() == 0) {
    return;
  }

  {  // Transform
    geometry_msgs::msg::TransformStamped tf_target_frame;
    try {
      tf_target_frame = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }
    Eigen::Matrix4d transform = tf2::transformToEigen(tf_target_frame.transform).matrix();
    pc = pc.Transform(transform);
    sensor_origin_ = transform.block<2, 1>(0, 3).cast<float>();
  }

  // Static background subtraction (in target frame)
  if (enable_static_bg_subtraction_) {
    // Collect positions of confirmed dynamic tracks from the previous update
    std::vector<Eigen::Vector2f> dynamic_positions;
    for (const auto & [id, track] : tracker_->getTracks()) {
      if (track.confirmed && track.dynamic) {
        dynamic_positions.emplace_back(track.kf->state(0), track.kf->state(1));
      }
    }

    std_msgs::msg::Header target_header = msg->header;
    target_header.frame_id = target_frame_;
    pc = static_bg_subtractor_->filter(pc, target_header, dynamic_positions, sensor_origin_);

    if (pc.points_.size() == 0) {
      return;
    }
  }

  // Radius outlier removal: remove points with too few neighbours within a radius
  if (enable_radius_outlier_removal_) {
    auto [filtered_pc, _] = pc.RemoveRadiusOutliers(
      radius_outlier_removal_min_points_, radius_outlier_removal_radius_);
    pc = std::move(*filtered_pc);

    if (pc.points_.size() == 0) {
      return;
    }
  }

  if (publish_filtered_pcl_) {
    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl_msg.header.frame_id = target_frame_;
    pcl_msg.header.stamp = msg->header.stamp;
    pcl_msg.height = 1;
    pcl_msg.width = static_cast<uint32_t>(pc.points_.size());
    pcl_msg.is_dense = true;
    pcl_msg.is_bigendian = false;

    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x"; field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32; field_x.count = 1;
    field_y.name = "y"; field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32; field_y.count = 1;
    field_z.name = "z"; field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32; field_z.count = 1;
    pcl_msg.fields = {field_x, field_y, field_z};
    pcl_msg.point_step = 12;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);

    for (size_t i = 0; i < pc.points_.size(); ++i) {
      const auto & pt = pc.points_[i];
      float fx = static_cast<float>(pt.x());
      float fy = static_cast<float>(pt.y());
      float fz = static_cast<float>(pt.z());
      std::memcpy(&pcl_msg.data[i * 12 + 0], &fx, 4);
      std::memcpy(&pcl_msg.data[i * 12 + 4], &fy, 4);
      std::memcpy(&pcl_msg.data[i * 12 + 8], &fz, 4);
    }

    filtered_pcl_pub_->publish(pcl_msg);
  }

  // Segment and calculate centroids
  const std::vector<open3d::geometry::PointCloud> clusters = segment(pc);
  std::vector<std::optional<open3d::geometry::AxisAlignedBoundingBox>> bboxes;
  std::vector<Eigen::Vector2f> centroids;
  for (const auto & cluster : clusters) {
    centroids.push_back(calculateCentroid(cluster));
    try {
      bboxes.push_back(cluster.GetAxisAlignedBoundingBox());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not compute bounding box for cluster: %s", e.what());
      bboxes.push_back(std::nullopt);
    }
  }

  // Update tracker
  tracker_->updateTracks(centroids);
  RCLCPP_DEBUG(
    get_logger(), "Track update: %zu total tracks",
    tracker_->getTracks().size());

  // Publish
  lidar_objects_tracker_msgs::msg::TrackedObjects tracked_objects_msg;
  const auto tracks = tracker_->getTracks();
  tracked_objects_msg.objects.reserve(tracks.size());
  for (const auto & [id, track] : tracks) {
    if (!track.confirmed) {
      continue;
    }
    lidar_objects_tracker_msgs::msg::TrackedObject tracked_object_msg;
    const Eigen::Vector4f & state = track.kf->state;
    tracked_object_msg.header.frame_id = target_frame_;
    tracked_object_msg.header.stamp = msg->header.stamp;
    tracked_object_msg.id = id;
    tracked_object_msg.position.x = state(0);
    tracked_object_msg.position.y = state(1);
    tracked_object_msg.velocity.x = state(2);
    tracked_object_msg.velocity.y = state(3);
    tracked_object_msg.confirmed = track.confirmed;
    tracked_object_msg.birth_time = track.birth_time;
    tracked_objects_msg.objects.push_back(tracked_object_msg);
  }
  tracked_objects_pub_->publish(tracked_objects_msg);

  // Cache visualization state for the timer
  if (visualize_) {
    vis_state_.centroids = centroids;
    vis_state_.bboxes = bboxes;
    vis_state_.tracks = tracks;
    vis_state_.sensor_origin = sensor_origin_;
    vis_state_.stamp = this->now();
    vis_state_.valid = true;
  }
}

void ObjectsTracker::visualizationTimerCallback()
{
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // Always send DELETEALL first so stale markers are cleared when data is absent
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array->markers.push_back(delete_all);

  // Check if data is stale (no scan received for more than 1 second)
  const rclcpp::Duration stale_threshold = rclcpp::Duration::from_seconds(1.0);
  const bool data_fresh = vis_state_.valid &&
    (now() - vis_state_.stamp) < stale_threshold;

  if (data_fresh) {
    const auto & centroids = vis_state_.centroids;
    const auto & bboxes = vis_state_.bboxes;
    const auto & tracks = vis_state_.tracks;
    const rclcpp::Time & stamp = vis_state_.stamp;

    // Sensor origin
    if (vis_state_.sensor_origin.has_value()) {
      visualization_msgs::msg::Marker marker_sensor;
      marker_sensor.header.frame_id = target_frame_;
      marker_sensor.header.stamp = stamp;
      marker_sensor.ns = "sensor_origin";
      marker_sensor.id = 0;
      marker_sensor.type = visualization_msgs::msg::Marker::SPHERE;
      marker_sensor.action = visualization_msgs::msg::Marker::ADD;
      marker_sensor.pose.position.x = vis_state_.sensor_origin->x();
      marker_sensor.pose.position.y = vis_state_.sensor_origin->y();
      marker_sensor.pose.position.z = 0.0;
      marker_sensor.pose.orientation.w = 1.0;
      marker_sensor.scale.x = 0.15;
      marker_sensor.scale.y = 0.15;
      marker_sensor.scale.z = 0.15;
      marker_sensor.color.r = 1.0;
      marker_sensor.color.g = 1.0;
      marker_sensor.color.b = 0.0;
      marker_sensor.color.a = 1.0;
      marker_array->markers.push_back(marker_sensor);
    }

    // Cluster centroids and bounding boxes
    visualization_msgs::msg::Marker marker_centroids;
    marker_centroids.header.frame_id = target_frame_;
    marker_centroids.header.stamp = stamp;
    marker_centroids.ns = "centroids";
    marker_centroids.id = 0;
    marker_centroids.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_centroids.action = visualization_msgs::msg::Marker::ADD;
    marker_centroids.scale.x = 0.2;
    marker_centroids.scale.y = 0.2;
    marker_centroids.scale.z = 0.01;
    marker_centroids.color.r = 1.0;
    marker_centroids.color.g = 0.0;
    marker_centroids.color.b = 0.0;
    marker_centroids.color.a = 0.5;

    visualization_msgs::msg::Marker default_bbox;
    default_bbox.header.frame_id = target_frame_;
    default_bbox.header.stamp = stamp;
    default_bbox.ns = "bounding_boxes";
    default_bbox.type = visualization_msgs::msg::Marker::LINE_LIST;
    default_bbox.action = visualization_msgs::msg::Marker::ADD;
    default_bbox.scale.x = 0.01;
    default_bbox.color.r = 1.0;
    default_bbox.color.g = 1.0;
    default_bbox.color.b = 0.8;
    default_bbox.color.a = 1.0;

    for (size_t i = 0; i < centroids.size(); ++i) {
      // Get centroid and bbox
      const auto & centroid = centroids[i];
      const auto & bbox = bboxes[i];

      // Centroid marker
      geometry_msgs::msg::Point p;
      p.x = centroid.x();
      p.y = centroid.y();
      p.z = 0.0;
      marker_centroids.points.push_back(p);

      // Bounding box marker
      if (bbox.has_value()) {
        visualization_msgs::msg::Marker bbox_marker = default_bbox;
        bbox_marker.id = static_cast<int>(i);
        const Eigen::Vector3d min_bound = bbox->min_bound_;
        const Eigen::Vector3d max_bound = bbox->max_bound_;
        std::vector<Eigen::Vector3d> corners(4);
        corners[0] = Eigen::Vector3d(min_bound.x(), min_bound.y(), 0.0);
        corners[1] = Eigen::Vector3d(max_bound.x(), min_bound.y(), 0.0);
        corners[2] = Eigen::Vector3d(max_bound.x(), max_bound.y(), 0.0);
        corners[3] = Eigen::Vector3d(min_bound.x(), max_bound.y(), 0.0);
        // Lines
        for (size_t j = 0; j < 4; ++j) {
          geometry_msgs::msg::Point p1, p2;
          p1.x = corners[j].x();
          p1.y = corners[j].y();
          p1.z = 0.0;
          p2.x = corners[(j + 1) % 4].x();
          p2.y = corners[(j + 1) % 4].y();
          p2.z = 0.0;
          bbox_marker.points.push_back(p1);
          bbox_marker.points.push_back(p2);
        }
        marker_array->markers.push_back(bbox_marker);
      }
    }
    marker_array->markers.push_back(marker_centroids);

    // Tracked objects
    for (const auto & [id, track] : tracks) {
      const Eigen::Vector4f & state = track.kf->state;
      const Eigen::Matrix4f & P = track.kf->covariance;

      if (!track.confirmed) {
        // For unconfirmed tracks, only show existence probability
        visualization_msgs::msg::Marker marker_id;
        marker_id.header.frame_id = target_frame_;
        marker_id.header.stamp = stamp;
        marker_id.ns = "unconfirmed_track_existence";
        marker_id.id = id;
        marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_id.action = visualization_msgs::msg::Marker::ADD;
        marker_id.scale.z = 0.1;
        marker_id.color.r = 1.0;
        marker_id.color.g = 1.0;
        marker_id.color.b = 1.0;
        marker_id.color.a = 1.0;
        marker_id.pose.position.x = state(0);
        marker_id.pose.position.y = state(1);
        marker_id.pose.position.z = 0.5;
        std::stringstream ss;
        ss << "exist_prob:" << std::fixed << std::setprecision(0) << track.existence_probability * 100.0f << "%";
        marker_id.text = ss.str();
        marker_array->markers.push_back(marker_id);

        continue;
      }
      // Size and alpha based on covariance
      const float pos_std = std::sqrt((P(0, 0) + P(1, 1)) / 2.0f);
      const float scale = pos_std;
      const float alpha = 1.0f - pos_std;

      visualization_msgs::msg::Marker marker_track;
      marker_track.header.frame_id = target_frame_;
      marker_track.header.stamp = stamp;
      marker_track.ns = "tracked_objects";
      marker_track.id = id;
      marker_track.type = visualization_msgs::msg::Marker::SPHERE;
      marker_track.action = visualization_msgs::msg::Marker::ADD;
      marker_track.scale.x = scale;
      marker_track.scale.y = scale;
      marker_track.scale.z = 0.01;
      marker_track.color.r = 0.0;
      marker_track.color.g = 1.0;
      marker_track.color.b = 0.0;
      marker_track.color.a = alpha;

      marker_track.pose.position.x = state(0);
      marker_track.pose.position.y = state(1);
      marker_track.pose.position.z = 0.0;
      marker_array->markers.push_back(marker_track);

      // Velocity arrow
      visualization_msgs::msg::Marker marker_velocity;
      marker_velocity.header.frame_id = target_frame_;
      marker_velocity.header.stamp = stamp;
      marker_velocity.ns = "tracked_object_velocities";
      marker_velocity.id = id;
      marker_velocity.type = visualization_msgs::msg::Marker::ARROW;
      marker_velocity.action = visualization_msgs::msg::Marker::ADD;
      marker_velocity.scale.x = 0.03;  // shaft diameter
      marker_velocity.scale.y = 0.06;  // head diameter
      marker_velocity.scale.z = 0.06;  // head length
      marker_velocity.color.r = 0.0;
      marker_velocity.color.g = 1.0;
      marker_velocity.color.b = 0.0;
      marker_velocity.color.a = alpha;
      geometry_msgs::msg::Point start_point;
      start_point.x = state(0);
      start_point.y = state(1);
      start_point.z = 0.0;
      marker_velocity.points.push_back(start_point);
      geometry_msgs::msg::Point end_point;
      end_point.x = state(0) + state(2) * 1.0f;  // 1 second ahead
      end_point.y = state(1) + state(3) * 1.0f;
      end_point.z = 0.0;
      marker_velocity.points.push_back(end_point);
      marker_array->markers.push_back(marker_velocity);

      // Add text marker for ID
      visualization_msgs::msg::Marker marker_id;
      marker_id.header.frame_id = target_frame_;
      marker_id.header.stamp = stamp;
      marker_id.ns = "tracked_object_ids";
      marker_id.id = id;
      marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker_id.action = visualization_msgs::msg::Marker::ADD;
      marker_id.scale.z = 0.1;
      marker_id.color.r = 1.0;
      marker_id.color.g = 1.0;
      marker_id.color.b = 1.0;
      marker_id.color.a = 1.0;
      marker_id.pose.position.x = state(0);
      marker_id.pose.position.y = state(1);
      marker_id.pose.position.z = 0.5;
      std::stringstream ss;
      ss << "#" << id << "\n";
      ss << "exist_prob:" << std::fixed << std::setprecision(0) << track.existence_probability * 100.0f << "%";
      marker_id.text = ss.str();
      marker_array->markers.push_back(marker_id);
    }

    // Unconfirmed tracks: existence probability text
    for (const auto & [id, track] : tracks) {
      if (track.confirmed) {
        continue;
      }
      const Eigen::Vector4f & state = track.kf->state;

      visualization_msgs::msg::Marker marker_unconfirmed_text;
      marker_unconfirmed_text.header.frame_id = target_frame_;
      marker_unconfirmed_text.header.stamp = stamp;
      marker_unconfirmed_text.ns = "unconfirmed_track_ids";
      marker_unconfirmed_text.id = id;
      marker_unconfirmed_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker_unconfirmed_text.action = visualization_msgs::msg::Marker::ADD;
      marker_unconfirmed_text.scale.z = 0.08;
      marker_unconfirmed_text.color.r = 1.0;
      marker_unconfirmed_text.color.g = 1.0;
      marker_unconfirmed_text.color.b = 1.0;
      marker_unconfirmed_text.color.a = 1.0;
      marker_unconfirmed_text.pose.position.x = state(0);
      marker_unconfirmed_text.pose.position.y = state(1);
      marker_unconfirmed_text.pose.position.z = 0.3;
      std::stringstream ss;
      ss << "exist_prob:" << std::fixed << std::setprecision(0) << track.existence_probability * 100.0f << "%";
      marker_unconfirmed_text.text = ss.str();
      marker_array->markers.push_back(marker_unconfirmed_text);
    }

    // Frozen disk and shadow from static background subtractor
    if (enable_static_bg_subtraction_) {
      std_msgs::msg::Header bg_header;
      bg_header.frame_id = target_frame_;
      bg_header.stamp = stamp;
      for (auto & m : static_bg_subtractor_->getFrozenMaskMarkers(bg_header)) {
        marker_array->markers.push_back(std::move(m));
      }
    }

    marker_pub_->publish(std::move(marker_array));
  }
}

open3d::geometry::PointCloud ObjectsTracker::laserScanToPointCloud(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  open3d::geometry::PointCloud pc;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float & range = msg->ranges[i];
    if (std::isfinite(range)) {
      float angle = msg->angle_min + i * msg->angle_increment;
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      pc.points_.emplace_back(Eigen::Vector3d(x, y, 0.0));
    }
  }

  return pc;
}

std::vector<open3d::geometry::PointCloud> ObjectsTracker::segment(
  const open3d::geometry::PointCloud & pc) const
{
  std::vector<int> labels = pc.ClusterDBSCAN(cluster_neighbor_radius_, cluster_min_points_);
  std::vector<open3d::geometry::PointCloud> clusters;
  int max_label = *std::max_element(labels.begin(), labels.end());
  clusters.resize(max_label + 1);
  for (size_t i = 0; i < labels.size(); ++i) {
    int label = labels[i];
    if (label != -1) {
      clusters[label].points_.push_back(pc.points_[i]);
    }
  }

  return clusters;
}

Eigen::Vector2f ObjectsTracker::calculateCentroid(const open3d::geometry::PointCloud & cluster)
{
  Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
  for (const auto & point : cluster.points_) {
    centroid.x() += static_cast<float>(point.x());
    centroid.y() += static_cast<float>(point.y());
  }

  centroid /= static_cast<float>(cluster.points_.size());
  return centroid;
}

}  // namespace lidar_objects_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_objects_tracker::ObjectsTracker)
