/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_
#define LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_

#include <open3d/geometry/PointCloud.h>

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "lidar_objects_tracker/tracker_base.hpp"
#include "lidar_objects_tracker/static_background_subtractor.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lidar_objects_tracker_msgs/msg/tracked_object.hpp"
#include "lidar_objects_tracker_msgs/msg/tracked_objects.hpp"

namespace lidar_objects_tracker
{

class ObjectsTracker : public rclcpp::Node
{
public:
  explicit ObjectsTracker(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);

  static open3d::geometry::PointCloud laserScanToPointCloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  std::vector<open3d::geometry::PointCloud> segment(
    const open3d::geometry::PointCloud & pc) const;
  static Eigen::Vector2f calculateCentroid(const open3d::geometry::PointCloud & cluster);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // TODO(redvinaa): publish static and dynamic scans?

  std::unique_ptr<StaticBackgroundSubtractor> static_bg_subtractor_;

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<lidar_objects_tracker_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcl_pub_;

  std::unique_ptr<TrackerBase> tracker_;

  // Parameters
  std::string target_frame_;
  double cluster_neighbor_radius_;
  size_t cluster_min_points_;
  bool visualize_;
  bool enable_static_bg_subtraction_;
  bool publish_filtered_pcl_;
  bool enable_radius_outlier_removal_;
  int radius_outlier_removal_min_points_;
  double radius_outlier_removal_radius_;
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_
