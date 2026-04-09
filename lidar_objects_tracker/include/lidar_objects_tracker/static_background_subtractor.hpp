/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_
#define LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include <open3d/geometry/PointCloud.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace lidar_objects_tracker
{

/** @brief Header-only static background subtractor using a local occupancy grid.
 *
 * For each received scan, cells that were hit have their probability increased,
 * all other cells are decreased. Points that fall on cells above the threshold
 * are considered static background and are filtered out.
 *
 * Parameters (namespaced under "static_bg_subtractor"):
 *   - resolution        : grid cell size in metres (default 0.1)
 *   - hit_increment     : probability increase on a hit (default 0.1)
 *   - miss_decrement    : probability decrease when not hit (default 0.02)
 *   - threshold         : cells above this probability are treated as static (default 0.7)
 *   - publish_grid      : publish the occupancy grid for debug (default false)
 *   - grid_half_size    : half-width/height of the grid in metres (default 20.0)
 */
class StaticBackgroundSubtractor
{
public:
  explicit StaticBackgroundSubtractor(rclcpp::Node & node)
  : logger_(node.get_logger())
  {
    node.declare_parameter<double>("static_bg_subtractor.resolution", 0.1);
    node.declare_parameter<double>("static_bg_subtractor.hit_increment", 0.1);
    node.declare_parameter<double>("static_bg_subtractor.miss_decrement", 0.02);
    node.declare_parameter<double>("static_bg_subtractor.threshold", 0.7);
    node.declare_parameter<bool>("static_bg_subtractor.publish_grid", false);
    node.declare_parameter<double>("static_bg_subtractor.grid_half_size", 20.0);

    resolution_     = static_cast<float>(node.get_parameter("static_bg_subtractor.resolution").as_double());
    hit_increment_  = static_cast<float>(node.get_parameter("static_bg_subtractor.hit_increment").as_double());
    miss_decrement_ = static_cast<float>(node.get_parameter("static_bg_subtractor.miss_decrement").as_double());
    threshold_      = static_cast<float>(node.get_parameter("static_bg_subtractor.threshold").as_double());
    publish_grid_   = node.get_parameter("static_bg_subtractor.publish_grid").as_bool();
    grid_half_size_ = static_cast<float>(node.get_parameter("static_bg_subtractor.grid_half_size").as_double());

    // Grid dimensions: centred at origin
    grid_size_ = static_cast<int>(std::ceil(2.0f * grid_half_size_ / resolution_));
    grid_.assign(grid_size_ * grid_size_, 0.0f);

    if (publish_grid_) {
      grid_pub_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
        "static_background_grid", rclcpp::QoS(1).transient_local());
    }

    RCLCPP_INFO(
      logger_,
      "StaticBackgroundSubtractor: resolution=%.3f, grid=%dx%d cells (%.1fm x %.1fm)",
      resolution_, grid_size_, grid_size_,
      2.0f * grid_half_size_, 2.0f * grid_half_size_);
  }

  /** @brief Update the occupancy grid and return only non-static points.
   *
   * The input point cloud must already be in the sensor/local frame because
   * the grid is maintained in that same frame (robot-centric).
   *
   * @param pc     Input point cloud (local frame, z ignored)
   * @param header Header from the original LaserScan (for grid publication stamp / frame)
   * @return       Filtered point cloud with static background points removed
   */
  open3d::geometry::PointCloud filter(
    const open3d::geometry::PointCloud & pc,
    const std_msgs::msg::Header & header)
  {
    // --- 1. Mark which cells were hit this scan ---
    std::vector<bool> hit(grid_size_ * grid_size_, false);

    for (const auto & pt : pc.points_) {
      const int ix = worldToIndex(static_cast<float>(pt.x()));
      const int iy = worldToIndex(static_cast<float>(pt.y()));
      if (isValid(ix, iy)) {
        hit[cellIndex(ix, iy)] = true;
      }
    }

    // --- 2. Update probabilities for every cell ---
    for (int i = 0; i < grid_size_ * grid_size_; ++i) {
      if (hit[i]) {
        grid_[i] = std::min(1.0f, grid_[i] + hit_increment_);
      } else {
        grid_[i] = std::max(0.0f, grid_[i] - miss_decrement_);
      }
    }

    // --- 3. Filter out points whose cell is above threshold ---
    open3d::geometry::PointCloud filtered;
    for (const auto & pt : pc.points_) {
      const int ix = worldToIndex(static_cast<float>(pt.x()));
      const int iy = worldToIndex(static_cast<float>(pt.y()));
      if (!isValid(ix, iy) || grid_[cellIndex(ix, iy)] < threshold_) {
        filtered.points_.push_back(pt);
      }
    }

    // --- 4. Optionally publish grid ---
    if (publish_grid_) {
      publishGrid(header);
    }

    return filtered;
  }

private:
  // Convert world coordinate (metres) to grid index
  inline int worldToIndex(float world) const
  {
    return static_cast<int>(std::floor((world + grid_half_size_) / resolution_));
  }

  inline bool isValid(int ix, int iy) const
  {
    return ix >= 0 && ix < grid_size_ && iy >= 0 && iy < grid_size_;
  }

  inline int cellIndex(int ix, int iy) const
  {
    return iy * grid_size_ + ix;
  }

  void publishGrid(const std_msgs::msg::Header & header)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header = header;
    msg.info.resolution = resolution_;
    msg.info.width = static_cast<uint32_t>(grid_size_);
    msg.info.height = static_cast<uint32_t>(grid_size_);
    msg.info.origin.position.x = -grid_half_size_;
    msg.info.origin.position.y = -grid_half_size_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_size_ * grid_size_);
    for (int i = 0; i < grid_size_ * grid_size_; ++i) {
      // OccupancyGrid uses [0, 100], -1 = unknown
      msg.data[i] = static_cast<int8_t>(grid_[i] * 100.0f);
    }

    grid_pub_->publish(msg);
  }

  rclcpp::Logger logger_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  float resolution_;
  float hit_increment_;
  float miss_decrement_;
  float threshold_;
  bool publish_grid_;
  float grid_half_size_;

  int grid_size_;                // cells per axis
  std::vector<float> grid_;     // flat [iy * grid_size_ + ix], values in [0, 1]
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_
