/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_
#define LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <open3d/geometry/PointCloud.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace lidar_objects_tracker
{

/** @brief Header-only static background subtractor using a local occupancy grid.
 *
 * For each received scan, cells that were hit have their probability increased,
 * all other cells are decreased. Points that fall on cells above the threshold
 * are considered static background and are filtered out.
 *
 * An inflation is applied to the thresholded binary grid each
 * scan: every cell neighbouring an occupied cell is also treated as occupied.
 * This closes the small gaps that cause flickering at object edges.
 *
 * Parameters (namespaced under "static_bg_subtractor"):
 *   - resolution        : grid cell size in metres
 *   - hit_increment     : probability increase on a hit
 *   - miss_decrement    : probability decrease when not hit
 *   - threshold         : cells above this probability are treated as static
 *   - inflation_radius  : number of cells to inflate around each occupied cell
 *   - publish_grid                : publish the occupancy grid for debug
 *   - grid_size                    : width/height of the grid in metres
 *   - initialize_with_first_scan   : set hit cells to max probability on the first scan
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
    node.declare_parameter<int>("static_bg_subtractor.inflation_radius", 1);
    node.declare_parameter<bool>("static_bg_subtractor.publish_grid", false);
    node.declare_parameter<double>("static_bg_subtractor.grid_size", 10.0);
    node.declare_parameter<bool>("static_bg_subtractor.initialize_with_first_scan", true);
    node.declare_parameter<double>("static_bg_subtractor.dynamic_track_clear_radius", 0.5);

    resolution_ =
      static_cast<float>(node.get_parameter("static_bg_subtractor.resolution").as_double());
    hit_increment_ =
      static_cast<float>(node.get_parameter("static_bg_subtractor.hit_increment").as_double());
    miss_decrement_ =
      static_cast<float>(node.get_parameter("static_bg_subtractor.miss_decrement").as_double());
    threshold_ =
      static_cast<float>(node.get_parameter("static_bg_subtractor.threshold").as_double());
    inflation_radius_ = node.get_parameter("static_bg_subtractor.inflation_radius").as_int();
    publish_grid_ = node.get_parameter("static_bg_subtractor.publish_grid").as_bool();
    grid_world_size_ =
      static_cast<float>(node.get_parameter("static_bg_subtractor.grid_size").as_double());
    initialize_with_first_scan_ =
      node.get_parameter("static_bg_subtractor.initialize_with_first_scan").as_bool();
    dynamic_track_clear_radius_ =
      static_cast<float>(
        node.get_parameter("static_bg_subtractor.dynamic_track_clear_radius").as_double());

    // Grid dimensions: centred at origin
    grid_size_ = static_cast<int>(std::ceil(grid_world_size_ / resolution_));
    grid_.assign(grid_size_ * grid_size_, 0.0f);

    if (publish_grid_) {
      grid_pub_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
        "static_background_grid", rclcpp::QoS(1).transient_local());
    }

    RCLCPP_INFO(
      logger_,
      "StaticBackgroundSubtractor: resolution=%.3f, grid=%dx%d cells (%.1fm x %.1fm), init_first_scan=%s",
      resolution_, grid_size_, grid_size_,
      grid_world_size_, grid_world_size_,
      initialize_with_first_scan_ ? "true" : "false");
  }

  /** @brief Update the occupancy grid and return only non-static points.
   *
   * The input point cloud must already be in the sensor/local frame because
   * the grid is maintained in that same frame (robot-centric).
   *
   * @param pc                 Input point cloud (local frame, z ignored)
   * @param header             Header from the original LaserScan (for grid publication stamp / frame)
   * @param dynamic_positions  Positions of confirmed dynamic tracks; cells under them are frozen
   * @param sensor_origin      Sensor position in the target frame; when provided, also freezes the
   *                           shadow behind each dynamic track along the ray from sensor to track
   * @return                   Filtered point cloud with static background points removed
   */
  open3d::geometry::PointCloud filter(
    const open3d::geometry::PointCloud & pc,
    const std_msgs::msg::Header & header,
    const std::vector<Eigen::Vector2f> & dynamic_positions = {},
    const std::optional<Eigen::Vector2f> & sensor_origin = std::nullopt)
  {
    // Mark which cells were hit this scan
    std::vector<bool> hit(grid_size_ * grid_size_, false);

    for (const auto & pt : pc.points_) {
      const int ix = worldToIndex(static_cast<float>(pt.x()));
      const int iy = worldToIndex(static_cast<float>(pt.y()));
      if (isValid(ix, iy)) {
        hit[cellIndex(ix, iy)] = true;
      }
    }

    // Mark cells under dynamic tracks and their shadows; probabilities will not be updated
    const int dynamic_cells =
      static_cast<int>(std::ceil(dynamic_track_clear_radius_ / resolution_));
    disk_frozen_.assign(grid_size_ * grid_size_, false);
    shadow_frozen_.assign(grid_size_ * grid_size_, false);
    for (const auto & pos : dynamic_positions) {
      // Freeze circular disk around the track
      const int cx = worldToIndex(pos.x());
      const int cy = worldToIndex(pos.y());
      const float r2 = dynamic_track_clear_radius_ * dynamic_track_clear_radius_;
      for (int dy = -dynamic_cells; dy <= dynamic_cells; ++dy) {
        for (int dx = -dynamic_cells; dx <= dynamic_cells; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (!isValid(nx, ny)) {
            continue;
          }
          const float wx = worldFromIndex(nx) - pos.x();
          const float wy = worldFromIndex(ny) - pos.y();
          if (wx * wx + wy * wy <= r2) {
            disk_frozen_[cellIndex(nx, ny)] = true;
          }
        }
      }
      // Freeze shadow: march the ray from sensor through the track outward to the grid edge
      if (sensor_origin.has_value()) {
        castShadow(*sensor_origin, pos, shadow_frozen_);
      }
    }
    // Merge into a single frozen mask for the update step
    std::vector<bool> frozen(grid_size_ * grid_size_, false);
    for (int i = 0; i < grid_size_ * grid_size_; ++i) {
      frozen[i] = disk_frozen_[i] || shadow_frozen_[i];
    }

    // Update probabilities, skipping frozen cells
    for (int i = 0; i < grid_size_ * grid_size_; ++i) {
      if (frozen[i]) {
        continue;
      }
      if (hit[i]) {
        grid_[i] = is_first_scan_ && initialize_with_first_scan_
          ? 1.0f
          : std::min(1.0f, grid_[i] + hit_increment_);
      } else {
        grid_[i] = std::max(0.0f, grid_[i] - miss_decrement_);
      }
    }
    is_first_scan_ = false;

    // Inflate thresholded grid: neighbours of occupied cells are also occupied
    const std::vector<bool> inflated = inflate(grid_, threshold_);

    // Filter out points whose cell is static after inflation
    open3d::geometry::PointCloud filtered;
    for (const auto & pt : pc.points_) {
      const int ix = worldToIndex(static_cast<float>(pt.x()));
      const int iy = worldToIndex(static_cast<float>(pt.y()));
      if (!isValid(ix, iy) || !inflated[cellIndex(ix, iy)]) {
        filtered.points_.push_back(pt);
      }
    }

    // Optionally publish grid
    if (publish_grid_) {
      publishGrid(header, inflated);
    }

    return filtered;
  }

  /** @brief Return disk and shadow frozen regions as CUBE_LIST markers to be appended to
   *  a MarkerArray by the caller. Uses the masks computed during the last filter() call.
   */
  std::vector<visualization_msgs::msg::Marker> getFrozenMaskMarkers(
    const std_msgs::msg::Header & header) const
  {
    auto makeMarker = [&](const char * ns, int id, float r, float g, float b)
    {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = resolution_;
      m.scale.y = resolution_;
      m.scale.z = 0.02f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.4f;
      m.pose.orientation.w = 1.0;
      return m;
    };

    // Disk: cyan; shadow: orange
    auto disk_marker = makeMarker("frozen_disk", 0, 0.0f, 1.0f, 1.0f);
    auto shadow_marker = makeMarker("frozen_shadow", 0, 1.0f, 0.5f, 0.0f);

    for (int iy = 0; iy < grid_size_; ++iy) {
      for (int ix = 0; ix < grid_size_; ++ix) {
        const int i = cellIndex(ix, iy);
        geometry_msgs::msg::Point p;
        p.x = worldFromIndex(ix);
        p.y = worldFromIndex(iy);
        p.z = 0.0;
        if (disk_frozen_[i]) {
          disk_marker.points.push_back(p);
        } else if (shadow_frozen_[i]) {
          shadow_marker.points.push_back(p);
        }
      }
    }

    return {disk_marker, shadow_marker};
  }

private:
  // Convert world coordinate (metres) to grid index
  inline int worldToIndex(float world) const
  {
    return static_cast<int>(std::floor((world + grid_world_size_ * 0.5f) / resolution_));
  }

  inline bool isValid(int ix, int iy) const
  {
    return ix >= 0 && ix < grid_size_ && iy >= 0 && iy < grid_size_;
  }

  inline int cellIndex(int ix, int iy) const
  {
    return iy * grid_size_ + ix;
  }

  /** @brief Freeze the shadow of the disk behind the track: sweeps rays across the full
   *  angular extent of the disk as seen from the sensor, marching each ray from the far
   *  edge of the disk to the grid boundary.
   */
  void castShadow(
    const Eigen::Vector2f & sensor_origin,
    const Eigen::Vector2f & track_pos,
    std::vector<bool> & frozen) const
  {
    const Eigen::Vector2f to_track = track_pos - sensor_origin;
    const float dist = to_track.norm();
    if (dist < 1e-6f) {
      return;
    }
    const float center_angle = std::atan2(to_track.y(), to_track.x());
    // Angular half-width of the disk as seen from the sensor
    const float half_angle = dist > dynamic_track_clear_radius_
      ? std::asin(dynamic_track_clear_radius_ / dist)
      : static_cast<float>(M_PI_2);
    // Angular step fine enough that no cell is skipped at the far grid corner
    const float max_range = grid_world_size_ * std::sqrt(2.0f);
    const float angular_step = resolution_ / max_range;
    const float radial_step = resolution_ * 0.5f;
    for (float angle = center_angle - half_angle;
         angle <= center_angle + half_angle + angular_step * 0.5f;
         angle += angular_step)
    {
      const Eigen::Vector2f unit(std::cos(angle), std::sin(angle));
      // Start just past the far edge of the disk
      float t = dist + dynamic_track_clear_radius_ + radial_step;
      Eigen::Vector2f p = sensor_origin + unit * t;
      while (isValid(worldToIndex(p.x()), worldToIndex(p.y()))) {
        frozen[cellIndex(worldToIndex(p.x()), worldToIndex(p.y()))] = true;
        t += radial_step;
        p = sensor_origin + unit * t;
      }
    }
  }

  // Convert grid index to world coordinate (cell centre)
  inline float worldFromIndex(int idx) const
  {
    return (idx + 0.5f) * resolution_ - grid_world_size_ * 0.5f;
  }

  /** @brief Inflate the thresholded grid: a cell is occupied if it or any
   *  neighbour within inflation_radius_ cells is above the threshold.
   */
  std::vector<bool> inflate(const std::vector<float> & grid, float thresh) const
  {
    const int n = grid_size_ * grid_size_;
    const int r = inflation_radius_;

    std::vector<bool> result(n, false);
    for (int iy = 0; iy < grid_size_; ++iy) {
      for (int ix = 0; ix < grid_size_; ++ix) {
        for (int dy = -r; dy <= r && !result[cellIndex(ix, iy)]; ++dy) {
          for (int dx = -r; dx <= r && !result[cellIndex(ix, iy)]; ++dx) {
            const int nx = ix + dx;
            const int ny = iy + dy;
            if (isValid(nx, ny) && grid[cellIndex(nx, ny)] >= thresh) {
              result[cellIndex(ix, iy)] = true;
            }
          }
        }
      }
    }
    return result;
  }

  void publishGrid(const std_msgs::msg::Header & header, const std::vector<bool> & inflated)
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header = header;
    msg.info.resolution = resolution_;
    msg.info.width = static_cast<uint32_t>(grid_size_);
    msg.info.height = static_cast<uint32_t>(grid_size_);
    msg.info.origin.position.x = -grid_world_size_ * 0.5f;
    msg.info.origin.position.y = -grid_world_size_ * 0.5f;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_size_ * grid_size_);
    for (int i = 0; i < grid_size_ * grid_size_; ++i) {
      // Show post-closing binary state: occupied=100, free=raw probability
      msg.data[i] = inflated[i] ? 100 : static_cast<int8_t>(grid_[i] * 100.0f);
    }

    grid_pub_->publish(msg);
  }

  rclcpp::Logger logger_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  float resolution_;
  float hit_increment_;
  float miss_decrement_;
  float threshold_;
  int inflation_radius_;  // cells to inflate around each occupied cell
  bool publish_grid_;
  float grid_world_size_;  // full width/height of the grid in metres
  bool initialize_with_first_scan_;
  float dynamic_track_clear_radius_;  // radius (m) around dynamic tracks where probability updates are frozen
  bool is_first_scan_{true};

  int grid_size_;                // cells per axis
  std::vector<float> grid_;     // flat [iy * grid_size_ + ix], values in [0, 1]
  std::vector<bool> disk_frozen_;    // cached from last filter() call
  std::vector<bool> shadow_frozen_;  // cached from last filter() call
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__STATIC_BACKGROUND_SUBTRACTOR_HPP_
