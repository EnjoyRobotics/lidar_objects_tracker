/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__TRACKER_BASE_HPP_
#define LIDAR_OBJECTS_TRACKER__TRACKER_BASE_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <vector>

#include "lidar_objects_tracker/kalman_filter.hpp"

namespace lidar_objects_tracker
{

struct Track
{
  std::shared_ptr<KalmanFilter2D> kf;
  float existence_probability;
  bool confirmed = false;
  rclcpp::Time birth_time;
};


/** @brief Abstract base class for multi-object trackers */
class TrackerBase
{
public:
  virtual ~TrackerBase() = default;

  /** @brief Update all tracks with new measurements
   * @param measurements 2D point measurements (centroids)
   */
  virtual void updateTracks(const std::vector<Eigen::Vector2f> & measurements) = 0;

  /** @brief Get the current set of tracks
   * @return const reference to the internal tracks map (id -> Track)
   */
  virtual const std::map<uint32_t, Track> & getTracks() const = 0;
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__TRACKER_BASE_HPP_
