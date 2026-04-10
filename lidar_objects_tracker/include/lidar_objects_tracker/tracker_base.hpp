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

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "lidar_objects_tracker/kalman_filter.hpp"

namespace lidar_objects_tracker
{

struct Track
{
  std::shared_ptr<KalmanFilter2D> kf;
  float existence_probability;
};

struct UpdateInfo
{
  // Global update info
  float dt;
  std::set<uint32_t> births;
  std::set<uint32_t> deaths;

  struct TrackUpdateInfo
  {
    std::map<size_t, float> measurement_weights;
  };

  // Per track update info
  std::map<uint32_t, TrackUpdateInfo> updates;
};

/** @brief Abstract base class for multi-object trackers
 *
 * Both LMBTracker and PMBMTracker implement this interface.
 * Note: uint32_t is used for IDs, size_t for indices
 */
class TrackerBase
{
public:
  virtual ~TrackerBase() = default;

  /** @brief Update all tracks with new measurements
   * @param measurements 2D point measurements (centroids)
   * @return UpdateInfo describing births, deaths and per-track association info
   */
  virtual UpdateInfo updateTracks(const std::vector<Eigen::Vector2f> & measurements) = 0;

  /** @brief Get the current set of tracks
   * @return const reference to the internal tracks map (id -> Track)
   */
  virtual const std::map<uint32_t, Track> & getTracks() const = 0;
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__TRACKER_BASE_HPP_
