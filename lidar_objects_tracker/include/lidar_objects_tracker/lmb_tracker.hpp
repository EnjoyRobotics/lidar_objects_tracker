/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_
#define LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

#include "lidar_objects_tracker/tracker_base.hpp"

namespace lidar_objects_tracker
{

/** @brief LMB Tracker for 2D point measurements
 *
 * Note: uint32_t is used for IDs, size_t for indices
 */
class LMBTracker : public TrackerBase
{
public:
  explicit LMBTracker(rclcpp::Node & node)
  : clock_(node.get_clock()),
    last_update_time_(node.get_clock()->now())
  {
    node.declare_parameter<double>("lmb_tracker.max_dt", 1.0);
    node.declare_parameter<double>("lmb_tracker.gate_threshold", 6.0);
    node.declare_parameter<double>("lmb_tracker.birth_existence_prob", 0.2);
    node.declare_parameter<double>("lmb_tracker.death_existence_prob", 0.05);
    node.declare_parameter<double>("lmb_tracker.survival_prob", 0.99);
    node.declare_parameter<double>("lmb_tracker.detection_prob", 0.99);
    node.declare_parameter<double>("lmb_tracker.confirmation_existence_prob", 0.7);
    node.declare_parameter<double>("lmb_tracker.merge_distance", 0.5);
    node.declare_parameter<double>("kalman_filter.pos_uncertainty", 0.2);
    node.declare_parameter<double>("kalman_filter.vel_uncertainty", 0.4);
    node.declare_parameter<double>("kalman_filter.acc_uncertainty", 1.0);
    max_dt_ = static_cast<float>(node.get_parameter("lmb_tracker.max_dt").as_double());
    gate_threshold_ =
      static_cast<float>(node.get_parameter("lmb_tracker.gate_threshold").as_double());
    birth_existence_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.birth_existence_prob").as_double());
    death_existence_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.death_existence_prob").as_double());
    survival_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.survival_prob").as_double());
    detection_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.detection_prob").as_double());
    confirmation_existence_prob_ =
      static_cast<float>(
      node.get_parameter("lmb_tracker.confirmation_existence_prob").as_double());
    merge_distance_ =
      static_cast<float>(node.get_parameter("lmb_tracker.merge_distance").as_double());
    kf_pos_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.pos_uncertainty").as_double());
    kf_vel_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.vel_uncertainty").as_double());
    kf_acc_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.acc_uncertainty").as_double());
  }

  UpdateInfo updateTracks(const std::vector<Eigen::Vector2f> & measurements) override
  {
    UpdateInfo update_info;

    // Used for births
    std::set<size_t> used_measurements;

    // Get dt since last update
    const rclcpp::Time current_time = clock_->now();
    const float dt = (current_time - last_update_time_).seconds();
    update_info.dt = dt;
    last_update_time_ = current_time;
    if (dt < 1e-6 || dt > max_dt_) {
      update_info.success = false;
      return update_info;
    }
    RCLCPP_DEBUG(logger_, "===============================");
    RCLCPP_DEBUG(logger_, "Updating tracks with dt: %.3f s", dt);

    for (auto & [id, track] : tracks_) {
      update_info.updates.emplace(id, UpdateInfo::TrackUpdateInfo{});

      // Predict kalman filter and update existence probability
      track.kf->predict(dt);
      auto & r = track.existence_probability;
      r = std::clamp(r * survival_prob_, 0.0f, 1.0f);  // Clamp to [0, 1]

      RCLCPP_DEBUG(
        logger_, "Track ID: %u, Existence Probability: %.3f",
        id, track.existence_probability);

      // Gate measurements
      std::set<size_t> gated_indices;
      std::stringstream ss;
      ss << "Using measurements: ";
      for (size_t i = 0; i < measurements.size(); ++i) {
        const auto & measurement = measurements[i];
        const float dist2 = track.kf->mahalanobisDistance2(measurement);
        if (dist2 < gate_threshold_) {
          gated_indices.insert(i);
          used_measurements.insert(i);
          ss << i << " ";
        }
      }
      RCLCPP_DEBUG(logger_, "%s", ss.str().c_str());

      // If no measurements, missed detection
      if (gated_indices.empty()) {
        track.existence_probability *= 1.0f - detection_prob_;

        RCLCPP_DEBUG(
          logger_, "Missed detection for Track ID: %u, New Existence Probability: %.3f",
          id, track.existence_probability);

        continue;
      }

      // Update with all gated measurements (simplified, equal weights)
      // TODO(redvinaa): Implement proper weighting
      Eigen::Vector2f combined_measurement = Eigen::Vector2f::Zero();
      const float w = 1.0f / static_cast<float>(gated_indices.size());
      for (const auto & meas_idx : gated_indices) {
        combined_measurement += measurements[meas_idx] / w;
        update_info.updates[id].measurement_weights[meas_idx] = w;
      }
      track.kf->update(combined_measurement);

      // Update existence probability
      r = 1 - (1 - r) * (1 - detection_prob_);

      // Confirm track once threshold is reached
      if (!track.confirmed && r >= confirmation_existence_prob_) {
        track.confirmed = true;
      }

      RCLCPP_DEBUG(
        logger_, "Updated Track ID: %u, New Existence Probability: %.3f",
        id, track.existence_probability);
    }

    // Deaths
    std::vector<uint32_t> tracks_to_erase;
    for (const auto & [id, track] : tracks_) {
      if (track.existence_probability < death_existence_prob_) {
        tracks_to_erase.push_back(id);
        update_info.deaths.insert(id);
        RCLCPP_DEBUG(logger_, "Deleting Track ID: %u due to low existence probability", id);
      }
    }
    for (const auto & id : tracks_to_erase) {
      tracks_.erase(id);
    }

    // Births
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (used_measurements.find(i) != used_measurements.end()) {
        continue;  // Measurement already used
      }

      // Create new track
      const auto & meas = measurements[i];
      Eigen::Vector4f x0;
      x0 << meas(0), meas(1), 0.0f, 0.0f;  // Initial velocity zero
      auto kf = std::make_shared<KalmanFilter2D>(
        x0,
        kf_pos_uncertainty_,
        kf_vel_uncertainty_,
        kf_acc_uncertainty_);

      // Assign new ID
      uint32_t new_id = 0;
      while (tracks_.find(new_id) != tracks_.end()) {
        ++new_id;
      }

      Track new_track;
      new_track.kf = kf;
      new_track.existence_probability = birth_existence_prob_;
      new_track.confirmed = false;
      new_track.birth_time = current_time;
      tracks_.emplace(new_id, std::move(new_track));

      update_info.births.insert(new_id);

      RCLCPP_DEBUG(
        logger_, "Created new Track ID: %u at position (%.2f, %.2f)",
        new_id, meas(0), meas(1));
    }

    // Merge nearby tracks: keep the older one, discard the newer
    if (merge_distance_ > 0.0f) {
      std::set<uint32_t> merged_away;
      for (auto it_a = tracks_.begin(); it_a != tracks_.end(); ++it_a) {
        if (merged_away.count(it_a->first)) {
          continue;
        }
        for (auto it_b = std::next(it_a); it_b != tracks_.end(); ++it_b) {
          if (merged_away.count(it_b->first)) {
            continue;
          }
          const Eigen::Vector2f pos_a = it_a->second.kf->state.head<2>();
          const Eigen::Vector2f pos_b = it_b->second.kf->state.head<2>();
          if ((pos_a - pos_b).norm() < merge_distance_) {
            // Keep the older track (earlier birth_time), discard the newer
            const bool a_is_older =
              it_a->second.birth_time <= it_b->second.birth_time;
            const uint32_t keep_id = a_is_older ? it_a->first : it_b->first;
            const uint32_t drop_id = a_is_older ? it_b->first : it_a->first;
            // Carry over confirmed status
            tracks_[keep_id].confirmed =
              tracks_[keep_id].confirmed || tracks_[drop_id].confirmed;
            merged_away.insert(drop_id);
            update_info.merges.insert(drop_id);
            RCLCPP_DEBUG(
              logger_, "Merging Track ID: %u into Track ID: %u",
              drop_id, keep_id);
          }
        }
      }
      for (const auto & id : merged_away) {
        tracks_.erase(id);
      }
    }

    return update_info;
  }

  inline const std::map<uint32_t, Track> & getTracks() const override
  {
    return tracks_;
  }

private:
  std::map<uint32_t, Track> tracks_;
  rclcpp::Clock::SharedPtr clock_;  // To get dt
  rclcpp::Logger logger_ = rclcpp::get_logger("LMBTracker");

  float max_dt_;
  rclcpp::Time last_update_time_;
  float gate_threshold_;  // ~95% confidence
  float birth_existence_prob_;  // Keep low so multiple confirmations needed
  float death_existence_prob_;
  float survival_prob_;  // P(existing object survives next step)
  float detection_prob_;  // P(existing object is detected)
  float confirmation_existence_prob_;  // Existence probability needed to confirm a track
  float merge_distance_;  // Euclidean distance below which two tracks are merged (m)
  float kf_pos_uncertainty_;  // for Kalman Filter initialization, m
  float kf_vel_uncertainty_;  // for Kalman Filter initialization, m/s
  float kf_acc_uncertainty_;  // for Kalman Filter initialization, m/s^2
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_
