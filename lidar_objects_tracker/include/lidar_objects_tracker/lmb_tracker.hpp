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

#include <algorithm>
#include <iomanip>
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
    node.declare_parameter<double>("lmb_tracker.clutter_intensity", 0.1);
    node.declare_parameter<double>("lmb_tracker.max_dt", 1.0);
    node.declare_parameter<double>("lmb_tracker.gate_threshold", 6.0);
    node.declare_parameter<double>("lmb_tracker.birth_existence_prob", 0.2);
    node.declare_parameter<double>("lmb_tracker.death_existence_prob", 0.05);
    node.declare_parameter<double>("lmb_tracker.survival_prob", 0.99);
    node.declare_parameter<double>("lmb_tracker.detection_prob", 0.99);
    node.declare_parameter<double>("lmb_tracker.confirmation_existence_prob", 0.7);
    node.declare_parameter<double>("lmb_tracker.merge_distance", 0.5);
    node.declare_parameter<double>("lmb_tracker.max_position_variance", 1.0);
    node.declare_parameter<double>("kalman_filter.pos_uncertainty", 0.2);
    node.declare_parameter<double>("kalman_filter.vel_uncertainty", 0.4);
    node.declare_parameter<double>("kalman_filter.acc_uncertainty", 1.0);
    clutter_intensity_ =
      static_cast<float>(node.get_parameter("lmb_tracker.clutter_intensity").as_double());
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
    max_position_variance_ =
      static_cast<float>(node.get_parameter("lmb_tracker.max_position_variance").as_double());
    kf_pos_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.pos_uncertainty").as_double());
    kf_vel_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.vel_uncertainty").as_double());
    kf_acc_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.acc_uncertainty").as_double());
  }

  bool updateTracks(const std::vector<Eigen::Vector2f> & measurements) override
  {
    // Used for births
    std::set<size_t> used_measurements;

    // Get dt since last update
    const rclcpp::Time current_time = clock_->now();
    const float dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    if (dt < 1e-6 || dt > max_dt_) {
      return false;
    }
    RCLCPP_DEBUG(logger_, "===============================");
    RCLCPP_DEBUG(logger_, "Updating tracks with dt: %.3f s", dt);

    // Step 1: Predict
    for (auto & [id, track] : tracks_) {
      track.kf->predict(dt);
      track.existence_probability =
        std::clamp(track.existence_probability * survival_prob_, 0.0f, 1.0f);

      RCLCPP_DEBUG(
        logger_, "Track ID: %u, Existence Probability: %.3f",
        id, track.existence_probability);
    }

    // Step 2: Association weights (LMB-style)
    // w_ij = P_D * r * L_ij  (unnormalized, for gated measurements only)
    // w_i0 = (1 - P_D) * r   (missed detection)
    // Normalize each track's weights by: sum_j(w_ij) + w_i0 + clutter_intensity
    struct TrackWeights
    {
      std::map<size_t, float> w;  // measurement index -> unnormalized weight
      float w0;                   // missed-detection unnormalized weight
      float normalizer;           // denominator for this track
    };
    std::map<uint32_t, TrackWeights> all_weights;

    for (auto & [id, track] : tracks_) {
      TrackWeights tw;
      tw.w0 = (1.0f - detection_prob_) * track.existence_probability;
      float sum_w = 0.0f;

      std::stringstream ss;
      ss << "Track " << id << " using measurements: ";
      for (size_t j = 0; j < measurements.size(); ++j) {
        const float d2 = track.kf->mahalanobisDistance2(measurements[j]);
        if (d2 >= gate_threshold_) {
          continue;
        }
        const float L = track.kf->likelihood(measurements[j]);
        const float w = detection_prob_ * track.existence_probability * L;
        tw.w[j] = w;
        sum_w += w;
        used_measurements.insert(j);
        ss << j << " ";
      }
      RCLCPP_DEBUG(logger_, "%s", ss.str().c_str());

      tw.normalizer = sum_w + tw.w0 + clutter_intensity_;
      all_weights[id] = std::move(tw);
    }

    // Step 3: Update tracks
    for (auto & [id, track] : tracks_) {
      auto & tw = all_weights[id];

      // Normalized association probabilities
      float sum_assoc = 0.0f;
      for (auto & [j, w] : tw.w) {
        w /= tw.normalizer;
        sum_assoc += w;
      }
      const float w0_norm = tw.w0 / tw.normalizer;

      // Existence update: r = sum of normalized association weights + w0
      track.existence_probability = std::clamp(sum_assoc + w0_norm, 0.0f, 1.0f);

      // State update: weighted mixture of KF updates
      if (sum_assoc > 0.0f) {
        Eigen::Vector4f x_new = Eigen::Vector4f::Zero();
        Eigen::Matrix4f P_new = Eigen::Matrix4f::Zero();

        // Include missed-detection component (predicted state, weighted by w0_norm)
        x_new += w0_norm * track.kf->state;
        P_new += w0_norm * track.kf->covariance;

        // Save predicted state/covariance before any KF update mutates it
        const Eigen::Vector4f x_pred = track.kf->state;
        const Eigen::Matrix4f P_pred = track.kf->covariance;

        for (auto & [j, w_norm] : tw.w) {
          // Restore predicted state so each update starts from the same prior
          track.kf->state = x_pred;
          track.kf->covariance = P_pred;
          track.kf->update(measurements[j]);
          x_new += w_norm * track.kf->state;
          P_new += w_norm * track.kf->covariance;
        }

        // Normalize (sum_assoc + w0_norm should already equal r, which is <= 1;
        // divide to guard against floating-point drift)
        const float total_w = sum_assoc + w0_norm;
        if (total_w > 1e-6f) {
          x_new /= total_w;
          P_new /= total_w;
        }

        track.kf->state = x_new;
        track.kf->covariance = P_new;

        RCLCPP_DEBUG(
          logger_, "Updated Track ID: %u, New Existence Probability: %.3f",
          id, track.existence_probability);
      } else {
        // Missed detection: keep predicted state as-is
        RCLCPP_DEBUG(
          logger_, "Missed detection for Track ID: %u, New Existence Probability: %.3f",
          id, track.existence_probability);
      }
    }

    // Step 4: Births — unassigned measurements spawn new tracks
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (used_measurements.count(i)) {
        continue;
      }

      const auto & meas = measurements[i];
      Eigen::Vector4f x0;
      x0 << meas(0), meas(1), 0.0f, 0.0f;
      auto kf = std::make_shared<KalmanFilter2D>(
        x0,
        kf_pos_uncertainty_,
        kf_vel_uncertainty_,
        kf_acc_uncertainty_);

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

      RCLCPP_DEBUG(
        logger_, "Created new Track ID: %u at position (%.2f, %.2f)",
        new_id, meas(0), meas(1));
    }

    // Step 5: Merge nearby confirmed tracks — keep older, transfer existence probability
    if (merge_distance_ > 0.0f) {
      std::set<uint32_t> merged_away;
      for (auto it_a = tracks_.begin(); it_a != tracks_.end(); ++it_a) {
        if (merged_away.count(it_a->first) || !it_a->second.confirmed) {
          continue;
        }
        for (auto it_b = std::next(it_a); it_b != tracks_.end(); ++it_b) {
          if (merged_away.count(it_b->first) || !it_b->second.confirmed) {
            continue;
          }
          const Eigen::Vector2f pos_a = it_a->second.kf->state.head<2>();
          const Eigen::Vector2f pos_b = it_b->second.kf->state.head<2>();
          if ((pos_a - pos_b).norm() < merge_distance_) {
            const bool a_is_older =
              it_a->second.birth_time <= it_b->second.birth_time;
            const uint32_t keep_id = a_is_older ? it_a->first : it_b->first;
            const uint32_t drop_id = a_is_older ? it_b->first : it_a->first;
            tracks_[keep_id].existence_probability = std::max(
              tracks_[keep_id].existence_probability,
              tracks_[drop_id].existence_probability);
            merged_away.insert(drop_id);
            RCLCPP_DEBUG(
              logger_, "Merging confirmed Track ID: %u into Track ID: %u",
              drop_id, keep_id);
          }
        }
      }
      for (const auto & id : merged_away) {
        tracks_.erase(id);
      }
    }

    // Step 6: Prune tracks with low existence probability; clamp excessive covariance
    std::vector<uint32_t> tracks_to_erase;
    for (const auto & [id, track] : tracks_) {
      if (track.existence_probability < death_existence_prob_) {
        tracks_to_erase.push_back(id);
        RCLCPP_DEBUG(logger_, "Deleting Track ID: %u due to low existence probability", id);
      }
    }
    for (const auto & id : tracks_to_erase) {
      tracks_.erase(id);
    }
    for (auto & [id, track] : tracks_) {
      track.kf->covariance(0, 0) = std::min(track.kf->covariance(0, 0), max_position_variance_);
      track.kf->covariance(1, 1) = std::min(track.kf->covariance(1, 1), max_position_variance_);
    }

    // Step 7: Confirm tracks that crossed the threshold
    for (auto & [id, track] : tracks_) {
      if (!track.confirmed && track.existence_probability >= confirmation_existence_prob_) {
        track.confirmed = true;
      }
    }

    return true;
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
  float clutter_intensity_;  // lambda_c: expected number of clutter measurements per unit volume
  float gate_threshold_;  // ~95% confidence
  float birth_existence_prob_;  // Keep low so multiple confirmations needed
  float death_existence_prob_;
  float survival_prob_;  // P(existing object survives next step)
  float detection_prob_;  // P(existing object is detected)
  float confirmation_existence_prob_;  // Existence probability needed to confirm a track
  float merge_distance_;  // Euclidean distance below which two confirmed tracks are merged (m)
  float max_position_variance_;  // P(0,0) and P(1,1) are clamped to this value (m^2)
  float kf_pos_uncertainty_;  // for Kalman Filter initialization, m
  float kf_vel_uncertainty_;  // for Kalman Filter initialization, m/s
  float kf_acc_uncertainty_;  // for Kalman Filter initialization, m/s^2
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_

