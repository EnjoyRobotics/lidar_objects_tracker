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
    // Gating parameters
    node.declare_parameter<double>("lmb_tracker.gating.mahal2", 9.0);
    node.declare_parameter<double>("lmb_tracker.gating.dist", 3.0);

    // Basic tracking parameters
    node.declare_parameter<double>("lmb_tracker.birth_existence_prob", 0.05);
    node.declare_parameter<double>("lmb_tracker.death_existence_prob", 0.01);
    node.declare_parameter<double>("lmb_tracker.detection_prob", 0.6);
    node.declare_parameter<double>("lmb_tracker.clutter_intensity", 0.1);
    node.declare_parameter<double>("lmb_tracker.confirmation_existence_prob", 0.7);
    node.declare_parameter<double>("lmb_tracker.dynamic_velocity_threshold", 0.3);

    // Merging parameters
    node.declare_parameter<double>("lmb_tracker.merging.mahal2", 13.0);
    node.declare_parameter<double>("lmb_tracker.merging.dist", 3.0);

    // Kalman Filter parameters
    node.declare_parameter<double>("kalman_filter.pos_uncertainty", 0.2);
    node.declare_parameter<double>("kalman_filter.vel_uncertainty", 0.4);
    node.declare_parameter<double>("kalman_filter.acc_uncertainty", 1.0);

    gate_mahal2_ =
      static_cast<float>(node.get_parameter("lmb_tracker.gating.mahal2").as_double());
    dist2_threshold_ = std::pow(
      static_cast<float>(node.get_parameter("lmb_tracker.gating.dist").as_double()), 2.0);
    birth_existence_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.birth_existence_prob").as_double());
    death_existence_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.death_existence_prob").as_double());
    detection_prob_ =
      static_cast<float>(node.get_parameter("lmb_tracker.detection_prob").as_double());
    clutter_intensity_ =
      static_cast<float>(node.get_parameter("lmb_tracker.clutter_intensity").as_double());
    confirmation_existence_prob_ =
      static_cast<float>(
      node.get_parameter("lmb_tracker.confirmation_existence_prob").as_double());
    dynamic_velocity_threshold_ =
      static_cast<float>(
      node.get_parameter("lmb_tracker.dynamic_velocity_threshold").as_double());
    merge_mahal2_ =
      static_cast<float>(node.get_parameter("lmb_tracker.merging.mahal2").as_double());
    merge_dist2_ = std::pow(
      static_cast<float>(node.get_parameter("lmb_tracker.merging.dist").as_double()), 2.0);
    kf_pos_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.pos_uncertainty").as_double());
    kf_vel_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.vel_uncertainty").as_double());
    kf_acc_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.acc_uncertainty").as_double());
  }

  void updateTracks(const std::vector<Eigen::Vector2f> & measurements) override
  {
    // Used for births
    std::set<size_t> used_measurements;

    // Get dt since last update
    const rclcpp::Time current_time = clock_->now();
    const float dt = std::max<float>(
        (current_time - last_update_time_).seconds(),
        1e-3f);  // Avoid zero or negative dt
    last_update_time_ = current_time;
    RCLCPP_DEBUG(logger_, "===============================");
    RCLCPP_DEBUG(logger_, "Updating tracks with dt: %.3f s", dt);

    // Predict
    for (auto & [id, track] : tracks_) {
      track.kf->predict(dt);
    }

    // Compute association likelihoods for each track
    // For Bernoulli update: r' = r*eta_D / ((1-r)*kappa + r*eta)
    // where eta_D = sum_j(P_D * L_j), eta = (1-P_D) + eta_D, kappa = clutter_intensity
    struct TrackAssociation
    {
      std::map<size_t, float> likelihood;  // measurement index -> P_D * L_j
      float sum_likelihood;                // sum of P_D * L_j over gated measurements
    };
    std::map<uint32_t, TrackAssociation> all_assoc;

    for (auto & [id, track] : tracks_) {
      TrackAssociation ta;
      float sum_likelihood = 0.0f;

      std::stringstream ss;
      ss << "Track " << id << " using measurements: ";
      for (size_t j = 0; j < measurements.size(); ++j) {
        const float dist2 = track.kf->euclideanDistance2(measurements[j]);
        if (dist2 > dist2_threshold_) {
          continue;
        }

        const float mahal2 = track.kf->mahalanobisDistance2(measurements[j]);
        if (mahal2 >= gate_mahal2_) {
          continue;
        }
        const float L = track.kf->likelihood(measurements[j]);
        const float pd_likelihood = L;
        ta.likelihood[j] = pd_likelihood;
        sum_likelihood += pd_likelihood;
        used_measurements.insert(j);
        ss << j << " ";
      }
      RCLCPP_DEBUG(logger_, "%s", ss.str().c_str());

      ta.sum_likelihood = sum_likelihood;
      all_assoc[id] = std::move(ta);
    }

    // Update existence probability using Bernoulli filter formula
    // if detected: r' = r * eta_D / ((1-r)*kappa + r*eta)
    // if not detected: r' = r * (1-P_D)
    for (auto & [id, track] : tracks_) {
      auto & ta = all_assoc[id];

      const float r_old = track.existence_probability;
      const float eta_D = ta.sum_likelihood;

      float r_new = r_old;

      if (eta_D > 1e-6f) {
        const float eta = (1.0f - detection_prob_) + eta_D;
        const float denominator =
          (1.0f - r_old) * clutter_intensity_ + r_old * eta;

        if (denominator > 1e-6f) {
          r_new = r_old * eta_D / denominator;
        }
      } else {
        // NO detection case → pure missed detection update
        r_new = r_old * (1.0f - detection_prob_);
      }

      track.existence_probability = std::clamp(r_new, 0.0f, 1.0f);

      RCLCPP_DEBUG(
        logger_,
        "Track ID: %u, r_old=%.3f, eta_D=%.3f, r_new=%.3f, confirmed=%s",
        id, r_old, eta_D, track.existence_probability,
        track.confirmed ? "true" : "false");
    }

    // State update (weighted mixture of KF updates)
    for (auto & [id, track] : tracks_) {
      auto & ta = all_assoc[id];

      const float r = track.existence_probability;

      if (ta.sum_likelihood > 1e-6f) {

        Eigen::Matrix4f P = track.kf->covariance;
        Eigen::Vector4f x = track.kf->state;

        Eigen::Matrix4f P_inv = P.inverse(); // (can later optimize)

        Eigen::Vector4f x_update = Eigen::Vector4f::Zero();
        Eigen::Matrix4f P_update = Eigen::Matrix4f::Zero();

        float total_w = (1.0f - detection_prob_) + ta.sum_likelihood;

        // missed detection component
        x_update += (1.0f - detection_prob_) * x;
        P_update += (1.0f - detection_prob_) * P;

        // measurement contributions (NO KF re-run)
        for (auto & [j, pd_likelihood] : ta.likelihood) {

          Eigen::Vector2f z = measurements[j];

          // innovation
          Eigen::Vector2f y = z - track.kf->H() * x;
          Eigen::Matrix2f S = track.kf->H() * P * track.kf->H().transpose() +
                              track.kf->R();

          Eigen::Matrix<float, 4, 2> K =
            P * track.kf->H().transpose() * S.inverse();

          Eigen::Vector4f x_j = x + K * y;
          Eigen::Matrix4f P_j =
            (Eigen::Matrix4f::Identity() - K * track.kf->H()) * P;

          x_update += pd_likelihood * x_j;
          P_update += pd_likelihood * P_j;
        }

        x_update /= total_w;
        P_update /= total_w;

        track.kf->state = x_update;
        track.kf->covariance = P_update;

        RCLCPP_DEBUG(
          logger_, "Updated Track ID: %u, state=(%.2f, %.2f), r=%.3f",
          id, track.kf->state(0), track.kf->state(1), track.existence_probability);
      } else {
        // Missed detection: keep predicted state as-is
        RCLCPP_DEBUG(
          logger_, "Missed detection for Track ID: %u, r=%.3f",
          id, track.existence_probability);
      }
    }

    // Births — unassigned measurements spawn new tracks
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

    // Merge nearby confirmed tracks — keep older, transfer existence probability
    if (merge_mahal2_ > 0.0f || merge_dist2_ > 0.0f) {
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

          // Check both conditions: position-only euclidean distance and full state mahalanobis distance
          const float pos_dist2 = (pos_a - pos_b).squaredNorm();
          const bool pos_matches = (merge_dist2_ > 0.0f) && (pos_dist2 <= merge_dist2_);

          const float state_mahal2 = it_a->second.kf->mahalanobisDistance2(pos_b);
          const bool state_matches = (merge_mahal2_ > 0.0f) && (state_mahal2 <= merge_mahal2_);

          // Both conditions must be true if both are set; if only one is set, only that one matters
          bool should_merge = false;
          if (merge_dist2_ > 0.0f && merge_mahal2_ > 0.0f) {
            should_merge = pos_matches && state_matches;
          } else if (merge_dist2_ > 0.0f) {
            should_merge = pos_matches;
          } else if (merge_mahal2_ > 0.0f) {
            should_merge = state_matches;
          }

          if (should_merge) {
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

    // Prune tracks with low existence probability
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

    // Confirm tracks that crossed the threshold; mark dynamic if velocity exceeds threshold
    for (auto & [id, track] : tracks_) {
      if (!track.confirmed && track.existence_probability >= confirmation_existence_prob_) {
        track.confirmed = true;
      }
      if (track.confirmed) {
        const float speed2 = track.kf->state.tail<2>().squaredNorm();
        if (speed2 >= dynamic_velocity_threshold_ * dynamic_velocity_threshold_) {
          track.dynamic = true;
        }
      }
    }
  }

  inline const std::map<uint32_t, Track> & getTracks() const override
  {
    return tracks_;
  }

private:
  std::map<uint32_t, Track> tracks_;
  rclcpp::Clock::SharedPtr clock_;  // To get dt
  rclcpp::Logger logger_ = rclcpp::get_logger("LMBTracker");

  rclcpp::Time last_update_time_;
  float gate_mahal2_;  // Max squared Mahalanobis distance for gating
  float dist2_threshold_;  // Max squared euclidean distance to associate measurement to track
  float clutter_intensity_;  // lambda_c: expected number of clutter measurements per unit volume
  float birth_existence_prob_;  // Keep low so multiple confirmations needed
  float death_existence_prob_;
  float detection_prob_;  // P(existing object is detected)
  float confirmation_existence_prob_;  // Existence probability needed to confirm a track
  float dynamic_velocity_threshold_;   // Speed (m/s) above which a confirmed track is marked dynamic
  float merge_mahal2_;  // Max squared Mahalanobis distance (state space) for merging two tracks
  float merge_dist2_;  // Max squared euclidean distance (position only) for merging two tracks
  float kf_pos_uncertainty_;  // for Kalman Filter initialization, m
  float kf_vel_uncertainty_;  // for Kalman Filter initialization, m/s
  float kf_acc_uncertainty_;  // for Kalman Filter initialization, m/s^2
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_

