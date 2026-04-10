/* Copyright 2026 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2026
 */

#ifndef LIDAR_OBJECTS_TRACKER__PMBM_TRACKER_HPP_
#define LIDAR_OBJECTS_TRACKER__PMBM_TRACKER_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "lidar_objects_tracker/tracker_base.hpp"

namespace lidar_objects_tracker
{

class PMBMTracker : public TrackerBase
{
public:
  explicit PMBMTracker(rclcpp::Node & node)
  : clock_(node.get_clock()),
    last_update_time_(node.get_clock()->now())
  {
    node.declare_parameter<double>("pmbm_tracker.max_dt", 1.0);
    node.declare_parameter<double>("pmbm_tracker.gate_threshold", 6.0);
    node.declare_parameter<double>("pmbm_tracker.survival_prob", 0.99);
    node.declare_parameter<double>("pmbm_tracker.detection_prob", 0.9);
    node.declare_parameter<double>("pmbm_tracker.birth_intensity", 0.05);
    node.declare_parameter<double>("pmbm_tracker.prune_threshold", 0.01);
    node.declare_parameter<double>("kalman_filter.pos_uncertainty", 0.2);
    node.declare_parameter<double>("kalman_filter.vel_uncertainty", 0.4);
    node.declare_parameter<double>("kalman_filter.acc_uncertainty", 1.0);

    max_dt_ = node.get_parameter("pmbm_tracker.max_dt").as_double();
    gate_threshold_ = node.get_parameter("pmbm_tracker.gate_threshold").as_double();
    survival_prob_ = node.get_parameter("pmbm_tracker.survival_prob").as_double();
    detection_prob_ = node.get_parameter("pmbm_tracker.detection_prob").as_double();
    birth_intensity_ = node.get_parameter("pmbm_tracker.birth_intensity").as_double();
    prune_threshold_ = node.get_parameter("pmbm_tracker.prune_threshold").as_double();
    kf_pos_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.pos_uncertainty").as_double());
    kf_vel_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.vel_uncertainty").as_double());
    kf_acc_uncertainty_ =
      static_cast<float>(node.get_parameter("kalman_filter.acc_uncertainty").as_double());
  }

  struct Bernoulli
  {
    float existence = 0.0f;
    std::shared_ptr<KalmanFilter2D> kf;
  };

  struct PoissonComponent
  {
    Eigen::Vector2f mean;
    float weight;
  };

  UpdateInfo updateTracks(const std::vector<Eigen::Vector2f> & measurements) override
  {
    UpdateInfo info;

    float dt = (clock_->now() - last_update_time_).seconds();
    last_update_time_ = clock_->now();
    info.dt = dt;

    if (dt <= 0.0 || dt > max_dt_) {
      throw std::runtime_error("Invalid dt");
    }

    // 1. Predict Bernoulli tracks
    for (auto & [id, b] : tracks_) {
      b.kf->predict(dt);
      b.existence *= survival_prob_;
    }

    // 2. Data association (very simple: nearest gated)
    std::set<size_t> used_measurements;

    for (auto & [id, b] : tracks_) {
      float best_dist = gate_threshold_;
      int best_idx = -1;

      for (size_t i = 0; i < measurements.size(); ++i) {
        if (used_measurements.count(i)) {
          continue;
        }

        float d = b.kf->mahalanobisDistance2(measurements[i]);
        if (d < best_dist) {
          best_dist = d;
          best_idx = static_cast<int>(i);
        }
      }

      if (best_idx >= 0) {
        // Detection update
        b.kf->update(measurements[best_idx]);
        b.existence = 1.0f - (1.0f - b.existence) * (1.0f - detection_prob_);
        used_measurements.insert(best_idx);

        info.updates[id].measurement_weights[best_idx] = 1.0f;
      } else {
        // Missed detection
        b.existence *= (1.0f - detection_prob_);
      }
    }

    // 3. Prune weak tracks
    std::vector<uint32_t> to_remove;
    for (auto & [id, b] : tracks_) {
      if (b.existence < prune_threshold_) {
        to_remove.push_back(id);
        info.deaths.insert(id);
      }
    }
    for (auto id : to_remove) {
      tracks_.erase(id);
    }

    // 4. Convert Poisson (unassigned measurements) to Bernoulli births
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (used_measurements.count(i)) {
        continue;
      }

      Eigen::Vector4f x0;
      x0.head<2>() = measurements[i];
      x0.tail<2>() = Eigen::Vector2f::Zero();

      auto kf = std::make_shared<KalmanFilter2D>(
        x0, kf_pos_uncertainty_, kf_vel_uncertainty_,
        kf_acc_uncertainty_);

      Bernoulli b;
      b.kf = kf;
      b.existence = birth_intensity_;

      uint32_t new_id = next_id_++;
      tracks_[new_id] = b;

      info.births.insert(new_id);
    }

    return info;
  }

  const std::map<uint32_t, Track> & getTracks() const override
  {
    // Convert internal Bernoulli to Track interface
    cached_tracks_.clear();
    for (const auto & [id, b] : tracks_) {
      Track t;
      t.kf = b.kf;
      t.existence_probability = b.existence;
      cached_tracks_[id] = t;
    }
    return cached_tracks_;
  }

private:
  std::map<uint32_t, Bernoulli> tracks_;
  mutable std::map<uint32_t, Track> cached_tracks_;

  uint32_t next_id_ = 0;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_update_time_;

  float max_dt_;
  float gate_threshold_;
  float survival_prob_;
  float detection_prob_;
  float birth_intensity_;
  float prune_threshold_;
  float kf_pos_uncertainty_;
  float kf_vel_uncertainty_;
  float kf_acc_uncertainty_;
};

}  // namespace lidar_objects_tracker

#endif  // LIDAR_OBJECTS_TRACKER__PMBM_TRACKER_HPP_
