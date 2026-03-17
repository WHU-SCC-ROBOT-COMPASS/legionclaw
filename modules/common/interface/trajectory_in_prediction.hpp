/**
 * @file    trajectory_in_prediction.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <vector>

#include "modules/common/interface/trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrajectoryInPrediction {
public:
  TrajectoryInPrediction() {
    trajectory_points_mutex_ = std::make_shared<std::mutex>();

    probability_ = 0.0;
    clear_trajectory_points();
  }
  ~TrajectoryInPrediction() = default;

  inline void set_probability(const double &probability) {
    probability_ = probability;
    probability_ptr_ = &probability_;
  }

  inline const double &probability() const { return probability_; }

  inline double *mutable_probability() { return &probability_; }

  inline bool has_probability() { return (probability_ptr_ != nullptr); }

  inline void set_trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> *trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.assign(trajectory_points->begin(),
                              trajectory_points->end());
  }

  inline void set_trajectory_points(
      const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_ = trajectory_points;
  }

  inline void
  set_trajectory_points(const uint32_t index,
                        legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_[index] = trajectory_points;
  }

  inline void add_trajectory_points(
      const legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.emplace_back(trajectory_points);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return &trajectory_points_;
  }

  inline void trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points.assign(trajectory_points_.begin(),
                             trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
  trajectory_points() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_;
  }

  inline uint32_t trajectory_points_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_.size();
  }

  inline void clear_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.clear();
    trajectory_points_.shrink_to_fit();
  }

  inline bool has_trajectory_points() {
    return (trajectory_points_size() != 0);
  }

  void operator=(const TrajectoryInPrediction &trajectory_in_prediction) {
    CopyFrom(trajectory_in_prediction);
  }

  void CopyFrom(const TrajectoryInPrediction &trajectory_in_prediction) {
    probability_ = trajectory_in_prediction.probability();
    trajectory_points_ = trajectory_in_prediction.trajectory_points();
  }

protected:
  std::shared_ptr<std::mutex> trajectory_points_mutex_;
  // probability of this trajectory
  double probability_;
  double *probability_ptr_ = nullptr;
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points_;
};
} // namespace interface
} // namespace legionclaw
