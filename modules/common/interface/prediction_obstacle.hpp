/**
 * @file    prediction_obstacle.hpp
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

#include "modules/common/interface/obstacle_intent.hpp"
#include "modules/common/interface/obstacle_interactive_tag.hpp"
#include "modules/common/interface/obstacle_priority.hpp"
#include "modules/common/interface/perception_obstacle.hpp"
#include "modules/common/interface/trajectory_in_prediction.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PredictionObstacle {
public:
  PredictionObstacle() {
    trajectory_mutex_ = std::make_shared<std::mutex>();

    timestamp_ = 0.0;
    predicted_period_ = 0.0;
    clear_trajectory();
    is_static_ = false;
  }
  ~PredictionObstacle() = default;

  inline void set_perception_obstacle(
      const legionclaw::interface::PerceptionObstacle &perception_obstacle) {
    perception_obstacle_ = perception_obstacle;
    perception_obstacle_ptr_ = &perception_obstacle_;
  }

  inline const legionclaw::interface::PerceptionObstacle &
  perception_obstacle() const {
    return perception_obstacle_;
  }

  inline legionclaw::interface::PerceptionObstacle *mutable_perception_obstacle() {
    return &perception_obstacle_;
  }

  inline bool has_perception_obstacle() {
    return (perception_obstacle_ptr_ != nullptr);
  }

  inline void set_timestamp(const double &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_predicted_period(const double &predicted_period) {
    predicted_period_ = predicted_period;
    predicted_period_ptr_ = &predicted_period_;
  }

  inline const double &predicted_period() const { return predicted_period_; }

  inline double *mutable_predicted_period() { return &predicted_period_; }

  inline bool has_predicted_period() {
    return (predicted_period_ptr_ != nullptr);
  }

  inline void set_trajectory(
      std::vector<legionclaw::interface::TrajectoryInPrediction> *trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_.assign(trajectory->begin(), trajectory->end());
  }

  inline void set_trajectory(
      const std::vector<legionclaw::interface::TrajectoryInPrediction> &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_ = trajectory;
  }

  inline void
  set_trajectory(const uint32_t index,
                 legionclaw::interface::TrajectoryInPrediction &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_[index] = trajectory;
  }

  inline void
  add_trajectory(const legionclaw::interface::TrajectoryInPrediction &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_.emplace_back(trajectory);
  }

  inline const legionclaw::interface::TrajectoryInPrediction &
  trajectory(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return trajectory_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryInPrediction> *
  mutable_trajectory() {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return &trajectory_;
  }

  inline void trajectory(
      std::vector<legionclaw::interface::TrajectoryInPrediction> &trajectory) const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory.assign(trajectory_.begin(), trajectory_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryInPrediction> &
  trajectory() const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return trajectory_;
  }

  inline uint32_t trajectory_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return trajectory_.size();
  }

  inline void clear_trajectory() {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_.clear();
    trajectory_.shrink_to_fit();
  }

  inline bool has_trajectory() { return (trajectory_size() != 0); }

  inline void set_intent(const legionclaw::interface::ObstacleIntent &intent) {
    intent_ = intent;
    intent_ptr_ = &intent_;
  }

  inline const legionclaw::interface::ObstacleIntent &intent() const {
    return intent_;
  }

  inline legionclaw::interface::ObstacleIntent *mutable_intent() { return &intent_; }

  inline bool has_intent() { return (intent_ptr_ != nullptr); }

  inline void set_priority(const legionclaw::interface::ObstaclePriority &priority) {
    priority_ = priority;
    priority_ptr_ = &priority_;
  }

  inline const legionclaw::interface::ObstaclePriority &priority() const {
    return priority_;
  }

  inline legionclaw::interface::ObstaclePriority *mutable_priority() {
    return &priority_;
  }

  inline bool has_priority() { return (priority_ptr_ != nullptr); }

  inline void set_interactive_tag(
      const legionclaw::interface::ObstacleInteractiveTag &interactive_tag) {
    interactive_tag_ = interactive_tag;
    interactive_tag_ptr_ = &interactive_tag_;
  }

  inline const legionclaw::interface::ObstacleInteractiveTag &
  interactive_tag() const {
    return interactive_tag_;
  }

  inline legionclaw::interface::ObstacleInteractiveTag *mutable_interactive_tag() {
    return &interactive_tag_;
  }

  inline bool has_interactive_tag() {
    return (interactive_tag_ptr_ != nullptr);
  }

  inline void set_is_static(const bool &is_static) {
    is_static_ = is_static;
    is_static_ptr_ = &is_static_;
  }

  inline const bool &is_static() const { return is_static_; }

  inline bool *mutable_is_static() { return &is_static_; }

  inline bool has_is_static() { return (is_static_ptr_ != nullptr); }

  void operator=(const PredictionObstacle &prediction_obstacle) {
    CopyFrom(prediction_obstacle);
  }

  void CopyFrom(const PredictionObstacle &prediction_obstacle) {
    perception_obstacle_ = prediction_obstacle.perception_obstacle();
    timestamp_ = prediction_obstacle.timestamp();
    predicted_period_ = prediction_obstacle.predicted_period();
    trajectory_ = prediction_obstacle.trajectory();
    intent_ = prediction_obstacle.intent();
    priority_ = prediction_obstacle.priority();
    interactive_tag_ = prediction_obstacle.interactive_tag();
    is_static_ = prediction_obstacle.is_static();
  }

protected:
  std::shared_ptr<std::mutex> trajectory_mutex_;
  // perception info of obstacle
  legionclaw::interface::PerceptionObstacle perception_obstacle_;
  legionclaw::interface::PerceptionObstacle *perception_obstacle_ptr_ = nullptr;
  // GPS time in seconds
  double timestamp_;
  double *timestamp_ptr_ = nullptr;
  // the length of the time for this prediction (e.g. 10s)
  double predicted_period_;
  double *predicted_period_ptr_ = nullptr;
  // can have multiple trajectories per obstacle
  std::vector<legionclaw::interface::TrajectoryInPrediction> trajectory_;
  //估计障碍物的意图        0-UNKNOWN, 1-STOP, 2-STATIONARY, 3-移动 MOVING,
  //4-HANGE_LANE, 5-LOW_ACCELERATION, 6-HIGH_ACCELERATION, 7-LOW_DECELERATION,
  //8-HIGH_DECELERATION,
  legionclaw::interface::ObstacleIntent intent_;
  legionclaw::interface::ObstacleIntent *intent_ptr_ = nullptr;
  //障碍物优先级               1-CAUTION, 2-NORMAL, 3-IGNORE
  legionclaw::interface::ObstaclePriority priority_;
  legionclaw::interface::ObstaclePriority *priority_ptr_ = nullptr;
  legionclaw::interface::ObstacleInteractiveTag interactive_tag_;
  legionclaw::interface::ObstacleInteractiveTag *interactive_tag_ptr_ = nullptr;
  // is obstacle static (default = false)
  bool is_static_;
  bool *is_static_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
