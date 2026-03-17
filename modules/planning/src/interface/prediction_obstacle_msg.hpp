/**
 * @file    prediction_obstacle_msg.hpp
 * @author  zdhy
 * @date    2021-07-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <stdint.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>


#include "modules/common/interface/obstacle_intent.hpp"
#include "modules/planning/src/interface/obstacle_msg.hpp"
// #include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/interface/trajectory_in_prediction.hpp"

using namespace legionclaw::common::math;
/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
class PredictionObstacleMsg {
 public:
  PredictionObstacleMsg() {
    trajectory_mutex_ = std::make_shared<std::mutex>();
    polygon_lists_mutex_ = std::make_shared<std::mutex>();

    timestamp_ = 0.0;
    predicted_period_ = 0.0;
    clear_trajectory();
    clear_polygon_lists();
    // intent_ = legionclaw::interface::ObstacleIntent::Type::INTENT_UNKNOWN_STATE;
    is_static_ = false;
    id_ = "";
  }
  ~PredictionObstacleMsg() = default;

  inline void set_perception_obstacle(
      const legionclaw::planning::ObstacleMsg &perception_obstacle) {
    perception_obstacle_ = perception_obstacle;
  }

  inline const legionclaw::planning::ObstacleMsg &perception_obstacle() const {
    return perception_obstacle_;
  }

  inline legionclaw::planning::ObstacleMsg *mutable_perception_obstacle() {
    return &perception_obstacle_;
  }

  inline void set_timestamp(const double &timestamp) { timestamp_ = timestamp; }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline void set_predicted_period(const double &predicted_period) {
    predicted_period_ = predicted_period;
  }

  inline const double &predicted_period() const { return predicted_period_; }

  inline double *mutable_predicted_period() { return &predicted_period_; }

  inline void set_trajectory(
      std::vector<legionclaw::interface::TrajectoryInPrediction> *trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_.assign(trajectory->begin(), trajectory->end());
  }

  inline void set_trajectory(
      const std::vector<legionclaw::interface::TrajectoryInPrediction>
          &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_ = trajectory;
  }

  inline void set_trajectory(
      const uint32_t index,
      legionclaw::interface::TrajectoryInPrediction &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_[index] = trajectory;
  }

  inline void add_trajectory(
      const legionclaw::interface::TrajectoryInPrediction &trajectory) {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory_.emplace_back(trajectory);
  }

  inline const legionclaw::interface::TrajectoryInPrediction &trajectory(
      uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return trajectory_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryInPrediction>
      *mutable_trajectory() {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    return &trajectory_;
  }

  inline void trajectory(std::vector<legionclaw::interface::TrajectoryInPrediction>
                             &trajectory) const {
    std::lock_guard<std::mutex> lock(*trajectory_mutex_);
    trajectory.assign(trajectory_.begin(), trajectory_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryInPrediction>
      &trajectory() const {
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
  }

  inline void set_polygon_lists(
      std::vector<std::vector<Polygon2d>> *polygon_lists) {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists_.assign(polygon_lists->begin(), polygon_lists->end());
  }

  inline void set_polygon_lists(
      const std::vector<std::vector<Polygon2d>> &polygon_lists) {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists_ = polygon_lists;
  }

  inline void set_polygon_lists(const uint32_t index,
                                std::vector<Polygon2d> &polygon_lists) {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists_[index] = polygon_lists;
  }

  inline void add_polygon_lists(const std::vector<Polygon2d> &polygon_lists) {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists_.emplace_back(polygon_lists);
  }

  inline const std::vector<Polygon2d> &polygon_lists(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    return polygon_lists_[index];
  }

  inline std::vector<std::vector<Polygon2d>> *mutable_polygon_lists() {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    return &polygon_lists_;
  }

  inline void polygon_lists(
      std::vector<std::vector<Polygon2d>> &polygon_lists) const {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists.assign(polygon_lists_.begin(), polygon_lists_.end());
  }

  inline const std::vector<std::vector<Polygon2d>> &polygon_lists() const {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    return polygon_lists_;
  }

  inline uint32_t polygon_lists_size() const {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    return polygon_lists_.size();
  }

  inline void clear_polygon_lists() {
    std::lock_guard<std::mutex> lock(*polygon_lists_mutex_);
    polygon_lists_.clear();
  }

  inline void set_intent(const legionclaw::interface::ObstacleIntent& intent) {
    intent_ = intent;
  }

  inline const legionclaw::interface::ObstacleIntent& intent() const {
    return intent_;
  }

  inline legionclaw::interface::ObstacleIntent* mutable_intent() {
    return &intent_;
  }

  inline void set_is_static(const bool &is_static) { is_static_ = is_static; }

  inline const bool &is_static() const { return is_static_; }

  inline bool *mutable_is_static() { return &is_static_; }

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

 protected:
  std::shared_ptr<std::mutex> trajectory_mutex_;
  std::shared_ptr<std::mutex> polygon_lists_mutex_;
  // perception info of obstacle
  legionclaw::planning::ObstacleMsg perception_obstacle_;
  // GPS time in seconds
  double timestamp_;
  // the length of the time for this prediction (e.g. 10s)
  double predicted_period_;
  // can have multiple trajectories per obstacle
  std::vector<legionclaw::interface::TrajectoryInPrediction> trajectory_;
  std::vector<std::vector<Polygon2d>> polygon_lists_;
  //估计障碍物的意图        0-UNKNOWN, 1-STOP, 2-STATIONARY, 3-移动 MOVING,
  // 4-HANGE_LANE, 5-LOW_ACCELERATION, 6-HIGH_ACCELERATION, 7-LOW_DECELERATION,
  // 8-HIGH_DECELERATION,
  legionclaw::interface::ObstacleIntent intent_;
  // is obstacle static (default = false)
  bool is_static_;
  std::string id_;
};
}  // namespace planning
}  // namespace legionclaw
