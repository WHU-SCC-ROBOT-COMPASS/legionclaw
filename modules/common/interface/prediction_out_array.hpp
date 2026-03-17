/**
 * @file    prediction_out_array.hpp
 * @author  zdhy
 * @date    2021-12-07
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

#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PredictionOutArray {
public:
  PredictionOutArray() {
    prediction_obstacles_mutex_ = std::make_shared<std::mutex>();

    clear_prediction_obstacles();
    change_origin_flag_ =
        legionclaw::interface::Location::ChangeOriginFlag::CHANGE_NULL;
    start_timestamp_ = 0.0;
    end_timestamp_ = 0.0;
    self_intent_ = legionclaw::interface::PredictionOutArray::SelfIntent::UNKNOWN;
    scenario_ = legionclaw::interface::ADCTrajectory::Scenario::SCENARIO_UNKNOWN;
  }
  ~PredictionOutArray() = default;

  enum SelfIntent {
    UNKNOWN = 0,
    STOP = 1,
    CRUISE = 2,
    CHANGE_LANE = 3,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline void set_prediction_obstacles(
      std::vector<legionclaw::interface::PredictionObstacle> *prediction_obstacles) {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles_.assign(prediction_obstacles->begin(),
                                 prediction_obstacles->end());
  }

  inline void set_prediction_obstacles(
      const std::vector<legionclaw::interface::PredictionObstacle>
          &prediction_obstacles) {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles_ = prediction_obstacles;
  }

  inline void set_prediction_obstacles(
      const uint32_t index,
      legionclaw::interface::PredictionObstacle &prediction_obstacles) {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles_[index] = prediction_obstacles;
  }

  inline void add_prediction_obstacles(
      const legionclaw::interface::PredictionObstacle &prediction_obstacles) {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles_.emplace_back(prediction_obstacles);
  }

  inline const legionclaw::interface::PredictionObstacle &
  prediction_obstacles(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    return prediction_obstacles_[index];
  }

  inline std::vector<legionclaw::interface::PredictionObstacle> *
  mutable_prediction_obstacles() {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    return &prediction_obstacles_;
  }

  inline void prediction_obstacles(
      std::vector<legionclaw::interface::PredictionObstacle> &prediction_obstacles)
      const {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles.assign(prediction_obstacles_.begin(),
                                prediction_obstacles_.end());
  }

  inline const std::vector<legionclaw::interface::PredictionObstacle> &
  prediction_obstacles() const {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    return prediction_obstacles_;
  }

  inline uint32_t prediction_obstacles_size() const {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    return prediction_obstacles_.size();
  }

  inline void clear_prediction_obstacles() {
    std::lock_guard<std::mutex> lock(*prediction_obstacles_mutex_);
    prediction_obstacles_.clear();
  }

  inline void set_change_origin_flag(
      const legionclaw::interface::Location::ChangeOriginFlag &change_origin_flag) {
    change_origin_flag_ = change_origin_flag;
  }

  inline const legionclaw::interface::Location::ChangeOriginFlag &
  change_origin_flag() const {
    return change_origin_flag_;
  }

  inline legionclaw::interface::Location::ChangeOriginFlag *
  mutable_change_origin_flag() {
    return &change_origin_flag_;
  }

  inline void set_start_timestamp(const double &start_timestamp) {
    start_timestamp_ = start_timestamp;
  }

  inline const double &start_timestamp() const { return start_timestamp_; }

  inline double *mutable_start_timestamp() { return &start_timestamp_; }

  inline void set_end_timestamp(const double &end_timestamp) {
    end_timestamp_ = end_timestamp;
  }

  inline const double &end_timestamp() const { return end_timestamp_; }

  inline double *mutable_end_timestamp() { return &end_timestamp_; }

  inline void set_self_intent(
      const legionclaw::interface::PredictionOutArray::SelfIntent &self_intent) {
    self_intent_ = self_intent;
  }

  inline const legionclaw::interface::PredictionOutArray::SelfIntent &
  self_intent() const {
    return self_intent_;
  }

  inline legionclaw::interface::PredictionOutArray::SelfIntent *
  mutable_self_intent() {
    return &self_intent_;
  }

  inline void
  set_scenario(const legionclaw::interface::ADCTrajectory::Scenario &scenario) {
    scenario_ = scenario;
  }

  inline const legionclaw::interface::ADCTrajectory::Scenario &scenario() const {
    return scenario_;
  }

  inline legionclaw::interface::ADCTrajectory::Scenario *mutable_scenario() {
    return &scenario_;
  }

protected:
  std::shared_ptr<std::mutex> prediction_obstacles_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  // make prediction for multiple obstacles
  std::vector<legionclaw::interface::PredictionObstacle> prediction_obstacles_;
  // 0:坐标系切换成功 1:坐标系切换中 2:坐标系切换故障
  legionclaw::interface::Location::ChangeOriginFlag change_origin_flag_;
  // start timestamp
  double start_timestamp_;
  // end timestamp
  double end_timestamp_;
  //自动驾驶车辆意图  0-UNKNOWN, 1-STOP, 2-CRUISE, 3-CHANGE_LANE
  legionclaw::interface::PredictionOutArray::SelfIntent self_intent_;
  //场景              0-UNKNOWN, 1000-CRUISE, 1001-CRUISE_URBAN
  // 1002-CRUISE_HIGHWAY, 2000-JUNCTION  2001-JUNCTION_TRAFFIC_LIGHT
  // 2002-JUNCTION_STOP_SIGN
  legionclaw::interface::ADCTrajectory::Scenario scenario_;
};
} // namespace interface
} // namespace legionclaw
