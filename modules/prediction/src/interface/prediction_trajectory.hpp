/**
 * @file              prediction_trajectory.hpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-07-07 06:29:25
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#pragma once

#include <iostream>

#include "modules/common/interface/trajectory_in_prediction.hpp"

using namespace std;

namespace legionclaw {
namespace prediction {

enum PredictMode
{
 PREDICT_MODE_VELOCITY = 0,
 PREDICT_MODE_VECTORMAP = 1
};

enum PredictDir
{
 PREDICT_DIR_STRAIGHT = 0,
 PREDICT_DIR_LEFT = 1,
 PREDICT_DIR_RIGHT = 2
};

/**
 * @class PredictionTrajectory
 *
 * @brief 消息的描述.
 */
class PredictionTrajectory {
 public:
  PredictionTrajectory() = default;

  virtual ~PredictionTrajectory() = default;

  inline const int predict_mode() const { return predict_mode_; }

  inline void set_predict_mode(const int predict_mode) {
    predict_mode_ = predict_mode;
  }

  inline const int predict_direction() const { return predict_direction_; }

  inline void set_predict_direction(const int predict_direction) {
    predict_direction_ = predict_direction;
  }

  inline const legionclaw::interface::TrajectoryInPrediction trajectory_in_prediction()  const { return trajectory_in_prediction_; }

  inline legionclaw::interface::TrajectoryInPrediction *mutable_trajectory_in_prediction() { return &trajectory_in_prediction_; }

  inline void set_trajectory_in_prediction(legionclaw::interface::TrajectoryInPrediction trajectory_in_prediction) { 
    trajectory_in_prediction_ = trajectory_in_prediction;
  }

 private:
  int predict_mode_ = 0;
  int predict_direction_ = 0;
  legionclaw::interface::TrajectoryInPrediction trajectory_in_prediction_;

};
}  // namespace prediction
}  // namespace legionclaw
