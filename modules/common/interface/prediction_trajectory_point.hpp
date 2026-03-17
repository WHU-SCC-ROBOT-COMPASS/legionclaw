/**
 * @file    prediction_trajectory_point.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PredictionTrajectoryPoint {
public:
  PredictionTrajectoryPoint() { timestamp_ = 0.0; }
  ~PredictionTrajectoryPoint() = default;

  inline void set_predition_path_point(
      const legionclaw::interface::Point3D &predition_path_point) {
    predition_path_point_ = predition_path_point;
    predition_path_point_ptr_ = &predition_path_point_;
  }

  inline const legionclaw::interface::Point3D &predition_path_point() const {
    return predition_path_point_;
  }

  inline legionclaw::interface::Point3D *mutable_predition_path_point() {
    return &predition_path_point_;
  }

  inline bool has_predition_path_point() {
    return (predition_path_point_ptr_ != nullptr);
  }

  inline void set_timestamp(const double &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  void operator=(const PredictionTrajectoryPoint &prediction_trajectory_point) {
    CopyFrom(prediction_trajectory_point);
  }

  void CopyFrom(const PredictionTrajectoryPoint &prediction_trajectory_point) {
    predition_path_point_ = prediction_trajectory_point.predition_path_point();
    timestamp_ = prediction_trajectory_point.timestamp();
  }

protected:
  legionclaw::interface::Point3D predition_path_point_;
  legionclaw::interface::Point3D *predition_path_point_ptr_ = nullptr;
  double timestamp_;
  double *timestamp_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
