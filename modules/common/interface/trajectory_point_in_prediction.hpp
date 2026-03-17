/**
 * @file    trajectory_point_in_prediction.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/path_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrajectoryPointInPrediction {
public:
  TrajectoryPointInPrediction() {
    v_ = 0.0;
    a_ = 0.0;
    relative_time_ = 0.0;
  }
  ~TrajectoryPointInPrediction() = default;

  inline void set_path_point(const legionclaw::interface::PathPoint &path_point) {
    path_point_ = path_point;
    path_point_ptr_ = &path_point_;
  }

  inline const legionclaw::interface::PathPoint &path_point() const {
    return path_point_;
  }

  inline legionclaw::interface::PathPoint *mutable_path_point() {
    return &path_point_;
  }

  inline bool has_path_point() { return (path_point_ptr_ != nullptr); }

  inline void set_v(const double &v) {
    v_ = v;
    v_ptr_ = &v_;
  }

  inline const double &v() const { return v_; }

  inline double *mutable_v() { return &v_; }

  inline bool has_v() { return (v_ptr_ != nullptr); }

  inline void set_a(const double &a) {
    a_ = a;
    a_ptr_ = &a_;
  }

  inline const double &a() const { return a_; }

  inline double *mutable_a() { return &a_; }

  inline bool has_a() { return (a_ptr_ != nullptr); }

  inline void set_relative_time(const double &relative_time) {
    relative_time_ = relative_time;
    relative_time_ptr_ = &relative_time_;
  }

  inline const double &relative_time() const { return relative_time_; }

  inline double *mutable_relative_time() { return &relative_time_; }

  inline bool has_relative_time() { return (relative_time_ptr_ != nullptr); }

  void
  operator=(const TrajectoryPointInPrediction &trajectory_point_in_prediction) {
    CopyFrom(trajectory_point_in_prediction);
  }

  void
  CopyFrom(const TrajectoryPointInPrediction &trajectory_point_in_prediction) {
    path_point_ = trajectory_point_in_prediction.path_point();
    v_ = trajectory_point_in_prediction.v();
    a_ = trajectory_point_in_prediction.a();
    relative_time_ = trajectory_point_in_prediction.relative_time();
  }

protected:
  // path point
  legionclaw::interface::PathPoint path_point_;
  legionclaw::interface::PathPoint *path_point_ptr_ = nullptr;
  // linear velocity
  double v_;
  double *v_ptr_ = nullptr;
  // linear acceleration
  double a_;
  double *a_ptr_ = nullptr;
  // relative time from beginning of the trajectory
  double relative_time_;
  double *relative_time_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
