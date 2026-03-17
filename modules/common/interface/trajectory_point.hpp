/**
 * @file    trajectory_point.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/enum/enum.h"
#include "modules/common/interface/path_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrajectoryPoint {
public:
  TrajectoryPoint() {
    v_ = 0.0;
    a_ = 0.0;
    relative_time_ = 0.0;
    da_ = 0.0;
    is_steer_valid_ = false;
    front_steer_ = 0.0;
    rear_steer_ = 0.0;
    gear_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
  }
  ~TrajectoryPoint() = default;

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

  inline void set_da(const double &da) {
    da_ = da;
    da_ptr_ = &da_;
  }

  inline const double &da() const { return da_; }

  inline double *mutable_da() { return &da_; }

  inline bool has_da() { return (da_ptr_ != nullptr); }

  inline void set_is_steer_valid(const bool &is_steer_valid) {
    is_steer_valid_ = is_steer_valid;
    is_steer_valid_ptr_ = &is_steer_valid_;
  }

  inline const bool &is_steer_valid() const { return is_steer_valid_; }

  inline bool *mutable_is_steer_valid() { return &is_steer_valid_; }

  inline bool has_is_steer_valid() { return (is_steer_valid_ptr_ != nullptr); }

  inline void set_front_steer(const double &front_steer) {
    front_steer_ = front_steer;
    front_steer_ptr_ = &front_steer_;
  }

  inline const double &front_steer() const { return front_steer_; }

  inline double *mutable_front_steer() { return &front_steer_; }

  inline bool has_front_steer() { return (front_steer_ptr_ != nullptr); }

  inline void set_rear_steer(const double &rear_steer) {
    rear_steer_ = rear_steer;
    rear_steer_ptr_ = &rear_steer_;
  }

  inline const double &rear_steer() const { return rear_steer_; }

  inline double *mutable_rear_steer() { return &rear_steer_; }

  inline bool has_rear_steer() { return (rear_steer_ptr_ != nullptr); }

  inline void set_gear(const legionclaw::common::GearPosition &gear) {
    gear_ = gear;
    gear_ptr_ = &gear_;
  }

  inline const legionclaw::common::GearPosition &gear() const { return gear_; }

  inline legionclaw::common::GearPosition *mutable_gear() { return &gear_; }

  inline bool has_gear() { return (gear_ptr_ != nullptr); }

  void operator=(const TrajectoryPoint &trajectory_point) {
    CopyFrom(trajectory_point);
  }

  void CopyFrom(const TrajectoryPoint &trajectory_point) {
    path_point_ = trajectory_point.path_point();
    v_ = trajectory_point.v();
    a_ = trajectory_point.a();
    relative_time_ = trajectory_point.relative_time();
    da_ = trajectory_point.da();
    is_steer_valid_ = trajectory_point.is_steer_valid();
    front_steer_ = trajectory_point.front_steer();
    rear_steer_ = trajectory_point.rear_steer();
    gear_ = trajectory_point.gear();
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
  // longitudinal jerk
  double da_;
  double *da_ptr_ = nullptr;
  //转向是否有效
  bool is_steer_valid_;
  bool *is_steer_valid_ptr_ = nullptr;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double front_steer_;
  double *front_steer_ptr_ = nullptr;
  //后轮转角
  double rear_steer_;
  double *rear_steer_ptr_ = nullptr;
  //档位
  legionclaw::common::GearPosition gear_;
  legionclaw::common::GearPosition *gear_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
