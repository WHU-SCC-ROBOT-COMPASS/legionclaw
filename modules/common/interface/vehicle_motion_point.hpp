/**
 * @file    vehicle_motion_point.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleMotionPoint {
public:
  VehicleMotionPoint() { steer_ = 0.0; }
  ~VehicleMotionPoint() = default;

  inline void set_trajectory_point(
      const legionclaw::interface::TrajectoryPoint &trajectory_point) {
    trajectory_point_ = trajectory_point;
    trajectory_point_ptr_ = &trajectory_point_;
  }

  inline const legionclaw::interface::TrajectoryPoint &trajectory_point() const {
    return trajectory_point_;
  }

  inline legionclaw::interface::TrajectoryPoint *mutable_trajectory_point() {
    return &trajectory_point_;
  }

  inline bool has_trajectory_point() {
    return (trajectory_point_ptr_ != nullptr);
  }

  inline void set_steer(const double &steer) {
    steer_ = steer;
    steer_ptr_ = &steer_;
  }

  inline const double &steer() const { return steer_; }

  inline double *mutable_steer() { return &steer_; }

  inline bool has_steer() { return (steer_ptr_ != nullptr); }

  void operator=(const VehicleMotionPoint &vehicle_motion_point) {
    CopyFrom(vehicle_motion_point);
  }

  void CopyFrom(const VehicleMotionPoint &vehicle_motion_point) {
    trajectory_point_ = vehicle_motion_point.trajectory_point();
    steer_ = vehicle_motion_point.steer();
  }

protected:
  // trajectory point
  legionclaw::interface::TrajectoryPoint trajectory_point_;
  legionclaw::interface::TrajectoryPoint *trajectory_point_ptr_ = nullptr;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer_;
  double *steer_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
