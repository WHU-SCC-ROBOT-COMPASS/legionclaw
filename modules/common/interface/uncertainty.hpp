/**
 * @file    uncertainty.hpp
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
class Uncertainty {
public:
  Uncertainty() = default;
  ~Uncertainty() = default;

  inline void
  set_position_std_dev(const legionclaw::interface::Point3D &position_std_dev) {
    position_std_dev_ = position_std_dev;
    position_std_dev_ptr_ = &position_std_dev_;
  }

  inline const legionclaw::interface::Point3D &position_std_dev() const {
    return position_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_position_std_dev() {
    return &position_std_dev_;
  }

  inline bool has_position_std_dev() {
    return (position_std_dev_ptr_ != nullptr);
  }

  inline void set_orientation_std_dev(
      const legionclaw::interface::Point3D &orientation_std_dev) {
    orientation_std_dev_ = orientation_std_dev;
    orientation_std_dev_ptr_ = &orientation_std_dev_;
  }

  inline const legionclaw::interface::Point3D &orientation_std_dev() const {
    return orientation_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_orientation_std_dev() {
    return &orientation_std_dev_;
  }

  inline bool has_orientation_std_dev() {
    return (orientation_std_dev_ptr_ != nullptr);
  }

  inline void set_linear_velocity_std_dev(
      const legionclaw::interface::Point3D &linear_velocity_std_dev) {
    linear_velocity_std_dev_ = linear_velocity_std_dev;
    linear_velocity_std_dev_ptr_ = &linear_velocity_std_dev_;
  }

  inline const legionclaw::interface::Point3D &linear_velocity_std_dev() const {
    return linear_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_velocity_std_dev() {
    return &linear_velocity_std_dev_;
  }

  inline bool has_linear_velocity_std_dev() {
    return (linear_velocity_std_dev_ptr_ != nullptr);
  }

  inline void set_linear_acceleration_std_dev(
      const legionclaw::interface::Point3D &linear_acceleration_std_dev) {
    linear_acceleration_std_dev_ = linear_acceleration_std_dev;
    linear_acceleration_std_dev_ptr_ = &linear_acceleration_std_dev_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration_std_dev() const {
    return linear_acceleration_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration_std_dev() {
    return &linear_acceleration_std_dev_;
  }

  inline bool has_linear_acceleration_std_dev() {
    return (linear_acceleration_std_dev_ptr_ != nullptr);
  }

  inline void set_angular_velocity_std_dev(
      const legionclaw::interface::Point3D &angular_velocity_std_dev) {
    angular_velocity_std_dev_ = angular_velocity_std_dev;
    angular_velocity_std_dev_ptr_ = &angular_velocity_std_dev_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity_std_dev() const {
    return angular_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity_std_dev() {
    return &angular_velocity_std_dev_;
  }

  inline bool has_angular_velocity_std_dev() {
    return (angular_velocity_std_dev_ptr_ != nullptr);
  }

  void operator=(const Uncertainty &uncertainty) { CopyFrom(uncertainty); }

  void CopyFrom(const Uncertainty &uncertainty) {
    position_std_dev_ = uncertainty.position_std_dev();
    orientation_std_dev_ = uncertainty.orientation_std_dev();
    linear_velocity_std_dev_ = uncertainty.linear_velocity_std_dev();
    linear_acceleration_std_dev_ = uncertainty.linear_acceleration_std_dev();
    angular_velocity_std_dev_ = uncertainty.angular_velocity_std_dev();
  }

protected:
  // Standard deviation of position, east/north/up in meters.
  legionclaw::interface::Point3D position_std_dev_;
  legionclaw::interface::Point3D *position_std_dev_ptr_ = nullptr;
  // Standard deviation of quaternion qx/qy/qz, unitless.
  legionclaw::interface::Point3D orientation_std_dev_;
  legionclaw::interface::Point3D *orientation_std_dev_ptr_ = nullptr;
  // Standard deviation of linear velocity, east/north/up in meters per second.
  legionclaw::interface::Point3D linear_velocity_std_dev_;
  legionclaw::interface::Point3D *linear_velocity_std_dev_ptr_ = nullptr;
  // Standard deviation of linear acceleration, right/forward/up in meters per
  // square second.
  legionclaw::interface::Point3D linear_acceleration_std_dev_;
  legionclaw::interface::Point3D *linear_acceleration_std_dev_ptr_ = nullptr;
  // Standard deviation of angular velocity, right/forward/up in radians per
  // second.
  legionclaw::interface::Point3D angular_velocity_std_dev_;
  legionclaw::interface::Point3D *angular_velocity_std_dev_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
