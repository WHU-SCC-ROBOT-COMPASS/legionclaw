/**
 * @file    example.h
 * @author  editor
 * @date    2020-05-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>



using namespace std;

namespace legionclaw {
namespace localization {
/**
 * @class Uncertainty
 *
 * @brief 消息的描述.
 */
class Uncertainty {
 public:
  Uncertainty() = default;
  ~Uncertainty() = default;

  void set_position_std_dev(legionclaw::interface::Point3D position_std_dev) {
    position_std_dev_ = position_std_dev;
  }

  legionclaw::interface::Point3D position_std_dev() const { return position_std_dev_; }

  inline legionclaw::interface::Point3D *mutable_position_std_dev() {
    return &position_std_dev_;
  }

  void set_orientation_std_dev(legionclaw::interface::Point3D orientation_std_dev) {
    orientation_std_dev_ = orientation_std_dev;
  }

  legionclaw::interface::Point3D orientation_std_dev() const {
    return orientation_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_orientation_std_dev() {
    return &orientation_std_dev_;
  }

  void set_linear_velocity_std_dev(
      legionclaw::interface::Point3D linear_velocity_std_dev) {
    linear_velocity_std_dev_ = linear_velocity_std_dev;
  }

  legionclaw::interface::Point3D linear_velocity_std_dev() const {
    return linear_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_velocity_std_dev() {
    return &linear_velocity_std_dev_;
  }

  void set_linear_acceleration_std_dev(
      legionclaw::interface::Point3D linear_acceleration_std_dev) {
    linear_acceleration_std_dev_ = linear_acceleration_std_dev;
  }

  legionclaw::interface::Point3D linear_acceleration_std_dev() const {
    return linear_acceleration_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration_std_dev() {
    return &linear_acceleration_std_dev_;
  }

  void set_angular_velocity_std_dev(
      legionclaw::interface::Point3D angular_velocity_std_dev) {
    angular_velocity_std_dev_ = angular_velocity_std_dev;
  }

  legionclaw::interface::Point3D angular_velocity_std_dev() const {
    return angular_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity_std_dev() {
    return &angular_velocity_std_dev_;
  }

 protected:
  // Standard deviation of position, east/north/up in meters.
  legionclaw::interface::Point3D position_std_dev_;
  // Standard deviation of quaternion qx/qy/qz, unitless.
  legionclaw::interface::Point3D orientation_std_dev_;
  // Standard deviation of linear velocity, east/north/up in meters per second.
  legionclaw::interface::Point3D linear_velocity_std_dev_;
  // tandard deviation of linear acceleration, right/forward/up in meters
  // persquare second.
  legionclaw::interface::Point3D linear_acceleration_std_dev_;
  // Standard deviation of angular velocity, right/forward/up in radians
  // persecond.
  legionclaw::interface::Point3D angular_velocity_std_dev_;
};
}  // namespace localization
}  // namespace legionclaw
