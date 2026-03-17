/**
 * @file    pose.hpp
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
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/quaternion.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Pose {
public:
  Pose() { heading_ = 0.0; }
  ~Pose() = default;

  inline void set_position(const legionclaw::interface::PointENU &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::PointENU &position() const {
    return position_;
  }

  inline legionclaw::interface::PointENU *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_orientation(const legionclaw::interface::Quaternion &orientation) {
    orientation_ = orientation;
    orientation_ptr_ = &orientation_;
  }

  inline const legionclaw::interface::Quaternion &orientation() const {
    return orientation_;
  }

  inline legionclaw::interface::Quaternion *mutable_orientation() {
    return &orientation_;
  }

  inline bool has_orientation() { return (orientation_ptr_ != nullptr); }

  inline void
  set_linear_velocity(const legionclaw::interface::Point3D &linear_velocity) {
    linear_velocity_ = linear_velocity;
    linear_velocity_ptr_ = &linear_velocity_;
  }

  inline const legionclaw::interface::Point3D &linear_velocity() const {
    return linear_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_velocity() {
    return &linear_velocity_;
  }

  inline bool has_linear_velocity() {
    return (linear_velocity_ptr_ != nullptr);
  }

  inline void set_linear_acceleration(
      const legionclaw::interface::Point3D &linear_acceleration) {
    linear_acceleration_ = linear_acceleration;
    linear_acceleration_ptr_ = &linear_acceleration_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration() const {
    return linear_acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration() {
    return &linear_acceleration_;
  }

  inline bool has_linear_acceleration() {
    return (linear_acceleration_ptr_ != nullptr);
  }

  inline void
  set_angular_velocity(const legionclaw::interface::Point3D &angular_velocity) {
    angular_velocity_ = angular_velocity;
    angular_velocity_ptr_ = &angular_velocity_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity() const {
    return angular_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity() {
    return &angular_velocity_;
  }

  inline bool has_angular_velocity() {
    return (angular_velocity_ptr_ != nullptr);
  }

  inline void set_heading(const double &heading) {
    heading_ = heading;
    heading_ptr_ = &heading_;
  }

  inline const double &heading() const { return heading_; }

  inline double *mutable_heading() { return &heading_; }

  inline bool has_heading() { return (heading_ptr_ != nullptr); }

  inline void set_linear_acceleration_vrf(
      const legionclaw::interface::Point3D &linear_acceleration_vrf) {
    linear_acceleration_vrf_ = linear_acceleration_vrf;
    linear_acceleration_vrf_ptr_ = &linear_acceleration_vrf_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration_vrf() const {
    return linear_acceleration_vrf_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration_vrf() {
    return &linear_acceleration_vrf_;
  }

  inline bool has_linear_acceleration_vrf() {
    return (linear_acceleration_vrf_ptr_ != nullptr);
  }

  inline void set_angular_velocity_vrf(
      const legionclaw::interface::Point3D &angular_velocity_vrf) {
    angular_velocity_vrf_ = angular_velocity_vrf;
    angular_velocity_vrf_ptr_ = &angular_velocity_vrf_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity_vrf() const {
    return angular_velocity_vrf_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity_vrf() {
    return &angular_velocity_vrf_;
  }

  inline bool has_angular_velocity_vrf() {
    return (angular_velocity_vrf_ptr_ != nullptr);
  }

  inline void set_euler_angles(const legionclaw::interface::Point3D &euler_angles) {
    euler_angles_ = euler_angles;
    euler_angles_ptr_ = &euler_angles_;
  }

  inline const legionclaw::interface::Point3D &euler_angles() const {
    return euler_angles_;
  }

  inline legionclaw::interface::Point3D *mutable_euler_angles() {
    return &euler_angles_;
  }

  inline bool has_euler_angles() { return (euler_angles_ptr_ != nullptr); }

  void operator=(const Pose &pose) { CopyFrom(pose); }

  void CopyFrom(const Pose &pose) {
    position_ = pose.position();
    orientation_ = pose.orientation();
    linear_velocity_ = pose.linear_velocity();
    linear_acceleration_ = pose.linear_acceleration();
    angular_velocity_ = pose.angular_velocity();
    heading_ = pose.heading();
    linear_acceleration_vrf_ = pose.linear_acceleration_vrf();
    angular_velocity_vrf_ = pose.angular_velocity_vrf();
    euler_angles_ = pose.euler_angles();
  }

protected:
  // Position of the vehicle reference point (VRP) in the map reference frame.
  // The VRP is the center of rear axle.
  legionclaw::interface::PointENU position_;
  legionclaw::interface::PointENU *position_ptr_ = nullptr;
  // A quaternion that represents the rotation from the IMU coordinate
  // (Right/Forward/Up) to the world coordinate (East/North/Up).
  legionclaw::interface::Quaternion orientation_;
  legionclaw::interface::Quaternion *orientation_ptr_ = nullptr;
  // Linear velocity of the VRP in the map reference frame.East/north/up in
  // meters per second.
  legionclaw::interface::Point3D linear_velocity_;
  legionclaw::interface::Point3D *linear_velocity_ptr_ = nullptr;
  // Linear acceleration of the VRP in the map reference frame. East/north/up in
  // meters per square second.
  legionclaw::interface::Point3D linear_acceleration_;
  legionclaw::interface::Point3D *linear_acceleration_ptr_ = nullptr;
  // Angular velocity of the vehicle in the map reference frame.Around
  // east/north/up axes in radians per second.
  legionclaw::interface::Point3D angular_velocity_;
  legionclaw::interface::Point3D *angular_velocity_ptr_ = nullptr;
  // The heading is zero when the car is facing East and positive when facing
  // North
  double heading_;
  double *heading_ptr_ = nullptr;
  // Linear acceleration of the VRP in the vehicle reference
  // frame.Right/forward/up in meters per square second.
  legionclaw::interface::Point3D linear_acceleration_vrf_;
  legionclaw::interface::Point3D *linear_acceleration_vrf_ptr_ = nullptr;
  // Angular velocity of the VRP in the vehicle reference frame. Around
  // right/forward/up axes in radians per second.
  legionclaw::interface::Point3D angular_velocity_vrf_;
  legionclaw::interface::Point3D *angular_velocity_vrf_ptr_ = nullptr;
  //// Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
  ///// in world coordinate (East/North/Up)   // The roll, in (-pi/2, pi/2),
  ///corresponds to a rotation around the y-axis.   // The pitch, in [-pi, pi),
  ///corresponds to a rotation around the x-axis.   // The yaw, in [-pi, pi),
  ///corresponds to a rotation around the z-axis.   // The direction of rotation
  ///follows the right-hand rule.
  legionclaw::interface::Point3D euler_angles_;
  legionclaw::interface::Point3D *euler_angles_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
