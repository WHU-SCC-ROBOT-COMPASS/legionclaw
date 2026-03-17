/**
 * @file    vehicle_state.hpp
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
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/pose.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleState {
public:
  VehicleState() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    timestamp_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
    heading_ = 0.0;
    kappa_ = 0.0;
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    linear_acceleration_ = 0.0;
    gear_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    front_steering_value_ = 0.0;
    rear_steering_value_ = 0.0;
  }
  ~VehicleState() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_x(const double &x) {
    x_ = x;
    x_ptr_ = &x_;
  }

  inline const double &x() const { return x_; }

  inline double *mutable_x() { return &x_; }

  inline bool has_x() { return (x_ptr_ != nullptr); }

  inline void set_y(const double &y) {
    y_ = y;
    y_ptr_ = &y_;
  }

  inline const double &y() const { return y_; }

  inline double *mutable_y() { return &y_; }

  inline bool has_y() { return (y_ptr_ != nullptr); }

  inline void set_z(const double &z) {
    z_ = z;
    z_ptr_ = &z_;
  }

  inline const double &z() const { return z_; }

  inline double *mutable_z() { return &z_; }

  inline bool has_z() { return (z_ptr_ != nullptr); }

  inline void set_timestamp(const double &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_roll(const double &roll) {
    roll_ = roll;
    roll_ptr_ = &roll_;
  }

  inline const double &roll() const { return roll_; }

  inline double *mutable_roll() { return &roll_; }

  inline bool has_roll() { return (roll_ptr_ != nullptr); }

  inline void set_pitch(const double &pitch) {
    pitch_ = pitch;
    pitch_ptr_ = &pitch_;
  }

  inline const double &pitch() const { return pitch_; }

  inline double *mutable_pitch() { return &pitch_; }

  inline bool has_pitch() { return (pitch_ptr_ != nullptr); }

  inline void set_yaw(const double &yaw) {
    yaw_ = yaw;
    yaw_ptr_ = &yaw_;
  }

  inline const double &yaw() const { return yaw_; }

  inline double *mutable_yaw() { return &yaw_; }

  inline bool has_yaw() { return (yaw_ptr_ != nullptr); }

  inline void set_heading(const double &heading) {
    heading_ = heading;
    heading_ptr_ = &heading_;
  }

  inline const double &heading() const { return heading_; }

  inline double *mutable_heading() { return &heading_; }

  inline bool has_heading() { return (heading_ptr_ != nullptr); }

  inline void set_kappa(const double &kappa) {
    kappa_ = kappa;
    kappa_ptr_ = &kappa_;
  }

  inline const double &kappa() const { return kappa_; }

  inline double *mutable_kappa() { return &kappa_; }

  inline bool has_kappa() { return (kappa_ptr_ != nullptr); }

  inline void set_linear_velocity(const double &linear_velocity) {
    linear_velocity_ = linear_velocity;
    linear_velocity_ptr_ = &linear_velocity_;
  }

  inline const double &linear_velocity() const { return linear_velocity_; }

  inline double *mutable_linear_velocity() { return &linear_velocity_; }

  inline bool has_linear_velocity() {
    return (linear_velocity_ptr_ != nullptr);
  }

  inline void set_angular_velocity(const double &angular_velocity) {
    angular_velocity_ = angular_velocity;
    angular_velocity_ptr_ = &angular_velocity_;
  }

  inline const double &angular_velocity() const { return angular_velocity_; }

  inline double *mutable_angular_velocity() { return &angular_velocity_; }

  inline bool has_angular_velocity() {
    return (angular_velocity_ptr_ != nullptr);
  }

  inline void set_linear_acceleration(const double &linear_acceleration) {
    linear_acceleration_ = linear_acceleration;
    linear_acceleration_ptr_ = &linear_acceleration_;
  }

  inline const double &linear_acceleration() const {
    return linear_acceleration_;
  }

  inline double *mutable_linear_acceleration() { return &linear_acceleration_; }

  inline bool has_linear_acceleration() {
    return (linear_acceleration_ptr_ != nullptr);
  }

  inline void set_gear(const legionclaw::common::GearPosition &gear) {
    gear_ = gear;
    gear_ptr_ = &gear_;
  }

  inline const legionclaw::common::GearPosition &gear() const { return gear_; }

  inline legionclaw::common::GearPosition *mutable_gear() { return &gear_; }

  inline bool has_gear() { return (gear_ptr_ != nullptr); }

  inline void set_driving_mode(const legionclaw::common::DrivingMode &driving_mode) {
    driving_mode_ = driving_mode;
    driving_mode_ptr_ = &driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &driving_mode() const {
    return driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_driving_mode() {
    return &driving_mode_;
  }

  inline bool has_driving_mode() { return (driving_mode_ptr_ != nullptr); }

  inline void set_pose(const legionclaw::interface::Pose &pose) {
    pose_ = pose;
    pose_ptr_ = &pose_;
  }

  inline const legionclaw::interface::Pose &pose() const { return pose_; }

  inline legionclaw::interface::Pose *mutable_pose() { return &pose_; }

  inline bool has_pose() { return (pose_ptr_ != nullptr); }

  inline void set_front_steering_value(const double &front_steering_value) {
    front_steering_value_ = front_steering_value;
    front_steering_value_ptr_ = &front_steering_value_;
  }

  inline const double &front_steering_value() const {
    return front_steering_value_;
  }

  inline double *mutable_front_steering_value() {
    return &front_steering_value_;
  }

  inline bool has_front_steering_value() {
    return (front_steering_value_ptr_ != nullptr);
  }

  inline void set_rear_steering_value(const double &rear_steering_value) {
    rear_steering_value_ = rear_steering_value;
    rear_steering_value_ptr_ = &rear_steering_value_;
  }

  inline const double &rear_steering_value() const {
    return rear_steering_value_;
  }

  inline double *mutable_rear_steering_value() { return &rear_steering_value_; }

  inline bool has_rear_steering_value() {
    return (rear_steering_value_ptr_ != nullptr);
  }

  void operator=(const VehicleState &vehicle_state) { CopyFrom(vehicle_state); }

  void CopyFrom(const VehicleState &vehicle_state) {
    header_ = vehicle_state.header();
    x_ = vehicle_state.x();
    y_ = vehicle_state.y();
    z_ = vehicle_state.z();
    timestamp_ = vehicle_state.timestamp();
    roll_ = vehicle_state.roll();
    pitch_ = vehicle_state.pitch();
    yaw_ = vehicle_state.yaw();
    heading_ = vehicle_state.heading();
    kappa_ = vehicle_state.kappa();
    linear_velocity_ = vehicle_state.linear_velocity();
    angular_velocity_ = vehicle_state.angular_velocity();
    linear_acceleration_ = vehicle_state.linear_acceleration();
    gear_ = vehicle_state.gear();
    driving_mode_ = vehicle_state.driving_mode();
    pose_ = vehicle_state.pose();
    front_steering_value_ = vehicle_state.front_steering_value();
    rear_steering_value_ = vehicle_state.rear_steering_value();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  double x_;
  double *x_ptr_ = nullptr;
  double y_;
  double *y_ptr_ = nullptr;
  double z_;
  double *z_ptr_ = nullptr;
  double timestamp_;
  double *timestamp_ptr_ = nullptr;
  double roll_;
  double *roll_ptr_ = nullptr;
  double pitch_;
  double *pitch_ptr_ = nullptr;
  double yaw_;
  double *yaw_ptr_ = nullptr;
  double heading_;
  double *heading_ptr_ = nullptr;
  double kappa_;
  double *kappa_ptr_ = nullptr;
  double linear_velocity_;
  double *linear_velocity_ptr_ = nullptr;
  double angular_velocity_;
  double *angular_velocity_ptr_ = nullptr;
  double linear_acceleration_;
  double *linear_acceleration_ptr_ = nullptr;
  legionclaw::common::GearPosition gear_;
  legionclaw::common::GearPosition *gear_ptr_ = nullptr;
  legionclaw::common::DrivingMode driving_mode_;
  legionclaw::common::DrivingMode *driving_mode_ptr_ = nullptr;
  legionclaw::interface::Pose pose_;
  legionclaw::interface::Pose *pose_ptr_ = nullptr;
  double front_steering_value_;
  double *front_steering_value_ptr_ = nullptr;
  double rear_steering_value_;
  double *rear_steering_value_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
