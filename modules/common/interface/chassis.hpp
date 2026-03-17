/**
 * @file    chassis.hpp
 * @author  zdhy
 * @date    2024-02-21
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

#include "modules/common/enum/enum.h"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Chassis {
public:
  Chassis() {
    chassis_error_code_mutex_ = std::make_shared<std::mutex>();

    moving_status_ = legionclaw::common::MovingStatus::UNKNOWN_VEHICLE_STATE;
    driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    steer_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    steering_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    front_steering_value_ = 0.0;
    rear_steering_value_ = 0.0;
    steering_torque_nm_ = 0.0;
    front_steering_rate_dps_ = 0.0;
    rear_steering_rate_dps_ = 0.0;
    accel_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    accel_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    accel_value_ = 0.0;
    brake_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    brake_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    brake_value_ = 0.0;
    backup_brake_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    backup_brake_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    backup_brake_value_ = 0.0;
    epb_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    epb_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    epb_level_ = legionclaw::common::EPBLevel::RELEASED;
    engine_status_ = legionclaw::common::EngineStauts::STOPPED;
    engine_rpm_ = 0.0;
    engine_torque_ = 0.0;
    speed_mps_ = 0.0;
    odometer_m_ = 0.0;
    fuel_range_m_ = 0;
    gear_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    gear_status_ = legionclaw::common::ControlStatus::CONTROLLABLE;
    gear_location_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    driver_seat_belt_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    high_beam_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    low_beam_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    horn_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    turn_lamp_status_ = legionclaw::common::TurnSignal::TURN_INACTIVE_SIGNAL;
    front_wiper_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_wiper_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    position_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    front_fog_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_fog_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    brake_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    alarm_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    lf_door_status_ = legionclaw::common::DoorStatus::CLOSEED;
    rf_door_status_ = legionclaw::common::DoorStatus::CLOSEED;
    lr_door_status_ = legionclaw::common::DoorStatus::CLOSEED;
    rr_door_status_ = legionclaw::common::DoorStatus::CLOSEED;
    rearview_mirror_status_ =
        legionclaw::common::FoldUnfoldStatus::FOLD_UNFOLD_STATUS_UNKNOWN;
    trunk_status_ = legionclaw::common::DoorStatus::CLOSEED;
    engine_bay_door_status_ = legionclaw::common::DoorStatus::CLOSEED;
    wheel_direction_rr_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_rr_ = 0.0;
    wheel_direction_rl_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_rl_ = 0.0;
    wheel_direction_fr_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_fr_ = 0.0;
    wheel_direction_fl_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_fl_ = 0.0;
    is_tire_pressure_ok_ = legionclaw::common::FailureStatus::FAILURE_STATUS_UNKNOWN;
    is_tire_pressure_lf_valid_ = legionclaw::common::IsValid::INVALID;
    tire_pressure_lf_ = 0.0;
    is_tire_pressure_rf_valid_ = legionclaw::common::IsValid::INVALID;
    tire_pressure_rf_ = 0.0;
    is_tire_pressure_lr_valid_ = legionclaw::common::IsValid::INVALID;
    tire_pressure_lr_ = 0.0;
    is_tire_pressure_rr_valid_ = legionclaw::common::IsValid::INVALID;
    tire_pressure_rr_ = 0.0;
    battery_power_percentage_ = 0.0;
    air_bag_status_ = legionclaw::common::FailureStatus::FAILURE_STATUS_UNKNOWN;
    charging_gun_status_ = legionclaw::common::PlugStatus::PLUG_STATUS_UNKNOWN;
    vehicle_power_status_ =
        legionclaw::common::FailureStatus::FAILURE_STATUS_UNKNOWN;
    clear_chassis_error_code();
  }
  ~Chassis() = default;

  enum ErrorCode {
    NO_ERROR = 0,
    CMD_NOT_IN_PERIOD = 1,
    CHASSIS_ERROR = 2,
    MANUAL_INTERVENTION = 3,
    CHASSIS_CAN_NOT_IN_PERIOD = 4,
    SPEED_IS_INVALID = 5,
    UNKNOWN_ERROR = 6,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_moving_status(const legionclaw::common::MovingStatus &moving_status) {
    moving_status_ = moving_status;
    moving_status_ptr_ = &moving_status_;
  }

  inline const legionclaw::common::MovingStatus &moving_status() const {
    return moving_status_;
  }

  inline legionclaw::common::MovingStatus *mutable_moving_status() {
    return &moving_status_;
  }

  inline bool has_moving_status() { return (moving_status_ptr_ != nullptr); }

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

  inline void
  set_steer_driving_mode(const legionclaw::common::DrivingMode &steer_driving_mode) {
    steer_driving_mode_ = steer_driving_mode;
    steer_driving_mode_ptr_ = &steer_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &steer_driving_mode() const {
    return steer_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_steer_driving_mode() {
    return &steer_driving_mode_;
  }

  inline bool has_steer_driving_mode() {
    return (steer_driving_mode_ptr_ != nullptr);
  }

  inline void
  set_steering_status(const legionclaw::common::ControlStatus &steering_status) {
    steering_status_ = steering_status;
    steering_status_ptr_ = &steering_status_;
  }

  inline const legionclaw::common::ControlStatus &steering_status() const {
    return steering_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_steering_status() {
    return &steering_status_;
  }

  inline bool has_steering_status() {
    return (steering_status_ptr_ != nullptr);
  }

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

  inline void set_steering_torque_nm(const double &steering_torque_nm) {
    steering_torque_nm_ = steering_torque_nm;
    steering_torque_nm_ptr_ = &steering_torque_nm_;
  }

  inline const double &steering_torque_nm() const {
    return steering_torque_nm_;
  }

  inline double *mutable_steering_torque_nm() { return &steering_torque_nm_; }

  inline bool has_steering_torque_nm() {
    return (steering_torque_nm_ptr_ != nullptr);
  }

  inline void
  set_front_steering_rate_dps(const double &front_steering_rate_dps) {
    front_steering_rate_dps_ = front_steering_rate_dps;
    front_steering_rate_dps_ptr_ = &front_steering_rate_dps_;
  }

  inline const double &front_steering_rate_dps() const {
    return front_steering_rate_dps_;
  }

  inline double *mutable_front_steering_rate_dps() {
    return &front_steering_rate_dps_;
  }

  inline bool has_front_steering_rate_dps() {
    return (front_steering_rate_dps_ptr_ != nullptr);
  }

  inline void set_rear_steering_rate_dps(const double &rear_steering_rate_dps) {
    rear_steering_rate_dps_ = rear_steering_rate_dps;
    rear_steering_rate_dps_ptr_ = &rear_steering_rate_dps_;
  }

  inline const double &rear_steering_rate_dps() const {
    return rear_steering_rate_dps_;
  }

  inline double *mutable_rear_steering_rate_dps() {
    return &rear_steering_rate_dps_;
  }

  inline bool has_rear_steering_rate_dps() {
    return (rear_steering_rate_dps_ptr_ != nullptr);
  }

  inline void
  set_accel_driving_mode(const legionclaw::common::DrivingMode &accel_driving_mode) {
    accel_driving_mode_ = accel_driving_mode;
    accel_driving_mode_ptr_ = &accel_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &accel_driving_mode() const {
    return accel_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_accel_driving_mode() {
    return &accel_driving_mode_;
  }

  inline bool has_accel_driving_mode() {
    return (accel_driving_mode_ptr_ != nullptr);
  }

  inline void
  set_accel_status(const legionclaw::common::ControlStatus &accel_status) {
    accel_status_ = accel_status;
    accel_status_ptr_ = &accel_status_;
  }

  inline const legionclaw::common::ControlStatus &accel_status() const {
    return accel_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_accel_status() {
    return &accel_status_;
  }

  inline bool has_accel_status() { return (accel_status_ptr_ != nullptr); }

  inline void set_accel_value(const double &accel_value) {
    accel_value_ = accel_value;
    accel_value_ptr_ = &accel_value_;
  }

  inline const double &accel_value() const { return accel_value_; }

  inline double *mutable_accel_value() { return &accel_value_; }

  inline bool has_accel_value() { return (accel_value_ptr_ != nullptr); }

  inline void
  set_brake_driving_mode(const legionclaw::common::DrivingMode &brake_driving_mode) {
    brake_driving_mode_ = brake_driving_mode;
    brake_driving_mode_ptr_ = &brake_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &brake_driving_mode() const {
    return brake_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_brake_driving_mode() {
    return &brake_driving_mode_;
  }

  inline bool has_brake_driving_mode() {
    return (brake_driving_mode_ptr_ != nullptr);
  }

  inline void
  set_brake_status(const legionclaw::common::ControlStatus &brake_status) {
    brake_status_ = brake_status;
    brake_status_ptr_ = &brake_status_;
  }

  inline const legionclaw::common::ControlStatus &brake_status() const {
    return brake_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_brake_status() {
    return &brake_status_;
  }

  inline bool has_brake_status() { return (brake_status_ptr_ != nullptr); }

  inline void set_brake_value(const double &brake_value) {
    brake_value_ = brake_value;
    brake_value_ptr_ = &brake_value_;
  }

  inline const double &brake_value() const { return brake_value_; }

  inline double *mutable_brake_value() { return &brake_value_; }

  inline bool has_brake_value() { return (brake_value_ptr_ != nullptr); }

  inline void set_backup_brake_driving_mode(
      const legionclaw::common::DrivingMode &backup_brake_driving_mode) {
    backup_brake_driving_mode_ = backup_brake_driving_mode;
    backup_brake_driving_mode_ptr_ = &backup_brake_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &backup_brake_driving_mode() const {
    return backup_brake_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_backup_brake_driving_mode() {
    return &backup_brake_driving_mode_;
  }

  inline bool has_backup_brake_driving_mode() {
    return (backup_brake_driving_mode_ptr_ != nullptr);
  }

  inline void set_backup_brake_status(
      const legionclaw::common::ControlStatus &backup_brake_status) {
    backup_brake_status_ = backup_brake_status;
    backup_brake_status_ptr_ = &backup_brake_status_;
  }

  inline const legionclaw::common::ControlStatus &backup_brake_status() const {
    return backup_brake_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_backup_brake_status() {
    return &backup_brake_status_;
  }

  inline bool has_backup_brake_status() {
    return (backup_brake_status_ptr_ != nullptr);
  }

  inline void set_backup_brake_value(const double &backup_brake_value) {
    backup_brake_value_ = backup_brake_value;
    backup_brake_value_ptr_ = &backup_brake_value_;
  }

  inline const double &backup_brake_value() const {
    return backup_brake_value_;
  }

  inline double *mutable_backup_brake_value() { return &backup_brake_value_; }

  inline bool has_backup_brake_value() {
    return (backup_brake_value_ptr_ != nullptr);
  }

  inline void
  set_epb_driving_mode(const legionclaw::common::DrivingMode &epb_driving_mode) {
    epb_driving_mode_ = epb_driving_mode;
    epb_driving_mode_ptr_ = &epb_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &epb_driving_mode() const {
    return epb_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_epb_driving_mode() {
    return &epb_driving_mode_;
  }

  inline bool has_epb_driving_mode() {
    return (epb_driving_mode_ptr_ != nullptr);
  }

  inline void set_epb_status(const legionclaw::common::ControlStatus &epb_status) {
    epb_status_ = epb_status;
    epb_status_ptr_ = &epb_status_;
  }

  inline const legionclaw::common::ControlStatus &epb_status() const {
    return epb_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_epb_status() {
    return &epb_status_;
  }

  inline bool has_epb_status() { return (epb_status_ptr_ != nullptr); }

  inline void set_epb_level(const legionclaw::common::EPBLevel &epb_level) {
    epb_level_ = epb_level;
    epb_level_ptr_ = &epb_level_;
  }

  inline const legionclaw::common::EPBLevel &epb_level() const { return epb_level_; }

  inline legionclaw::common::EPBLevel *mutable_epb_level() { return &epb_level_; }

  inline bool has_epb_level() { return (epb_level_ptr_ != nullptr); }

  inline void
  set_engine_status(const legionclaw::common::EngineStauts &engine_status) {
    engine_status_ = engine_status;
    engine_status_ptr_ = &engine_status_;
  }

  inline const legionclaw::common::EngineStauts &engine_status() const {
    return engine_status_;
  }

  inline legionclaw::common::EngineStauts *mutable_engine_status() {
    return &engine_status_;
  }

  inline bool has_engine_status() { return (engine_status_ptr_ != nullptr); }

  inline void set_engine_rpm(const double &engine_rpm) {
    engine_rpm_ = engine_rpm;
    engine_rpm_ptr_ = &engine_rpm_;
  }

  inline const double &engine_rpm() const { return engine_rpm_; }

  inline double *mutable_engine_rpm() { return &engine_rpm_; }

  inline bool has_engine_rpm() { return (engine_rpm_ptr_ != nullptr); }

  inline void set_engine_torque(const double &engine_torque) {
    engine_torque_ = engine_torque;
    engine_torque_ptr_ = &engine_torque_;
  }

  inline const double &engine_torque() const { return engine_torque_; }

  inline double *mutable_engine_torque() { return &engine_torque_; }

  inline bool has_engine_torque() { return (engine_torque_ptr_ != nullptr); }

  inline void set_speed_mps(const double &speed_mps) {
    speed_mps_ = speed_mps;
    speed_mps_ptr_ = &speed_mps_;
  }

  inline const double &speed_mps() const { return speed_mps_; }

  inline double *mutable_speed_mps() { return &speed_mps_; }

  inline bool has_speed_mps() { return (speed_mps_ptr_ != nullptr); }

  inline void set_odometer_m(const double &odometer_m) {
    odometer_m_ = odometer_m;
    odometer_m_ptr_ = &odometer_m_;
  }

  inline const double &odometer_m() const { return odometer_m_; }

  inline double *mutable_odometer_m() { return &odometer_m_; }

  inline bool has_odometer_m() { return (odometer_m_ptr_ != nullptr); }

  inline void set_fuel_range_m(const int32_t &fuel_range_m) {
    fuel_range_m_ = fuel_range_m;
    fuel_range_m_ptr_ = &fuel_range_m_;
  }

  inline const int32_t &fuel_range_m() const { return fuel_range_m_; }

  inline int32_t *mutable_fuel_range_m() { return &fuel_range_m_; }

  inline bool has_fuel_range_m() { return (fuel_range_m_ptr_ != nullptr); }

  inline void
  set_gear_driving_mode(const legionclaw::common::DrivingMode &gear_driving_mode) {
    gear_driving_mode_ = gear_driving_mode;
    gear_driving_mode_ptr_ = &gear_driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &gear_driving_mode() const {
    return gear_driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_gear_driving_mode() {
    return &gear_driving_mode_;
  }

  inline bool has_gear_driving_mode() {
    return (gear_driving_mode_ptr_ != nullptr);
  }

  inline void set_gear_status(const legionclaw::common::ControlStatus &gear_status) {
    gear_status_ = gear_status;
    gear_status_ptr_ = &gear_status_;
  }

  inline const legionclaw::common::ControlStatus &gear_status() const {
    return gear_status_;
  }

  inline legionclaw::common::ControlStatus *mutable_gear_status() {
    return &gear_status_;
  }

  inline bool has_gear_status() { return (gear_status_ptr_ != nullptr); }

  inline void
  set_gear_location(const legionclaw::common::GearPosition &gear_location) {
    gear_location_ = gear_location;
    gear_location_ptr_ = &gear_location_;
  }

  inline const legionclaw::common::GearPosition &gear_location() const {
    return gear_location_;
  }

  inline legionclaw::common::GearPosition *mutable_gear_location() {
    return &gear_location_;
  }

  inline bool has_gear_location() { return (gear_location_ptr_ != nullptr); }

  inline void
  set_driver_seat_belt(const legionclaw::common::SwitchStatus &driver_seat_belt) {
    driver_seat_belt_ = driver_seat_belt;
    driver_seat_belt_ptr_ = &driver_seat_belt_;
  }

  inline const legionclaw::common::SwitchStatus &driver_seat_belt() const {
    return driver_seat_belt_;
  }

  inline legionclaw::common::SwitchStatus *mutable_driver_seat_belt() {
    return &driver_seat_belt_;
  }

  inline bool has_driver_seat_belt() {
    return (driver_seat_belt_ptr_ != nullptr);
  }

  inline void
  set_high_beam_status(const legionclaw::common::SwitchStatus &high_beam_status) {
    high_beam_status_ = high_beam_status;
    high_beam_status_ptr_ = &high_beam_status_;
  }

  inline const legionclaw::common::SwitchStatus &high_beam_status() const {
    return high_beam_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_high_beam_status() {
    return &high_beam_status_;
  }

  inline bool has_high_beam_status() {
    return (high_beam_status_ptr_ != nullptr);
  }

  inline void
  set_low_beam_status(const legionclaw::common::SwitchStatus &low_beam_status) {
    low_beam_status_ = low_beam_status;
    low_beam_status_ptr_ = &low_beam_status_;
  }

  inline const legionclaw::common::SwitchStatus &low_beam_status() const {
    return low_beam_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_low_beam_status() {
    return &low_beam_status_;
  }

  inline bool has_low_beam_status() {
    return (low_beam_status_ptr_ != nullptr);
  }

  inline void set_horn_status(const legionclaw::common::SwitchStatus &horn_status) {
    horn_status_ = horn_status;
    horn_status_ptr_ = &horn_status_;
  }

  inline const legionclaw::common::SwitchStatus &horn_status() const {
    return horn_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_horn_status() {
    return &horn_status_;
  }

  inline bool has_horn_status() { return (horn_status_ptr_ != nullptr); }

  inline void
  set_turn_lamp_status(const legionclaw::common::TurnSignal &turn_lamp_status) {
    turn_lamp_status_ = turn_lamp_status;
    turn_lamp_status_ptr_ = &turn_lamp_status_;
  }

  inline const legionclaw::common::TurnSignal &turn_lamp_status() const {
    return turn_lamp_status_;
  }

  inline legionclaw::common::TurnSignal *mutable_turn_lamp_status() {
    return &turn_lamp_status_;
  }

  inline bool has_turn_lamp_status() {
    return (turn_lamp_status_ptr_ != nullptr);
  }

  inline void set_front_wiper_status(
      const legionclaw::common::SwitchStatus &front_wiper_status) {
    front_wiper_status_ = front_wiper_status;
    front_wiper_status_ptr_ = &front_wiper_status_;
  }

  inline const legionclaw::common::SwitchStatus &front_wiper_status() const {
    return front_wiper_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_wiper_status() {
    return &front_wiper_status_;
  }

  inline bool has_front_wiper_status() {
    return (front_wiper_status_ptr_ != nullptr);
  }

  inline void
  set_rear_wiper_status(const legionclaw::common::SwitchStatus &rear_wiper_status) {
    rear_wiper_status_ = rear_wiper_status;
    rear_wiper_status_ptr_ = &rear_wiper_status_;
  }

  inline const legionclaw::common::SwitchStatus &rear_wiper_status() const {
    return rear_wiper_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_wiper_status() {
    return &rear_wiper_status_;
  }

  inline bool has_rear_wiper_status() {
    return (rear_wiper_status_ptr_ != nullptr);
  }

  inline void set_position_lamp_status(
      const legionclaw::common::SwitchStatus &position_lamp_status) {
    position_lamp_status_ = position_lamp_status;
    position_lamp_status_ptr_ = &position_lamp_status_;
  }

  inline const legionclaw::common::SwitchStatus &position_lamp_status() const {
    return position_lamp_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_position_lamp_status() {
    return &position_lamp_status_;
  }

  inline bool has_position_lamp_status() {
    return (position_lamp_status_ptr_ != nullptr);
  }

  inline void set_front_fog_lamp_status(
      const legionclaw::common::SwitchStatus &front_fog_lamp_status) {
    front_fog_lamp_status_ = front_fog_lamp_status;
    front_fog_lamp_status_ptr_ = &front_fog_lamp_status_;
  }

  inline const legionclaw::common::SwitchStatus &front_fog_lamp_status() const {
    return front_fog_lamp_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_fog_lamp_status() {
    return &front_fog_lamp_status_;
  }

  inline bool has_front_fog_lamp_status() {
    return (front_fog_lamp_status_ptr_ != nullptr);
  }

  inline void set_rear_fog_lamp_status(
      const legionclaw::common::SwitchStatus &rear_fog_lamp_status) {
    rear_fog_lamp_status_ = rear_fog_lamp_status;
    rear_fog_lamp_status_ptr_ = &rear_fog_lamp_status_;
  }

  inline const legionclaw::common::SwitchStatus &rear_fog_lamp_status() const {
    return rear_fog_lamp_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_fog_lamp_status() {
    return &rear_fog_lamp_status_;
  }

  inline bool has_rear_fog_lamp_status() {
    return (rear_fog_lamp_status_ptr_ != nullptr);
  }

  inline void
  set_brake_lamp_status(const legionclaw::common::SwitchStatus &brake_lamp_status) {
    brake_lamp_status_ = brake_lamp_status;
    brake_lamp_status_ptr_ = &brake_lamp_status_;
  }

  inline const legionclaw::common::SwitchStatus &brake_lamp_status() const {
    return brake_lamp_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_brake_lamp_status() {
    return &brake_lamp_status_;
  }

  inline bool has_brake_lamp_status() {
    return (brake_lamp_status_ptr_ != nullptr);
  }

  inline void
  set_alarm_lamp_status(const legionclaw::common::SwitchStatus &alarm_lamp_status) {
    alarm_lamp_status_ = alarm_lamp_status;
    alarm_lamp_status_ptr_ = &alarm_lamp_status_;
  }

  inline const legionclaw::common::SwitchStatus &alarm_lamp_status() const {
    return alarm_lamp_status_;
  }

  inline legionclaw::common::SwitchStatus *mutable_alarm_lamp_status() {
    return &alarm_lamp_status_;
  }

  inline bool has_alarm_lamp_status() {
    return (alarm_lamp_status_ptr_ != nullptr);
  }

  inline void
  set_lf_door_status(const legionclaw::common::DoorStatus &lf_door_status) {
    lf_door_status_ = lf_door_status;
    lf_door_status_ptr_ = &lf_door_status_;
  }

  inline const legionclaw::common::DoorStatus &lf_door_status() const {
    return lf_door_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_lf_door_status() {
    return &lf_door_status_;
  }

  inline bool has_lf_door_status() { return (lf_door_status_ptr_ != nullptr); }

  inline void
  set_rf_door_status(const legionclaw::common::DoorStatus &rf_door_status) {
    rf_door_status_ = rf_door_status;
    rf_door_status_ptr_ = &rf_door_status_;
  }

  inline const legionclaw::common::DoorStatus &rf_door_status() const {
    return rf_door_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_rf_door_status() {
    return &rf_door_status_;
  }

  inline bool has_rf_door_status() { return (rf_door_status_ptr_ != nullptr); }

  inline void
  set_lr_door_status(const legionclaw::common::DoorStatus &lr_door_status) {
    lr_door_status_ = lr_door_status;
    lr_door_status_ptr_ = &lr_door_status_;
  }

  inline const legionclaw::common::DoorStatus &lr_door_status() const {
    return lr_door_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_lr_door_status() {
    return &lr_door_status_;
  }

  inline bool has_lr_door_status() { return (lr_door_status_ptr_ != nullptr); }

  inline void
  set_rr_door_status(const legionclaw::common::DoorStatus &rr_door_status) {
    rr_door_status_ = rr_door_status;
    rr_door_status_ptr_ = &rr_door_status_;
  }

  inline const legionclaw::common::DoorStatus &rr_door_status() const {
    return rr_door_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_rr_door_status() {
    return &rr_door_status_;
  }

  inline bool has_rr_door_status() { return (rr_door_status_ptr_ != nullptr); }

  inline void set_rearview_mirror_status(
      const legionclaw::common::FoldUnfoldStatus &rearview_mirror_status) {
    rearview_mirror_status_ = rearview_mirror_status;
    rearview_mirror_status_ptr_ = &rearview_mirror_status_;
  }

  inline const legionclaw::common::FoldUnfoldStatus &rearview_mirror_status() const {
    return rearview_mirror_status_;
  }

  inline legionclaw::common::FoldUnfoldStatus *mutable_rearview_mirror_status() {
    return &rearview_mirror_status_;
  }

  inline bool has_rearview_mirror_status() {
    return (rearview_mirror_status_ptr_ != nullptr);
  }

  inline void set_trunk_status(const legionclaw::common::DoorStatus &trunk_status) {
    trunk_status_ = trunk_status;
    trunk_status_ptr_ = &trunk_status_;
  }

  inline const legionclaw::common::DoorStatus &trunk_status() const {
    return trunk_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_trunk_status() {
    return &trunk_status_;
  }

  inline bool has_trunk_status() { return (trunk_status_ptr_ != nullptr); }

  inline void set_engine_bay_door_status(
      const legionclaw::common::DoorStatus &engine_bay_door_status) {
    engine_bay_door_status_ = engine_bay_door_status;
    engine_bay_door_status_ptr_ = &engine_bay_door_status_;
  }

  inline const legionclaw::common::DoorStatus &engine_bay_door_status() const {
    return engine_bay_door_status_;
  }

  inline legionclaw::common::DoorStatus *mutable_engine_bay_door_status() {
    return &engine_bay_door_status_;
  }

  inline bool has_engine_bay_door_status() {
    return (engine_bay_door_status_ptr_ != nullptr);
  }

  inline void set_wheel_direction_rr(
      const legionclaw::common::WheelSpeedType &wheel_direction_rr) {
    wheel_direction_rr_ = wheel_direction_rr;
    wheel_direction_rr_ptr_ = &wheel_direction_rr_;
  }

  inline const legionclaw::common::WheelSpeedType &wheel_direction_rr() const {
    return wheel_direction_rr_;
  }

  inline legionclaw::common::WheelSpeedType *mutable_wheel_direction_rr() {
    return &wheel_direction_rr_;
  }

  inline bool has_wheel_direction_rr() {
    return (wheel_direction_rr_ptr_ != nullptr);
  }

  inline void set_wheel_spd_rr(const double &wheel_spd_rr) {
    wheel_spd_rr_ = wheel_spd_rr;
    wheel_spd_rr_ptr_ = &wheel_spd_rr_;
  }

  inline const double &wheel_spd_rr() const { return wheel_spd_rr_; }

  inline double *mutable_wheel_spd_rr() { return &wheel_spd_rr_; }

  inline bool has_wheel_spd_rr() { return (wheel_spd_rr_ptr_ != nullptr); }

  inline void set_wheel_direction_rl(
      const legionclaw::common::WheelSpeedType &wheel_direction_rl) {
    wheel_direction_rl_ = wheel_direction_rl;
    wheel_direction_rl_ptr_ = &wheel_direction_rl_;
  }

  inline const legionclaw::common::WheelSpeedType &wheel_direction_rl() const {
    return wheel_direction_rl_;
  }

  inline legionclaw::common::WheelSpeedType *mutable_wheel_direction_rl() {
    return &wheel_direction_rl_;
  }

  inline bool has_wheel_direction_rl() {
    return (wheel_direction_rl_ptr_ != nullptr);
  }

  inline void set_wheel_spd_rl(const double &wheel_spd_rl) {
    wheel_spd_rl_ = wheel_spd_rl;
    wheel_spd_rl_ptr_ = &wheel_spd_rl_;
  }

  inline const double &wheel_spd_rl() const { return wheel_spd_rl_; }

  inline double *mutable_wheel_spd_rl() { return &wheel_spd_rl_; }

  inline bool has_wheel_spd_rl() { return (wheel_spd_rl_ptr_ != nullptr); }

  inline void set_wheel_direction_fr(
      const legionclaw::common::WheelSpeedType &wheel_direction_fr) {
    wheel_direction_fr_ = wheel_direction_fr;
    wheel_direction_fr_ptr_ = &wheel_direction_fr_;
  }

  inline const legionclaw::common::WheelSpeedType &wheel_direction_fr() const {
    return wheel_direction_fr_;
  }

  inline legionclaw::common::WheelSpeedType *mutable_wheel_direction_fr() {
    return &wheel_direction_fr_;
  }

  inline bool has_wheel_direction_fr() {
    return (wheel_direction_fr_ptr_ != nullptr);
  }

  inline void set_wheel_spd_fr(const double &wheel_spd_fr) {
    wheel_spd_fr_ = wheel_spd_fr;
    wheel_spd_fr_ptr_ = &wheel_spd_fr_;
  }

  inline const double &wheel_spd_fr() const { return wheel_spd_fr_; }

  inline double *mutable_wheel_spd_fr() { return &wheel_spd_fr_; }

  inline bool has_wheel_spd_fr() { return (wheel_spd_fr_ptr_ != nullptr); }

  inline void set_wheel_direction_fl(
      const legionclaw::common::WheelSpeedType &wheel_direction_fl) {
    wheel_direction_fl_ = wheel_direction_fl;
    wheel_direction_fl_ptr_ = &wheel_direction_fl_;
  }

  inline const legionclaw::common::WheelSpeedType &wheel_direction_fl() const {
    return wheel_direction_fl_;
  }

  inline legionclaw::common::WheelSpeedType *mutable_wheel_direction_fl() {
    return &wheel_direction_fl_;
  }

  inline bool has_wheel_direction_fl() {
    return (wheel_direction_fl_ptr_ != nullptr);
  }

  inline void set_wheel_spd_fl(const double &wheel_spd_fl) {
    wheel_spd_fl_ = wheel_spd_fl;
    wheel_spd_fl_ptr_ = &wheel_spd_fl_;
  }

  inline const double &wheel_spd_fl() const { return wheel_spd_fl_; }

  inline double *mutable_wheel_spd_fl() { return &wheel_spd_fl_; }

  inline bool has_wheel_spd_fl() { return (wheel_spd_fl_ptr_ != nullptr); }

  inline void set_is_tire_pressure_ok(
      const legionclaw::common::FailureStatus &is_tire_pressure_ok) {
    is_tire_pressure_ok_ = is_tire_pressure_ok;
    is_tire_pressure_ok_ptr_ = &is_tire_pressure_ok_;
  }

  inline const legionclaw::common::FailureStatus &is_tire_pressure_ok() const {
    return is_tire_pressure_ok_;
  }

  inline legionclaw::common::FailureStatus *mutable_is_tire_pressure_ok() {
    return &is_tire_pressure_ok_;
  }

  inline bool has_is_tire_pressure_ok() {
    return (is_tire_pressure_ok_ptr_ != nullptr);
  }

  inline void set_is_tire_pressure_lf_valid(
      const legionclaw::common::IsValid &is_tire_pressure_lf_valid) {
    is_tire_pressure_lf_valid_ = is_tire_pressure_lf_valid;
    is_tire_pressure_lf_valid_ptr_ = &is_tire_pressure_lf_valid_;
  }

  inline const legionclaw::common::IsValid &is_tire_pressure_lf_valid() const {
    return is_tire_pressure_lf_valid_;
  }

  inline legionclaw::common::IsValid *mutable_is_tire_pressure_lf_valid() {
    return &is_tire_pressure_lf_valid_;
  }

  inline bool has_is_tire_pressure_lf_valid() {
    return (is_tire_pressure_lf_valid_ptr_ != nullptr);
  }

  inline void set_tire_pressure_lf(const double &tire_pressure_lf) {
    tire_pressure_lf_ = tire_pressure_lf;
    tire_pressure_lf_ptr_ = &tire_pressure_lf_;
  }

  inline const double &tire_pressure_lf() const { return tire_pressure_lf_; }

  inline double *mutable_tire_pressure_lf() { return &tire_pressure_lf_; }

  inline bool has_tire_pressure_lf() {
    return (tire_pressure_lf_ptr_ != nullptr);
  }

  inline void set_is_tire_pressure_rf_valid(
      const legionclaw::common::IsValid &is_tire_pressure_rf_valid) {
    is_tire_pressure_rf_valid_ = is_tire_pressure_rf_valid;
    is_tire_pressure_rf_valid_ptr_ = &is_tire_pressure_rf_valid_;
  }

  inline const legionclaw::common::IsValid &is_tire_pressure_rf_valid() const {
    return is_tire_pressure_rf_valid_;
  }

  inline legionclaw::common::IsValid *mutable_is_tire_pressure_rf_valid() {
    return &is_tire_pressure_rf_valid_;
  }

  inline bool has_is_tire_pressure_rf_valid() {
    return (is_tire_pressure_rf_valid_ptr_ != nullptr);
  }

  inline void set_tire_pressure_rf(const double &tire_pressure_rf) {
    tire_pressure_rf_ = tire_pressure_rf;
    tire_pressure_rf_ptr_ = &tire_pressure_rf_;
  }

  inline const double &tire_pressure_rf() const { return tire_pressure_rf_; }

  inline double *mutable_tire_pressure_rf() { return &tire_pressure_rf_; }

  inline bool has_tire_pressure_rf() {
    return (tire_pressure_rf_ptr_ != nullptr);
  }

  inline void set_is_tire_pressure_lr_valid(
      const legionclaw::common::IsValid &is_tire_pressure_lr_valid) {
    is_tire_pressure_lr_valid_ = is_tire_pressure_lr_valid;
    is_tire_pressure_lr_valid_ptr_ = &is_tire_pressure_lr_valid_;
  }

  inline const legionclaw::common::IsValid &is_tire_pressure_lr_valid() const {
    return is_tire_pressure_lr_valid_;
  }

  inline legionclaw::common::IsValid *mutable_is_tire_pressure_lr_valid() {
    return &is_tire_pressure_lr_valid_;
  }

  inline bool has_is_tire_pressure_lr_valid() {
    return (is_tire_pressure_lr_valid_ptr_ != nullptr);
  }

  inline void set_tire_pressure_lr(const double &tire_pressure_lr) {
    tire_pressure_lr_ = tire_pressure_lr;
    tire_pressure_lr_ptr_ = &tire_pressure_lr_;
  }

  inline const double &tire_pressure_lr() const { return tire_pressure_lr_; }

  inline double *mutable_tire_pressure_lr() { return &tire_pressure_lr_; }

  inline bool has_tire_pressure_lr() {
    return (tire_pressure_lr_ptr_ != nullptr);
  }

  inline void set_is_tire_pressure_rr_valid(
      const legionclaw::common::IsValid &is_tire_pressure_rr_valid) {
    is_tire_pressure_rr_valid_ = is_tire_pressure_rr_valid;
    is_tire_pressure_rr_valid_ptr_ = &is_tire_pressure_rr_valid_;
  }

  inline const legionclaw::common::IsValid &is_tire_pressure_rr_valid() const {
    return is_tire_pressure_rr_valid_;
  }

  inline legionclaw::common::IsValid *mutable_is_tire_pressure_rr_valid() {
    return &is_tire_pressure_rr_valid_;
  }

  inline bool has_is_tire_pressure_rr_valid() {
    return (is_tire_pressure_rr_valid_ptr_ != nullptr);
  }

  inline void set_tire_pressure_rr(const double &tire_pressure_rr) {
    tire_pressure_rr_ = tire_pressure_rr;
    tire_pressure_rr_ptr_ = &tire_pressure_rr_;
  }

  inline const double &tire_pressure_rr() const { return tire_pressure_rr_; }

  inline double *mutable_tire_pressure_rr() { return &tire_pressure_rr_; }

  inline bool has_tire_pressure_rr() {
    return (tire_pressure_rr_ptr_ != nullptr);
  }

  inline void
  set_battery_power_percentage(const double &battery_power_percentage) {
    battery_power_percentage_ = battery_power_percentage;
    battery_power_percentage_ptr_ = &battery_power_percentage_;
  }

  inline const double &battery_power_percentage() const {
    return battery_power_percentage_;
  }

  inline double *mutable_battery_power_percentage() {
    return &battery_power_percentage_;
  }

  inline bool has_battery_power_percentage() {
    return (battery_power_percentage_ptr_ != nullptr);
  }

  inline void
  set_air_bag_status(const legionclaw::common::FailureStatus &air_bag_status) {
    air_bag_status_ = air_bag_status;
    air_bag_status_ptr_ = &air_bag_status_;
  }

  inline const legionclaw::common::FailureStatus &air_bag_status() const {
    return air_bag_status_;
  }

  inline legionclaw::common::FailureStatus *mutable_air_bag_status() {
    return &air_bag_status_;
  }

  inline bool has_air_bag_status() { return (air_bag_status_ptr_ != nullptr); }

  inline void set_charging_gun_status(
      const legionclaw::common::PlugStatus &charging_gun_status) {
    charging_gun_status_ = charging_gun_status;
    charging_gun_status_ptr_ = &charging_gun_status_;
  }

  inline const legionclaw::common::PlugStatus &charging_gun_status() const {
    return charging_gun_status_;
  }

  inline legionclaw::common::PlugStatus *mutable_charging_gun_status() {
    return &charging_gun_status_;
  }

  inline bool has_charging_gun_status() {
    return (charging_gun_status_ptr_ != nullptr);
  }

  inline void set_vehicle_power_status(
      const legionclaw::common::FailureStatus &vehicle_power_status) {
    vehicle_power_status_ = vehicle_power_status;
    vehicle_power_status_ptr_ = &vehicle_power_status_;
  }

  inline const legionclaw::common::FailureStatus &vehicle_power_status() const {
    return vehicle_power_status_;
  }

  inline legionclaw::common::FailureStatus *mutable_vehicle_power_status() {
    return &vehicle_power_status_;
  }

  inline bool has_vehicle_power_status() {
    return (vehicle_power_status_ptr_ != nullptr);
  }

  inline void set_chassis_error_code(
      std::vector<legionclaw::interface::Chassis::ErrorCode> *chassis_error_code) {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code_.assign(chassis_error_code->begin(),
                               chassis_error_code->end());
  }

  inline void
  set_chassis_error_code(const std::vector<legionclaw::interface::Chassis::ErrorCode>
                             &chassis_error_code) {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code_ = chassis_error_code;
  }

  inline void set_chassis_error_code(
      const uint32_t index,
      legionclaw::interface::Chassis::ErrorCode &chassis_error_code) {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code_[index] = chassis_error_code;
  }

  inline void add_chassis_error_code(
      const legionclaw::interface::Chassis::ErrorCode &chassis_error_code) {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code_.emplace_back(chassis_error_code);
  }

  inline const legionclaw::interface::Chassis::ErrorCode &
  chassis_error_code(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    return chassis_error_code_[index];
  }

  inline std::vector<legionclaw::interface::Chassis::ErrorCode> *
  mutable_chassis_error_code() {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    return &chassis_error_code_;
  }

  inline void chassis_error_code(
      std::vector<legionclaw::interface::Chassis::ErrorCode> &chassis_error_code)
      const {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code.assign(chassis_error_code_.begin(),
                              chassis_error_code_.end());
  }

  inline const std::vector<legionclaw::interface::Chassis::ErrorCode> &
  chassis_error_code() const {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    return chassis_error_code_;
  }

  inline uint32_t chassis_error_code_size() const {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    return chassis_error_code_.size();
  }

  inline void clear_chassis_error_code() {
    std::lock_guard<std::mutex> lock(*chassis_error_code_mutex_);
    chassis_error_code_.clear();
    chassis_error_code_.shrink_to_fit();
  }

  inline bool has_chassis_error_code() {
    return (chassis_error_code_size() != 0);
  }

  void operator=(const Chassis &chassis) { CopyFrom(chassis); }

  void CopyFrom(const Chassis &chassis) {
    header_ = chassis.header();
    moving_status_ = chassis.moving_status();
    driving_mode_ = chassis.driving_mode();
    steer_driving_mode_ = chassis.steer_driving_mode();
    steering_status_ = chassis.steering_status();
    front_steering_value_ = chassis.front_steering_value();
    rear_steering_value_ = chassis.rear_steering_value();
    steering_torque_nm_ = chassis.steering_torque_nm();
    front_steering_rate_dps_ = chassis.front_steering_rate_dps();
    rear_steering_rate_dps_ = chassis.rear_steering_rate_dps();
    accel_driving_mode_ = chassis.accel_driving_mode();
    accel_status_ = chassis.accel_status();
    accel_value_ = chassis.accel_value();
    brake_driving_mode_ = chassis.brake_driving_mode();
    brake_status_ = chassis.brake_status();
    brake_value_ = chassis.brake_value();
    backup_brake_driving_mode_ = chassis.backup_brake_driving_mode();
    backup_brake_status_ = chassis.backup_brake_status();
    backup_brake_value_ = chassis.backup_brake_value();
    epb_driving_mode_ = chassis.epb_driving_mode();
    epb_status_ = chassis.epb_status();
    epb_level_ = chassis.epb_level();
    engine_status_ = chassis.engine_status();
    engine_rpm_ = chassis.engine_rpm();
    engine_torque_ = chassis.engine_torque();
    speed_mps_ = chassis.speed_mps();
    odometer_m_ = chassis.odometer_m();
    fuel_range_m_ = chassis.fuel_range_m();
    gear_driving_mode_ = chassis.gear_driving_mode();
    gear_status_ = chassis.gear_status();
    gear_location_ = chassis.gear_location();
    driver_seat_belt_ = chassis.driver_seat_belt();
    high_beam_status_ = chassis.high_beam_status();
    low_beam_status_ = chassis.low_beam_status();
    horn_status_ = chassis.horn_status();
    turn_lamp_status_ = chassis.turn_lamp_status();
    front_wiper_status_ = chassis.front_wiper_status();
    rear_wiper_status_ = chassis.rear_wiper_status();
    position_lamp_status_ = chassis.position_lamp_status();
    front_fog_lamp_status_ = chassis.front_fog_lamp_status();
    rear_fog_lamp_status_ = chassis.rear_fog_lamp_status();
    brake_lamp_status_ = chassis.brake_lamp_status();
    alarm_lamp_status_ = chassis.alarm_lamp_status();
    lf_door_status_ = chassis.lf_door_status();
    rf_door_status_ = chassis.rf_door_status();
    lr_door_status_ = chassis.lr_door_status();
    rr_door_status_ = chassis.rr_door_status();
    rearview_mirror_status_ = chassis.rearview_mirror_status();
    trunk_status_ = chassis.trunk_status();
    engine_bay_door_status_ = chassis.engine_bay_door_status();
    wheel_direction_rr_ = chassis.wheel_direction_rr();
    wheel_spd_rr_ = chassis.wheel_spd_rr();
    wheel_direction_rl_ = chassis.wheel_direction_rl();
    wheel_spd_rl_ = chassis.wheel_spd_rl();
    wheel_direction_fr_ = chassis.wheel_direction_fr();
    wheel_spd_fr_ = chassis.wheel_spd_fr();
    wheel_direction_fl_ = chassis.wheel_direction_fl();
    wheel_spd_fl_ = chassis.wheel_spd_fl();
    is_tire_pressure_ok_ = chassis.is_tire_pressure_ok();
    is_tire_pressure_lf_valid_ = chassis.is_tire_pressure_lf_valid();
    tire_pressure_lf_ = chassis.tire_pressure_lf();
    is_tire_pressure_rf_valid_ = chassis.is_tire_pressure_rf_valid();
    tire_pressure_rf_ = chassis.tire_pressure_rf();
    is_tire_pressure_lr_valid_ = chassis.is_tire_pressure_lr_valid();
    tire_pressure_lr_ = chassis.tire_pressure_lr();
    is_tire_pressure_rr_valid_ = chassis.is_tire_pressure_rr_valid();
    tire_pressure_rr_ = chassis.tire_pressure_rr();
    battery_power_percentage_ = chassis.battery_power_percentage();
    air_bag_status_ = chassis.air_bag_status();
    charging_gun_status_ = chassis.charging_gun_status();
    vehicle_power_status_ = chassis.vehicle_power_status();
    chassis_error_code_ = chassis.chassis_error_code();
  }

protected:
  std::shared_ptr<std::mutex> chassis_error_code_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //移动状态
  legionclaw::common::MovingStatus moving_status_;
  legionclaw::common::MovingStatus *moving_status_ptr_ = nullptr;
  //驾驶状态
  legionclaw::common::DrivingMode driving_mode_;
  legionclaw::common::DrivingMode *driving_mode_ptr_ = nullptr;
  //线控转向驾驶模式
  legionclaw::common::DrivingMode steer_driving_mode_;
  legionclaw::common::DrivingMode *steer_driving_mode_ptr_ = nullptr;
  //线控转向执行器状态
  legionclaw::common::ControlStatus steering_status_;
  legionclaw::common::ControlStatus *steering_status_ptr_ = nullptr;
  //方向盘转角
  double front_steering_value_;
  double *front_steering_value_ptr_ = nullptr;
  //后轮转角
  double rear_steering_value_;
  double *rear_steering_value_ptr_ = nullptr;
  //方向盘扭矩
  double steering_torque_nm_;
  double *steering_torque_nm_ptr_ = nullptr;
  //方向盘转速
  double front_steering_rate_dps_;
  double *front_steering_rate_dps_ptr_ = nullptr;
  //方向盘转速
  double rear_steering_rate_dps_;
  double *rear_steering_rate_dps_ptr_ = nullptr;
  //线控驱动驾驶模式
  legionclaw::common::DrivingMode accel_driving_mode_;
  legionclaw::common::DrivingMode *accel_driving_mode_ptr_ = nullptr;
  //线控驱动状态
  legionclaw::common::ControlStatus accel_status_;
  legionclaw::common::ControlStatus *accel_status_ptr_ = nullptr;
  //线控驱动值（油门值）
  double accel_value_;
  double *accel_value_ptr_ = nullptr;
  //线控制动驾驶模式
  legionclaw::common::DrivingMode brake_driving_mode_;
  legionclaw::common::DrivingMode *brake_driving_mode_ptr_ = nullptr;
  //线控制动状态
  legionclaw::common::ControlStatus brake_status_;
  legionclaw::common::ControlStatus *brake_status_ptr_ = nullptr;
  //线控制动值（刹车值）
  double brake_value_;
  double *brake_value_ptr_ = nullptr;
  //备用线控制动驾驶模式
  legionclaw::common::DrivingMode backup_brake_driving_mode_;
  legionclaw::common::DrivingMode *backup_brake_driving_mode_ptr_ = nullptr;
  //备用线控制动状态
  legionclaw::common::ControlStatus backup_brake_status_;
  legionclaw::common::ControlStatus *backup_brake_status_ptr_ = nullptr;
  //备用线控制动值
  double backup_brake_value_;
  double *backup_brake_value_ptr_ = nullptr;
  // EPB驾驶模式
  legionclaw::common::DrivingMode epb_driving_mode_;
  legionclaw::common::DrivingMode *epb_driving_mode_ptr_ = nullptr;
  // EPB状态
  legionclaw::common::ControlStatus epb_status_;
  legionclaw::common::ControlStatus *epb_status_ptr_ = nullptr;
  // EPB档位
  legionclaw::common::EPBLevel epb_level_;
  legionclaw::common::EPBLevel *epb_level_ptr_ = nullptr;
  //发动机状态
  legionclaw::common::EngineStauts engine_status_;
  legionclaw::common::EngineStauts *engine_status_ptr_ = nullptr;
  //发动机转速
  double engine_rpm_;
  double *engine_rpm_ptr_ = nullptr;
  //发动机扭矩
  double engine_torque_;
  double *engine_torque_ptr_ = nullptr;
  //车速（m/s）
  double speed_mps_;
  double *speed_mps_ptr_ = nullptr;
  //里程(m)
  double odometer_m_;
  double *odometer_m_ptr_ = nullptr;
  //油表值
  int32_t fuel_range_m_;
  int32_t *fuel_range_m_ptr_ = nullptr;
  //档位驾驶模式
  legionclaw::common::DrivingMode gear_driving_mode_;
  legionclaw::common::DrivingMode *gear_driving_mode_ptr_ = nullptr;
  //档位状态
  legionclaw::common::ControlStatus gear_status_;
  legionclaw::common::ControlStatus *gear_status_ptr_ = nullptr;
  //档位值
  legionclaw::common::GearPosition gear_location_;
  legionclaw::common::GearPosition *gear_location_ptr_ = nullptr;
  //主驾驶位安全带状态
  legionclaw::common::SwitchStatus driver_seat_belt_;
  legionclaw::common::SwitchStatus *driver_seat_belt_ptr_ = nullptr;
  //远光灯状态
  legionclaw::common::SwitchStatus high_beam_status_;
  legionclaw::common::SwitchStatus *high_beam_status_ptr_ = nullptr;
  //近光灯状态
  legionclaw::common::SwitchStatus low_beam_status_;
  legionclaw::common::SwitchStatus *low_beam_status_ptr_ = nullptr;
  //喇叭状态
  legionclaw::common::SwitchStatus horn_status_;
  legionclaw::common::SwitchStatus *horn_status_ptr_ = nullptr;
  //转向灯状态
  legionclaw::common::TurnSignal turn_lamp_status_;
  legionclaw::common::TurnSignal *turn_lamp_status_ptr_ = nullptr;
  //前雨刮器状态
  legionclaw::common::SwitchStatus front_wiper_status_;
  legionclaw::common::SwitchStatus *front_wiper_status_ptr_ = nullptr;
  //后雨刮器状态
  legionclaw::common::SwitchStatus rear_wiper_status_;
  legionclaw::common::SwitchStatus *rear_wiper_status_ptr_ = nullptr;
  //位置灯状态
  legionclaw::common::SwitchStatus position_lamp_status_;
  legionclaw::common::SwitchStatus *position_lamp_status_ptr_ = nullptr;
  //前雾灯状态
  legionclaw::common::SwitchStatus front_fog_lamp_status_;
  legionclaw::common::SwitchStatus *front_fog_lamp_status_ptr_ = nullptr;
  //后雾灯状态
  legionclaw::common::SwitchStatus rear_fog_lamp_status_;
  legionclaw::common::SwitchStatus *rear_fog_lamp_status_ptr_ = nullptr;
  //刹车灯状态
  legionclaw::common::SwitchStatus brake_lamp_status_;
  legionclaw::common::SwitchStatus *brake_lamp_status_ptr_ = nullptr;
  //报警灯状态
  legionclaw::common::SwitchStatus alarm_lamp_status_;
  legionclaw::common::SwitchStatus *alarm_lamp_status_ptr_ = nullptr;
  //左前门状态
  legionclaw::common::DoorStatus lf_door_status_;
  legionclaw::common::DoorStatus *lf_door_status_ptr_ = nullptr;
  //右前门状态
  legionclaw::common::DoorStatus rf_door_status_;
  legionclaw::common::DoorStatus *rf_door_status_ptr_ = nullptr;
  //左后门状态
  legionclaw::common::DoorStatus lr_door_status_;
  legionclaw::common::DoorStatus *lr_door_status_ptr_ = nullptr;
  //右后门状态
  legionclaw::common::DoorStatus rr_door_status_;
  legionclaw::common::DoorStatus *rr_door_status_ptr_ = nullptr;
  //后视镜状态
  legionclaw::common::FoldUnfoldStatus rearview_mirror_status_;
  legionclaw::common::FoldUnfoldStatus *rearview_mirror_status_ptr_ = nullptr;
  //后备箱盖状态
  legionclaw::common::DoorStatus trunk_status_;
  legionclaw::common::DoorStatus *trunk_status_ptr_ = nullptr;
  //发送机舱盖状态
  legionclaw::common::DoorStatus engine_bay_door_status_;
  legionclaw::common::DoorStatus *engine_bay_door_status_ptr_ = nullptr;
  //右后轮方向
  legionclaw::common::WheelSpeedType wheel_direction_rr_;
  legionclaw::common::WheelSpeedType *wheel_direction_rr_ptr_ = nullptr;
  //右后轮速
  double wheel_spd_rr_;
  double *wheel_spd_rr_ptr_ = nullptr;
  //左后轮方向
  legionclaw::common::WheelSpeedType wheel_direction_rl_;
  legionclaw::common::WheelSpeedType *wheel_direction_rl_ptr_ = nullptr;
  //左后轮值
  double wheel_spd_rl_;
  double *wheel_spd_rl_ptr_ = nullptr;
  //右前轮方向
  legionclaw::common::WheelSpeedType wheel_direction_fr_;
  legionclaw::common::WheelSpeedType *wheel_direction_fr_ptr_ = nullptr;
  //右前轮速
  double wheel_spd_fr_;
  double *wheel_spd_fr_ptr_ = nullptr;
  //左前轮方向
  legionclaw::common::WheelSpeedType wheel_direction_fl_;
  legionclaw::common::WheelSpeedType *wheel_direction_fl_ptr_ = nullptr;
  //左前轮速
  double wheel_spd_fl_;
  double *wheel_spd_fl_ptr_ = nullptr;
  //胎压是否正常
  legionclaw::common::FailureStatus is_tire_pressure_ok_;
  legionclaw::common::FailureStatus *is_tire_pressure_ok_ptr_ = nullptr;
  //左前轮胎压是否有效
  legionclaw::common::IsValid is_tire_pressure_lf_valid_;
  legionclaw::common::IsValid *is_tire_pressure_lf_valid_ptr_ = nullptr;
  //左前轮胎压
  double tire_pressure_lf_;
  double *tire_pressure_lf_ptr_ = nullptr;
  //右前轮胎压是否有效
  legionclaw::common::IsValid is_tire_pressure_rf_valid_;
  legionclaw::common::IsValid *is_tire_pressure_rf_valid_ptr_ = nullptr;
  //右前轮胎压
  double tire_pressure_rf_;
  double *tire_pressure_rf_ptr_ = nullptr;
  //左后轮胎压是否有效
  legionclaw::common::IsValid is_tire_pressure_lr_valid_;
  legionclaw::common::IsValid *is_tire_pressure_lr_valid_ptr_ = nullptr;
  //左后轮胎压
  double tire_pressure_lr_;
  double *tire_pressure_lr_ptr_ = nullptr;
  //右后轮胎压是否有效
  legionclaw::common::IsValid is_tire_pressure_rr_valid_;
  legionclaw::common::IsValid *is_tire_pressure_rr_valid_ptr_ = nullptr;
  //右后轮胎压
  double tire_pressure_rr_;
  double *tire_pressure_rr_ptr_ = nullptr;
  //电池电量值
  double battery_power_percentage_;
  double *battery_power_percentage_ptr_ = nullptr;
  //气囊状态
  legionclaw::common::FailureStatus air_bag_status_;
  legionclaw::common::FailureStatus *air_bag_status_ptr_ = nullptr;
  //充电枪状态
  legionclaw::common::PlugStatus charging_gun_status_;
  legionclaw::common::PlugStatus *charging_gun_status_ptr_ = nullptr;
  //电量表状态
  legionclaw::common::FailureStatus vehicle_power_status_;
  legionclaw::common::FailureStatus *vehicle_power_status_ptr_ = nullptr;
  //底盘错误码
  std::vector<legionclaw::interface::Chassis::ErrorCode> chassis_error_code_;
};
} // namespace interface
} // namespace legionclaw
