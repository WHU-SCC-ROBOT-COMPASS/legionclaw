/**
 * @file    control_command.hpp
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

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ControlCommand {
public:
  ControlCommand() {
    steer_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    front_steering_target_ = 0.0;
    rear_steering_target_ = 0.0;
    front_steering_rate_ = 0.0;
    rear_steering_rate_ = 0.0;
    accel_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    accel_value_ = 0.0;
    brake_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    brake_value_ = 0.0;
    backup_brake_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    backup_brake_value_ = 0.0;
    epb_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    epb_level_ = legionclaw::common::EPBLevel::RELEASED;
    gear_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    emergency_brake_enable_ = 0;
    gear_location_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    speed_ = 0.0;
    acceleration_ = 0.0;
    turn_lamp_ctrl_ = legionclaw::common::TurnSignal::TURN_INACTIVE_SIGNAL;
    high_beam_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    low_beam_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    horn_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    front_wiper_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_wiper_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    position_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    front_fog_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_fog_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    brake_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    alarm_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    lf_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    rf_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    lr_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    rr_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
  }
  ~ControlCommand() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

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

  inline void set_front_steering_target(const double &front_steering_target) {
    front_steering_target_ = front_steering_target;
    front_steering_target_ptr_ = &front_steering_target_;
  }

  inline const double &front_steering_target() const {
    return front_steering_target_;
  }

  inline double *mutable_front_steering_target() {
    return &front_steering_target_;
  }

  inline bool has_front_steering_target() {
    return (front_steering_target_ptr_ != nullptr);
  }

  inline void set_rear_steering_target(const double &rear_steering_target) {
    rear_steering_target_ = rear_steering_target;
    rear_steering_target_ptr_ = &rear_steering_target_;
  }

  inline const double &rear_steering_target() const {
    return rear_steering_target_;
  }

  inline double *mutable_rear_steering_target() {
    return &rear_steering_target_;
  }

  inline bool has_rear_steering_target() {
    return (rear_steering_target_ptr_ != nullptr);
  }

  inline void set_front_steering_rate(const double &front_steering_rate) {
    front_steering_rate_ = front_steering_rate;
    front_steering_rate_ptr_ = &front_steering_rate_;
  }

  inline const double &front_steering_rate() const {
    return front_steering_rate_;
  }

  inline double *mutable_front_steering_rate() { return &front_steering_rate_; }

  inline bool has_front_steering_rate() {
    return (front_steering_rate_ptr_ != nullptr);
  }

  inline void set_rear_steering_rate(const double &rear_steering_rate) {
    rear_steering_rate_ = rear_steering_rate;
    rear_steering_rate_ptr_ = &rear_steering_rate_;
  }

  inline const double &rear_steering_rate() const {
    return rear_steering_rate_;
  }

  inline double *mutable_rear_steering_rate() { return &rear_steering_rate_; }

  inline bool has_rear_steering_rate() {
    return (rear_steering_rate_ptr_ != nullptr);
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

  inline void set_epb_level(const legionclaw::common::EPBLevel &epb_level) {
    epb_level_ = epb_level;
    epb_level_ptr_ = &epb_level_;
  }

  inline const legionclaw::common::EPBLevel &epb_level() const { return epb_level_; }

  inline legionclaw::common::EPBLevel *mutable_epb_level() { return &epb_level_; }

  inline bool has_epb_level() { return (epb_level_ptr_ != nullptr); }

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

  inline void set_emergency_brake_enable(const int8_t &emergency_brake_enable) {
    emergency_brake_enable_ = emergency_brake_enable;
    emergency_brake_enable_ptr_ = &emergency_brake_enable_;
  }

  inline const int8_t &emergency_brake_enable() const {
    return emergency_brake_enable_;
  }

  inline int8_t *mutable_emergency_brake_enable() {
    return &emergency_brake_enable_;
  }

  inline bool has_emergency_brake_enable() {
    return (emergency_brake_enable_ptr_ != nullptr);
  }

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

  inline void set_speed(const double &speed) {
    speed_ = speed;
    speed_ptr_ = &speed_;
  }

  inline const double &speed() const { return speed_; }

  inline double *mutable_speed() { return &speed_; }

  inline bool has_speed() { return (speed_ptr_ != nullptr); }

  inline void set_acceleration(const double &acceleration) {
    acceleration_ = acceleration;
    acceleration_ptr_ = &acceleration_;
  }

  inline const double &acceleration() const { return acceleration_; }

  inline double *mutable_acceleration() { return &acceleration_; }

  inline bool has_acceleration() { return (acceleration_ptr_ != nullptr); }

  inline void
  set_turn_lamp_ctrl(const legionclaw::common::TurnSignal &turn_lamp_ctrl) {
    turn_lamp_ctrl_ = turn_lamp_ctrl;
    turn_lamp_ctrl_ptr_ = &turn_lamp_ctrl_;
  }

  inline const legionclaw::common::TurnSignal &turn_lamp_ctrl() const {
    return turn_lamp_ctrl_;
  }

  inline legionclaw::common::TurnSignal *mutable_turn_lamp_ctrl() {
    return &turn_lamp_ctrl_;
  }

  inline bool has_turn_lamp_ctrl() { return (turn_lamp_ctrl_ptr_ != nullptr); }

  inline void
  set_high_beam_ctrl(const legionclaw::common::SwitchStatus &high_beam_ctrl) {
    high_beam_ctrl_ = high_beam_ctrl;
    high_beam_ctrl_ptr_ = &high_beam_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &high_beam_ctrl() const {
    return high_beam_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_high_beam_ctrl() {
    return &high_beam_ctrl_;
  }

  inline bool has_high_beam_ctrl() { return (high_beam_ctrl_ptr_ != nullptr); }

  inline void
  set_low_beam_ctrl(const legionclaw::common::SwitchStatus &low_beam_ctrl) {
    low_beam_ctrl_ = low_beam_ctrl;
    low_beam_ctrl_ptr_ = &low_beam_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &low_beam_ctrl() const {
    return low_beam_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_low_beam_ctrl() {
    return &low_beam_ctrl_;
  }

  inline bool has_low_beam_ctrl() { return (low_beam_ctrl_ptr_ != nullptr); }

  inline void set_horn_ctrl(const legionclaw::common::SwitchStatus &horn_ctrl) {
    horn_ctrl_ = horn_ctrl;
    horn_ctrl_ptr_ = &horn_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &horn_ctrl() const {
    return horn_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_horn_ctrl() {
    return &horn_ctrl_;
  }

  inline bool has_horn_ctrl() { return (horn_ctrl_ptr_ != nullptr); }

  inline void
  set_front_wiper_ctrl(const legionclaw::common::SwitchStatus &front_wiper_ctrl) {
    front_wiper_ctrl_ = front_wiper_ctrl;
    front_wiper_ctrl_ptr_ = &front_wiper_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &front_wiper_ctrl() const {
    return front_wiper_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_wiper_ctrl() {
    return &front_wiper_ctrl_;
  }

  inline bool has_front_wiper_ctrl() {
    return (front_wiper_ctrl_ptr_ != nullptr);
  }

  inline void
  set_rear_wiper_ctrl(const legionclaw::common::SwitchStatus &rear_wiper_ctrl) {
    rear_wiper_ctrl_ = rear_wiper_ctrl;
    rear_wiper_ctrl_ptr_ = &rear_wiper_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &rear_wiper_ctrl() const {
    return rear_wiper_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_wiper_ctrl() {
    return &rear_wiper_ctrl_;
  }

  inline bool has_rear_wiper_ctrl() {
    return (rear_wiper_ctrl_ptr_ != nullptr);
  }

  inline void set_position_lamp_ctrl(
      const legionclaw::common::SwitchStatus &position_lamp_ctrl) {
    position_lamp_ctrl_ = position_lamp_ctrl;
    position_lamp_ctrl_ptr_ = &position_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &position_lamp_ctrl() const {
    return position_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_position_lamp_ctrl() {
    return &position_lamp_ctrl_;
  }

  inline bool has_position_lamp_ctrl() {
    return (position_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_front_fog_lamp_ctrl(
      const legionclaw::common::SwitchStatus &front_fog_lamp_ctrl) {
    front_fog_lamp_ctrl_ = front_fog_lamp_ctrl;
    front_fog_lamp_ctrl_ptr_ = &front_fog_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &front_fog_lamp_ctrl() const {
    return front_fog_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_fog_lamp_ctrl() {
    return &front_fog_lamp_ctrl_;
  }

  inline bool has_front_fog_lamp_ctrl() {
    return (front_fog_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_rear_fog_lamp_ctrl(
      const legionclaw::common::SwitchStatus &rear_fog_lamp_ctrl) {
    rear_fog_lamp_ctrl_ = rear_fog_lamp_ctrl;
    rear_fog_lamp_ctrl_ptr_ = &rear_fog_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &rear_fog_lamp_ctrl() const {
    return rear_fog_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_fog_lamp_ctrl() {
    return &rear_fog_lamp_ctrl_;
  }

  inline bool has_rear_fog_lamp_ctrl() {
    return (rear_fog_lamp_ctrl_ptr_ != nullptr);
  }

  inline void
  set_brake_lamp_ctrl(const legionclaw::common::SwitchStatus &brake_lamp_ctrl) {
    brake_lamp_ctrl_ = brake_lamp_ctrl;
    brake_lamp_ctrl_ptr_ = &brake_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &brake_lamp_ctrl() const {
    return brake_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_brake_lamp_ctrl() {
    return &brake_lamp_ctrl_;
  }

  inline bool has_brake_lamp_ctrl() {
    return (brake_lamp_ctrl_ptr_ != nullptr);
  }

  inline void
  set_alarm_lamp_ctrl(const legionclaw::common::SwitchStatus &alarm_lamp_ctrl) {
    alarm_lamp_ctrl_ = alarm_lamp_ctrl;
    alarm_lamp_ctrl_ptr_ = &alarm_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &alarm_lamp_ctrl() const {
    return alarm_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_alarm_lamp_ctrl() {
    return &alarm_lamp_ctrl_;
  }

  inline bool has_alarm_lamp_ctrl() {
    return (alarm_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_lf_door_ctrl(const legionclaw::common::DoorStatus &lf_door_ctrl) {
    lf_door_ctrl_ = lf_door_ctrl;
    lf_door_ctrl_ptr_ = &lf_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &lf_door_ctrl() const {
    return lf_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_lf_door_ctrl() {
    return &lf_door_ctrl_;
  }

  inline bool has_lf_door_ctrl() { return (lf_door_ctrl_ptr_ != nullptr); }

  inline void set_rf_door_ctrl(const legionclaw::common::DoorStatus &rf_door_ctrl) {
    rf_door_ctrl_ = rf_door_ctrl;
    rf_door_ctrl_ptr_ = &rf_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &rf_door_ctrl() const {
    return rf_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_rf_door_ctrl() {
    return &rf_door_ctrl_;
  }

  inline bool has_rf_door_ctrl() { return (rf_door_ctrl_ptr_ != nullptr); }

  inline void set_lr_door_ctrl(const legionclaw::common::DoorStatus &lr_door_ctrl) {
    lr_door_ctrl_ = lr_door_ctrl;
    lr_door_ctrl_ptr_ = &lr_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &lr_door_ctrl() const {
    return lr_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_lr_door_ctrl() {
    return &lr_door_ctrl_;
  }

  inline bool has_lr_door_ctrl() { return (lr_door_ctrl_ptr_ != nullptr); }

  inline void set_rr_door_ctrl(const legionclaw::common::DoorStatus &rr_door_ctrl) {
    rr_door_ctrl_ = rr_door_ctrl;
    rr_door_ctrl_ptr_ = &rr_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &rr_door_ctrl() const {
    return rr_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_rr_door_ctrl() {
    return &rr_door_ctrl_;
  }

  inline bool has_rr_door_ctrl() { return (rr_door_ctrl_ptr_ != nullptr); }

  void operator=(const ControlCommand &control_command) {
    CopyFrom(control_command);
  }

  void CopyFrom(const ControlCommand &control_command) {
    header_ = control_command.header();
    steer_driving_mode_ = control_command.steer_driving_mode();
    front_steering_target_ = control_command.front_steering_target();
    rear_steering_target_ = control_command.rear_steering_target();
    front_steering_rate_ = control_command.front_steering_rate();
    rear_steering_rate_ = control_command.rear_steering_rate();
    accel_driving_mode_ = control_command.accel_driving_mode();
    accel_value_ = control_command.accel_value();
    brake_driving_mode_ = control_command.brake_driving_mode();
    brake_value_ = control_command.brake_value();
    backup_brake_driving_mode_ = control_command.backup_brake_driving_mode();
    backup_brake_value_ = control_command.backup_brake_value();
    epb_driving_mode_ = control_command.epb_driving_mode();
    epb_level_ = control_command.epb_level();
    gear_driving_mode_ = control_command.gear_driving_mode();
    emergency_brake_enable_ = control_command.emergency_brake_enable();
    gear_location_ = control_command.gear_location();
    speed_ = control_command.speed();
    acceleration_ = control_command.acceleration();
    turn_lamp_ctrl_ = control_command.turn_lamp_ctrl();
    high_beam_ctrl_ = control_command.high_beam_ctrl();
    low_beam_ctrl_ = control_command.low_beam_ctrl();
    horn_ctrl_ = control_command.horn_ctrl();
    front_wiper_ctrl_ = control_command.front_wiper_ctrl();
    rear_wiper_ctrl_ = control_command.rear_wiper_ctrl();
    position_lamp_ctrl_ = control_command.position_lamp_ctrl();
    front_fog_lamp_ctrl_ = control_command.front_fog_lamp_ctrl();
    rear_fog_lamp_ctrl_ = control_command.rear_fog_lamp_ctrl();
    brake_lamp_ctrl_ = control_command.brake_lamp_ctrl();
    alarm_lamp_ctrl_ = control_command.alarm_lamp_ctrl();
    lf_door_ctrl_ = control_command.lf_door_ctrl();
    rf_door_ctrl_ = control_command.rf_door_ctrl();
    lr_door_ctrl_ = control_command.lr_door_ctrl();
    rr_door_ctrl_ = control_command.rr_door_ctrl();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode steer_driving_mode_;
  legionclaw::common::DrivingMode *steer_driving_mode_ptr_ = nullptr;
  // target steering angle
  double front_steering_target_;
  double *front_steering_target_ptr_ = nullptr;
  // target steering angle
  double rear_steering_target_;
  double *rear_steering_target_ptr_ = nullptr;
  // target non-directional steering rate
  double front_steering_rate_;
  double *front_steering_rate_ptr_ = nullptr;
  // target non-directional steering rate
  double rear_steering_rate_;
  double *rear_steering_rate_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode accel_driving_mode_;
  legionclaw::common::DrivingMode *accel_driving_mode_ptr_ = nullptr;
  // target accel value
  double accel_value_;
  double *accel_value_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode brake_driving_mode_;
  legionclaw::common::DrivingMode *brake_driving_mode_ptr_ = nullptr;
  // target brake
  double brake_value_;
  double *brake_value_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode backup_brake_driving_mode_;
  legionclaw::common::DrivingMode *backup_brake_driving_mode_ptr_ = nullptr;
  // target brake
  double backup_brake_value_;
  double *backup_brake_value_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode epb_driving_mode_;
  legionclaw::common::DrivingMode *epb_driving_mode_ptr_ = nullptr;
  // 0=RELEASE 1=APPLIY 2=INVALID
  legionclaw::common::EPBLevel epb_level_;
  legionclaw::common::EPBLevel *epb_level_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
  legionclaw::common::DrivingMode gear_driving_mode_;
  legionclaw::common::DrivingMode *gear_driving_mode_ptr_ = nullptr;
  // 0=disenable  1=enable
  int8_t emergency_brake_enable_;
  int8_t *emergency_brake_enable_ptr_ = nullptr;
  // 0=GEAR_NEUTRAL 1=GEAR_DRIVE 2=GEAR_REVERSE 3=GEAR_PARKING 4=GEAR_LOW
  // 5=GEAR_INVALID 6=GEAR_NONE
  legionclaw::common::GearPosition gear_location_;
  legionclaw::common::GearPosition *gear_location_ptr_ = nullptr;
  // target speed, in m/s
  double speed_;
  double *speed_ptr_ = nullptr;
  // target acceleration in m`s^-2
  double acceleration_;
  double *acceleration_ptr_ = nullptr;
  //转向灯 0 关闭 1 开启 2 不响应
  legionclaw::common::TurnSignal turn_lamp_ctrl_;
  legionclaw::common::TurnSignal *turn_lamp_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus high_beam_ctrl_;
  legionclaw::common::SwitchStatus *high_beam_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus low_beam_ctrl_;
  legionclaw::common::SwitchStatus *low_beam_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus horn_ctrl_;
  legionclaw::common::SwitchStatus *horn_ctrl_ptr_ = nullptr;
  //前雨刮器 0 关闭 1 开启
  legionclaw::common::SwitchStatus front_wiper_ctrl_;
  legionclaw::common::SwitchStatus *front_wiper_ctrl_ptr_ = nullptr;
  //后雨刮器 0 关闭 1 开启
  legionclaw::common::SwitchStatus rear_wiper_ctrl_;
  legionclaw::common::SwitchStatus *rear_wiper_ctrl_ptr_ = nullptr;
  //位置灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus position_lamp_ctrl_;
  legionclaw::common::SwitchStatus *position_lamp_ctrl_ptr_ = nullptr;
  //前雾灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus front_fog_lamp_ctrl_;
  legionclaw::common::SwitchStatus *front_fog_lamp_ctrl_ptr_ = nullptr;
  //后雾灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus rear_fog_lamp_ctrl_;
  legionclaw::common::SwitchStatus *rear_fog_lamp_ctrl_ptr_ = nullptr;
  //刹车灯 一般情况自动控制 0 关闭 1 开启
  legionclaw::common::SwitchStatus brake_lamp_ctrl_;
  legionclaw::common::SwitchStatus *brake_lamp_ctrl_ptr_ = nullptr;
  //警报灯 双闪 0 关闭 1 开启
  legionclaw::common::SwitchStatus alarm_lamp_ctrl_;
  legionclaw::common::SwitchStatus *alarm_lamp_ctrl_ptr_ = nullptr;
  //左前门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus lf_door_ctrl_;
  legionclaw::common::DoorStatus *lf_door_ctrl_ptr_ = nullptr;
  //右前门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus rf_door_ctrl_;
  legionclaw::common::DoorStatus *rf_door_ctrl_ptr_ = nullptr;
  //左后门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus lr_door_ctrl_;
  legionclaw::common::DoorStatus *lr_door_ctrl_ptr_ = nullptr;
  //右后门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus rr_door_ctrl_;
  legionclaw::common::DoorStatus *rr_door_ctrl_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
