/**
 * @file    wheel_info.hpp
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
class WheelInfo {
public:
  WheelInfo() {
    moving_status_ = legionclaw::common::MovingStatus::UNKNOWN_VEHICLE_STATE;
    steer_driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    steering_value_ = 0.0;
    steering_torque_nm_ = 0.0;
    steering_rate_dps_ = 0.0;
    speed_mps_ = 0.0;
    veh_spd_vld_ = false;
    gear_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    gear_vld_ = false;
    wheel_direction_rr_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_rr_ = 0.0;
    wheel_direction_rl_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_rl_ = 0.0;
    wheel_direction_fr_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_fr_ = 0.0;
    wheel_direction_fl_ = legionclaw::common::WheelSpeedType::FORWARD;
    wheel_spd_fl_ = 0.0;
    yaw_rate_ = 0.0;
    wss_fl_edges_sum_ = 0;
    wss_fr_edges_sum_ = 0;
    wss_rl_edges_sum_ = 0;
    wss_rr_edges_sum_ = 0;
    wss_fl_edges_sum_vld_ = false;
    wss_fr_edges_sum_vld_ = false;
    wss_rl_edges_sum_vld_ = false;
    wss_rr_edges_sum_vld_ = false;
    veh_lat_accel_ = 0.0;
    veh_lgt_accel_ = 0.0;
    veh_lat_accel_vld_ = false;
    veh_lgt_accel_vld_ = false;
  }
  ~WheelInfo() = default;

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

  inline void set_steering_value(const double &steering_value) {
    steering_value_ = steering_value;
    steering_value_ptr_ = &steering_value_;
  }

  inline const double &steering_value() const { return steering_value_; }

  inline double *mutable_steering_value() { return &steering_value_; }

  inline bool has_steering_value() { return (steering_value_ptr_ != nullptr); }

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

  inline void set_steering_rate_dps(const double &steering_rate_dps) {
    steering_rate_dps_ = steering_rate_dps;
    steering_rate_dps_ptr_ = &steering_rate_dps_;
  }

  inline const double &steering_rate_dps() const { return steering_rate_dps_; }

  inline double *mutable_steering_rate_dps() { return &steering_rate_dps_; }

  inline bool has_steering_rate_dps() {
    return (steering_rate_dps_ptr_ != nullptr);
  }

  inline void set_speed_mps(const double &speed_mps) {
    speed_mps_ = speed_mps;
    speed_mps_ptr_ = &speed_mps_;
  }

  inline const double &speed_mps() const { return speed_mps_; }

  inline double *mutable_speed_mps() { return &speed_mps_; }

  inline bool has_speed_mps() { return (speed_mps_ptr_ != nullptr); }

  inline void set_veh_spd_vld(const bool &veh_spd_vld) {
    veh_spd_vld_ = veh_spd_vld;
    veh_spd_vld_ptr_ = &veh_spd_vld_;
  }

  inline const bool &veh_spd_vld() const { return veh_spd_vld_; }

  inline bool *mutable_veh_spd_vld() { return &veh_spd_vld_; }

  inline bool has_veh_spd_vld() { return (veh_spd_vld_ptr_ != nullptr); }

  inline void set_gear(const legionclaw::common::GearPosition &gear) {
    gear_ = gear;
    gear_ptr_ = &gear_;
  }

  inline const legionclaw::common::GearPosition &gear() const { return gear_; }

  inline legionclaw::common::GearPosition *mutable_gear() { return &gear_; }

  inline bool has_gear() { return (gear_ptr_ != nullptr); }

  inline void set_gear_vld(const bool &gear_vld) {
    gear_vld_ = gear_vld;
    gear_vld_ptr_ = &gear_vld_;
  }

  inline const bool &gear_vld() const { return gear_vld_; }

  inline bool *mutable_gear_vld() { return &gear_vld_; }

  inline bool has_gear_vld() { return (gear_vld_ptr_ != nullptr); }

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

  inline void set_yaw_rate(const double &yaw_rate) {
    yaw_rate_ = yaw_rate;
    yaw_rate_ptr_ = &yaw_rate_;
  }

  inline const double &yaw_rate() const { return yaw_rate_; }

  inline double *mutable_yaw_rate() { return &yaw_rate_; }

  inline bool has_yaw_rate() { return (yaw_rate_ptr_ != nullptr); }

  inline void set_wss_fl_edges_sum(const int32_t &wss_fl_edges_sum) {
    wss_fl_edges_sum_ = wss_fl_edges_sum;
    wss_fl_edges_sum_ptr_ = &wss_fl_edges_sum_;
  }

  inline const int32_t &wss_fl_edges_sum() const { return wss_fl_edges_sum_; }

  inline int32_t *mutable_wss_fl_edges_sum() { return &wss_fl_edges_sum_; }

  inline bool has_wss_fl_edges_sum() {
    return (wss_fl_edges_sum_ptr_ != nullptr);
  }

  inline void set_wss_fr_edges_sum(const int32_t &wss_fr_edges_sum) {
    wss_fr_edges_sum_ = wss_fr_edges_sum;
    wss_fr_edges_sum_ptr_ = &wss_fr_edges_sum_;
  }

  inline const int32_t &wss_fr_edges_sum() const { return wss_fr_edges_sum_; }

  inline int32_t *mutable_wss_fr_edges_sum() { return &wss_fr_edges_sum_; }

  inline bool has_wss_fr_edges_sum() {
    return (wss_fr_edges_sum_ptr_ != nullptr);
  }

  inline void set_wss_rl_edges_sum(const int32_t &wss_rl_edges_sum) {
    wss_rl_edges_sum_ = wss_rl_edges_sum;
    wss_rl_edges_sum_ptr_ = &wss_rl_edges_sum_;
  }

  inline const int32_t &wss_rl_edges_sum() const { return wss_rl_edges_sum_; }

  inline int32_t *mutable_wss_rl_edges_sum() { return &wss_rl_edges_sum_; }

  inline bool has_wss_rl_edges_sum() {
    return (wss_rl_edges_sum_ptr_ != nullptr);
  }

  inline void set_wss_rr_edges_sum(const int32_t &wss_rr_edges_sum) {
    wss_rr_edges_sum_ = wss_rr_edges_sum;
    wss_rr_edges_sum_ptr_ = &wss_rr_edges_sum_;
  }

  inline const int32_t &wss_rr_edges_sum() const { return wss_rr_edges_sum_; }

  inline int32_t *mutable_wss_rr_edges_sum() { return &wss_rr_edges_sum_; }

  inline bool has_wss_rr_edges_sum() {
    return (wss_rr_edges_sum_ptr_ != nullptr);
  }

  inline void set_wss_fl_edges_sum_vld(const bool &wss_fl_edges_sum_vld) {
    wss_fl_edges_sum_vld_ = wss_fl_edges_sum_vld;
    wss_fl_edges_sum_vld_ptr_ = &wss_fl_edges_sum_vld_;
  }

  inline const bool &wss_fl_edges_sum_vld() const {
    return wss_fl_edges_sum_vld_;
  }

  inline bool *mutable_wss_fl_edges_sum_vld() { return &wss_fl_edges_sum_vld_; }

  inline bool has_wss_fl_edges_sum_vld() {
    return (wss_fl_edges_sum_vld_ptr_ != nullptr);
  }

  inline void set_wss_fr_edges_sum_vld(const bool &wss_fr_edges_sum_vld) {
    wss_fr_edges_sum_vld_ = wss_fr_edges_sum_vld;
    wss_fr_edges_sum_vld_ptr_ = &wss_fr_edges_sum_vld_;
  }

  inline const bool &wss_fr_edges_sum_vld() const {
    return wss_fr_edges_sum_vld_;
  }

  inline bool *mutable_wss_fr_edges_sum_vld() { return &wss_fr_edges_sum_vld_; }

  inline bool has_wss_fr_edges_sum_vld() {
    return (wss_fr_edges_sum_vld_ptr_ != nullptr);
  }

  inline void set_wss_rl_edges_sum_vld(const bool &wss_rl_edges_sum_vld) {
    wss_rl_edges_sum_vld_ = wss_rl_edges_sum_vld;
    wss_rl_edges_sum_vld_ptr_ = &wss_rl_edges_sum_vld_;
  }

  inline const bool &wss_rl_edges_sum_vld() const {
    return wss_rl_edges_sum_vld_;
  }

  inline bool *mutable_wss_rl_edges_sum_vld() { return &wss_rl_edges_sum_vld_; }

  inline bool has_wss_rl_edges_sum_vld() {
    return (wss_rl_edges_sum_vld_ptr_ != nullptr);
  }

  inline void set_wss_rr_edges_sum_vld(const bool &wss_rr_edges_sum_vld) {
    wss_rr_edges_sum_vld_ = wss_rr_edges_sum_vld;
    wss_rr_edges_sum_vld_ptr_ = &wss_rr_edges_sum_vld_;
  }

  inline const bool &wss_rr_edges_sum_vld() const {
    return wss_rr_edges_sum_vld_;
  }

  inline bool *mutable_wss_rr_edges_sum_vld() { return &wss_rr_edges_sum_vld_; }

  inline bool has_wss_rr_edges_sum_vld() {
    return (wss_rr_edges_sum_vld_ptr_ != nullptr);
  }

  inline void set_veh_lat_accel(const double &veh_lat_accel) {
    veh_lat_accel_ = veh_lat_accel;
    veh_lat_accel_ptr_ = &veh_lat_accel_;
  }

  inline const double &veh_lat_accel() const { return veh_lat_accel_; }

  inline double *mutable_veh_lat_accel() { return &veh_lat_accel_; }

  inline bool has_veh_lat_accel() { return (veh_lat_accel_ptr_ != nullptr); }

  inline void set_veh_lgt_accel(const double &veh_lgt_accel) {
    veh_lgt_accel_ = veh_lgt_accel;
    veh_lgt_accel_ptr_ = &veh_lgt_accel_;
  }

  inline const double &veh_lgt_accel() const { return veh_lgt_accel_; }

  inline double *mutable_veh_lgt_accel() { return &veh_lgt_accel_; }

  inline bool has_veh_lgt_accel() { return (veh_lgt_accel_ptr_ != nullptr); }

  inline void set_veh_lat_accel_vld(const bool &veh_lat_accel_vld) {
    veh_lat_accel_vld_ = veh_lat_accel_vld;
    veh_lat_accel_vld_ptr_ = &veh_lat_accel_vld_;
  }

  inline const bool &veh_lat_accel_vld() const { return veh_lat_accel_vld_; }

  inline bool *mutable_veh_lat_accel_vld() { return &veh_lat_accel_vld_; }

  inline bool has_veh_lat_accel_vld() {
    return (veh_lat_accel_vld_ptr_ != nullptr);
  }

  inline void set_veh_lgt_accel_vld(const bool &veh_lgt_accel_vld) {
    veh_lgt_accel_vld_ = veh_lgt_accel_vld;
    veh_lgt_accel_vld_ptr_ = &veh_lgt_accel_vld_;
  }

  inline const bool &veh_lgt_accel_vld() const { return veh_lgt_accel_vld_; }

  inline bool *mutable_veh_lgt_accel_vld() { return &veh_lgt_accel_vld_; }

  inline bool has_veh_lgt_accel_vld() {
    return (veh_lgt_accel_vld_ptr_ != nullptr);
  }

  void operator=(const WheelInfo &wheel_info) { CopyFrom(wheel_info); }

  void CopyFrom(const WheelInfo &wheel_info) {
    header_ = wheel_info.header();
    moving_status_ = wheel_info.moving_status();
    steer_driving_mode_ = wheel_info.steer_driving_mode();
    steering_value_ = wheel_info.steering_value();
    steering_torque_nm_ = wheel_info.steering_torque_nm();
    steering_rate_dps_ = wheel_info.steering_rate_dps();
    speed_mps_ = wheel_info.speed_mps();
    veh_spd_vld_ = wheel_info.veh_spd_vld();
    gear_ = wheel_info.gear();
    gear_vld_ = wheel_info.gear_vld();
    wheel_direction_rr_ = wheel_info.wheel_direction_rr();
    wheel_spd_rr_ = wheel_info.wheel_spd_rr();
    wheel_direction_rl_ = wheel_info.wheel_direction_rl();
    wheel_spd_rl_ = wheel_info.wheel_spd_rl();
    wheel_direction_fr_ = wheel_info.wheel_direction_fr();
    wheel_spd_fr_ = wheel_info.wheel_spd_fr();
    wheel_direction_fl_ = wheel_info.wheel_direction_fl();
    wheel_spd_fl_ = wheel_info.wheel_spd_fl();
    yaw_rate_ = wheel_info.yaw_rate();
    wss_fl_edges_sum_ = wheel_info.wss_fl_edges_sum();
    wss_fr_edges_sum_ = wheel_info.wss_fr_edges_sum();
    wss_rl_edges_sum_ = wheel_info.wss_rl_edges_sum();
    wss_rr_edges_sum_ = wheel_info.wss_rr_edges_sum();
    wss_fl_edges_sum_vld_ = wheel_info.wss_fl_edges_sum_vld();
    wss_fr_edges_sum_vld_ = wheel_info.wss_fr_edges_sum_vld();
    wss_rl_edges_sum_vld_ = wheel_info.wss_rl_edges_sum_vld();
    wss_rr_edges_sum_vld_ = wheel_info.wss_rr_edges_sum_vld();
    veh_lat_accel_ = wheel_info.veh_lat_accel();
    veh_lgt_accel_ = wheel_info.veh_lgt_accel();
    veh_lat_accel_vld_ = wheel_info.veh_lat_accel_vld();
    veh_lgt_accel_vld_ = wheel_info.veh_lgt_accel_vld();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // 0=UNKNOWN_VEHICLE_STATE  1=STATIONARY  2=MOVING_FORWARD  3=MOVING_BACKWARD
  legionclaw::common::MovingStatus moving_status_;
  legionclaw::common::MovingStatus *moving_status_ptr_ = nullptr;
  // 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE  4=EMERGENCY_MODE
  legionclaw::common::DrivingMode steer_driving_mode_;
  legionclaw::common::DrivingMode *steer_driving_mode_ptr_ = nullptr;
  // Real steering location
  double steering_value_;
  double *steering_value_ptr_ = nullptr;
  // Applied steering torque in [Nm]
  double steering_torque_nm_;
  double *steering_torque_nm_ptr_ = nullptr;
  // degree/s
  double steering_rate_dps_;
  double *steering_rate_dps_ptr_ = nullptr;
  // Vehicle Speed in meters per second
  double speed_mps_;
  double *speed_mps_ptr_ = nullptr;
  //车速有效性
  bool veh_spd_vld_;
  bool *veh_spd_vld_ptr_ = nullptr;
  //档位
  legionclaw::common::GearPosition gear_;
  legionclaw::common::GearPosition *gear_ptr_ = nullptr;
  //档位有效性
  bool gear_vld_;
  bool *gear_vld_ptr_ = nullptr;
  // 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
  legionclaw::common::WheelSpeedType wheel_direction_rr_;
  legionclaw::common::WheelSpeedType *wheel_direction_rr_ptr_ = nullptr;
  // Actual speed of the right rear wheel
  double wheel_spd_rr_;
  double *wheel_spd_rr_ptr_ = nullptr;
  // 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
  legionclaw::common::WheelSpeedType wheel_direction_rl_;
  legionclaw::common::WheelSpeedType *wheel_direction_rl_ptr_ = nullptr;
  // Actual speed of the left rear wheel
  double wheel_spd_rl_;
  double *wheel_spd_rl_ptr_ = nullptr;
  // 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
  legionclaw::common::WheelSpeedType wheel_direction_fr_;
  legionclaw::common::WheelSpeedType *wheel_direction_fr_ptr_ = nullptr;
  // Actual speed of the right front wheel
  double wheel_spd_fr_;
  double *wheel_spd_fr_ptr_ = nullptr;
  // 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
  legionclaw::common::WheelSpeedType wheel_direction_fl_;
  legionclaw::common::WheelSpeedType *wheel_direction_fl_ptr_ = nullptr;
  // Actual speed of the left front wheel
  double wheel_spd_fl_;
  double *wheel_spd_fl_ptr_ = nullptr;
  // Yaw rate[degree/s] [-163.84|163.835]
  double yaw_rate_;
  double *yaw_rate_ptr_ = nullptr;
  //左前轮边缘和
  int32_t wss_fl_edges_sum_;
  int32_t *wss_fl_edges_sum_ptr_ = nullptr;
  //右前轮边缘和
  int32_t wss_fr_edges_sum_;
  int32_t *wss_fr_edges_sum_ptr_ = nullptr;
  //左后轮边缘和
  int32_t wss_rl_edges_sum_;
  int32_t *wss_rl_edges_sum_ptr_ = nullptr;
  //右后轮边缘和
  int32_t wss_rr_edges_sum_;
  int32_t *wss_rr_edges_sum_ptr_ = nullptr;
  //左前轮边缘和有效位（0：无效 1：有效）
  bool wss_fl_edges_sum_vld_;
  bool *wss_fl_edges_sum_vld_ptr_ = nullptr;
  //右前轮边缘和有效位（0：无效 1：有效）
  bool wss_fr_edges_sum_vld_;
  bool *wss_fr_edges_sum_vld_ptr_ = nullptr;
  //左后轮边缘和有效位（0：无效 1：有效）
  bool wss_rl_edges_sum_vld_;
  bool *wss_rl_edges_sum_vld_ptr_ = nullptr;
  //右后轮边缘和有效位（0：无效 1：有效）
  bool wss_rr_edges_sum_vld_;
  bool *wss_rr_edges_sum_vld_ptr_ = nullptr;
  //车辆横向加速度
  double veh_lat_accel_;
  double *veh_lat_accel_ptr_ = nullptr;
  //车辆纵向加速度
  double veh_lgt_accel_;
  double *veh_lgt_accel_ptr_ = nullptr;
  //车辆横向加速度有效性
  bool veh_lat_accel_vld_;
  bool *veh_lat_accel_vld_ptr_ = nullptr;
  //车辆纵向加速度有效性
  bool veh_lgt_accel_vld_;
  bool *veh_lgt_accel_vld_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
