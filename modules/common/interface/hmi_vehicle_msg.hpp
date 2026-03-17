/**
 * @file    hmi_vehicle_msg.hpp
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
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/point_llh.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HMIVehicleMsg {
public:
  HMIVehicleMsg() {
    chassis_error_code_mutex_ = std::make_shared<std::mutex>();

    driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    gear_location_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    steering_value_ = 0.0;
    accel_value_ = 0.0;
    brake_value_ = 0.0;
    speed_mps_ = 0.0;
    pitch_ = 0.0;
    roll_ = 0.0;
    heading_ = 0.0;
    rtk_flag_ = legionclaw::interface::Location::RTKFlag::RTK_NULL;
    origin_lat_ = 0.0;
    origin_lon_ = 0.0;
    current_ins_yaw_ = 0.0;
    auto_safe_ = 0;
    battery_power_percentage_ = 0.0;
    charging_gun_status_ = legionclaw::common::PlugStatus::PLUG_STATUS_UNKNOWN;
    clear_chassis_error_code();
    brake_lamp_status_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    turn_lamp_status_ = legionclaw::common::TurnSignal::TURN_INACTIVE_SIGNAL;
  }
  ~HMIVehicleMsg() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

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

  inline void set_steering_value(const double &steering_value) {
    steering_value_ = steering_value;
    steering_value_ptr_ = &steering_value_;
  }

  inline const double &steering_value() const { return steering_value_; }

  inline double *mutable_steering_value() { return &steering_value_; }

  inline bool has_steering_value() { return (steering_value_ptr_ != nullptr); }

  inline void set_accel_value(const double &accel_value) {
    accel_value_ = accel_value;
    accel_value_ptr_ = &accel_value_;
  }

  inline const double &accel_value() const { return accel_value_; }

  inline double *mutable_accel_value() { return &accel_value_; }

  inline bool has_accel_value() { return (accel_value_ptr_ != nullptr); }

  inline void set_brake_value(const double &brake_value) {
    brake_value_ = brake_value;
    brake_value_ptr_ = &brake_value_;
  }

  inline const double &brake_value() const { return brake_value_; }

  inline double *mutable_brake_value() { return &brake_value_; }

  inline bool has_brake_value() { return (brake_value_ptr_ != nullptr); }

  inline void set_speed_mps(const double &speed_mps) {
    speed_mps_ = speed_mps;
    speed_mps_ptr_ = &speed_mps_;
  }

  inline const double &speed_mps() const { return speed_mps_; }

  inline double *mutable_speed_mps() { return &speed_mps_; }

  inline bool has_speed_mps() { return (speed_mps_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::PointLLH &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::PointLLH &position() const {
    return position_;
  }

  inline legionclaw::interface::PointLLH *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_utm_position(const legionclaw::interface::PointENU &utm_position) {
    utm_position_ = utm_position;
    utm_position_ptr_ = &utm_position_;
  }

  inline const legionclaw::interface::PointENU &utm_position() const {
    return utm_position_;
  }

  inline legionclaw::interface::PointENU *mutable_utm_position() {
    return &utm_position_;
  }

  inline bool has_utm_position() { return (utm_position_ptr_ != nullptr); }

  inline void set_pitch(const double &pitch) {
    pitch_ = pitch;
    pitch_ptr_ = &pitch_;
  }

  inline const double &pitch() const { return pitch_; }

  inline double *mutable_pitch() { return &pitch_; }

  inline bool has_pitch() { return (pitch_ptr_ != nullptr); }

  inline void set_roll(const double &roll) {
    roll_ = roll;
    roll_ptr_ = &roll_;
  }

  inline const double &roll() const { return roll_; }

  inline double *mutable_roll() { return &roll_; }

  inline bool has_roll() { return (roll_ptr_ != nullptr); }

  inline void set_heading(const double &heading) {
    heading_ = heading;
    heading_ptr_ = &heading_;
  }

  inline const double &heading() const { return heading_; }

  inline double *mutable_heading() { return &heading_; }

  inline bool has_heading() { return (heading_ptr_ != nullptr); }

  inline void
  set_rtk_flag(const legionclaw::interface::Location::RTKFlag &rtk_flag) {
    rtk_flag_ = rtk_flag;
    rtk_flag_ptr_ = &rtk_flag_;
  }

  inline const legionclaw::interface::Location::RTKFlag &rtk_flag() const {
    return rtk_flag_;
  }

  inline legionclaw::interface::Location::RTKFlag *mutable_rtk_flag() {
    return &rtk_flag_;
  }

  inline bool has_rtk_flag() { return (rtk_flag_ptr_ != nullptr); }

  inline void set_origin_lat(const double &origin_lat) {
    origin_lat_ = origin_lat;
    origin_lat_ptr_ = &origin_lat_;
  }

  inline const double &origin_lat() const { return origin_lat_; }

  inline double *mutable_origin_lat() { return &origin_lat_; }

  inline bool has_origin_lat() { return (origin_lat_ptr_ != nullptr); }

  inline void set_origin_lon(const double &origin_lon) {
    origin_lon_ = origin_lon;
    origin_lon_ptr_ = &origin_lon_;
  }

  inline const double &origin_lon() const { return origin_lon_; }

  inline double *mutable_origin_lon() { return &origin_lon_; }

  inline bool has_origin_lon() { return (origin_lon_ptr_ != nullptr); }

  inline void set_current_ins_yaw(const double &current_ins_yaw) {
    current_ins_yaw_ = current_ins_yaw;
    current_ins_yaw_ptr_ = &current_ins_yaw_;
  }

  inline const double &current_ins_yaw() const { return current_ins_yaw_; }

  inline double *mutable_current_ins_yaw() { return &current_ins_yaw_; }

  inline bool has_current_ins_yaw() {
    return (current_ins_yaw_ptr_ != nullptr);
  }

  inline void set_auto_safe(const int32_t &auto_safe) {
    auto_safe_ = auto_safe;
    auto_safe_ptr_ = &auto_safe_;
  }

  inline const int32_t &auto_safe() const { return auto_safe_; }

  inline int32_t *mutable_auto_safe() { return &auto_safe_; }

  inline bool has_auto_safe() { return (auto_safe_ptr_ != nullptr); }

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

  inline void
  set_mercator_position(const legionclaw::interface::PointENU &mercator_position) {
    mercator_position_ = mercator_position;
    mercator_position_ptr_ = &mercator_position_;
  }

  inline const legionclaw::interface::PointENU &mercator_position() const {
    return mercator_position_;
  }

  inline legionclaw::interface::PointENU *mutable_mercator_position() {
    return &mercator_position_;
  }

  inline bool has_mercator_position() {
    return (mercator_position_ptr_ != nullptr);
  }

  void operator=(const HMIVehicleMsg &hmi_vehicle_msg) {
    CopyFrom(hmi_vehicle_msg);
  }

  void CopyFrom(const HMIVehicleMsg &hmi_vehicle_msg) {
    header_ = hmi_vehicle_msg.header();
    driving_mode_ = hmi_vehicle_msg.driving_mode();
    gear_location_ = hmi_vehicle_msg.gear_location();
    steering_value_ = hmi_vehicle_msg.steering_value();
    accel_value_ = hmi_vehicle_msg.accel_value();
    brake_value_ = hmi_vehicle_msg.brake_value();
    speed_mps_ = hmi_vehicle_msg.speed_mps();
    position_ = hmi_vehicle_msg.position();
    utm_position_ = hmi_vehicle_msg.utm_position();
    pitch_ = hmi_vehicle_msg.pitch();
    roll_ = hmi_vehicle_msg.roll();
    heading_ = hmi_vehicle_msg.heading();
    rtk_flag_ = hmi_vehicle_msg.rtk_flag();
    origin_lat_ = hmi_vehicle_msg.origin_lat();
    origin_lon_ = hmi_vehicle_msg.origin_lon();
    current_ins_yaw_ = hmi_vehicle_msg.current_ins_yaw();
    auto_safe_ = hmi_vehicle_msg.auto_safe();
    battery_power_percentage_ = hmi_vehicle_msg.battery_power_percentage();
    charging_gun_status_ = hmi_vehicle_msg.charging_gun_status();
    chassis_error_code_ = hmi_vehicle_msg.chassis_error_code();
    brake_lamp_status_ = hmi_vehicle_msg.brake_lamp_status();
    turn_lamp_status_ = hmi_vehicle_msg.turn_lamp_status();
    mercator_position_ = hmi_vehicle_msg.mercator_position();
  }

protected:
  std::shared_ptr<std::mutex> chassis_error_code_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //驾驶状态
  legionclaw::common::DrivingMode driving_mode_;
  legionclaw::common::DrivingMode *driving_mode_ptr_ = nullptr;
  //档位值
  legionclaw::common::GearPosition gear_location_;
  legionclaw::common::GearPosition *gear_location_ptr_ = nullptr;
  //方向盘转角
  double steering_value_;
  double *steering_value_ptr_ = nullptr;
  //线控驱动值（油门值）
  double accel_value_;
  double *accel_value_ptr_ = nullptr;
  //线控制动值（刹车值）
  double brake_value_;
  double *brake_value_ptr_ = nullptr;
  //车速（m/s）
  double speed_mps_;
  double *speed_mps_ptr_ = nullptr;
  //经纬度高程
  legionclaw::interface::PointLLH position_;
  legionclaw::interface::PointLLH *position_ptr_ = nullptr;
  //相对参考点xyz坐标
  legionclaw::interface::PointENU utm_position_;
  legionclaw::interface::PointENU *utm_position_ptr_ = nullptr;
  //方向(单位弧度，-π～π,正东:0, 逆时针。都满足右手定则)
  double pitch_;
  double *pitch_ptr_ = nullptr;
  double roll_;
  double *roll_ptr_ = nullptr;
  double heading_;
  double *heading_ptr_ = nullptr;
  // RTK标志值
  legionclaw::interface::Location::RTKFlag rtk_flag_;
  legionclaw::interface::Location::RTKFlag *rtk_flag_ptr_ = nullptr;
  //// 参考点纬度 GCJ-02格式
  double origin_lat_;
  double *origin_lat_ptr_ = nullptr;
  //// 参考点经度 GCJ-02格式
  double origin_lon_;
  double *origin_lon_ptr_ = nullptr;
  double current_ins_yaw_;
  double *current_ins_yaw_ptr_ = nullptr;
  ////  安全驾驶0:正常 1:左前车门没关 2:右前车门没关 3:左后车门没关
  ///4:右后车门没关 5:未系安全带 6: 未打开反观镜 7:车辆未静止
  int32_t auto_safe_;
  int32_t *auto_safe_ptr_ = nullptr;
  //电池电量值
  double battery_power_percentage_;
  double *battery_power_percentage_ptr_ = nullptr;
  //充电枪状态
  legionclaw::common::PlugStatus charging_gun_status_;
  legionclaw::common::PlugStatus *charging_gun_status_ptr_ = nullptr;
  //底盘错误码
  std::vector<legionclaw::interface::Chassis::ErrorCode> chassis_error_code_;
  //刹车灯状态
  legionclaw::common::SwitchStatus brake_lamp_status_;
  legionclaw::common::SwitchStatus *brake_lamp_status_ptr_ = nullptr;
  //转向灯状态
  legionclaw::common::TurnSignal turn_lamp_status_;
  legionclaw::common::TurnSignal *turn_lamp_status_ptr_ = nullptr;
  //转坐标系后的参考点
  legionclaw::interface::PointENU mercator_position_;
  legionclaw::interface::PointENU *mercator_position_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
