/**
 * @file    simple_longitudinal_debug.hpp
 * @author  jiang <jiangchengjie@indrv.cn>
 * @date    2019-10-11
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */

#ifndef CONTROLLER_LON_CONTROLLER_SIMPLE_LONGITUDINAL_DEBUG_H_
#define CONTROLLER_LON_CONTROLLER_SIMPLE_LONGITUDINAL_DEBUG_H_

#include "modules/common/interface/trajectory_point.hpp"
/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace control {
/**
 * @class SimpleLongitudinalDebug
 *
 * @brief SimpleLongitudinalDebug.
 */
class SimpleLongitudinalDebug {
 public:
  SimpleLongitudinalDebug() = default;
  ~SimpleLongitudinalDebug() = default;

  void set_station_reference(double station_reference) {
    station_reference_ = station_reference;
  }

  double station_reference() const { return station_reference_; }

  void set_station_error(double station_error) {
    station_error_ = station_error;
  }

  double station_error() const { return station_error_; }

  void set_station_error_limited(double station_error_limited) {
    station_error_limited_ = station_error_limited;
  }

  double station_error_limited() const { return station_error_limited_; }

  void set_preview_station_error(double preview_station_error) {
    preview_station_error_ = preview_station_error;
  }

  double preview_station_error() const { return preview_station_error_; }

  void set_speed_reference(double speed_reference) {
    speed_reference_ = speed_reference;
  }

  double speed_reference() const { return speed_reference_; }

  void set_speed_error(double speed_error) { speed_error_ = speed_error; }

  double speed_error() const { return speed_error_; }

  void set_speed_controller_input_limited(
      double speed_controller_input_limited) {
    speed_controller_input_limited_ = speed_controller_input_limited;
  }

  double speed_controller_input_limited() const {
    return speed_controller_input_limited_;
  }

  void set_preview_speed_reference(double preview_speed_reference) {
    preview_speed_reference_ = preview_speed_reference;
  }

  double preview_speed_reference() const { return preview_speed_reference_; }

  void set_preview_speed_error(double preview_speed_error) {
    preview_speed_error_ = preview_speed_error;
  }

  double preview_speed_error() const { return preview_speed_error_; }

  void set_preview_acceleration_reference(
      double preview_acceleration_reference) {
    preview_acceleration_reference_ = preview_acceleration_reference;
  }

  double preview_acceleration_reference() const {
    return preview_acceleration_reference_;
  }

  void set_acceleration_cmd_closeloop(double acceleration_cmd_closeloop) {
    acceleration_cmd_closeloop_ = acceleration_cmd_closeloop;
  }

  double acceleration_cmd_closeloop() const {
    return acceleration_cmd_closeloop_;
  }

  void set_acceleration_cmd(double acceleration_cmd) {
    acceleration_cmd_ = acceleration_cmd;
  }

  double acceleration_cmd() const { return acceleration_cmd_; }

  void set_acceleration_lookup(double acceleration_lookup) {
    acceleration_lookup_ = acceleration_lookup;
  }

  double acceleration_lookup() const { return acceleration_lookup_; }

  void set_speed_lookup(double speed_lookup) { speed_lookup_ = speed_lookup; }

  double speed_lookup() const { return speed_lookup_; }

  void set_calibration_value(double calibration_value) {
    calibration_value_ = calibration_value;
  }

  double calibration_value() const { return calibration_value_; }

  void set_throttle_cmd(double throttle_cmd) { throttle_cmd_ = throttle_cmd; }

  double throttle_cmd() const { return throttle_cmd_; }

  void set_brake_cmd(double brake_cmd) { brake_cmd_ = brake_cmd; }

  double brake_cmd() const { return brake_cmd_; }

  void set_is_full_stop(bool is_full_stop) { is_full_stop_ = is_full_stop; }

  bool is_full_stop() const { return is_full_stop_; }

  void set_slope_offset_compensation(double slope_offset_compensation) {
    slope_offset_compensation_ = slope_offset_compensation;
  }

  double slope_offset_compensation() const {
    return slope_offset_compensation_;
  }

  void set_aero_offset_compensation(double aero_offset_compensation) {
    aero_offset_compensation_ = aero_offset_compensation;
  }

  double aero_offset_compensation() const { return aero_offset_compensation_; }

  void set_turning_offset_compensation(double turning_offset_compensation) {
    turning_offset_compensation_ = turning_offset_compensation;
  }

  double turning_offset_compensation() const {
    return turning_offset_compensation_;
  }

  void set_rolling_offset_compensation(double rolling_offset_compensation) {
    rolling_offset_compensation_ = rolling_offset_compensation;
  }

  double rolling_offset_compensation() const {
    return rolling_offset_compensation_;
  }

  void set_current_station(double current_station) {
    current_station_ = current_station;
  }

  double current_station() const { return current_station_; }

  void set_path_remain(double path_remain) { path_remain_ = path_remain; }

  double path_remain() const { return path_remain_; }

  void set_pid_saturation_status(int32_t pid_saturation_status) {
    pid_saturation_status_ = pid_saturation_status;
  }

  int32_t pid_saturation_status() const { return pid_saturation_status_; }

  void set_leadlag_saturation_status(int32_t leadlag_saturation_status) {
    leadlag_saturation_status_ = leadlag_saturation_status;
  }

  int32_t leadlag_saturation_status() const {
    return leadlag_saturation_status_;
  }

  void set_speed_offset(double speed_offset) { speed_offset_ = speed_offset; }

  double speed_offset() const { return speed_offset_; }

  void set_current_speed(double current_speed) {
    current_speed_ = current_speed;
  }

  double current_speed() const { return current_speed_; }

  void set_acceleration_reference(double acceleration_reference) {
    acceleration_reference_ = acceleration_reference;
  }

  double acceleration_reference() const { return acceleration_reference_; }

  void set_current_acceleration(double current_acceleration) {
    current_acceleration_ = current_acceleration;
  }

  double current_acceleration() const { return current_acceleration_; }

  void set_acceleration_error(double acceleration_error) {
    acceleration_error_ = acceleration_error;
  }

  double acceleration_error() const { return acceleration_error_; }

  void set_jerk_reference(double jerk_reference) {
    jerk_reference_ = jerk_reference;
  }

  double jerk_reference() const { return jerk_reference_; }

  void set_current_jerk(double current_jerk) { current_jerk_ = current_jerk; }

  double current_jerk() const { return current_jerk_; }

  void set_jerk_error(double jerk_error) { jerk_error_ = jerk_error; }

  double jerk_error() const { return jerk_error_; }

  void set_current_matched_point(
      legionclaw::interface::TrajectoryPoint current_matched_point) {
    current_matched_point_ = current_matched_point;
  }

  legionclaw::interface::TrajectoryPoint current_matched_point() const {
    return current_matched_point_;
  }

  void set_current_reference_point(
      legionclaw::interface::TrajectoryPoint current_reference_point) {
    current_reference_point_ = current_reference_point;
  }

  legionclaw::interface::TrajectoryPoint current_reference_point() const {
    return current_reference_point_;
  }

  void set_preview_reference_point(
      legionclaw::interface::TrajectoryPoint preview_reference_point) {
    preview_reference_point_ = preview_reference_point;
  }

  legionclaw::interface::TrajectoryPoint preview_reference_point() const {
    return preview_reference_point_;
  }

 protected:
  double station_reference_;
  double station_error_;
  double station_error_limited_;
  double preview_station_error_;
  double speed_reference_;
  double speed_error_;
  double speed_controller_input_limited_;
  double preview_speed_reference_;
  double preview_speed_error_;
  double preview_acceleration_reference_;
  double acceleration_cmd_closeloop_;
  double acceleration_cmd_;
  double acceleration_lookup_;
  double speed_lookup_;
  double calibration_value_;
  double throttle_cmd_;
  double brake_cmd_;
  bool is_full_stop_;
  double slope_offset_compensation_;
  double aero_offset_compensation_;
  double turning_offset_compensation_;
  double rolling_offset_compensation_;
  double current_station_;
  double path_remain_;
  int32_t pid_saturation_status_;
  int32_t leadlag_saturation_status_;
  double speed_offset_;
  double current_speed_;
  double acceleration_reference_;
  double current_acceleration_;
  double acceleration_error_;
  double jerk_reference_;
  double current_jerk_;
  double jerk_error_;
  legionclaw::interface::TrajectoryPoint current_matched_point_;
  legionclaw::interface::TrajectoryPoint current_reference_point_;
  legionclaw::interface::TrajectoryPoint preview_reference_point_;
};
}  // namespace control
}  // namespace legionclaw
#endif  // CONTROLLER_LON_CONTROLLER_SIMPLE_LONGITUDINAL_DEBUG_H_
