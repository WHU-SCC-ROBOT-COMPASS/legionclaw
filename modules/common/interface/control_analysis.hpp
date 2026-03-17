/**
 * @file    control_analysis.hpp
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
class ControlAnalysis {
public:
  ControlAnalysis() {
    driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    driving_mode_fd_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    gear_location_fd_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    gear_location_cmd_ = legionclaw::common::GearPosition::GEAR_NEUTRAL;
    epb_level_fd_ = legionclaw::common::EPBLevel::RELEASED;
    epb_level_cmd_ = legionclaw::common::EPBLevel::RELEASED;
    speed_mps_ = 0.0;
    speed_reference_ = 0.0;
    accel_value_fd_ = 0.0;
    accel_value_cmd_ = 0.0;
    brake_value_fd_ = 0.0;
    brake_value_cmd_ = 0.0;
    path_remain_ = 0.0;
    has_stop_point_ = false;
    is_full_stop_ = false;
    is_stopped_ = false;
    lon_acc_jerk_ = 0.0;
    acceleration_cmd_ = 0.0;
    acceleration_cmd_closeloop_ = 0.0;
    preview_acceleration_reference_ = 0.0;
    slope_offset_compensation_ = 0.0;
    turning_offset_compensation_ = 0.0;
    speed_error_limited_ = 0.0;
    speed_error_ = 0.0;
    speed_offset_ = 0.0;
    station_error_limited_ = 0.0;
    station_error_ = 0.0;
    lon_target_point_s_ = 0.0;
    lon_calculate_time_ = 0;
    lon_calculate_time_max_ = 0;
    ref_curvature_ = 0.0;
    ref_heading_ = 0.0;
    current_heading_ = 0.0;
    heading_error_ = 0.0;
    heading_error_rate_ = 0.0;
    lateral_error_ = 0.0;
    lateral_error_rate_ = 0.0;
    lon_error_ = 0.0;
    front_steering_value_fd_ = 0.0;
    front_steering_target_ = 0.0;
    front_steering_rate_ = 0.0;
    front_steer_angle_feedforward_ = 0.0;
    front_steer_angle_feedback_ = 0.0;
    front_steer_angle_lateral_contribution_ = 0.0;
    front_steer_angle_lateral_rate_contribution_ = 0.0;
    front_steer_angle_heading_contribution_ = 0.0;
    front_steer_angle_heading_rate_contribution_ = 0.0;
    rear_steering_value_fd_ = 0.0;
    rear_steering_target_ = 0.0;
    rear_steering_rate_ = 0.0;
    rear_steer_angle_feedforward_ = 0.0;
    rear_steer_angle_feedback_ = 0.0;
    rear_steer_angle_lateral_contribution_ = 0.0;
    rear_steer_angle_lateral_rate_contribution_ = 0.0;
    rear_steer_angle_heading_contribution_ = 0.0;
    rear_steer_angle_heading_rate_contribution_ = 0.0;
    matrix_k_00_ = 0.0;
    matrix_k_01_ = 0.0;
    matrix_k_02_ = 0.0;
    matrix_k_03_ = 0.0;
    matrix_k_10_ = 0.0;
    matrix_k_11_ = 0.0;
    matrix_k_12_ = 0.0;
    matrix_k_13_ = 0.0;
    matrix_state_0_ = 0.0;
    matrix_state_1_ = 0.0;
    matrix_state_2_ = 0.0;
    matrix_state_3_ = 0.0;
    matrix_q_updated_0_ = 0.0;
    matrix_q_updated_1_ = 0.0;
    matrix_q_updated_2_ = 0.0;
    matrix_q_updated_3_ = 0.0;
    current_x_ = 0.0;
    current_y_ = 0.0;
    target_point_x_ = 0.0;
    target_point_y_ = 0.0;
    lat_target_point_s_ = 0.0;
    lqr_calculate_time_ = 0;
    lqr_calculate_time_max_ = 0;
  }
  ~ControlAnalysis() = default;

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
  set_driving_mode_fd(const legionclaw::common::DrivingMode &driving_mode_fd) {
    driving_mode_fd_ = driving_mode_fd;
    driving_mode_fd_ptr_ = &driving_mode_fd_;
  }

  inline const legionclaw::common::DrivingMode &driving_mode_fd() const {
    return driving_mode_fd_;
  }

  inline legionclaw::common::DrivingMode *mutable_driving_mode_fd() {
    return &driving_mode_fd_;
  }

  inline bool has_driving_mode_fd() {
    return (driving_mode_fd_ptr_ != nullptr);
  }

  inline void
  set_gear_location_fd(const legionclaw::common::GearPosition &gear_location_fd) {
    gear_location_fd_ = gear_location_fd;
    gear_location_fd_ptr_ = &gear_location_fd_;
  }

  inline const legionclaw::common::GearPosition &gear_location_fd() const {
    return gear_location_fd_;
  }

  inline legionclaw::common::GearPosition *mutable_gear_location_fd() {
    return &gear_location_fd_;
  }

  inline bool has_gear_location_fd() {
    return (gear_location_fd_ptr_ != nullptr);
  }

  inline void
  set_gear_location_cmd(const legionclaw::common::GearPosition &gear_location_cmd) {
    gear_location_cmd_ = gear_location_cmd;
    gear_location_cmd_ptr_ = &gear_location_cmd_;
  }

  inline const legionclaw::common::GearPosition &gear_location_cmd() const {
    return gear_location_cmd_;
  }

  inline legionclaw::common::GearPosition *mutable_gear_location_cmd() {
    return &gear_location_cmd_;
  }

  inline bool has_gear_location_cmd() {
    return (gear_location_cmd_ptr_ != nullptr);
  }

  inline void set_epb_level_fd(const legionclaw::common::EPBLevel &epb_level_fd) {
    epb_level_fd_ = epb_level_fd;
    epb_level_fd_ptr_ = &epb_level_fd_;
  }

  inline const legionclaw::common::EPBLevel &epb_level_fd() const {
    return epb_level_fd_;
  }

  inline legionclaw::common::EPBLevel *mutable_epb_level_fd() {
    return &epb_level_fd_;
  }

  inline bool has_epb_level_fd() { return (epb_level_fd_ptr_ != nullptr); }

  inline void set_epb_level_cmd(const legionclaw::common::EPBLevel &epb_level_cmd) {
    epb_level_cmd_ = epb_level_cmd;
    epb_level_cmd_ptr_ = &epb_level_cmd_;
  }

  inline const legionclaw::common::EPBLevel &epb_level_cmd() const {
    return epb_level_cmd_;
  }

  inline legionclaw::common::EPBLevel *mutable_epb_level_cmd() {
    return &epb_level_cmd_;
  }

  inline bool has_epb_level_cmd() { return (epb_level_cmd_ptr_ != nullptr); }

  inline void set_speed_mps(const double &speed_mps) {
    speed_mps_ = speed_mps;
    speed_mps_ptr_ = &speed_mps_;
  }

  inline const double &speed_mps() const { return speed_mps_; }

  inline double *mutable_speed_mps() { return &speed_mps_; }

  inline bool has_speed_mps() { return (speed_mps_ptr_ != nullptr); }

  inline void set_speed_reference(const double &speed_reference) {
    speed_reference_ = speed_reference;
    speed_reference_ptr_ = &speed_reference_;
  }

  inline const double &speed_reference() const { return speed_reference_; }

  inline double *mutable_speed_reference() { return &speed_reference_; }

  inline bool has_speed_reference() {
    return (speed_reference_ptr_ != nullptr);
  }

  inline void set_accel_value_fd(const double &accel_value_fd) {
    accel_value_fd_ = accel_value_fd;
    accel_value_fd_ptr_ = &accel_value_fd_;
  }

  inline const double &accel_value_fd() const { return accel_value_fd_; }

  inline double *mutable_accel_value_fd() { return &accel_value_fd_; }

  inline bool has_accel_value_fd() { return (accel_value_fd_ptr_ != nullptr); }

  inline void set_accel_value_cmd(const double &accel_value_cmd) {
    accel_value_cmd_ = accel_value_cmd;
    accel_value_cmd_ptr_ = &accel_value_cmd_;
  }

  inline const double &accel_value_cmd() const { return accel_value_cmd_; }

  inline double *mutable_accel_value_cmd() { return &accel_value_cmd_; }

  inline bool has_accel_value_cmd() {
    return (accel_value_cmd_ptr_ != nullptr);
  }

  inline void set_brake_value_fd(const double &brake_value_fd) {
    brake_value_fd_ = brake_value_fd;
    brake_value_fd_ptr_ = &brake_value_fd_;
  }

  inline const double &brake_value_fd() const { return brake_value_fd_; }

  inline double *mutable_brake_value_fd() { return &brake_value_fd_; }

  inline bool has_brake_value_fd() { return (brake_value_fd_ptr_ != nullptr); }

  inline void set_brake_value_cmd(const double &brake_value_cmd) {
    brake_value_cmd_ = brake_value_cmd;
    brake_value_cmd_ptr_ = &brake_value_cmd_;
  }

  inline const double &brake_value_cmd() const { return brake_value_cmd_; }

  inline double *mutable_brake_value_cmd() { return &brake_value_cmd_; }

  inline bool has_brake_value_cmd() {
    return (brake_value_cmd_ptr_ != nullptr);
  }

  inline void set_path_remain(const double &path_remain) {
    path_remain_ = path_remain;
    path_remain_ptr_ = &path_remain_;
  }

  inline const double &path_remain() const { return path_remain_; }

  inline double *mutable_path_remain() { return &path_remain_; }

  inline bool has_path_remain() { return (path_remain_ptr_ != nullptr); }

  inline void set_has_stop_point(const bool &has_stop_point) {
    has_stop_point_ = has_stop_point;
    has_stop_point_ptr_ = &has_stop_point_;
  }

  inline const bool &has_stop_point() const { return has_stop_point_; }

  inline bool *mutable_has_stop_point() { return &has_stop_point_; }

  inline bool has_has_stop_point() { return (has_stop_point_ptr_ != nullptr); }

  inline void set_is_full_stop(const bool &is_full_stop) {
    is_full_stop_ = is_full_stop;
    is_full_stop_ptr_ = &is_full_stop_;
  }

  inline const bool &is_full_stop() const { return is_full_stop_; }

  inline bool *mutable_is_full_stop() { return &is_full_stop_; }

  inline bool has_is_full_stop() { return (is_full_stop_ptr_ != nullptr); }

  inline void set_is_stopped(const bool &is_stopped) {
    is_stopped_ = is_stopped;
    is_stopped_ptr_ = &is_stopped_;
  }

  inline const bool &is_stopped() const { return is_stopped_; }

  inline bool *mutable_is_stopped() { return &is_stopped_; }

  inline bool has_is_stopped() { return (is_stopped_ptr_ != nullptr); }

  inline void set_lon_acc_jerk(const double &lon_acc_jerk) {
    lon_acc_jerk_ = lon_acc_jerk;
    lon_acc_jerk_ptr_ = &lon_acc_jerk_;
  }

  inline const double &lon_acc_jerk() const { return lon_acc_jerk_; }

  inline double *mutable_lon_acc_jerk() { return &lon_acc_jerk_; }

  inline bool has_lon_acc_jerk() { return (lon_acc_jerk_ptr_ != nullptr); }

  inline void set_acceleration_cmd(const double &acceleration_cmd) {
    acceleration_cmd_ = acceleration_cmd;
    acceleration_cmd_ptr_ = &acceleration_cmd_;
  }

  inline const double &acceleration_cmd() const { return acceleration_cmd_; }

  inline double *mutable_acceleration_cmd() { return &acceleration_cmd_; }

  inline bool has_acceleration_cmd() {
    return (acceleration_cmd_ptr_ != nullptr);
  }

  inline void
  set_acceleration_cmd_closeloop(const double &acceleration_cmd_closeloop) {
    acceleration_cmd_closeloop_ = acceleration_cmd_closeloop;
    acceleration_cmd_closeloop_ptr_ = &acceleration_cmd_closeloop_;
  }

  inline const double &acceleration_cmd_closeloop() const {
    return acceleration_cmd_closeloop_;
  }

  inline double *mutable_acceleration_cmd_closeloop() {
    return &acceleration_cmd_closeloop_;
  }

  inline bool has_acceleration_cmd_closeloop() {
    return (acceleration_cmd_closeloop_ptr_ != nullptr);
  }

  inline void set_preview_acceleration_reference(
      const double &preview_acceleration_reference) {
    preview_acceleration_reference_ = preview_acceleration_reference;
    preview_acceleration_reference_ptr_ = &preview_acceleration_reference_;
  }

  inline const double &preview_acceleration_reference() const {
    return preview_acceleration_reference_;
  }

  inline double *mutable_preview_acceleration_reference() {
    return &preview_acceleration_reference_;
  }

  inline bool has_preview_acceleration_reference() {
    return (preview_acceleration_reference_ptr_ != nullptr);
  }

  inline void
  set_slope_offset_compensation(const double &slope_offset_compensation) {
    slope_offset_compensation_ = slope_offset_compensation;
    slope_offset_compensation_ptr_ = &slope_offset_compensation_;
  }

  inline const double &slope_offset_compensation() const {
    return slope_offset_compensation_;
  }

  inline double *mutable_slope_offset_compensation() {
    return &slope_offset_compensation_;
  }

  inline bool has_slope_offset_compensation() {
    return (slope_offset_compensation_ptr_ != nullptr);
  }

  inline void
  set_turning_offset_compensation(const double &turning_offset_compensation) {
    turning_offset_compensation_ = turning_offset_compensation;
    turning_offset_compensation_ptr_ = &turning_offset_compensation_;
  }

  inline const double &turning_offset_compensation() const {
    return turning_offset_compensation_;
  }

  inline double *mutable_turning_offset_compensation() {
    return &turning_offset_compensation_;
  }

  inline bool has_turning_offset_compensation() {
    return (turning_offset_compensation_ptr_ != nullptr);
  }

  inline void set_speed_error_limited(const double &speed_error_limited) {
    speed_error_limited_ = speed_error_limited;
    speed_error_limited_ptr_ = &speed_error_limited_;
  }

  inline const double &speed_error_limited() const {
    return speed_error_limited_;
  }

  inline double *mutable_speed_error_limited() { return &speed_error_limited_; }

  inline bool has_speed_error_limited() {
    return (speed_error_limited_ptr_ != nullptr);
  }

  inline void set_speed_error(const double &speed_error) {
    speed_error_ = speed_error;
    speed_error_ptr_ = &speed_error_;
  }

  inline const double &speed_error() const { return speed_error_; }

  inline double *mutable_speed_error() { return &speed_error_; }

  inline bool has_speed_error() { return (speed_error_ptr_ != nullptr); }

  inline void set_speed_offset(const double &speed_offset) {
    speed_offset_ = speed_offset;
    speed_offset_ptr_ = &speed_offset_;
  }

  inline const double &speed_offset() const { return speed_offset_; }

  inline double *mutable_speed_offset() { return &speed_offset_; }

  inline bool has_speed_offset() { return (speed_offset_ptr_ != nullptr); }

  inline void set_station_error_limited(const double &station_error_limited) {
    station_error_limited_ = station_error_limited;
    station_error_limited_ptr_ = &station_error_limited_;
  }

  inline const double &station_error_limited() const {
    return station_error_limited_;
  }

  inline double *mutable_station_error_limited() {
    return &station_error_limited_;
  }

  inline bool has_station_error_limited() {
    return (station_error_limited_ptr_ != nullptr);
  }

  inline void set_station_error(const double &station_error) {
    station_error_ = station_error;
    station_error_ptr_ = &station_error_;
  }

  inline const double &station_error() const { return station_error_; }

  inline double *mutable_station_error() { return &station_error_; }

  inline bool has_station_error() { return (station_error_ptr_ != nullptr); }

  inline void set_lon_target_point_s(const double &lon_target_point_s) {
    lon_target_point_s_ = lon_target_point_s;
    lon_target_point_s_ptr_ = &lon_target_point_s_;
  }

  inline const double &lon_target_point_s() const {
    return lon_target_point_s_;
  }

  inline double *mutable_lon_target_point_s() { return &lon_target_point_s_; }

  inline bool has_lon_target_point_s() {
    return (lon_target_point_s_ptr_ != nullptr);
  }

  inline void set_lon_calculate_time(const int64_t &lon_calculate_time) {
    lon_calculate_time_ = lon_calculate_time;
    lon_calculate_time_ptr_ = &lon_calculate_time_;
  }

  inline const int64_t &lon_calculate_time() const {
    return lon_calculate_time_;
  }

  inline int64_t *mutable_lon_calculate_time() { return &lon_calculate_time_; }

  inline bool has_lon_calculate_time() {
    return (lon_calculate_time_ptr_ != nullptr);
  }

  inline void
  set_lon_calculate_time_max(const int64_t &lon_calculate_time_max) {
    lon_calculate_time_max_ = lon_calculate_time_max;
    lon_calculate_time_max_ptr_ = &lon_calculate_time_max_;
  }

  inline const int64_t &lon_calculate_time_max() const {
    return lon_calculate_time_max_;
  }

  inline int64_t *mutable_lon_calculate_time_max() {
    return &lon_calculate_time_max_;
  }

  inline bool has_lon_calculate_time_max() {
    return (lon_calculate_time_max_ptr_ != nullptr);
  }

  inline void set_ref_curvature(const double &ref_curvature) {
    ref_curvature_ = ref_curvature;
    ref_curvature_ptr_ = &ref_curvature_;
  }

  inline const double &ref_curvature() const { return ref_curvature_; }

  inline double *mutable_ref_curvature() { return &ref_curvature_; }

  inline bool has_ref_curvature() { return (ref_curvature_ptr_ != nullptr); }

  inline void set_ref_heading(const double &ref_heading) {
    ref_heading_ = ref_heading;
    ref_heading_ptr_ = &ref_heading_;
  }

  inline const double &ref_heading() const { return ref_heading_; }

  inline double *mutable_ref_heading() { return &ref_heading_; }

  inline bool has_ref_heading() { return (ref_heading_ptr_ != nullptr); }

  inline void set_current_heading(const double &current_heading) {
    current_heading_ = current_heading;
    current_heading_ptr_ = &current_heading_;
  }

  inline const double &current_heading() const { return current_heading_; }

  inline double *mutable_current_heading() { return &current_heading_; }

  inline bool has_current_heading() {
    return (current_heading_ptr_ != nullptr);
  }

  inline void set_heading_error(const double &heading_error) {
    heading_error_ = heading_error;
    heading_error_ptr_ = &heading_error_;
  }

  inline const double &heading_error() const { return heading_error_; }

  inline double *mutable_heading_error() { return &heading_error_; }

  inline bool has_heading_error() { return (heading_error_ptr_ != nullptr); }

  inline void set_heading_error_rate(const double &heading_error_rate) {
    heading_error_rate_ = heading_error_rate;
    heading_error_rate_ptr_ = &heading_error_rate_;
  }

  inline const double &heading_error_rate() const {
    return heading_error_rate_;
  }

  inline double *mutable_heading_error_rate() { return &heading_error_rate_; }

  inline bool has_heading_error_rate() {
    return (heading_error_rate_ptr_ != nullptr);
  }

  inline void set_lateral_error(const double &lateral_error) {
    lateral_error_ = lateral_error;
    lateral_error_ptr_ = &lateral_error_;
  }

  inline const double &lateral_error() const { return lateral_error_; }

  inline double *mutable_lateral_error() { return &lateral_error_; }

  inline bool has_lateral_error() { return (lateral_error_ptr_ != nullptr); }

  inline void set_lateral_error_rate(const double &lateral_error_rate) {
    lateral_error_rate_ = lateral_error_rate;
    lateral_error_rate_ptr_ = &lateral_error_rate_;
  }

  inline const double &lateral_error_rate() const {
    return lateral_error_rate_;
  }

  inline double *mutable_lateral_error_rate() { return &lateral_error_rate_; }

  inline bool has_lateral_error_rate() {
    return (lateral_error_rate_ptr_ != nullptr);
  }

  inline void set_lon_error(const double &lon_error) {
    lon_error_ = lon_error;
    lon_error_ptr_ = &lon_error_;
  }

  inline const double &lon_error() const { return lon_error_; }

  inline double *mutable_lon_error() { return &lon_error_; }

  inline bool has_lon_error() { return (lon_error_ptr_ != nullptr); }

  inline void
  set_front_steering_value_fd(const double &front_steering_value_fd) {
    front_steering_value_fd_ = front_steering_value_fd;
    front_steering_value_fd_ptr_ = &front_steering_value_fd_;
  }

  inline const double &front_steering_value_fd() const {
    return front_steering_value_fd_;
  }

  inline double *mutable_front_steering_value_fd() {
    return &front_steering_value_fd_;
  }

  inline bool has_front_steering_value_fd() {
    return (front_steering_value_fd_ptr_ != nullptr);
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

  inline void set_front_steer_angle_feedforward(
      const double &front_steer_angle_feedforward) {
    front_steer_angle_feedforward_ = front_steer_angle_feedforward;
    front_steer_angle_feedforward_ptr_ = &front_steer_angle_feedforward_;
  }

  inline const double &front_steer_angle_feedforward() const {
    return front_steer_angle_feedforward_;
  }

  inline double *mutable_front_steer_angle_feedforward() {
    return &front_steer_angle_feedforward_;
  }

  inline bool has_front_steer_angle_feedforward() {
    return (front_steer_angle_feedforward_ptr_ != nullptr);
  }

  inline void
  set_front_steer_angle_feedback(const double &front_steer_angle_feedback) {
    front_steer_angle_feedback_ = front_steer_angle_feedback;
    front_steer_angle_feedback_ptr_ = &front_steer_angle_feedback_;
  }

  inline const double &front_steer_angle_feedback() const {
    return front_steer_angle_feedback_;
  }

  inline double *mutable_front_steer_angle_feedback() {
    return &front_steer_angle_feedback_;
  }

  inline bool has_front_steer_angle_feedback() {
    return (front_steer_angle_feedback_ptr_ != nullptr);
  }

  inline void set_front_steer_angle_lateral_contribution(
      const double &front_steer_angle_lateral_contribution) {
    front_steer_angle_lateral_contribution_ =
        front_steer_angle_lateral_contribution;
    front_steer_angle_lateral_contribution_ptr_ =
        &front_steer_angle_lateral_contribution_;
  }

  inline const double &front_steer_angle_lateral_contribution() const {
    return front_steer_angle_lateral_contribution_;
  }

  inline double *mutable_front_steer_angle_lateral_contribution() {
    return &front_steer_angle_lateral_contribution_;
  }

  inline bool has_front_steer_angle_lateral_contribution() {
    return (front_steer_angle_lateral_contribution_ptr_ != nullptr);
  }

  inline void set_front_steer_angle_lateral_rate_contribution(
      const double &front_steer_angle_lateral_rate_contribution) {
    front_steer_angle_lateral_rate_contribution_ =
        front_steer_angle_lateral_rate_contribution;
    front_steer_angle_lateral_rate_contribution_ptr_ =
        &front_steer_angle_lateral_rate_contribution_;
  }

  inline const double &front_steer_angle_lateral_rate_contribution() const {
    return front_steer_angle_lateral_rate_contribution_;
  }

  inline double *mutable_front_steer_angle_lateral_rate_contribution() {
    return &front_steer_angle_lateral_rate_contribution_;
  }

  inline bool has_front_steer_angle_lateral_rate_contribution() {
    return (front_steer_angle_lateral_rate_contribution_ptr_ != nullptr);
  }

  inline void set_front_steer_angle_heading_contribution(
      const double &front_steer_angle_heading_contribution) {
    front_steer_angle_heading_contribution_ =
        front_steer_angle_heading_contribution;
    front_steer_angle_heading_contribution_ptr_ =
        &front_steer_angle_heading_contribution_;
  }

  inline const double &front_steer_angle_heading_contribution() const {
    return front_steer_angle_heading_contribution_;
  }

  inline double *mutable_front_steer_angle_heading_contribution() {
    return &front_steer_angle_heading_contribution_;
  }

  inline bool has_front_steer_angle_heading_contribution() {
    return (front_steer_angle_heading_contribution_ptr_ != nullptr);
  }

  inline void set_front_steer_angle_heading_rate_contribution(
      const double &front_steer_angle_heading_rate_contribution) {
    front_steer_angle_heading_rate_contribution_ =
        front_steer_angle_heading_rate_contribution;
    front_steer_angle_heading_rate_contribution_ptr_ =
        &front_steer_angle_heading_rate_contribution_;
  }

  inline const double &front_steer_angle_heading_rate_contribution() const {
    return front_steer_angle_heading_rate_contribution_;
  }

  inline double *mutable_front_steer_angle_heading_rate_contribution() {
    return &front_steer_angle_heading_rate_contribution_;
  }

  inline bool has_front_steer_angle_heading_rate_contribution() {
    return (front_steer_angle_heading_rate_contribution_ptr_ != nullptr);
  }

  inline void set_rear_steering_value_fd(const double &rear_steering_value_fd) {
    rear_steering_value_fd_ = rear_steering_value_fd;
    rear_steering_value_fd_ptr_ = &rear_steering_value_fd_;
  }

  inline const double &rear_steering_value_fd() const {
    return rear_steering_value_fd_;
  }

  inline double *mutable_rear_steering_value_fd() {
    return &rear_steering_value_fd_;
  }

  inline bool has_rear_steering_value_fd() {
    return (rear_steering_value_fd_ptr_ != nullptr);
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
  set_rear_steer_angle_feedforward(const double &rear_steer_angle_feedforward) {
    rear_steer_angle_feedforward_ = rear_steer_angle_feedforward;
    rear_steer_angle_feedforward_ptr_ = &rear_steer_angle_feedforward_;
  }

  inline const double &rear_steer_angle_feedforward() const {
    return rear_steer_angle_feedforward_;
  }

  inline double *mutable_rear_steer_angle_feedforward() {
    return &rear_steer_angle_feedforward_;
  }

  inline bool has_rear_steer_angle_feedforward() {
    return (rear_steer_angle_feedforward_ptr_ != nullptr);
  }

  inline void
  set_rear_steer_angle_feedback(const double &rear_steer_angle_feedback) {
    rear_steer_angle_feedback_ = rear_steer_angle_feedback;
    rear_steer_angle_feedback_ptr_ = &rear_steer_angle_feedback_;
  }

  inline const double &rear_steer_angle_feedback() const {
    return rear_steer_angle_feedback_;
  }

  inline double *mutable_rear_steer_angle_feedback() {
    return &rear_steer_angle_feedback_;
  }

  inline bool has_rear_steer_angle_feedback() {
    return (rear_steer_angle_feedback_ptr_ != nullptr);
  }

  inline void set_rear_steer_angle_lateral_contribution(
      const double &rear_steer_angle_lateral_contribution) {
    rear_steer_angle_lateral_contribution_ =
        rear_steer_angle_lateral_contribution;
    rear_steer_angle_lateral_contribution_ptr_ =
        &rear_steer_angle_lateral_contribution_;
  }

  inline const double &rear_steer_angle_lateral_contribution() const {
    return rear_steer_angle_lateral_contribution_;
  }

  inline double *mutable_rear_steer_angle_lateral_contribution() {
    return &rear_steer_angle_lateral_contribution_;
  }

  inline bool has_rear_steer_angle_lateral_contribution() {
    return (rear_steer_angle_lateral_contribution_ptr_ != nullptr);
  }

  inline void set_rear_steer_angle_lateral_rate_contribution(
      const double &rear_steer_angle_lateral_rate_contribution) {
    rear_steer_angle_lateral_rate_contribution_ =
        rear_steer_angle_lateral_rate_contribution;
    rear_steer_angle_lateral_rate_contribution_ptr_ =
        &rear_steer_angle_lateral_rate_contribution_;
  }

  inline const double &rear_steer_angle_lateral_rate_contribution() const {
    return rear_steer_angle_lateral_rate_contribution_;
  }

  inline double *mutable_rear_steer_angle_lateral_rate_contribution() {
    return &rear_steer_angle_lateral_rate_contribution_;
  }

  inline bool has_rear_steer_angle_lateral_rate_contribution() {
    return (rear_steer_angle_lateral_rate_contribution_ptr_ != nullptr);
  }

  inline void set_rear_steer_angle_heading_contribution(
      const double &rear_steer_angle_heading_contribution) {
    rear_steer_angle_heading_contribution_ =
        rear_steer_angle_heading_contribution;
    rear_steer_angle_heading_contribution_ptr_ =
        &rear_steer_angle_heading_contribution_;
  }

  inline const double &rear_steer_angle_heading_contribution() const {
    return rear_steer_angle_heading_contribution_;
  }

  inline double *mutable_rear_steer_angle_heading_contribution() {
    return &rear_steer_angle_heading_contribution_;
  }

  inline bool has_rear_steer_angle_heading_contribution() {
    return (rear_steer_angle_heading_contribution_ptr_ != nullptr);
  }

  inline void set_rear_steer_angle_heading_rate_contribution(
      const double &rear_steer_angle_heading_rate_contribution) {
    rear_steer_angle_heading_rate_contribution_ =
        rear_steer_angle_heading_rate_contribution;
    rear_steer_angle_heading_rate_contribution_ptr_ =
        &rear_steer_angle_heading_rate_contribution_;
  }

  inline const double &rear_steer_angle_heading_rate_contribution() const {
    return rear_steer_angle_heading_rate_contribution_;
  }

  inline double *mutable_rear_steer_angle_heading_rate_contribution() {
    return &rear_steer_angle_heading_rate_contribution_;
  }

  inline bool has_rear_steer_angle_heading_rate_contribution() {
    return (rear_steer_angle_heading_rate_contribution_ptr_ != nullptr);
  }

  inline void set_matrix_k_00(const double &matrix_k_00) {
    matrix_k_00_ = matrix_k_00;
    matrix_k_00_ptr_ = &matrix_k_00_;
  }

  inline const double &matrix_k_00() const { return matrix_k_00_; }

  inline double *mutable_matrix_k_00() { return &matrix_k_00_; }

  inline bool has_matrix_k_00() { return (matrix_k_00_ptr_ != nullptr); }

  inline void set_matrix_k_01(const double &matrix_k_01) {
    matrix_k_01_ = matrix_k_01;
    matrix_k_01_ptr_ = &matrix_k_01_;
  }

  inline const double &matrix_k_01() const { return matrix_k_01_; }

  inline double *mutable_matrix_k_01() { return &matrix_k_01_; }

  inline bool has_matrix_k_01() { return (matrix_k_01_ptr_ != nullptr); }

  inline void set_matrix_k_02(const double &matrix_k_02) {
    matrix_k_02_ = matrix_k_02;
    matrix_k_02_ptr_ = &matrix_k_02_;
  }

  inline const double &matrix_k_02() const { return matrix_k_02_; }

  inline double *mutable_matrix_k_02() { return &matrix_k_02_; }

  inline bool has_matrix_k_02() { return (matrix_k_02_ptr_ != nullptr); }

  inline void set_matrix_k_03(const double &matrix_k_03) {
    matrix_k_03_ = matrix_k_03;
    matrix_k_03_ptr_ = &matrix_k_03_;
  }

  inline const double &matrix_k_03() const { return matrix_k_03_; }

  inline double *mutable_matrix_k_03() { return &matrix_k_03_; }

  inline bool has_matrix_k_03() { return (matrix_k_03_ptr_ != nullptr); }

  inline void set_matrix_k_10(const double &matrix_k_10) {
    matrix_k_10_ = matrix_k_10;
    matrix_k_10_ptr_ = &matrix_k_10_;
  }

  inline const double &matrix_k_10() const { return matrix_k_10_; }

  inline double *mutable_matrix_k_10() { return &matrix_k_10_; }

  inline bool has_matrix_k_10() { return (matrix_k_10_ptr_ != nullptr); }

  inline void set_matrix_k_11(const double &matrix_k_11) {
    matrix_k_11_ = matrix_k_11;
    matrix_k_11_ptr_ = &matrix_k_11_;
  }

  inline const double &matrix_k_11() const { return matrix_k_11_; }

  inline double *mutable_matrix_k_11() { return &matrix_k_11_; }

  inline bool has_matrix_k_11() { return (matrix_k_11_ptr_ != nullptr); }

  inline void set_matrix_k_12(const double &matrix_k_12) {
    matrix_k_12_ = matrix_k_12;
    matrix_k_12_ptr_ = &matrix_k_12_;
  }

  inline const double &matrix_k_12() const { return matrix_k_12_; }

  inline double *mutable_matrix_k_12() { return &matrix_k_12_; }

  inline bool has_matrix_k_12() { return (matrix_k_12_ptr_ != nullptr); }

  inline void set_matrix_k_13(const double &matrix_k_13) {
    matrix_k_13_ = matrix_k_13;
    matrix_k_13_ptr_ = &matrix_k_13_;
  }

  inline const double &matrix_k_13() const { return matrix_k_13_; }

  inline double *mutable_matrix_k_13() { return &matrix_k_13_; }

  inline bool has_matrix_k_13() { return (matrix_k_13_ptr_ != nullptr); }

  inline void set_matrix_state_0(const double &matrix_state_0) {
    matrix_state_0_ = matrix_state_0;
    matrix_state_0_ptr_ = &matrix_state_0_;
  }

  inline const double &matrix_state_0() const { return matrix_state_0_; }

  inline double *mutable_matrix_state_0() { return &matrix_state_0_; }

  inline bool has_matrix_state_0() { return (matrix_state_0_ptr_ != nullptr); }

  inline void set_matrix_state_1(const double &matrix_state_1) {
    matrix_state_1_ = matrix_state_1;
    matrix_state_1_ptr_ = &matrix_state_1_;
  }

  inline const double &matrix_state_1() const { return matrix_state_1_; }

  inline double *mutable_matrix_state_1() { return &matrix_state_1_; }

  inline bool has_matrix_state_1() { return (matrix_state_1_ptr_ != nullptr); }

  inline void set_matrix_state_2(const double &matrix_state_2) {
    matrix_state_2_ = matrix_state_2;
    matrix_state_2_ptr_ = &matrix_state_2_;
  }

  inline const double &matrix_state_2() const { return matrix_state_2_; }

  inline double *mutable_matrix_state_2() { return &matrix_state_2_; }

  inline bool has_matrix_state_2() { return (matrix_state_2_ptr_ != nullptr); }

  inline void set_matrix_state_3(const double &matrix_state_3) {
    matrix_state_3_ = matrix_state_3;
    matrix_state_3_ptr_ = &matrix_state_3_;
  }

  inline const double &matrix_state_3() const { return matrix_state_3_; }

  inline double *mutable_matrix_state_3() { return &matrix_state_3_; }

  inline bool has_matrix_state_3() { return (matrix_state_3_ptr_ != nullptr); }

  inline void set_matrix_q_updated_0(const double &matrix_q_updated_0) {
    matrix_q_updated_0_ = matrix_q_updated_0;
    matrix_q_updated_0_ptr_ = &matrix_q_updated_0_;
  }

  inline const double &matrix_q_updated_0() const {
    return matrix_q_updated_0_;
  }

  inline double *mutable_matrix_q_updated_0() { return &matrix_q_updated_0_; }

  inline bool has_matrix_q_updated_0() {
    return (matrix_q_updated_0_ptr_ != nullptr);
  }

  inline void set_matrix_q_updated_1(const double &matrix_q_updated_1) {
    matrix_q_updated_1_ = matrix_q_updated_1;
    matrix_q_updated_1_ptr_ = &matrix_q_updated_1_;
  }

  inline const double &matrix_q_updated_1() const {
    return matrix_q_updated_1_;
  }

  inline double *mutable_matrix_q_updated_1() { return &matrix_q_updated_1_; }

  inline bool has_matrix_q_updated_1() {
    return (matrix_q_updated_1_ptr_ != nullptr);
  }

  inline void set_matrix_q_updated_2(const double &matrix_q_updated_2) {
    matrix_q_updated_2_ = matrix_q_updated_2;
    matrix_q_updated_2_ptr_ = &matrix_q_updated_2_;
  }

  inline const double &matrix_q_updated_2() const {
    return matrix_q_updated_2_;
  }

  inline double *mutable_matrix_q_updated_2() { return &matrix_q_updated_2_; }

  inline bool has_matrix_q_updated_2() {
    return (matrix_q_updated_2_ptr_ != nullptr);
  }

  inline void set_matrix_q_updated_3(const double &matrix_q_updated_3) {
    matrix_q_updated_3_ = matrix_q_updated_3;
    matrix_q_updated_3_ptr_ = &matrix_q_updated_3_;
  }

  inline const double &matrix_q_updated_3() const {
    return matrix_q_updated_3_;
  }

  inline double *mutable_matrix_q_updated_3() { return &matrix_q_updated_3_; }

  inline bool has_matrix_q_updated_3() {
    return (matrix_q_updated_3_ptr_ != nullptr);
  }

  inline void set_current_x(const double &current_x) {
    current_x_ = current_x;
    current_x_ptr_ = &current_x_;
  }

  inline const double &current_x() const { return current_x_; }

  inline double *mutable_current_x() { return &current_x_; }

  inline bool has_current_x() { return (current_x_ptr_ != nullptr); }

  inline void set_current_y(const double &current_y) {
    current_y_ = current_y;
    current_y_ptr_ = &current_y_;
  }

  inline const double &current_y() const { return current_y_; }

  inline double *mutable_current_y() { return &current_y_; }

  inline bool has_current_y() { return (current_y_ptr_ != nullptr); }

  inline void set_target_point_x(const double &target_point_x) {
    target_point_x_ = target_point_x;
    target_point_x_ptr_ = &target_point_x_;
  }

  inline const double &target_point_x() const { return target_point_x_; }

  inline double *mutable_target_point_x() { return &target_point_x_; }

  inline bool has_target_point_x() { return (target_point_x_ptr_ != nullptr); }

  inline void set_target_point_y(const double &target_point_y) {
    target_point_y_ = target_point_y;
    target_point_y_ptr_ = &target_point_y_;
  }

  inline const double &target_point_y() const { return target_point_y_; }

  inline double *mutable_target_point_y() { return &target_point_y_; }

  inline bool has_target_point_y() { return (target_point_y_ptr_ != nullptr); }

  inline void set_lat_target_point_s(const double &lat_target_point_s) {
    lat_target_point_s_ = lat_target_point_s;
    lat_target_point_s_ptr_ = &lat_target_point_s_;
  }

  inline const double &lat_target_point_s() const {
    return lat_target_point_s_;
  }

  inline double *mutable_lat_target_point_s() { return &lat_target_point_s_; }

  inline bool has_lat_target_point_s() {
    return (lat_target_point_s_ptr_ != nullptr);
  }

  inline void set_lqr_calculate_time(const int64_t &lqr_calculate_time) {
    lqr_calculate_time_ = lqr_calculate_time;
    lqr_calculate_time_ptr_ = &lqr_calculate_time_;
  }

  inline const int64_t &lqr_calculate_time() const {
    return lqr_calculate_time_;
  }

  inline int64_t *mutable_lqr_calculate_time() { return &lqr_calculate_time_; }

  inline bool has_lqr_calculate_time() {
    return (lqr_calculate_time_ptr_ != nullptr);
  }

  inline void
  set_lqr_calculate_time_max(const int64_t &lqr_calculate_time_max) {
    lqr_calculate_time_max_ = lqr_calculate_time_max;
    lqr_calculate_time_max_ptr_ = &lqr_calculate_time_max_;
  }

  inline const int64_t &lqr_calculate_time_max() const {
    return lqr_calculate_time_max_;
  }

  inline int64_t *mutable_lqr_calculate_time_max() {
    return &lqr_calculate_time_max_;
  }

  inline bool has_lqr_calculate_time_max() {
    return (lqr_calculate_time_max_ptr_ != nullptr);
  }

  void operator=(const ControlAnalysis &control_analysis) {
    CopyFrom(control_analysis);
  }

  void CopyFrom(const ControlAnalysis &control_analysis) {
    header_ = control_analysis.header();
    driving_mode_ = control_analysis.driving_mode();
    driving_mode_fd_ = control_analysis.driving_mode_fd();
    gear_location_fd_ = control_analysis.gear_location_fd();
    gear_location_cmd_ = control_analysis.gear_location_cmd();
    epb_level_fd_ = control_analysis.epb_level_fd();
    epb_level_cmd_ = control_analysis.epb_level_cmd();
    speed_mps_ = control_analysis.speed_mps();
    speed_reference_ = control_analysis.speed_reference();
    accel_value_fd_ = control_analysis.accel_value_fd();
    accel_value_cmd_ = control_analysis.accel_value_cmd();
    brake_value_fd_ = control_analysis.brake_value_fd();
    brake_value_cmd_ = control_analysis.brake_value_cmd();
    path_remain_ = control_analysis.path_remain();
    has_stop_point_ = control_analysis.has_stop_point();
    is_full_stop_ = control_analysis.is_full_stop();
    is_stopped_ = control_analysis.is_stopped();
    lon_acc_jerk_ = control_analysis.lon_acc_jerk();
    acceleration_cmd_ = control_analysis.acceleration_cmd();
    acceleration_cmd_closeloop_ = control_analysis.acceleration_cmd_closeloop();
    preview_acceleration_reference_ =
        control_analysis.preview_acceleration_reference();
    slope_offset_compensation_ = control_analysis.slope_offset_compensation();
    turning_offset_compensation_ =
        control_analysis.turning_offset_compensation();
    speed_error_limited_ = control_analysis.speed_error_limited();
    speed_error_ = control_analysis.speed_error();
    speed_offset_ = control_analysis.speed_offset();
    station_error_limited_ = control_analysis.station_error_limited();
    station_error_ = control_analysis.station_error();
    lon_target_point_s_ = control_analysis.lon_target_point_s();
    lon_calculate_time_ = control_analysis.lon_calculate_time();
    lon_calculate_time_max_ = control_analysis.lon_calculate_time_max();
    ref_curvature_ = control_analysis.ref_curvature();
    ref_heading_ = control_analysis.ref_heading();
    current_heading_ = control_analysis.current_heading();
    heading_error_ = control_analysis.heading_error();
    heading_error_rate_ = control_analysis.heading_error_rate();
    lateral_error_ = control_analysis.lateral_error();
    lateral_error_rate_ = control_analysis.lateral_error_rate();
    lon_error_ = control_analysis.lon_error();
    front_steering_value_fd_ = control_analysis.front_steering_value_fd();
    front_steering_target_ = control_analysis.front_steering_target();
    front_steering_rate_ = control_analysis.front_steering_rate();
    front_steer_angle_feedforward_ =
        control_analysis.front_steer_angle_feedforward();
    front_steer_angle_feedback_ = control_analysis.front_steer_angle_feedback();
    front_steer_angle_lateral_contribution_ =
        control_analysis.front_steer_angle_lateral_contribution();
    front_steer_angle_lateral_rate_contribution_ =
        control_analysis.front_steer_angle_lateral_rate_contribution();
    front_steer_angle_heading_contribution_ =
        control_analysis.front_steer_angle_heading_contribution();
    front_steer_angle_heading_rate_contribution_ =
        control_analysis.front_steer_angle_heading_rate_contribution();
    rear_steering_value_fd_ = control_analysis.rear_steering_value_fd();
    rear_steering_target_ = control_analysis.rear_steering_target();
    rear_steering_rate_ = control_analysis.rear_steering_rate();
    rear_steer_angle_feedforward_ =
        control_analysis.rear_steer_angle_feedforward();
    rear_steer_angle_feedback_ = control_analysis.rear_steer_angle_feedback();
    rear_steer_angle_lateral_contribution_ =
        control_analysis.rear_steer_angle_lateral_contribution();
    rear_steer_angle_lateral_rate_contribution_ =
        control_analysis.rear_steer_angle_lateral_rate_contribution();
    rear_steer_angle_heading_contribution_ =
        control_analysis.rear_steer_angle_heading_contribution();
    rear_steer_angle_heading_rate_contribution_ =
        control_analysis.rear_steer_angle_heading_rate_contribution();
    matrix_k_00_ = control_analysis.matrix_k_00();
    matrix_k_01_ = control_analysis.matrix_k_01();
    matrix_k_02_ = control_analysis.matrix_k_02();
    matrix_k_03_ = control_analysis.matrix_k_03();
    matrix_k_10_ = control_analysis.matrix_k_10();
    matrix_k_11_ = control_analysis.matrix_k_11();
    matrix_k_12_ = control_analysis.matrix_k_12();
    matrix_k_13_ = control_analysis.matrix_k_13();
    matrix_state_0_ = control_analysis.matrix_state_0();
    matrix_state_1_ = control_analysis.matrix_state_1();
    matrix_state_2_ = control_analysis.matrix_state_2();
    matrix_state_3_ = control_analysis.matrix_state_3();
    matrix_q_updated_0_ = control_analysis.matrix_q_updated_0();
    matrix_q_updated_1_ = control_analysis.matrix_q_updated_1();
    matrix_q_updated_2_ = control_analysis.matrix_q_updated_2();
    matrix_q_updated_3_ = control_analysis.matrix_q_updated_3();
    current_x_ = control_analysis.current_x();
    current_y_ = control_analysis.current_y();
    target_point_x_ = control_analysis.target_point_x();
    target_point_y_ = control_analysis.target_point_y();
    lat_target_point_s_ = control_analysis.lat_target_point_s();
    lqr_calculate_time_ = control_analysis.lqr_calculate_time();
    lqr_calculate_time_max_ = control_analysis.lqr_calculate_time_max();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //驾驶模式
  legionclaw::common::DrivingMode driving_mode_;
  legionclaw::common::DrivingMode *driving_mode_ptr_ = nullptr;
  //驾驶模式反馈
  legionclaw::common::DrivingMode driving_mode_fd_;
  legionclaw::common::DrivingMode *driving_mode_fd_ptr_ = nullptr;
  //档位反馈
  legionclaw::common::GearPosition gear_location_fd_;
  legionclaw::common::GearPosition *gear_location_fd_ptr_ = nullptr;
  //档位命令
  legionclaw::common::GearPosition gear_location_cmd_;
  legionclaw::common::GearPosition *gear_location_cmd_ptr_ = nullptr;
  // EPB状态反馈
  legionclaw::common::EPBLevel epb_level_fd_;
  legionclaw::common::EPBLevel *epb_level_fd_ptr_ = nullptr;
  // EPB命令
  legionclaw::common::EPBLevel epb_level_cmd_;
  legionclaw::common::EPBLevel *epb_level_cmd_ptr_ = nullptr;
  //车辆速度
  double speed_mps_;
  double *speed_mps_ptr_ = nullptr;
  //参考速度
  double speed_reference_;
  double *speed_reference_ptr_ = nullptr;
  //油门反馈
  double accel_value_fd_;
  double *accel_value_fd_ptr_ = nullptr;
  //油门命令
  double accel_value_cmd_;
  double *accel_value_cmd_ptr_ = nullptr;
  //刹车反馈
  double brake_value_fd_;
  double *brake_value_fd_ptr_ = nullptr;
  //刹车命令
  double brake_value_cmd_;
  double *brake_value_cmd_ptr_ = nullptr;
  //剩余轨迹里程
  double path_remain_;
  double *path_remain_ptr_ = nullptr;
  //是否有停车点
  bool has_stop_point_;
  bool *has_stop_point_ptr_ = nullptr;
  //是否有停车轨迹
  bool is_full_stop_;
  bool *is_full_stop_ptr_ = nullptr;
  //是否停车
  bool is_stopped_;
  bool *is_stopped_ptr_ = nullptr;
  //纵向加加速度
  double lon_acc_jerk_;
  double *lon_acc_jerk_ptr_ = nullptr;
  //加速度命令
  double acceleration_cmd_;
  double *acceleration_cmd_ptr_ = nullptr;
  //闭环加速度命令
  double acceleration_cmd_closeloop_;
  double *acceleration_cmd_closeloop_ptr_ = nullptr;
  //预描参考加速度
  double preview_acceleration_reference_;
  double *preview_acceleration_reference_ptr_ = nullptr;
  //坡道补偿
  double slope_offset_compensation_;
  double *slope_offset_compensation_ptr_ = nullptr;
  //转向补偿
  double turning_offset_compensation_;
  double *turning_offset_compensation_ptr_ = nullptr;
  //速度差限值
  double speed_error_limited_;
  double *speed_error_limited_ptr_ = nullptr;
  //速度差
  double speed_error_;
  double *speed_error_ptr_ = nullptr;
  double speed_offset_;
  double *speed_offset_ptr_ = nullptr;
  //位置误差限值
  double station_error_limited_;
  double *station_error_limited_ptr_ = nullptr;
  //位置误差
  double station_error_;
  double *station_error_ptr_ = nullptr;
  //纵向控制目标点里程
  double lon_target_point_s_;
  double *lon_target_point_s_ptr_ = nullptr;
  //纵向计算时间
  int64_t lon_calculate_time_;
  int64_t *lon_calculate_time_ptr_ = nullptr;
  //纵向时间限值
  int64_t lon_calculate_time_max_;
  int64_t *lon_calculate_time_max_ptr_ = nullptr;
  //参考曲率
  double ref_curvature_;
  double *ref_curvature_ptr_ = nullptr;
  //参考航向角
  double ref_heading_;
  double *ref_heading_ptr_ = nullptr;
  //当前航向角
  double current_heading_;
  double *current_heading_ptr_ = nullptr;
  //航向角误差
  double heading_error_;
  double *heading_error_ptr_ = nullptr;
  //航向角误差率
  double heading_error_rate_;
  double *heading_error_rate_ptr_ = nullptr;
  //横向误差
  double lateral_error_;
  double *lateral_error_ptr_ = nullptr;
  //横向误差率
  double lateral_error_rate_;
  double *lateral_error_rate_ptr_ = nullptr;
  //纵向误差
  double lon_error_;
  double *lon_error_ptr_ = nullptr;
  //前轮转向反馈
  double front_steering_value_fd_;
  double *front_steering_value_fd_ptr_ = nullptr;
  //前轮目标转向
  double front_steering_target_;
  double *front_steering_target_ptr_ = nullptr;
  //前轮转向角速率
  double front_steering_rate_;
  double *front_steering_rate_ptr_ = nullptr;
  //前轮前馈计算
  double front_steer_angle_feedforward_;
  double *front_steer_angle_feedforward_ptr_ = nullptr;
  //前轮反馈计算
  double front_steer_angle_feedback_;
  double *front_steer_angle_feedback_ptr_ = nullptr;
  //前轮横向误差贡献
  double front_steer_angle_lateral_contribution_;
  double *front_steer_angle_lateral_contribution_ptr_ = nullptr;
  //前轮横向误差率贡献
  double front_steer_angle_lateral_rate_contribution_;
  double *front_steer_angle_lateral_rate_contribution_ptr_ = nullptr;
  //前轮航向角误差贡献
  double front_steer_angle_heading_contribution_;
  double *front_steer_angle_heading_contribution_ptr_ = nullptr;
  //前轮航向角误差率贡献
  double front_steer_angle_heading_rate_contribution_;
  double *front_steer_angle_heading_rate_contribution_ptr_ = nullptr;
  //后轮转向反馈
  double rear_steering_value_fd_;
  double *rear_steering_value_fd_ptr_ = nullptr;
  //后轮目标转向
  double rear_steering_target_;
  double *rear_steering_target_ptr_ = nullptr;
  //后轮转向角速率
  double rear_steering_rate_;
  double *rear_steering_rate_ptr_ = nullptr;
  //后轮前馈计算
  double rear_steer_angle_feedforward_;
  double *rear_steer_angle_feedforward_ptr_ = nullptr;
  //后轮反馈计算
  double rear_steer_angle_feedback_;
  double *rear_steer_angle_feedback_ptr_ = nullptr;
  //后轮横向误差贡献
  double rear_steer_angle_lateral_contribution_;
  double *rear_steer_angle_lateral_contribution_ptr_ = nullptr;
  //后轮横向误差率贡献
  double rear_steer_angle_lateral_rate_contribution_;
  double *rear_steer_angle_lateral_rate_contribution_ptr_ = nullptr;
  //后轮航向角误差贡献
  double rear_steer_angle_heading_contribution_;
  double *rear_steer_angle_heading_contribution_ptr_ = nullptr;
  //后轮航向角误差率贡献
  double rear_steer_angle_heading_rate_contribution_;
  double *rear_steer_angle_heading_rate_contribution_ptr_ = nullptr;
  // k矩阵
  double matrix_k_00_;
  double *matrix_k_00_ptr_ = nullptr;
  // k矩阵
  double matrix_k_01_;
  double *matrix_k_01_ptr_ = nullptr;
  // k矩阵
  double matrix_k_02_;
  double *matrix_k_02_ptr_ = nullptr;
  // k矩阵
  double matrix_k_03_;
  double *matrix_k_03_ptr_ = nullptr;
  // k矩阵
  double matrix_k_10_;
  double *matrix_k_10_ptr_ = nullptr;
  // k矩阵
  double matrix_k_11_;
  double *matrix_k_11_ptr_ = nullptr;
  // k矩阵
  double matrix_k_12_;
  double *matrix_k_12_ptr_ = nullptr;
  // k矩阵
  double matrix_k_13_;
  double *matrix_k_13_ptr_ = nullptr;
  //状态矩阵
  double matrix_state_0_;
  double *matrix_state_0_ptr_ = nullptr;
  //状态矩阵
  double matrix_state_1_;
  double *matrix_state_1_ptr_ = nullptr;
  //状态矩阵
  double matrix_state_2_;
  double *matrix_state_2_ptr_ = nullptr;
  //状态矩阵
  double matrix_state_3_;
  double *matrix_state_3_ptr_ = nullptr;
  //权重矩阵
  double matrix_q_updated_0_;
  double *matrix_q_updated_0_ptr_ = nullptr;
  //权重矩阵
  double matrix_q_updated_1_;
  double *matrix_q_updated_1_ptr_ = nullptr;
  //权重矩阵
  double matrix_q_updated_2_;
  double *matrix_q_updated_2_ptr_ = nullptr;
  //权重矩阵
  double matrix_q_updated_3_;
  double *matrix_q_updated_3_ptr_ = nullptr;
  //当前位置x
  double current_x_;
  double *current_x_ptr_ = nullptr;
  //当前位置y
  double current_y_;
  double *current_y_ptr_ = nullptr;
  //目标点x
  double target_point_x_;
  double *target_point_x_ptr_ = nullptr;
  //目标点y
  double target_point_y_;
  double *target_point_y_ptr_ = nullptr;
  //横向目标点里程
  double lat_target_point_s_;
  double *lat_target_point_s_ptr_ = nullptr;
  // LQR计算时间
  int64_t lqr_calculate_time_;
  int64_t *lqr_calculate_time_ptr_ = nullptr;
  // LQR计算时间限值
  int64_t lqr_calculate_time_max_;
  int64_t *lqr_calculate_time_max_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
