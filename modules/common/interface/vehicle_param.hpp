/**
 * @file    vehicle_param.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>
#include <string>

#include "modules/common/enum/enum.h"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleParam {
public:
  VehicleParam() {
    brand_.clear();
    steer_mode_ = legionclaw::common::SteerMode::STEER_MODE_2WS;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    mass_fl_ = 0.0;
    mass_fr_ = 0.0;
    mass_rl_ = 0.0;
    mass_rr_ = 0.0;
    wheel_radius_ = 0.0;
    wheelbase_ = 0.0;
    front_edge_to_center_ = 0.0;
    back_edge_to_center_ = 0.0;
    lf_ = 0.0;
    lr_ = 0.0;
    cf_ = 0.0;
    cr_ = 0.0;
    steer_ratio_ = 0.0;
    rolling_coefficient_ = 0.0;
    air_density_ = 0.0;
    air_damping_coefficient_ = 0.0;
    max_front_steer_angle_ = 0.0;
    min_front_steer_angle_ = 0.0;
    max_rear_steer_angle_ = 0.0;
    min_rear_steer_angle_ = 0.0;
    speed_limit_ = 0.0;
    max_brake_value_ = 0.0;
    min_brake_value_ = 0.0;
    max_accel_value_ = 0.0;
    min_accel_value_ = 0.0;
    speed_deadzone_ = 0.0;
    standstill_acceleration_ = 0.0;
    max_front_steer_angle_rate_ = 0.0;
    max_rear_steer_angle_rate_ = 0.0;
    max_abs_speed_when_stopped_ = 0.0;
    max_abs_speed_when_stopped_duration_ = 0.0;
    brake_value_when_gear_transitioning_ = 0.0;
    accel_deadzone_ = 0.0;
    brake_deadzone_ = 0.0;
    acceleration_in_idle_ = 0.0;
    deceleration_in_idle_ = 0.0;
    max_acceleration_jerk_ = 0.0;
    max_acceleration_ = 0.0;
    max_deceleration_ = 0.0;
    min_turning_radius_ = 0.0;
  }
  ~VehicleParam() = default;

  inline void set_brand(const std::string &brand) {
    brand_ = brand;
    brand_ptr_ = &brand_;
  }

  inline const std::string &brand() const { return brand_; }

  inline std::string *mutable_brand() { return &brand_; }

  inline bool has_brand() { return (brand_ptr_ != nullptr); }

  inline void set_steer_mode(const legionclaw::common::SteerMode &steer_mode) {
    steer_mode_ = steer_mode;
    steer_mode_ptr_ = &steer_mode_;
  }

  inline const legionclaw::common::SteerMode &steer_mode() const {
    return steer_mode_;
  }

  inline legionclaw::common::SteerMode *mutable_steer_mode() { return &steer_mode_; }

  inline bool has_steer_mode() { return (steer_mode_ptr_ != nullptr); }

  inline void set_length(const double &length) {
    length_ = length;
    length_ptr_ = &length_;
  }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline bool has_length() { return (length_ptr_ != nullptr); }

  inline void set_width(const double &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const double &width() const { return width_; }

  inline double *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_height(const double &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const double &height() const { return height_; }

  inline double *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  inline void set_mass_fl(const double &mass_fl) {
    mass_fl_ = mass_fl;
    mass_fl_ptr_ = &mass_fl_;
  }

  inline const double &mass_fl() const { return mass_fl_; }

  inline double *mutable_mass_fl() { return &mass_fl_; }

  inline bool has_mass_fl() { return (mass_fl_ptr_ != nullptr); }

  inline void set_mass_fr(const double &mass_fr) {
    mass_fr_ = mass_fr;
    mass_fr_ptr_ = &mass_fr_;
  }

  inline const double &mass_fr() const { return mass_fr_; }

  inline double *mutable_mass_fr() { return &mass_fr_; }

  inline bool has_mass_fr() { return (mass_fr_ptr_ != nullptr); }

  inline void set_mass_rl(const double &mass_rl) {
    mass_rl_ = mass_rl;
    mass_rl_ptr_ = &mass_rl_;
  }

  inline const double &mass_rl() const { return mass_rl_; }

  inline double *mutable_mass_rl() { return &mass_rl_; }

  inline bool has_mass_rl() { return (mass_rl_ptr_ != nullptr); }

  inline void set_mass_rr(const double &mass_rr) {
    mass_rr_ = mass_rr;
    mass_rr_ptr_ = &mass_rr_;
  }

  inline const double &mass_rr() const { return mass_rr_; }

  inline double *mutable_mass_rr() { return &mass_rr_; }

  inline bool has_mass_rr() { return (mass_rr_ptr_ != nullptr); }

  inline void set_wheel_radius(const double &wheel_radius) {
    wheel_radius_ = wheel_radius;
    wheel_radius_ptr_ = &wheel_radius_;
  }

  inline const double &wheel_radius() const { return wheel_radius_; }

  inline double *mutable_wheel_radius() { return &wheel_radius_; }

  inline bool has_wheel_radius() { return (wheel_radius_ptr_ != nullptr); }

  inline void set_wheelbase(const double &wheelbase) {
    wheelbase_ = wheelbase;
    wheelbase_ptr_ = &wheelbase_;
  }

  inline const double &wheelbase() const { return wheelbase_; }

  inline double *mutable_wheelbase() { return &wheelbase_; }

  inline bool has_wheelbase() { return (wheelbase_ptr_ != nullptr); }

  inline void set_front_edge_to_center(const double &front_edge_to_center) {
    front_edge_to_center_ = front_edge_to_center;
    front_edge_to_center_ptr_ = &front_edge_to_center_;
  }

  inline const double &front_edge_to_center() const {
    return front_edge_to_center_;
  }

  inline double *mutable_front_edge_to_center() {
    return &front_edge_to_center_;
  }

  inline bool has_front_edge_to_center() {
    return (front_edge_to_center_ptr_ != nullptr);
  }

  inline void set_back_edge_to_center(const double &back_edge_to_center) {
    back_edge_to_center_ = back_edge_to_center;
    back_edge_to_center_ptr_ = &back_edge_to_center_;
  }

  inline const double &back_edge_to_center() const {
    return back_edge_to_center_;
  }

  inline double *mutable_back_edge_to_center() { return &back_edge_to_center_; }

  inline bool has_back_edge_to_center() {
    return (back_edge_to_center_ptr_ != nullptr);
  }

  inline void set_lf(const double &lf) {
    lf_ = lf;
    lf_ptr_ = &lf_;
  }

  inline const double &lf() const { return lf_; }

  inline double *mutable_lf() { return &lf_; }

  inline bool has_lf() { return (lf_ptr_ != nullptr); }

  inline void set_lr(const double &lr) {
    lr_ = lr;
    lr_ptr_ = &lr_;
  }

  inline const double &lr() const { return lr_; }

  inline double *mutable_lr() { return &lr_; }

  inline bool has_lr() { return (lr_ptr_ != nullptr); }

  inline void set_cf(const double &cf) {
    cf_ = cf;
    cf_ptr_ = &cf_;
  }

  inline const double &cf() const { return cf_; }

  inline double *mutable_cf() { return &cf_; }

  inline bool has_cf() { return (cf_ptr_ != nullptr); }

  inline void set_cr(const double &cr) {
    cr_ = cr;
    cr_ptr_ = &cr_;
  }

  inline const double &cr() const { return cr_; }

  inline double *mutable_cr() { return &cr_; }

  inline bool has_cr() { return (cr_ptr_ != nullptr); }

  inline void set_steer_ratio(const double &steer_ratio) {
    steer_ratio_ = steer_ratio;
    steer_ratio_ptr_ = &steer_ratio_;
  }

  inline const double &steer_ratio() const { return steer_ratio_; }

  inline double *mutable_steer_ratio() { return &steer_ratio_; }

  inline bool has_steer_ratio() { return (steer_ratio_ptr_ != nullptr); }

  inline void set_rolling_coefficient(const double &rolling_coefficient) {
    rolling_coefficient_ = rolling_coefficient;
    rolling_coefficient_ptr_ = &rolling_coefficient_;
  }

  inline const double &rolling_coefficient() const {
    return rolling_coefficient_;
  }

  inline double *mutable_rolling_coefficient() { return &rolling_coefficient_; }

  inline bool has_rolling_coefficient() {
    return (rolling_coefficient_ptr_ != nullptr);
  }

  inline void set_air_density(const double &air_density) {
    air_density_ = air_density;
    air_density_ptr_ = &air_density_;
  }

  inline const double &air_density() const { return air_density_; }

  inline double *mutable_air_density() { return &air_density_; }

  inline bool has_air_density() { return (air_density_ptr_ != nullptr); }

  inline void
  set_air_damping_coefficient(const double &air_damping_coefficient) {
    air_damping_coefficient_ = air_damping_coefficient;
    air_damping_coefficient_ptr_ = &air_damping_coefficient_;
  }

  inline const double &air_damping_coefficient() const {
    return air_damping_coefficient_;
  }

  inline double *mutable_air_damping_coefficient() {
    return &air_damping_coefficient_;
  }

  inline bool has_air_damping_coefficient() {
    return (air_damping_coefficient_ptr_ != nullptr);
  }

  inline void set_max_front_steer_angle(const double &max_front_steer_angle) {
    max_front_steer_angle_ = max_front_steer_angle;
    max_front_steer_angle_ptr_ = &max_front_steer_angle_;
  }

  inline const double &max_front_steer_angle() const {
    return max_front_steer_angle_;
  }

  inline double *mutable_max_front_steer_angle() {
    return &max_front_steer_angle_;
  }

  inline bool has_max_front_steer_angle() {
    return (max_front_steer_angle_ptr_ != nullptr);
  }

  inline void set_min_front_steer_angle(const double &min_front_steer_angle) {
    min_front_steer_angle_ = min_front_steer_angle;
    min_front_steer_angle_ptr_ = &min_front_steer_angle_;
  }

  inline const double &min_front_steer_angle() const {
    return min_front_steer_angle_;
  }

  inline double *mutable_min_front_steer_angle() {
    return &min_front_steer_angle_;
  }

  inline bool has_min_front_steer_angle() {
    return (min_front_steer_angle_ptr_ != nullptr);
  }

  inline void set_max_rear_steer_angle(const double &max_rear_steer_angle) {
    max_rear_steer_angle_ = max_rear_steer_angle;
    max_rear_steer_angle_ptr_ = &max_rear_steer_angle_;
  }

  inline const double &max_rear_steer_angle() const {
    return max_rear_steer_angle_;
  }

  inline double *mutable_max_rear_steer_angle() {
    return &max_rear_steer_angle_;
  }

  inline bool has_max_rear_steer_angle() {
    return (max_rear_steer_angle_ptr_ != nullptr);
  }

  inline void set_min_rear_steer_angle(const double &min_rear_steer_angle) {
    min_rear_steer_angle_ = min_rear_steer_angle;
    min_rear_steer_angle_ptr_ = &min_rear_steer_angle_;
  }

  inline const double &min_rear_steer_angle() const {
    return min_rear_steer_angle_;
  }

  inline double *mutable_min_rear_steer_angle() {
    return &min_rear_steer_angle_;
  }

  inline bool has_min_rear_steer_angle() {
    return (min_rear_steer_angle_ptr_ != nullptr);
  }

  inline void set_speed_limit(const double &speed_limit) {
    speed_limit_ = speed_limit;
    speed_limit_ptr_ = &speed_limit_;
  }

  inline const double &speed_limit() const { return speed_limit_; }

  inline double *mutable_speed_limit() { return &speed_limit_; }

  inline bool has_speed_limit() { return (speed_limit_ptr_ != nullptr); }

  inline void set_max_brake_value(const double &max_brake_value) {
    max_brake_value_ = max_brake_value;
    max_brake_value_ptr_ = &max_brake_value_;
  }

  inline const double &max_brake_value() const { return max_brake_value_; }

  inline double *mutable_max_brake_value() { return &max_brake_value_; }

  inline bool has_max_brake_value() {
    return (max_brake_value_ptr_ != nullptr);
  }

  inline void set_min_brake_value(const double &min_brake_value) {
    min_brake_value_ = min_brake_value;
    min_brake_value_ptr_ = &min_brake_value_;
  }

  inline const double &min_brake_value() const { return min_brake_value_; }

  inline double *mutable_min_brake_value() { return &min_brake_value_; }

  inline bool has_min_brake_value() {
    return (min_brake_value_ptr_ != nullptr);
  }

  inline void set_max_accel_value(const double &max_accel_value) {
    max_accel_value_ = max_accel_value;
    max_accel_value_ptr_ = &max_accel_value_;
  }

  inline const double &max_accel_value() const { return max_accel_value_; }

  inline double *mutable_max_accel_value() { return &max_accel_value_; }

  inline bool has_max_accel_value() {
    return (max_accel_value_ptr_ != nullptr);
  }

  inline void set_min_accel_value(const double &min_accel_value) {
    min_accel_value_ = min_accel_value;
    min_accel_value_ptr_ = &min_accel_value_;
  }

  inline const double &min_accel_value() const { return min_accel_value_; }

  inline double *mutable_min_accel_value() { return &min_accel_value_; }

  inline bool has_min_accel_value() {
    return (min_accel_value_ptr_ != nullptr);
  }

  inline void set_speed_deadzone(const double &speed_deadzone) {
    speed_deadzone_ = speed_deadzone;
    speed_deadzone_ptr_ = &speed_deadzone_;
  }

  inline const double &speed_deadzone() const { return speed_deadzone_; }

  inline double *mutable_speed_deadzone() { return &speed_deadzone_; }

  inline bool has_speed_deadzone() { return (speed_deadzone_ptr_ != nullptr); }

  inline void
  set_standstill_acceleration(const double &standstill_acceleration) {
    standstill_acceleration_ = standstill_acceleration;
    standstill_acceleration_ptr_ = &standstill_acceleration_;
  }

  inline const double &standstill_acceleration() const {
    return standstill_acceleration_;
  }

  inline double *mutable_standstill_acceleration() {
    return &standstill_acceleration_;
  }

  inline bool has_standstill_acceleration() {
    return (standstill_acceleration_ptr_ != nullptr);
  }

  inline void
  set_max_front_steer_angle_rate(const double &max_front_steer_angle_rate) {
    max_front_steer_angle_rate_ = max_front_steer_angle_rate;
    max_front_steer_angle_rate_ptr_ = &max_front_steer_angle_rate_;
  }

  inline const double &max_front_steer_angle_rate() const {
    return max_front_steer_angle_rate_;
  }

  inline double *mutable_max_front_steer_angle_rate() {
    return &max_front_steer_angle_rate_;
  }

  inline bool has_max_front_steer_angle_rate() {
    return (max_front_steer_angle_rate_ptr_ != nullptr);
  }

  inline void
  set_max_rear_steer_angle_rate(const double &max_rear_steer_angle_rate) {
    max_rear_steer_angle_rate_ = max_rear_steer_angle_rate;
    max_rear_steer_angle_rate_ptr_ = &max_rear_steer_angle_rate_;
  }

  inline const double &max_rear_steer_angle_rate() const {
    return max_rear_steer_angle_rate_;
  }

  inline double *mutable_max_rear_steer_angle_rate() {
    return &max_rear_steer_angle_rate_;
  }

  inline bool has_max_rear_steer_angle_rate() {
    return (max_rear_steer_angle_rate_ptr_ != nullptr);
  }

  inline void
  set_max_abs_speed_when_stopped(const double &max_abs_speed_when_stopped) {
    max_abs_speed_when_stopped_ = max_abs_speed_when_stopped;
    max_abs_speed_when_stopped_ptr_ = &max_abs_speed_when_stopped_;
  }

  inline const double &max_abs_speed_when_stopped() const {
    return max_abs_speed_when_stopped_;
  }

  inline double *mutable_max_abs_speed_when_stopped() {
    return &max_abs_speed_when_stopped_;
  }

  inline bool has_max_abs_speed_when_stopped() {
    return (max_abs_speed_when_stopped_ptr_ != nullptr);
  }

  inline void set_max_abs_speed_when_stopped_duration(
      const double &max_abs_speed_when_stopped_duration) {
    max_abs_speed_when_stopped_duration_ = max_abs_speed_when_stopped_duration;
    max_abs_speed_when_stopped_duration_ptr_ =
        &max_abs_speed_when_stopped_duration_;
  }

  inline const double &max_abs_speed_when_stopped_duration() const {
    return max_abs_speed_when_stopped_duration_;
  }

  inline double *mutable_max_abs_speed_when_stopped_duration() {
    return &max_abs_speed_when_stopped_duration_;
  }

  inline bool has_max_abs_speed_when_stopped_duration() {
    return (max_abs_speed_when_stopped_duration_ptr_ != nullptr);
  }

  inline void set_brake_value_when_gear_transitioning(
      const double &brake_value_when_gear_transitioning) {
    brake_value_when_gear_transitioning_ = brake_value_when_gear_transitioning;
    brake_value_when_gear_transitioning_ptr_ =
        &brake_value_when_gear_transitioning_;
  }

  inline const double &brake_value_when_gear_transitioning() const {
    return brake_value_when_gear_transitioning_;
  }

  inline double *mutable_brake_value_when_gear_transitioning() {
    return &brake_value_when_gear_transitioning_;
  }

  inline bool has_brake_value_when_gear_transitioning() {
    return (brake_value_when_gear_transitioning_ptr_ != nullptr);
  }

  inline void set_accel_deadzone(const double &accel_deadzone) {
    accel_deadzone_ = accel_deadzone;
    accel_deadzone_ptr_ = &accel_deadzone_;
  }

  inline const double &accel_deadzone() const { return accel_deadzone_; }

  inline double *mutable_accel_deadzone() { return &accel_deadzone_; }

  inline bool has_accel_deadzone() { return (accel_deadzone_ptr_ != nullptr); }

  inline void set_brake_deadzone(const double &brake_deadzone) {
    brake_deadzone_ = brake_deadzone;
    brake_deadzone_ptr_ = &brake_deadzone_;
  }

  inline const double &brake_deadzone() const { return brake_deadzone_; }

  inline double *mutable_brake_deadzone() { return &brake_deadzone_; }

  inline bool has_brake_deadzone() { return (brake_deadzone_ptr_ != nullptr); }

  inline void set_acceleration_in_idle(const double &acceleration_in_idle) {
    acceleration_in_idle_ = acceleration_in_idle;
    acceleration_in_idle_ptr_ = &acceleration_in_idle_;
  }

  inline const double &acceleration_in_idle() const {
    return acceleration_in_idle_;
  }

  inline double *mutable_acceleration_in_idle() {
    return &acceleration_in_idle_;
  }

  inline bool has_acceleration_in_idle() {
    return (acceleration_in_idle_ptr_ != nullptr);
  }

  inline void set_deceleration_in_idle(const double &deceleration_in_idle) {
    deceleration_in_idle_ = deceleration_in_idle;
    deceleration_in_idle_ptr_ = &deceleration_in_idle_;
  }

  inline const double &deceleration_in_idle() const {
    return deceleration_in_idle_;
  }

  inline double *mutable_deceleration_in_idle() {
    return &deceleration_in_idle_;
  }

  inline bool has_deceleration_in_idle() {
    return (deceleration_in_idle_ptr_ != nullptr);
  }

  inline void set_max_acceleration_jerk(const double &max_acceleration_jerk) {
    max_acceleration_jerk_ = max_acceleration_jerk;
    max_acceleration_jerk_ptr_ = &max_acceleration_jerk_;
  }

  inline const double &max_acceleration_jerk() const {
    return max_acceleration_jerk_;
  }

  inline double *mutable_max_acceleration_jerk() {
    return &max_acceleration_jerk_;
  }

  inline bool has_max_acceleration_jerk() {
    return (max_acceleration_jerk_ptr_ != nullptr);
  }

  inline void set_max_acceleration(const double &max_acceleration) {
    max_acceleration_ = max_acceleration;
    max_acceleration_ptr_ = &max_acceleration_;
  }

  inline const double &max_acceleration() const { return max_acceleration_; }

  inline double *mutable_max_acceleration() { return &max_acceleration_; }

  inline bool has_max_acceleration() {
    return (max_acceleration_ptr_ != nullptr);
  }

  inline void set_max_deceleration(const double &max_deceleration) {
    max_deceleration_ = max_deceleration;
    max_deceleration_ptr_ = &max_deceleration_;
  }

  inline const double &max_deceleration() const { return max_deceleration_; }

  inline double *mutable_max_deceleration() { return &max_deceleration_; }

  inline bool has_max_deceleration() {
    return (max_deceleration_ptr_ != nullptr);
  }

  inline void set_min_turning_radius(const double &min_turning_radius) {
    min_turning_radius_ = min_turning_radius;
    min_turning_radius_ptr_ = &min_turning_radius_;
  }

  inline const double &min_turning_radius() const {
    return min_turning_radius_;
  }

  inline double *mutable_min_turning_radius() { return &min_turning_radius_; }

  inline bool has_min_turning_radius() {
    return (min_turning_radius_ptr_ != nullptr);
  }

  void operator=(const VehicleParam &vehicle_param) { CopyFrom(vehicle_param); }

  void CopyFrom(const VehicleParam &vehicle_param) {
    brand_ = vehicle_param.brand();
    steer_mode_ = vehicle_param.steer_mode();
    length_ = vehicle_param.length();
    width_ = vehicle_param.width();
    height_ = vehicle_param.height();
    mass_fl_ = vehicle_param.mass_fl();
    mass_fr_ = vehicle_param.mass_fr();
    mass_rl_ = vehicle_param.mass_rl();
    mass_rr_ = vehicle_param.mass_rr();
    wheel_radius_ = vehicle_param.wheel_radius();
    wheelbase_ = vehicle_param.wheelbase();
    front_edge_to_center_ = vehicle_param.front_edge_to_center();
    back_edge_to_center_ = vehicle_param.back_edge_to_center();
    lf_ = vehicle_param.lf();
    lr_ = vehicle_param.lr();
    cf_ = vehicle_param.cf();
    cr_ = vehicle_param.cr();
    steer_ratio_ = vehicle_param.steer_ratio();
    rolling_coefficient_ = vehicle_param.rolling_coefficient();
    air_density_ = vehicle_param.air_density();
    air_damping_coefficient_ = vehicle_param.air_damping_coefficient();
    max_front_steer_angle_ = vehicle_param.max_front_steer_angle();
    min_front_steer_angle_ = vehicle_param.min_front_steer_angle();
    max_rear_steer_angle_ = vehicle_param.max_rear_steer_angle();
    min_rear_steer_angle_ = vehicle_param.min_rear_steer_angle();
    speed_limit_ = vehicle_param.speed_limit();
    max_brake_value_ = vehicle_param.max_brake_value();
    min_brake_value_ = vehicle_param.min_brake_value();
    max_accel_value_ = vehicle_param.max_accel_value();
    min_accel_value_ = vehicle_param.min_accel_value();
    speed_deadzone_ = vehicle_param.speed_deadzone();
    standstill_acceleration_ = vehicle_param.standstill_acceleration();
    max_front_steer_angle_rate_ = vehicle_param.max_front_steer_angle_rate();
    max_rear_steer_angle_rate_ = vehicle_param.max_rear_steer_angle_rate();
    max_abs_speed_when_stopped_ = vehicle_param.max_abs_speed_when_stopped();
    max_abs_speed_when_stopped_duration_ =
        vehicle_param.max_abs_speed_when_stopped_duration();
    brake_value_when_gear_transitioning_ =
        vehicle_param.brake_value_when_gear_transitioning();
    accel_deadzone_ = vehicle_param.accel_deadzone();
    brake_deadzone_ = vehicle_param.brake_deadzone();
    acceleration_in_idle_ = vehicle_param.acceleration_in_idle();
    deceleration_in_idle_ = vehicle_param.deceleration_in_idle();
    max_acceleration_jerk_ = vehicle_param.max_acceleration_jerk();
    max_acceleration_ = vehicle_param.max_acceleration();
    max_deceleration_ = vehicle_param.max_deceleration();
    min_turning_radius_ = vehicle_param.min_turning_radius();
  }

protected:
  std::string brand_;
  std::string *brand_ptr_ = nullptr;
  legionclaw::common::SteerMode steer_mode_;
  legionclaw::common::SteerMode *steer_mode_ptr_ = nullptr;
  double length_;
  double *length_ptr_ = nullptr;
  double width_;
  double *width_ptr_ = nullptr;
  double height_;
  double *height_ptr_ = nullptr;
  double mass_fl_;
  double *mass_fl_ptr_ = nullptr;
  double mass_fr_;
  double *mass_fr_ptr_ = nullptr;
  double mass_rl_;
  double *mass_rl_ptr_ = nullptr;
  double mass_rr_;
  double *mass_rr_ptr_ = nullptr;
  double wheel_radius_;
  double *wheel_radius_ptr_ = nullptr;
  double wheelbase_;
  double *wheelbase_ptr_ = nullptr;
  double front_edge_to_center_;
  double *front_edge_to_center_ptr_ = nullptr;
  double back_edge_to_center_;
  double *back_edge_to_center_ptr_ = nullptr;
  double lf_;
  double *lf_ptr_ = nullptr;
  double lr_;
  double *lr_ptr_ = nullptr;
  double cf_;
  double *cf_ptr_ = nullptr;
  double cr_;
  double *cr_ptr_ = nullptr;
  double steer_ratio_;
  double *steer_ratio_ptr_ = nullptr;
  double rolling_coefficient_;
  double *rolling_coefficient_ptr_ = nullptr;
  double air_density_;
  double *air_density_ptr_ = nullptr;
  double air_damping_coefficient_;
  double *air_damping_coefficient_ptr_ = nullptr;
  double max_front_steer_angle_;
  double *max_front_steer_angle_ptr_ = nullptr;
  double min_front_steer_angle_;
  double *min_front_steer_angle_ptr_ = nullptr;
  double max_rear_steer_angle_;
  double *max_rear_steer_angle_ptr_ = nullptr;
  double min_rear_steer_angle_;
  double *min_rear_steer_angle_ptr_ = nullptr;
  double speed_limit_;
  double *speed_limit_ptr_ = nullptr;
  double max_brake_value_;
  double *max_brake_value_ptr_ = nullptr;
  double min_brake_value_;
  double *min_brake_value_ptr_ = nullptr;
  double max_accel_value_;
  double *max_accel_value_ptr_ = nullptr;
  double min_accel_value_;
  double *min_accel_value_ptr_ = nullptr;
  double speed_deadzone_;
  double *speed_deadzone_ptr_ = nullptr;
  double standstill_acceleration_;
  double *standstill_acceleration_ptr_ = nullptr;
  double max_front_steer_angle_rate_;
  double *max_front_steer_angle_rate_ptr_ = nullptr;
  double max_rear_steer_angle_rate_;
  double *max_rear_steer_angle_rate_ptr_ = nullptr;
  double max_abs_speed_when_stopped_;
  double *max_abs_speed_when_stopped_ptr_ = nullptr;
  double max_abs_speed_when_stopped_duration_;
  double *max_abs_speed_when_stopped_duration_ptr_ = nullptr;
  double brake_value_when_gear_transitioning_;
  double *brake_value_when_gear_transitioning_ptr_ = nullptr;
  double accel_deadzone_;
  double *accel_deadzone_ptr_ = nullptr;
  double brake_deadzone_;
  double *brake_deadzone_ptr_ = nullptr;
  double acceleration_in_idle_;
  double *acceleration_in_idle_ptr_ = nullptr;
  double deceleration_in_idle_;
  double *deceleration_in_idle_ptr_ = nullptr;
  double max_acceleration_jerk_;
  double *max_acceleration_jerk_ptr_ = nullptr;
  double max_acceleration_;
  double *max_acceleration_ptr_ = nullptr;
  double max_deceleration_;
  double *max_deceleration_ptr_ = nullptr;
  double min_turning_radius_;
  double *min_turning_radius_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
