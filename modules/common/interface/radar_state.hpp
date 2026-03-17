/**
 * @file    radar_state.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/radar_state_error.hpp"
#include "modules/common/interface/radar_state_mode.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RadarState {
public:
  RadarState() {
    sensor_id_ = 0;
    nvm_read_status_ = 0;
    nvm_write_status_ = 0;
    max_distance_ = 0;
    sort_index_ = 0;
    radar_power_ = 0;
    ctl_relay_ = 0;
    output_type_ = 0;
    send_quality_ = 0;
    send_extinfo_ = 0;
    motion_rx_state_ = 0;
    rcs_threshold_ = 0;
    connector_direction_ = 0;
    radar_position_ = 0;
    hw_error_ = 0;
  }
  ~RadarState() = default;

  inline void set_sensor_id(const uint8_t &sensor_id) {
    sensor_id_ = sensor_id;
    sensor_id_ptr_ = &sensor_id_;
  }

  inline const uint8_t &sensor_id() const { return sensor_id_; }

  inline uint8_t *mutable_sensor_id() { return &sensor_id_; }

  inline bool has_sensor_id() { return (sensor_id_ptr_ != nullptr); }

  inline void set_nvm_read_status(const uint8_t &nvm_read_status) {
    nvm_read_status_ = nvm_read_status;
    nvm_read_status_ptr_ = &nvm_read_status_;
  }

  inline const uint8_t &nvm_read_status() const { return nvm_read_status_; }

  inline uint8_t *mutable_nvm_read_status() { return &nvm_read_status_; }

  inline bool has_nvm_read_status() {
    return (nvm_read_status_ptr_ != nullptr);
  }

  inline void set_nvm_write_status(const uint8_t &nvm_write_status) {
    nvm_write_status_ = nvm_write_status;
    nvm_write_status_ptr_ = &nvm_write_status_;
  }

  inline const uint8_t &nvm_write_status() const { return nvm_write_status_; }

  inline uint8_t *mutable_nvm_write_status() { return &nvm_write_status_; }

  inline bool has_nvm_write_status() {
    return (nvm_write_status_ptr_ != nullptr);
  }

  inline void set_radar_state_error(
      const legionclaw::interface::RadarStateError &radar_state_error) {
    radar_state_error_ = radar_state_error;
    radar_state_error_ptr_ = &radar_state_error_;
  }

  inline const legionclaw::interface::RadarStateError &radar_state_error() const {
    return radar_state_error_;
  }

  inline legionclaw::interface::RadarStateError *mutable_radar_state_error() {
    return &radar_state_error_;
  }

  inline bool has_radar_state_error() {
    return (radar_state_error_ptr_ != nullptr);
  }

  inline void set_radar_state_mode(
      const legionclaw::interface::RadarStateMode &radar_state_mode) {
    radar_state_mode_ = radar_state_mode;
    radar_state_mode_ptr_ = &radar_state_mode_;
  }

  inline const legionclaw::interface::RadarStateMode &radar_state_mode() const {
    return radar_state_mode_;
  }

  inline legionclaw::interface::RadarStateMode *mutable_radar_state_mode() {
    return &radar_state_mode_;
  }

  inline bool has_radar_state_mode() {
    return (radar_state_mode_ptr_ != nullptr);
  }

  inline void set_max_distance(const uint16_t &max_distance) {
    max_distance_ = max_distance;
    max_distance_ptr_ = &max_distance_;
  }

  inline const uint16_t &max_distance() const { return max_distance_; }

  inline uint16_t *mutable_max_distance() { return &max_distance_; }

  inline bool has_max_distance() { return (max_distance_ptr_ != nullptr); }

  inline void set_sort_index(const uint8_t &sort_index) {
    sort_index_ = sort_index;
    sort_index_ptr_ = &sort_index_;
  }

  inline const uint8_t &sort_index() const { return sort_index_; }

  inline uint8_t *mutable_sort_index() { return &sort_index_; }

  inline bool has_sort_index() { return (sort_index_ptr_ != nullptr); }

  inline void set_radar_power(const uint8_t &radar_power) {
    radar_power_ = radar_power;
    radar_power_ptr_ = &radar_power_;
  }

  inline const uint8_t &radar_power() const { return radar_power_; }

  inline uint8_t *mutable_radar_power() { return &radar_power_; }

  inline bool has_radar_power() { return (radar_power_ptr_ != nullptr); }

  inline void set_ctl_relay(const uint8_t &ctl_relay) {
    ctl_relay_ = ctl_relay;
    ctl_relay_ptr_ = &ctl_relay_;
  }

  inline const uint8_t &ctl_relay() const { return ctl_relay_; }

  inline uint8_t *mutable_ctl_relay() { return &ctl_relay_; }

  inline bool has_ctl_relay() { return (ctl_relay_ptr_ != nullptr); }

  inline void set_output_type(const uint8_t &output_type) {
    output_type_ = output_type;
    output_type_ptr_ = &output_type_;
  }

  inline const uint8_t &output_type() const { return output_type_; }

  inline uint8_t *mutable_output_type() { return &output_type_; }

  inline bool has_output_type() { return (output_type_ptr_ != nullptr); }

  inline void set_send_quality(const uint8_t &send_quality) {
    send_quality_ = send_quality;
    send_quality_ptr_ = &send_quality_;
  }

  inline const uint8_t &send_quality() const { return send_quality_; }

  inline uint8_t *mutable_send_quality() { return &send_quality_; }

  inline bool has_send_quality() { return (send_quality_ptr_ != nullptr); }

  inline void set_send_extinfo(const uint8_t &send_extinfo) {
    send_extinfo_ = send_extinfo;
    send_extinfo_ptr_ = &send_extinfo_;
  }

  inline const uint8_t &send_extinfo() const { return send_extinfo_; }

  inline uint8_t *mutable_send_extinfo() { return &send_extinfo_; }

  inline bool has_send_extinfo() { return (send_extinfo_ptr_ != nullptr); }

  inline void set_motion_rx_state(const uint8_t &motion_rx_state) {
    motion_rx_state_ = motion_rx_state;
    motion_rx_state_ptr_ = &motion_rx_state_;
  }

  inline const uint8_t &motion_rx_state() const { return motion_rx_state_; }

  inline uint8_t *mutable_motion_rx_state() { return &motion_rx_state_; }

  inline bool has_motion_rx_state() {
    return (motion_rx_state_ptr_ != nullptr);
  }

  inline void set_rcs_threshold(const uint8_t &rcs_threshold) {
    rcs_threshold_ = rcs_threshold;
    rcs_threshold_ptr_ = &rcs_threshold_;
  }

  inline const uint8_t &rcs_threshold() const { return rcs_threshold_; }

  inline uint8_t *mutable_rcs_threshold() { return &rcs_threshold_; }

  inline bool has_rcs_threshold() { return (rcs_threshold_ptr_ != nullptr); }

  inline void set_connector_direction(const uint8_t &connector_direction) {
    connector_direction_ = connector_direction;
    connector_direction_ptr_ = &connector_direction_;
  }

  inline const uint8_t &connector_direction() const {
    return connector_direction_;
  }

  inline uint8_t *mutable_connector_direction() {
    return &connector_direction_;
  }

  inline bool has_connector_direction() {
    return (connector_direction_ptr_ != nullptr);
  }

  inline void set_radar_position(const uint8_t &radar_position) {
    radar_position_ = radar_position;
    radar_position_ptr_ = &radar_position_;
  }

  inline const uint8_t &radar_position() const { return radar_position_; }

  inline uint8_t *mutable_radar_position() { return &radar_position_; }

  inline bool has_radar_position() { return (radar_position_ptr_ != nullptr); }

  inline void set_hw_error(const uint8_t &hw_error) {
    hw_error_ = hw_error;
    hw_error_ptr_ = &hw_error_;
  }

  inline const uint8_t &hw_error() const { return hw_error_; }

  inline uint8_t *mutable_hw_error() { return &hw_error_; }

  inline bool has_hw_error() { return (hw_error_ptr_ != nullptr); }

  void operator=(const RadarState &radar_state) { CopyFrom(radar_state); }

  void CopyFrom(const RadarState &radar_state) {
    sensor_id_ = radar_state.sensor_id();
    nvm_read_status_ = radar_state.nvm_read_status();
    nvm_write_status_ = radar_state.nvm_write_status();
    radar_state_error_ = radar_state.radar_state_error();
    radar_state_mode_ = radar_state.radar_state_mode();
    max_distance_ = radar_state.max_distance();
    sort_index_ = radar_state.sort_index();
    radar_power_ = radar_state.radar_power();
    ctl_relay_ = radar_state.ctl_relay();
    output_type_ = radar_state.output_type();
    send_quality_ = radar_state.send_quality();
    send_extinfo_ = radar_state.send_extinfo();
    motion_rx_state_ = radar_state.motion_rx_state();
    rcs_threshold_ = radar_state.rcs_threshold();
    connector_direction_ = radar_state.connector_direction();
    radar_position_ = radar_state.radar_position();
    hw_error_ = radar_state.hw_error();
  }

protected:
  uint8_t sensor_id_;
  uint8_t *sensor_id_ptr_ = nullptr;
  uint8_t nvm_read_status_;
  uint8_t *nvm_read_status_ptr_ = nullptr;
  uint8_t nvm_write_status_;
  uint8_t *nvm_write_status_ptr_ = nullptr;
  legionclaw::interface::RadarStateError radar_state_error_;
  legionclaw::interface::RadarStateError *radar_state_error_ptr_ = nullptr;
  legionclaw::interface::RadarStateMode radar_state_mode_;
  legionclaw::interface::RadarStateMode *radar_state_mode_ptr_ = nullptr;
  uint16_t max_distance_;
  uint16_t *max_distance_ptr_ = nullptr;
  uint8_t sort_index_;
  uint8_t *sort_index_ptr_ = nullptr;
  uint8_t radar_power_;
  uint8_t *radar_power_ptr_ = nullptr;
  uint8_t ctl_relay_;
  uint8_t *ctl_relay_ptr_ = nullptr;
  uint8_t output_type_;
  uint8_t *output_type_ptr_ = nullptr;
  uint8_t send_quality_;
  uint8_t *send_quality_ptr_ = nullptr;
  uint8_t send_extinfo_;
  uint8_t *send_extinfo_ptr_ = nullptr;
  uint8_t motion_rx_state_;
  uint8_t *motion_rx_state_ptr_ = nullptr;
  uint8_t rcs_threshold_;
  uint8_t *rcs_threshold_ptr_ = nullptr;
  uint8_t connector_direction_;
  uint8_t *connector_direction_ptr_ = nullptr;
  uint8_t radar_position_;
  uint8_t *radar_position_ptr_ = nullptr;
  uint8_t hw_error_;
  uint8_t *hw_error_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
