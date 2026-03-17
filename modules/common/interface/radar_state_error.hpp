/**
 * @file    radar_state_error.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RadarStateError {
public:
  RadarStateError() {
    persistent_error_ = 0;
    temporary_error_ = 0;
    interference_error_ = 0;
    temperature_error_ = 0;
    voltage_error_ = 0;
    block_error_ = 0;
    broadcast_error_ = 0;
    electric_axis_error_ = 0;
    config_error_ = 0;
    calibration_error_ = 0;
  }
  ~RadarStateError() = default;

  inline void set_persistent_error(const uint8_t &persistent_error) {
    persistent_error_ = persistent_error;
    persistent_error_ptr_ = &persistent_error_;
  }

  inline const uint8_t &persistent_error() const { return persistent_error_; }

  inline uint8_t *mutable_persistent_error() { return &persistent_error_; }

  inline bool has_persistent_error() {
    return (persistent_error_ptr_ != nullptr);
  }

  inline void set_temporary_error(const uint8_t &temporary_error) {
    temporary_error_ = temporary_error;
    temporary_error_ptr_ = &temporary_error_;
  }

  inline const uint8_t &temporary_error() const { return temporary_error_; }

  inline uint8_t *mutable_temporary_error() { return &temporary_error_; }

  inline bool has_temporary_error() {
    return (temporary_error_ptr_ != nullptr);
  }

  inline void set_interference_error(const uint8_t &interference_error) {
    interference_error_ = interference_error;
    interference_error_ptr_ = &interference_error_;
  }

  inline const uint8_t &interference_error() const {
    return interference_error_;
  }

  inline uint8_t *mutable_interference_error() { return &interference_error_; }

  inline bool has_interference_error() {
    return (interference_error_ptr_ != nullptr);
  }

  inline void set_temperature_error(const uint8_t &temperature_error) {
    temperature_error_ = temperature_error;
    temperature_error_ptr_ = &temperature_error_;
  }

  inline const uint8_t &temperature_error() const { return temperature_error_; }

  inline uint8_t *mutable_temperature_error() { return &temperature_error_; }

  inline bool has_temperature_error() {
    return (temperature_error_ptr_ != nullptr);
  }

  inline void set_voltage_error(const uint8_t &voltage_error) {
    voltage_error_ = voltage_error;
    voltage_error_ptr_ = &voltage_error_;
  }

  inline const uint8_t &voltage_error() const { return voltage_error_; }

  inline uint8_t *mutable_voltage_error() { return &voltage_error_; }

  inline bool has_voltage_error() { return (voltage_error_ptr_ != nullptr); }

  inline void set_block_error(const uint8_t &block_error) {
    block_error_ = block_error;
    block_error_ptr_ = &block_error_;
  }

  inline const uint8_t &block_error() const { return block_error_; }

  inline uint8_t *mutable_block_error() { return &block_error_; }

  inline bool has_block_error() { return (block_error_ptr_ != nullptr); }

  inline void set_broadcast_error(const uint8_t &broadcast_error) {
    broadcast_error_ = broadcast_error;
    broadcast_error_ptr_ = &broadcast_error_;
  }

  inline const uint8_t &broadcast_error() const { return broadcast_error_; }

  inline uint8_t *mutable_broadcast_error() { return &broadcast_error_; }

  inline bool has_broadcast_error() {
    return (broadcast_error_ptr_ != nullptr);
  }

  inline void set_electric_axis_error(const uint8_t &electric_axis_error) {
    electric_axis_error_ = electric_axis_error;
    electric_axis_error_ptr_ = &electric_axis_error_;
  }

  inline const uint8_t &electric_axis_error() const {
    return electric_axis_error_;
  }

  inline uint8_t *mutable_electric_axis_error() {
    return &electric_axis_error_;
  }

  inline bool has_electric_axis_error() {
    return (electric_axis_error_ptr_ != nullptr);
  }

  inline void set_config_error(const uint8_t &config_error) {
    config_error_ = config_error;
    config_error_ptr_ = &config_error_;
  }

  inline const uint8_t &config_error() const { return config_error_; }

  inline uint8_t *mutable_config_error() { return &config_error_; }

  inline bool has_config_error() { return (config_error_ptr_ != nullptr); }

  inline void set_calibration_error(const uint8_t &calibration_error) {
    calibration_error_ = calibration_error;
    calibration_error_ptr_ = &calibration_error_;
  }

  inline const uint8_t &calibration_error() const { return calibration_error_; }

  inline uint8_t *mutable_calibration_error() { return &calibration_error_; }

  inline bool has_calibration_error() {
    return (calibration_error_ptr_ != nullptr);
  }

  void operator=(const RadarStateError &radar_state_error) {
    CopyFrom(radar_state_error);
  }

  void CopyFrom(const RadarStateError &radar_state_error) {
    persistent_error_ = radar_state_error.persistent_error();
    temporary_error_ = radar_state_error.temporary_error();
    interference_error_ = radar_state_error.interference_error();
    temperature_error_ = radar_state_error.temperature_error();
    voltage_error_ = radar_state_error.voltage_error();
    block_error_ = radar_state_error.block_error();
    broadcast_error_ = radar_state_error.broadcast_error();
    electric_axis_error_ = radar_state_error.electric_axis_error();
    config_error_ = radar_state_error.config_error();
    calibration_error_ = radar_state_error.calibration_error();
  }

protected:
  uint8_t persistent_error_;
  uint8_t *persistent_error_ptr_ = nullptr;
  uint8_t temporary_error_;
  uint8_t *temporary_error_ptr_ = nullptr;
  uint8_t interference_error_;
  uint8_t *interference_error_ptr_ = nullptr;
  uint8_t temperature_error_;
  uint8_t *temperature_error_ptr_ = nullptr;
  uint8_t voltage_error_;
  uint8_t *voltage_error_ptr_ = nullptr;
  uint8_t block_error_;
  uint8_t *block_error_ptr_ = nullptr;
  uint8_t broadcast_error_;
  uint8_t *broadcast_error_ptr_ = nullptr;
  uint8_t electric_axis_error_;
  uint8_t *electric_axis_error_ptr_ = nullptr;
  uint8_t config_error_;
  uint8_t *config_error_ptr_ = nullptr;
  uint8_t calibration_error_;
  uint8_t *calibration_error_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
