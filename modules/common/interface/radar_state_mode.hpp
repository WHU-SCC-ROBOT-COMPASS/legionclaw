/**
 * @file    radar_state_mode.hpp
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
class RadarStateMode {
public:
  RadarStateMode() {
    can0_work_mode_ = 0;
    can1_work_mode_ = 0;
    dual_can_mode_ = 0;
    timming_mode_ = 0;
    power_mode_ = 0;
    performance_mode_ = 0;
  }
  ~RadarStateMode() = default;

  inline void set_can0_work_mode(const uint8_t &can0_work_mode) {
    can0_work_mode_ = can0_work_mode;
    can0_work_mode_ptr_ = &can0_work_mode_;
  }

  inline const uint8_t &can0_work_mode() const { return can0_work_mode_; }

  inline uint8_t *mutable_can0_work_mode() { return &can0_work_mode_; }

  inline bool has_can0_work_mode() { return (can0_work_mode_ptr_ != nullptr); }

  inline void set_can1_work_mode(const uint8_t &can1_work_mode) {
    can1_work_mode_ = can1_work_mode;
    can1_work_mode_ptr_ = &can1_work_mode_;
  }

  inline const uint8_t &can1_work_mode() const { return can1_work_mode_; }

  inline uint8_t *mutable_can1_work_mode() { return &can1_work_mode_; }

  inline bool has_can1_work_mode() { return (can1_work_mode_ptr_ != nullptr); }

  inline void set_dual_can_mode(const uint8_t &dual_can_mode) {
    dual_can_mode_ = dual_can_mode;
    dual_can_mode_ptr_ = &dual_can_mode_;
  }

  inline const uint8_t &dual_can_mode() const { return dual_can_mode_; }

  inline uint8_t *mutable_dual_can_mode() { return &dual_can_mode_; }

  inline bool has_dual_can_mode() { return (dual_can_mode_ptr_ != nullptr); }

  inline void set_timming_mode(const uint8_t &timming_mode) {
    timming_mode_ = timming_mode;
    timming_mode_ptr_ = &timming_mode_;
  }

  inline const uint8_t &timming_mode() const { return timming_mode_; }

  inline uint8_t *mutable_timming_mode() { return &timming_mode_; }

  inline bool has_timming_mode() { return (timming_mode_ptr_ != nullptr); }

  inline void set_power_mode(const uint8_t &power_mode) {
    power_mode_ = power_mode;
    power_mode_ptr_ = &power_mode_;
  }

  inline const uint8_t &power_mode() const { return power_mode_; }

  inline uint8_t *mutable_power_mode() { return &power_mode_; }

  inline bool has_power_mode() { return (power_mode_ptr_ != nullptr); }

  inline void set_performance_mode(const uint8_t &performance_mode) {
    performance_mode_ = performance_mode;
    performance_mode_ptr_ = &performance_mode_;
  }

  inline const uint8_t &performance_mode() const { return performance_mode_; }

  inline uint8_t *mutable_performance_mode() { return &performance_mode_; }

  inline bool has_performance_mode() {
    return (performance_mode_ptr_ != nullptr);
  }

  void operator=(const RadarStateMode &radar_state_mode) {
    CopyFrom(radar_state_mode);
  }

  void CopyFrom(const RadarStateMode &radar_state_mode) {
    can0_work_mode_ = radar_state_mode.can0_work_mode();
    can1_work_mode_ = radar_state_mode.can1_work_mode();
    dual_can_mode_ = radar_state_mode.dual_can_mode();
    timming_mode_ = radar_state_mode.timming_mode();
    power_mode_ = radar_state_mode.power_mode();
    performance_mode_ = radar_state_mode.performance_mode();
  }

protected:
  // can0工作模式
  uint8_t can0_work_mode_;
  uint8_t *can0_work_mode_ptr_ = nullptr;
  // can1工作模式
  uint8_t can1_work_mode_;
  uint8_t *can1_work_mode_ptr_ = nullptr;
  //双总线工作模式
  uint8_t dual_can_mode_;
  uint8_t *dual_can_mode_ptr_ = nullptr;
  //计时模式
  uint8_t timming_mode_;
  uint8_t *timming_mode_ptr_ = nullptr;
  //电源模式
  uint8_t power_mode_;
  uint8_t *power_mode_ptr_ = nullptr;
  //效能模式
  uint8_t performance_mode_;
  uint8_t *performance_mode_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
