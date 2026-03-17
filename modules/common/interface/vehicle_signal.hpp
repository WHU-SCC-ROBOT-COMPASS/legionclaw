/**
 * @file    vehicle_signal.hpp
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

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleSignal {
public:
  VehicleSignal() {
    turn_signal_ = legionclaw::common::TurnSignal::TURN_INACTIVE_SIGNAL;
    high_beam_ = false;
    low_beam_ = false;
    horn_ = false;
    emergency_light_ = false;
  }
  ~VehicleSignal() = default;

  inline void set_turn_signal(const legionclaw::common::TurnSignal &turn_signal) {
    turn_signal_ = turn_signal;
    turn_signal_ptr_ = &turn_signal_;
  }

  inline const legionclaw::common::TurnSignal &turn_signal() const {
    return turn_signal_;
  }

  inline legionclaw::common::TurnSignal *mutable_turn_signal() {
    return &turn_signal_;
  }

  inline bool has_turn_signal() { return (turn_signal_ptr_ != nullptr); }

  inline void set_high_beam(const bool &high_beam) {
    high_beam_ = high_beam;
    high_beam_ptr_ = &high_beam_;
  }

  inline const bool &high_beam() const { return high_beam_; }

  inline bool *mutable_high_beam() { return &high_beam_; }

  inline bool has_high_beam() { return (high_beam_ptr_ != nullptr); }

  inline void set_low_beam(const bool &low_beam) {
    low_beam_ = low_beam;
    low_beam_ptr_ = &low_beam_;
  }

  inline const bool &low_beam() const { return low_beam_; }

  inline bool *mutable_low_beam() { return &low_beam_; }

  inline bool has_low_beam() { return (low_beam_ptr_ != nullptr); }

  inline void set_horn(const bool &horn) {
    horn_ = horn;
    horn_ptr_ = &horn_;
  }

  inline const bool &horn() const { return horn_; }

  inline bool *mutable_horn() { return &horn_; }

  inline bool has_horn() { return (horn_ptr_ != nullptr); }

  inline void set_emergency_light(const bool &emergency_light) {
    emergency_light_ = emergency_light;
    emergency_light_ptr_ = &emergency_light_;
  }

  inline const bool &emergency_light() const { return emergency_light_; }

  inline bool *mutable_emergency_light() { return &emergency_light_; }

  inline bool has_emergency_light() {
    return (emergency_light_ptr_ != nullptr);
  }

  void operator=(const VehicleSignal &vehicle_signal) {
    CopyFrom(vehicle_signal);
  }

  void CopyFrom(const VehicleSignal &vehicle_signal) {
    turn_signal_ = vehicle_signal.turn_signal();
    high_beam_ = vehicle_signal.high_beam();
    low_beam_ = vehicle_signal.low_beam();
    horn_ = vehicle_signal.horn();
    emergency_light_ = vehicle_signal.emergency_light();
  }

protected:
  //转向信号
  legionclaw::common::TurnSignal turn_signal_;
  legionclaw::common::TurnSignal *turn_signal_ptr_ = nullptr;
  // high beam
  bool high_beam_;
  bool *high_beam_ptr_ = nullptr;
  // low beam
  bool low_beam_;
  bool *low_beam_ptr_ = nullptr;
  // horn
  bool horn_;
  bool *horn_ptr_ = nullptr;
  // emergency light
  bool emergency_light_;
  bool *emergency_light_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
