/**
 * @file    parking_state_display.hpp
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
class ParkingStateDisplay {
public:
  ParkingStateDisplay() {
    parking_type_ = legionclaw::common::ParkingType::INVALID_PARKING;
    moves_counter_ = 0;
    remian_distance_ = 0.0;
    display_info_ = legionclaw::interface::ParkingStateDisplay::InfoMessage::NORMAL;
    distance_to_leader_obj_ = 0.0;
    state_ = legionclaw::interface::ParkingStateDisplay::ParkingState::
        PARKING_STATE_NORMAL;
  }
  ~ParkingStateDisplay() = default;

  enum ParkingState {
    PARKING_STATE_NORMAL = 0,
    PARKING_STATE_ABNORMAL = 1,
    PARKING_STATE_FINISH = 2,
    PARKING_PAUSE_STATE_NORMAL = 3,
    PARKING_PAUSE_STATE_ABNORMAL = 4,
  };

  enum InfoMessage {
    NORMAL = 0,
    ABNORMAL = 1,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_parking_type(const legionclaw::common::ParkingType &parking_type) {
    parking_type_ = parking_type;
    parking_type_ptr_ = &parking_type_;
  }

  inline const legionclaw::common::ParkingType &parking_type() const {
    return parking_type_;
  }

  inline legionclaw::common::ParkingType *mutable_parking_type() {
    return &parking_type_;
  }

  inline bool has_parking_type() { return (parking_type_ptr_ != nullptr); }

  inline void set_moves_counter(const int32_t &moves_counter) {
    moves_counter_ = moves_counter;
    moves_counter_ptr_ = &moves_counter_;
  }

  inline const int32_t &moves_counter() const { return moves_counter_; }

  inline int32_t *mutable_moves_counter() { return &moves_counter_; }

  inline bool has_moves_counter() { return (moves_counter_ptr_ != nullptr); }

  inline void set_remian_distance(const double &remian_distance) {
    remian_distance_ = remian_distance;
    remian_distance_ptr_ = &remian_distance_;
  }

  inline const double &remian_distance() const { return remian_distance_; }

  inline double *mutable_remian_distance() { return &remian_distance_; }

  inline bool has_remian_distance() {
    return (remian_distance_ptr_ != nullptr);
  }

  inline void set_display_info(
      const legionclaw::interface::ParkingStateDisplay::InfoMessage &display_info) {
    display_info_ = display_info;
    display_info_ptr_ = &display_info_;
  }

  inline const legionclaw::interface::ParkingStateDisplay::InfoMessage &
  display_info() const {
    return display_info_;
  }

  inline legionclaw::interface::ParkingStateDisplay::InfoMessage *
  mutable_display_info() {
    return &display_info_;
  }

  inline bool has_display_info() { return (display_info_ptr_ != nullptr); }

  inline void set_distance_to_leader_obj(const double &distance_to_leader_obj) {
    distance_to_leader_obj_ = distance_to_leader_obj;
    distance_to_leader_obj_ptr_ = &distance_to_leader_obj_;
  }

  inline const double &distance_to_leader_obj() const {
    return distance_to_leader_obj_;
  }

  inline double *mutable_distance_to_leader_obj() {
    return &distance_to_leader_obj_;
  }

  inline bool has_distance_to_leader_obj() {
    return (distance_to_leader_obj_ptr_ != nullptr);
  }

  inline void
  set_state(const legionclaw::interface::ParkingStateDisplay::ParkingState &state) {
    state_ = state;
    state_ptr_ = &state_;
  }

  inline const legionclaw::interface::ParkingStateDisplay::ParkingState &
  state() const {
    return state_;
  }

  inline legionclaw::interface::ParkingStateDisplay::ParkingState *mutable_state() {
    return &state_;
  }

  inline bool has_state() { return (state_ptr_ != nullptr); }

  void operator=(const ParkingStateDisplay &parking_state_display) {
    CopyFrom(parking_state_display);
  }

  void CopyFrom(const ParkingStateDisplay &parking_state_display) {
    header_ = parking_state_display.header();
    parking_type_ = parking_state_display.parking_type();
    moves_counter_ = parking_state_display.moves_counter();
    remian_distance_ = parking_state_display.remian_distance();
    display_info_ = parking_state_display.display_info();
    distance_to_leader_obj_ = parking_state_display.distance_to_leader_obj();
    state_ = parking_state_display.state();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // 0：无效 1：垂直泊车 2：水平泊车 3：斜方泊车
  legionclaw::common::ParkingType parking_type_;
  legionclaw::common::ParkingType *parking_type_ptr_ = nullptr;
  //揉库次数
  int32_t moves_counter_;
  int32_t *moves_counter_ptr_ = nullptr;
  //剩余距离
  double remian_distance_;
  double *remian_distance_ptr_ = nullptr;
  //详细信息
  legionclaw::interface::ParkingStateDisplay::InfoMessage display_info_;
  legionclaw::interface::ParkingStateDisplay::InfoMessage *display_info_ptr_ =
      nullptr;
  //距离最近障碍物距离
  double distance_to_leader_obj_;
  double *distance_to_leader_obj_ptr_ = nullptr;
  legionclaw::interface::ParkingStateDisplay::ParkingState state_;
  legionclaw::interface::ParkingStateDisplay::ParkingState *state_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
