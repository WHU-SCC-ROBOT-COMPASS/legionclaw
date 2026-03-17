/**
 * @file    planning_cmd.hpp
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
class PlanningCmd {
public:
  PlanningCmd() {
    turn_lamp_ctrl_ = legionclaw::common::TurnSignal::TURN_INACTIVE_SIGNAL;
    high_beam_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    low_beam_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    horn_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    front_wiper_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_wiper_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    position_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    front_fog_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    rear_fog_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    brake_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    alarm_lamp_ctrl_ = legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF;
    lf_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    rf_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    lr_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
    rr_door_ctrl_ = legionclaw::common::DoorStatus::CLOSEED;
  }
  ~PlanningCmd() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_turn_lamp_ctrl(const legionclaw::common::TurnSignal &turn_lamp_ctrl) {
    turn_lamp_ctrl_ = turn_lamp_ctrl;
    turn_lamp_ctrl_ptr_ = &turn_lamp_ctrl_;
  }

  inline const legionclaw::common::TurnSignal &turn_lamp_ctrl() const {
    return turn_lamp_ctrl_;
  }

  inline legionclaw::common::TurnSignal *mutable_turn_lamp_ctrl() {
    return &turn_lamp_ctrl_;
  }

  inline bool has_turn_lamp_ctrl() { return (turn_lamp_ctrl_ptr_ != nullptr); }

  inline void
  set_high_beam_ctrl(const legionclaw::common::SwitchStatus &high_beam_ctrl) {
    high_beam_ctrl_ = high_beam_ctrl;
    high_beam_ctrl_ptr_ = &high_beam_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &high_beam_ctrl() const {
    return high_beam_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_high_beam_ctrl() {
    return &high_beam_ctrl_;
  }

  inline bool has_high_beam_ctrl() { return (high_beam_ctrl_ptr_ != nullptr); }

  inline void
  set_low_beam_ctrl(const legionclaw::common::SwitchStatus &low_beam_ctrl) {
    low_beam_ctrl_ = low_beam_ctrl;
    low_beam_ctrl_ptr_ = &low_beam_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &low_beam_ctrl() const {
    return low_beam_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_low_beam_ctrl() {
    return &low_beam_ctrl_;
  }

  inline bool has_low_beam_ctrl() { return (low_beam_ctrl_ptr_ != nullptr); }

  inline void set_horn_ctrl(const legionclaw::common::SwitchStatus &horn_ctrl) {
    horn_ctrl_ = horn_ctrl;
    horn_ctrl_ptr_ = &horn_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &horn_ctrl() const {
    return horn_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_horn_ctrl() {
    return &horn_ctrl_;
  }

  inline bool has_horn_ctrl() { return (horn_ctrl_ptr_ != nullptr); }

  inline void
  set_front_wiper_ctrl(const legionclaw::common::SwitchStatus &front_wiper_ctrl) {
    front_wiper_ctrl_ = front_wiper_ctrl;
    front_wiper_ctrl_ptr_ = &front_wiper_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &front_wiper_ctrl() const {
    return front_wiper_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_wiper_ctrl() {
    return &front_wiper_ctrl_;
  }

  inline bool has_front_wiper_ctrl() {
    return (front_wiper_ctrl_ptr_ != nullptr);
  }

  inline void
  set_rear_wiper_ctrl(const legionclaw::common::SwitchStatus &rear_wiper_ctrl) {
    rear_wiper_ctrl_ = rear_wiper_ctrl;
    rear_wiper_ctrl_ptr_ = &rear_wiper_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &rear_wiper_ctrl() const {
    return rear_wiper_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_wiper_ctrl() {
    return &rear_wiper_ctrl_;
  }

  inline bool has_rear_wiper_ctrl() {
    return (rear_wiper_ctrl_ptr_ != nullptr);
  }

  inline void set_position_lamp_ctrl(
      const legionclaw::common::SwitchStatus &position_lamp_ctrl) {
    position_lamp_ctrl_ = position_lamp_ctrl;
    position_lamp_ctrl_ptr_ = &position_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &position_lamp_ctrl() const {
    return position_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_position_lamp_ctrl() {
    return &position_lamp_ctrl_;
  }

  inline bool has_position_lamp_ctrl() {
    return (position_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_front_fog_lamp_ctrl(
      const legionclaw::common::SwitchStatus &front_fog_lamp_ctrl) {
    front_fog_lamp_ctrl_ = front_fog_lamp_ctrl;
    front_fog_lamp_ctrl_ptr_ = &front_fog_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &front_fog_lamp_ctrl() const {
    return front_fog_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_front_fog_lamp_ctrl() {
    return &front_fog_lamp_ctrl_;
  }

  inline bool has_front_fog_lamp_ctrl() {
    return (front_fog_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_rear_fog_lamp_ctrl(
      const legionclaw::common::SwitchStatus &rear_fog_lamp_ctrl) {
    rear_fog_lamp_ctrl_ = rear_fog_lamp_ctrl;
    rear_fog_lamp_ctrl_ptr_ = &rear_fog_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &rear_fog_lamp_ctrl() const {
    return rear_fog_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_rear_fog_lamp_ctrl() {
    return &rear_fog_lamp_ctrl_;
  }

  inline bool has_rear_fog_lamp_ctrl() {
    return (rear_fog_lamp_ctrl_ptr_ != nullptr);
  }

  inline void
  set_brake_lamp_ctrl(const legionclaw::common::SwitchStatus &brake_lamp_ctrl) {
    brake_lamp_ctrl_ = brake_lamp_ctrl;
    brake_lamp_ctrl_ptr_ = &brake_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &brake_lamp_ctrl() const {
    return brake_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_brake_lamp_ctrl() {
    return &brake_lamp_ctrl_;
  }

  inline bool has_brake_lamp_ctrl() {
    return (brake_lamp_ctrl_ptr_ != nullptr);
  }

  inline void
  set_alarm_lamp_ctrl(const legionclaw::common::SwitchStatus &alarm_lamp_ctrl) {
    alarm_lamp_ctrl_ = alarm_lamp_ctrl;
    alarm_lamp_ctrl_ptr_ = &alarm_lamp_ctrl_;
  }

  inline const legionclaw::common::SwitchStatus &alarm_lamp_ctrl() const {
    return alarm_lamp_ctrl_;
  }

  inline legionclaw::common::SwitchStatus *mutable_alarm_lamp_ctrl() {
    return &alarm_lamp_ctrl_;
  }

  inline bool has_alarm_lamp_ctrl() {
    return (alarm_lamp_ctrl_ptr_ != nullptr);
  }

  inline void set_lf_door_ctrl(const legionclaw::common::DoorStatus &lf_door_ctrl) {
    lf_door_ctrl_ = lf_door_ctrl;
    lf_door_ctrl_ptr_ = &lf_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &lf_door_ctrl() const {
    return lf_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_lf_door_ctrl() {
    return &lf_door_ctrl_;
  }

  inline bool has_lf_door_ctrl() { return (lf_door_ctrl_ptr_ != nullptr); }

  inline void set_rf_door_ctrl(const legionclaw::common::DoorStatus &rf_door_ctrl) {
    rf_door_ctrl_ = rf_door_ctrl;
    rf_door_ctrl_ptr_ = &rf_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &rf_door_ctrl() const {
    return rf_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_rf_door_ctrl() {
    return &rf_door_ctrl_;
  }

  inline bool has_rf_door_ctrl() { return (rf_door_ctrl_ptr_ != nullptr); }

  inline void set_lr_door_ctrl(const legionclaw::common::DoorStatus &lr_door_ctrl) {
    lr_door_ctrl_ = lr_door_ctrl;
    lr_door_ctrl_ptr_ = &lr_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &lr_door_ctrl() const {
    return lr_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_lr_door_ctrl() {
    return &lr_door_ctrl_;
  }

  inline bool has_lr_door_ctrl() { return (lr_door_ctrl_ptr_ != nullptr); }

  inline void set_rr_door_ctrl(const legionclaw::common::DoorStatus &rr_door_ctrl) {
    rr_door_ctrl_ = rr_door_ctrl;
    rr_door_ctrl_ptr_ = &rr_door_ctrl_;
  }

  inline const legionclaw::common::DoorStatus &rr_door_ctrl() const {
    return rr_door_ctrl_;
  }

  inline legionclaw::common::DoorStatus *mutable_rr_door_ctrl() {
    return &rr_door_ctrl_;
  }

  inline bool has_rr_door_ctrl() { return (rr_door_ctrl_ptr_ != nullptr); }

  void operator=(const PlanningCmd &planning_cmd) { CopyFrom(planning_cmd); }

  void CopyFrom(const PlanningCmd &planning_cmd) {
    header_ = planning_cmd.header();
    turn_lamp_ctrl_ = planning_cmd.turn_lamp_ctrl();
    high_beam_ctrl_ = planning_cmd.high_beam_ctrl();
    low_beam_ctrl_ = planning_cmd.low_beam_ctrl();
    horn_ctrl_ = planning_cmd.horn_ctrl();
    front_wiper_ctrl_ = planning_cmd.front_wiper_ctrl();
    rear_wiper_ctrl_ = planning_cmd.rear_wiper_ctrl();
    position_lamp_ctrl_ = planning_cmd.position_lamp_ctrl();
    front_fog_lamp_ctrl_ = planning_cmd.front_fog_lamp_ctrl();
    rear_fog_lamp_ctrl_ = planning_cmd.rear_fog_lamp_ctrl();
    brake_lamp_ctrl_ = planning_cmd.brake_lamp_ctrl();
    alarm_lamp_ctrl_ = planning_cmd.alarm_lamp_ctrl();
    lf_door_ctrl_ = planning_cmd.lf_door_ctrl();
    rf_door_ctrl_ = planning_cmd.rf_door_ctrl();
    lr_door_ctrl_ = planning_cmd.lr_door_ctrl();
    rr_door_ctrl_ = planning_cmd.rr_door_ctrl();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //转向灯 TURN_INACTIVE_SIGNAL:0 TURN_LEFT_SIGNAL:1 TURN_RIGHT_SIGNAL:2
  //TURN_INVALID_SIGNAL:3
  legionclaw::common::TurnSignal turn_lamp_ctrl_;
  legionclaw::common::TurnSignal *turn_lamp_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus high_beam_ctrl_;
  legionclaw::common::SwitchStatus *high_beam_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus low_beam_ctrl_;
  legionclaw::common::SwitchStatus *low_beam_ctrl_ptr_ = nullptr;
  // 0 关闭 1 开启 2 不响应
  legionclaw::common::SwitchStatus horn_ctrl_;
  legionclaw::common::SwitchStatus *horn_ctrl_ptr_ = nullptr;
  //前雨刮器 0 关闭 1 开启
  legionclaw::common::SwitchStatus front_wiper_ctrl_;
  legionclaw::common::SwitchStatus *front_wiper_ctrl_ptr_ = nullptr;
  //后雨刮器 0 关闭 1 开启
  legionclaw::common::SwitchStatus rear_wiper_ctrl_;
  legionclaw::common::SwitchStatus *rear_wiper_ctrl_ptr_ = nullptr;
  //位置灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus position_lamp_ctrl_;
  legionclaw::common::SwitchStatus *position_lamp_ctrl_ptr_ = nullptr;
  //前雾灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus front_fog_lamp_ctrl_;
  legionclaw::common::SwitchStatus *front_fog_lamp_ctrl_ptr_ = nullptr;
  //后雾灯 0 关闭 1 开启
  legionclaw::common::SwitchStatus rear_fog_lamp_ctrl_;
  legionclaw::common::SwitchStatus *rear_fog_lamp_ctrl_ptr_ = nullptr;
  //刹车灯 一般情况自动控制 0 关闭 1 开启
  legionclaw::common::SwitchStatus brake_lamp_ctrl_;
  legionclaw::common::SwitchStatus *brake_lamp_ctrl_ptr_ = nullptr;
  //警报灯 双闪 0 关闭 1 开启
  legionclaw::common::SwitchStatus alarm_lamp_ctrl_;
  legionclaw::common::SwitchStatus *alarm_lamp_ctrl_ptr_ = nullptr;
  //左前门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus lf_door_ctrl_;
  legionclaw::common::DoorStatus *lf_door_ctrl_ptr_ = nullptr;
  //右前门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus rf_door_ctrl_;
  legionclaw::common::DoorStatus *rf_door_ctrl_ptr_ = nullptr;
  //左后门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus lr_door_ctrl_;
  legionclaw::common::DoorStatus *lr_door_ctrl_ptr_ = nullptr;
  //右后门控制 0 关闭 1 开启
  legionclaw::common::DoorStatus rr_door_ctrl_;
  legionclaw::common::DoorStatus *rr_door_ctrl_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
