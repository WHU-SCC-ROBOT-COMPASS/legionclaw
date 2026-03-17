
/**
 * @file gear_state_machine_debug.h
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2018-07-07
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#pragma once
#include "modules/common/interface/chassis.hpp"
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {
class GearStateMachineDebug {
 public:
  GearStateMachineDebug() = default;
  ~GearStateMachineDebug() = default;

  void set_tar_gear(legionclaw::common::GearPosition tar_gear) {
    tar_gear_ = tar_gear;
  }

  legionclaw::common::GearPosition tar_gear() const { return tar_gear_; }

  void set_cur_gear(legionclaw::common::GearPosition cur_gear) {
    cur_gear_ = cur_gear;
  }

  legionclaw::common::GearPosition cur_gear() const { return cur_gear_; }

  void set_is_stopped(bool is_stopped) { is_stopped_ = is_stopped; }

  bool is_stopped() const { return is_stopped_; }

  void set_enable_epb_applied(bool enable_epb_applied) { enable_epb_applied_ = enable_epb_applied; }
  bool enable_epb_applied() const { return enable_epb_applied_; }


  void set_cur_epb_state(legionclaw::common::EPBLevel cur_epb_state) {
    cur_epb_state_ = cur_epb_state;
  }

  legionclaw::common::EPBLevel cur_epb_state() const { return cur_epb_state_; }

  void set_brake_value_when_gear_transitioning(
      double brake_value_when_gear_transitioning) {
    brake_value_when_gear_transitioning_ = brake_value_when_gear_transitioning;
  }

  double brake_value_when_gear_transitioning() const {
    return brake_value_when_gear_transitioning_;
  }

 protected:
  legionclaw::common::GearPosition tar_gear_;
  legionclaw::common::GearPosition cur_gear_;
  double cur_v_;
  bool is_stopped_;
  bool enable_epb_applied_;

  legionclaw::common::EPBLevel cur_epb_state_;

  double brake_value_when_gear_transitioning_;
};
}  // namespace control
}  // namespace legionclaw
