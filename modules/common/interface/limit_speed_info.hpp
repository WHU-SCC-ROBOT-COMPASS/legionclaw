/**
 * @file    limit_speed_info.hpp
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
class LimitSpeedInfo {
public:
  LimitSpeedInfo() {
    limitspeed_valid_flag_ = legionclaw::common::IsValid::INVALID;
    limit_speed_ = 0.0;
    limit_distance_ = 0.0;
  }
  ~LimitSpeedInfo() = default;

  inline void set_limitspeed_valid_flag(
      const legionclaw::common::IsValid &limitspeed_valid_flag) {
    limitspeed_valid_flag_ = limitspeed_valid_flag;
    limitspeed_valid_flag_ptr_ = &limitspeed_valid_flag_;
  }

  inline const legionclaw::common::IsValid &limitspeed_valid_flag() const {
    return limitspeed_valid_flag_;
  }

  inline legionclaw::common::IsValid *mutable_limitspeed_valid_flag() {
    return &limitspeed_valid_flag_;
  }

  inline bool has_limitspeed_valid_flag() {
    return (limitspeed_valid_flag_ptr_ != nullptr);
  }

  inline void set_limit_speed(const double &limit_speed) {
    limit_speed_ = limit_speed;
    limit_speed_ptr_ = &limit_speed_;
  }

  inline const double &limit_speed() const { return limit_speed_; }

  inline double *mutable_limit_speed() { return &limit_speed_; }

  inline bool has_limit_speed() { return (limit_speed_ptr_ != nullptr); }

  inline void set_limit_distance(const double &limit_distance) {
    limit_distance_ = limit_distance;
    limit_distance_ptr_ = &limit_distance_;
  }

  inline const double &limit_distance() const { return limit_distance_; }

  inline double *mutable_limit_distance() { return &limit_distance_; }

  inline bool has_limit_distance() { return (limit_distance_ptr_ != nullptr); }

  void operator=(const LimitSpeedInfo &limit_speed_info) {
    CopyFrom(limit_speed_info);
  }

  void CopyFrom(const LimitSpeedInfo &limit_speed_info) {
    limitspeed_valid_flag_ = limit_speed_info.limitspeed_valid_flag();
    limit_speed_ = limit_speed_info.limit_speed();
    limit_distance_ = limit_speed_info.limit_distance();
  }

protected:
  //限速有效性标志位
  legionclaw::common::IsValid limitspeed_valid_flag_;
  legionclaw::common::IsValid *limitspeed_valid_flag_ptr_ = nullptr;
  //限速大小（m/s）
  double limit_speed_;
  double *limit_speed_ptr_ = nullptr;
  //开始减速距离(m)
  double limit_distance_;
  double *limit_distance_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
