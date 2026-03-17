/**
 * @file    event.hpp
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

#include "modules/common/interface/time.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Event {
public:
  Event() {
    code_ = 0;
    reason_.clear();
  }
  ~Event() = default;

  inline void set_timestamp(const legionclaw::interface::Time &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const legionclaw::interface::Time &timestamp() const { return timestamp_; }

  inline legionclaw::interface::Time *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_code(const uint64_t &code) {
    code_ = code;
    code_ptr_ = &code_;
  }

  inline const uint64_t &code() const { return code_; }

  inline uint64_t *mutable_code() { return &code_; }

  inline bool has_code() { return (code_ptr_ != nullptr); }

  inline void set_reason(const std::string &reason) {
    reason_ = reason;
    reason_ptr_ = &reason_;
  }

  inline const std::string &reason() const { return reason_; }

  inline std::string *mutable_reason() { return &reason_; }

  inline bool has_reason() { return (reason_ptr_ != nullptr); }

  void operator=(const Event &event) { CopyFrom(event); }

  void CopyFrom(const Event &event) {
    timestamp_ = event.timestamp();
    code_ = event.code();
    reason_ = event.reason();
  }

protected:
  //时间戳
  legionclaw::interface::Time timestamp_;
  legionclaw::interface::Time *timestamp_ptr_ = nullptr;
  // 0：invalid； 1：不可泊车；2：可以泊车；3：泊车完成；
  // 4：不可泊出；5：可以泊出；6：泊出完成;   7 :
  // 不可行车；8：可以行车；9：行车完成； 1X：退出自动驾驶（视退出场景可细分）
  // 10：轨迹太短，请求退出自动驾驶;
  // 20：拨杆换道驳回；21：执行向左拨杆换道;22：执行向右拨杆换道；23：拨杆换道完成
  // 30：准备主动向左换道；31：执行主动向左换道；32：准备主动向右换道；33：执行主动向右换道；34：主动换道完成;
  // 147: localmap的车道线效果不好，请求人工接管或者退出自动驾驶
  uint64_t code_;
  uint64_t *code_ptr_ = nullptr;
  //原因
  std::string reason_;
  std::string *reason_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
