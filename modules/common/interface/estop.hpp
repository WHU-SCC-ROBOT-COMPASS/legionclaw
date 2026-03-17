/**
 * @file    estop.hpp
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

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class EStop {
public:
  EStop() {
    is_estop_ = false;
    reason_.clear();
  }
  ~EStop() = default;

  inline void set_is_estop(const bool &is_estop) {
    is_estop_ = is_estop;
    is_estop_ptr_ = &is_estop_;
  }

  inline const bool &is_estop() const { return is_estop_; }

  inline bool *mutable_is_estop() { return &is_estop_; }

  inline bool has_is_estop() { return (is_estop_ptr_ != nullptr); }

  inline void set_reason(const std::string &reason) {
    reason_ = reason;
    reason_ptr_ = &reason_;
  }

  inline const std::string &reason() const { return reason_; }

  inline std::string *mutable_reason() { return &reason_; }

  inline bool has_reason() { return (reason_ptr_ != nullptr); }

  void operator=(const EStop &estop) { CopyFrom(estop); }

  void CopyFrom(const EStop &estop) {
    is_estop_ = estop.is_estop();
    reason_ = estop.reason();
  }

protected:
  // is_estop == true when emergency stop is required
  bool is_estop_;
  bool *is_estop_ptr_ = nullptr;
  //原因
  std::string reason_;
  std::string *reason_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
