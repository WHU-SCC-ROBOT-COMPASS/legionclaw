/**
 * @file    status.hpp
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

#include "modules/common/enum/enum.h"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Status {
public:
  Status() {
    error_code_ = legionclaw::common::ErrorCode::LOCALIZATION_ERROR;
    msg_.clear();
  }
  ~Status() = default;

  inline void set_error_code(const legionclaw::common::ErrorCode &error_code) {
    error_code_ = error_code;
    error_code_ptr_ = &error_code_;
  }

  inline const legionclaw::common::ErrorCode &error_code() const {
    return error_code_;
  }

  inline legionclaw::common::ErrorCode *mutable_error_code() { return &error_code_; }

  inline bool has_error_code() { return (error_code_ptr_ != nullptr); }

  inline void set_msg(const std::string &msg) {
    msg_ = msg;
    msg_ptr_ = &msg_;
  }

  inline const std::string &msg() const { return msg_; }

  inline std::string *mutable_msg() { return &msg_; }

  inline bool has_msg() { return (msg_ptr_ != nullptr); }

  void operator=(const Status &status) { CopyFrom(status); }

  void CopyFrom(const Status &status) {
    error_code_ = status.error_code();
    msg_ = status.msg();
  }

protected:
  //错误码
  legionclaw::common::ErrorCode error_code_;
  legionclaw::common::ErrorCode *error_code_ptr_ = nullptr;
  //错误码描述
  std::string msg_;
  std::string *msg_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
