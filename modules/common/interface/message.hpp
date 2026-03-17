/**
 * @file    message.hpp
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
class Message {
public:
  Message() {
    code_ = 0;
    value_ = 0;
  }
  ~Message() = default;

  inline void set_code(const int32_t &code) {
    code_ = code;
    code_ptr_ = &code_;
  }

  inline const int32_t &code() const { return code_; }

  inline int32_t *mutable_code() { return &code_; }

  inline bool has_code() { return (code_ptr_ != nullptr); }

  inline void set_value(const int32_t &value) {
    value_ = value;
    value_ptr_ = &value_;
  }

  inline const int32_t &value() const { return value_; }

  inline int32_t *mutable_value() { return &value_; }

  inline bool has_value() { return (value_ptr_ != nullptr); }

  void operator=(const Message &message) { CopyFrom(message); }

  void CopyFrom(const Message &message) {
    code_ = message.code();
    value_ = message.value();
  }

protected:
  //指定需要显示的值的code定义，需要与self_id绑定 作为前缀
  int32_t code_;
  int32_t *code_ptr_ = nullptr;
  //对应显示的值或表达的状态
  int32_t value_;
  int32_t *value_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
