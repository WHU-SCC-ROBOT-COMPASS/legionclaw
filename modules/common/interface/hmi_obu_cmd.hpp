/**
 * @file    hmi_obu_cmd.hpp
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
class HMIObuCmd {
public:
  HMIObuCmd() {
    code_ = 0;
    val_ = 0;
  }
  ~HMIObuCmd() = default;

  inline void set_code(const int32_t &code) {
    code_ = code;
    code_ptr_ = &code_;
  }

  inline const int32_t &code() const { return code_; }

  inline int32_t *mutable_code() { return &code_; }

  inline bool has_code() { return (code_ptr_ != nullptr); }

  inline void set_val(const int32_t &val) {
    val_ = val;
    val_ptr_ = &val_;
  }

  inline const int32_t &val() const { return val_; }

  inline int32_t *mutable_val() { return &val_; }

  inline bool has_val() { return (val_ptr_ != nullptr); }

  void operator=(const HMIObuCmd &hmi_obu_cmd) { CopyFrom(hmi_obu_cmd); }

  void CopyFrom(const HMIObuCmd &hmi_obu_cmd) {
    code_ = hmi_obu_cmd.code();
    val_ = hmi_obu_cmd.val();
  }

protected:
  //指令编码：10001、自动驾驶使能开关；
  int32_t code_;
  int32_t *code_ptr_ = nullptr;
  //编码值：0、不响应；1、自定义状态；2、自定义状态...（更多参考：“ObuCmdMsg.md”）
  int32_t val_;
  int32_t *val_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
