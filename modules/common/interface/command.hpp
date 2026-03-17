/**
 * @file    command.hpp
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
class Command {
public:
  Command() {
    target_module_.clear();
    target_id_ = 0;
    target_switch_ = 0;
  }
  ~Command() = default;

  inline void set_target_module(const std::string &target_module) {
    target_module_ = target_module;
    target_module_ptr_ = &target_module_;
  }

  inline const std::string &target_module() const { return target_module_; }

  inline std::string *mutable_target_module() { return &target_module_; }

  inline bool has_target_module() { return (target_module_ptr_ != nullptr); }

  inline void set_target_id(const int8_t &target_id) {
    target_id_ = target_id;
    target_id_ptr_ = &target_id_;
  }

  inline const int8_t &target_id() const { return target_id_; }

  inline int8_t *mutable_target_id() { return &target_id_; }

  inline bool has_target_id() { return (target_id_ptr_ != nullptr); }

  inline void set_target_switch(const int32_t &target_switch) {
    target_switch_ = target_switch;
    target_switch_ptr_ = &target_switch_;
  }

  inline const int32_t &target_switch() const { return target_switch_; }

  inline int32_t *mutable_target_switch() { return &target_switch_; }

  inline bool has_target_switch() { return (target_switch_ptr_ != nullptr); }

  void operator=(const Command &command) { CopyFrom(command); }

  void CopyFrom(const Command &command) {
    target_module_ = command.target_module();
    target_id_ = command.target_id();
    target_switch_ = command.target_switch();
  }

protected:
  //目标节点名称
  std::string target_module_;
  std::string *target_module_ptr_ = nullptr;
  //目标节点ID
  int8_t target_id_;
  int8_t *target_id_ptr_ = nullptr;
  // 0:保持target_module的状态  1:打开target_module 2:关闭target_module
  int32_t target_switch_;
  int32_t *target_switch_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
