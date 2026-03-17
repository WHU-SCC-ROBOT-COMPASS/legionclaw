/**
 * @file    hmi_diagnostic_status.hpp
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
class HMIDiagnosticStatus {
public:
  HMIDiagnosticStatus() {
    id_ = 0;
    level_ = 0;
    name_.clear();
    message_.clear();
    hardware_id_.clear();
  }
  ~HMIDiagnosticStatus() = default;

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_level(const int32_t &level) {
    level_ = level;
    level_ptr_ = &level_;
  }

  inline const int32_t &level() const { return level_; }

  inline int32_t *mutable_level() { return &level_; }

  inline bool has_level() { return (level_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_message(const std::string &message) {
    message_ = message;
    message_ptr_ = &message_;
  }

  inline const std::string &message() const { return message_; }

  inline std::string *mutable_message() { return &message_; }

  inline bool has_message() { return (message_ptr_ != nullptr); }

  inline void set_hardware_id(const std::string &hardware_id) {
    hardware_id_ = hardware_id;
    hardware_id_ptr_ = &hardware_id_;
  }

  inline const std::string &hardware_id() const { return hardware_id_; }

  inline std::string *mutable_hardware_id() { return &hardware_id_; }

  inline bool has_hardware_id() { return (hardware_id_ptr_ != nullptr); }

  void operator=(const HMIDiagnosticStatus &hmi_diagnostic_status) {
    CopyFrom(hmi_diagnostic_status);
  }

  void CopyFrom(const HMIDiagnosticStatus &hmi_diagnostic_status) {
    id_ = hmi_diagnostic_status.id();
    level_ = hmi_diagnostic_status.level();
    name_ = hmi_diagnostic_status.name();
    message_ = hmi_diagnostic_status.message();
    hardware_id_ = hmi_diagnostic_status.hardware_id();
  }

protected:
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //// 诊断状态
  int32_t level_;
  int32_t *level_ptr_ = nullptr;
  ////测试/组件报告的描述
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //// 状态描述
  std::string message_;
  std::string *message_ptr_ = nullptr;
  //// 硬件的唯一描述
  std::string hardware_id_;
  std::string *hardware_id_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
