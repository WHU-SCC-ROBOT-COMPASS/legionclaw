/**
 * @file    command_respond.hpp
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

#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CommandRespond {
public:
  CommandRespond() {
    name_.clear();
    self_id_ = 0;
    respond_id_ = 0;
    status_ = 0;
    fail_code_ = 0;
  }
  ~CommandRespond() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_self_id(const int8_t &self_id) {
    self_id_ = self_id;
    self_id_ptr_ = &self_id_;
  }

  inline const int8_t &self_id() const { return self_id_; }

  inline int8_t *mutable_self_id() { return &self_id_; }

  inline bool has_self_id() { return (self_id_ptr_ != nullptr); }

  inline void set_respond_id(const int8_t &respond_id) {
    respond_id_ = respond_id;
    respond_id_ptr_ = &respond_id_;
  }

  inline const int8_t &respond_id() const { return respond_id_; }

  inline int8_t *mutable_respond_id() { return &respond_id_; }

  inline bool has_respond_id() { return (respond_id_ptr_ != nullptr); }

  inline void set_status(const int8_t &status) {
    status_ = status;
    status_ptr_ = &status_;
  }

  inline const int8_t &status() const { return status_; }

  inline int8_t *mutable_status() { return &status_; }

  inline bool has_status() { return (status_ptr_ != nullptr); }

  inline void set_fail_code(const int32_t &fail_code) {
    fail_code_ = fail_code;
    fail_code_ptr_ = &fail_code_;
  }

  inline const int32_t &fail_code() const { return fail_code_; }

  inline int32_t *mutable_fail_code() { return &fail_code_; }

  inline bool has_fail_code() { return (fail_code_ptr_ != nullptr); }

  void operator=(const CommandRespond &command_respond) {
    CopyFrom(command_respond);
  }

  void CopyFrom(const CommandRespond &command_respond) {
    header_ = command_respond.header();
    name_ = command_respond.name();
    self_id_ = command_respond.self_id();
    respond_id_ = command_respond.respond_id();
    status_ = command_respond.status();
    fail_code_ = command_respond.fail_code();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //发送指令的节点名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //发送节点的ID号，参考apps描述
  int8_t self_id_;
  int8_t *self_id_ptr_ = nullptr;
  int8_t respond_id_;
  int8_t *respond_id_ptr_ = nullptr;
  // 0 无响应 1 响应失败 2 响应成功
  int8_t status_;
  int8_t *status_ptr_ = nullptr;
  //失败原因
  int32_t fail_code_;
  int32_t *fail_code_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
