/**
 * @file    module_status.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/message.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ModuleStatus {
public:
  ModuleStatus() {
    messages_mutex_ = std::make_shared<std::mutex>();

    name_.clear();
    self_id_ = 0;
    status_ = 0;
    clear_messages();
  }
  ~ModuleStatus() = default;

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

  inline void set_status(const int32_t &status) {
    status_ = status;
    status_ptr_ = &status_;
  }

  inline const int32_t &status() const { return status_; }

  inline int32_t *mutable_status() { return &status_; }

  inline bool has_status() { return (status_ptr_ != nullptr); }

  inline void set_messages(std::vector<legionclaw::interface::Message> *messages) {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages_.assign(messages->begin(), messages->end());
  }

  inline void
  set_messages(const std::vector<legionclaw::interface::Message> &messages) {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages_ = messages;
  }

  inline void set_messages(const uint32_t index,
                           legionclaw::interface::Message &messages) {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages_[index] = messages;
  }

  inline void add_messages(const legionclaw::interface::Message &messages) {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages_.emplace_back(messages);
  }

  inline const legionclaw::interface::Message &messages(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    return messages_[index];
  }

  inline std::vector<legionclaw::interface::Message> *mutable_messages() {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    return &messages_;
  }

  inline void messages(std::vector<legionclaw::interface::Message> &messages) const {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages.assign(messages_.begin(), messages_.end());
  }

  inline const std::vector<legionclaw::interface::Message> &messages() const {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    return messages_;
  }

  inline uint32_t messages_size() const {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    return messages_.size();
  }

  inline void clear_messages() {
    std::lock_guard<std::mutex> lock(*messages_mutex_);
    messages_.clear();
    messages_.shrink_to_fit();
  }

  inline bool has_messages() { return (messages_size() != 0); }

  void operator=(const ModuleStatus &module_status) { CopyFrom(module_status); }

  void CopyFrom(const ModuleStatus &module_status) {
    header_ = module_status.header();
    name_ = module_status.name();
    self_id_ = module_status.self_id();
    status_ = module_status.status();
    messages_ = module_status.messages();
  }

protected:
  std::shared_ptr<std::mutex> messages_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //发送指令的节点名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //发送节点的ID号，参考apps描述
  int8_t self_id_;
  int8_t *self_id_ptr_ = nullptr;
  // 枚举方式
  // sef_id+status,比如设置1为开，2为关，当前sef_id是21，则在关闭状态下反馈值是
  // 212
  int32_t status_;
  int32_t *status_ptr_ = nullptr;
  //需要输出的消息列表：eg:要求显示剩余里程数为100米，self_id
  //为21，里程数id为08，code值为2108，value值为10000，精度为cm
  std::vector<legionclaw::interface::Message> messages_;
};
} // namespace interface
} // namespace legionclaw
