/**
 * @file    comm_command.hpp
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

#include "modules/common/interface/command.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CommCommand {
public:
  CommCommand() {
    commands_mutex_ = std::make_shared<std::mutex>();

    name_.clear();
    self_id_ = 0;
    clear_commands();
  }
  ~CommCommand() = default;

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

  inline void set_commands(std::vector<legionclaw::interface::Command> *commands) {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands_.assign(commands->begin(), commands->end());
  }

  inline void
  set_commands(const std::vector<legionclaw::interface::Command> &commands) {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands_ = commands;
  }

  inline void set_commands(const uint32_t index,
                           legionclaw::interface::Command &commands) {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands_[index] = commands;
  }

  inline void add_commands(const legionclaw::interface::Command &commands) {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands_.emplace_back(commands);
  }

  inline const legionclaw::interface::Command &commands(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    return commands_[index];
  }

  inline std::vector<legionclaw::interface::Command> *mutable_commands() {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    return &commands_;
  }

  inline void commands(std::vector<legionclaw::interface::Command> &commands) const {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands.assign(commands_.begin(), commands_.end());
  }

  inline const std::vector<legionclaw::interface::Command> &commands() const {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    return commands_;
  }

  inline uint32_t commands_size() const {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    return commands_.size();
  }

  inline void clear_commands() {
    std::lock_guard<std::mutex> lock(*commands_mutex_);
    commands_.clear();
    commands_.shrink_to_fit();
  }

  inline bool has_commands() { return (commands_size() != 0); }

  void operator=(const CommCommand &comm_command) { CopyFrom(comm_command); }

  void CopyFrom(const CommCommand &comm_command) {
    header_ = comm_command.header();
    name_ = comm_command.name();
    self_id_ = comm_command.self_id();
    commands_ = comm_command.commands();
  }

protected:
  std::shared_ptr<std::mutex> commands_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //发送指令的节点名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //发送节点的ID号，参考apps描述
  int8_t self_id_;
  int8_t *self_id_ptr_ = nullptr;
  //需要发送的指令，如果需要发送给多个节点，需要使用多个command
  std::vector<legionclaw::interface::Command> commands_;
};
} // namespace interface
} // namespace legionclaw
