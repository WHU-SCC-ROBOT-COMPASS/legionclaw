/**
 * @file    obu_cmd_msg.hpp
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
#include "modules/common/interface/obu_cmd.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ObuCmdMsg {
public:
  ObuCmdMsg() {
    obu_cmd_list_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    name_.clear();
    clear_obu_cmd_list();
  }
  ~ObuCmdMsg() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void
  set_obu_cmd_list(std::vector<legionclaw::interface::ObuCmd> *obu_cmd_list) {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list_.assign(obu_cmd_list->begin(), obu_cmd_list->end());
  }

  inline void
  set_obu_cmd_list(const std::vector<legionclaw::interface::ObuCmd> &obu_cmd_list) {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list_ = obu_cmd_list;
  }

  inline void set_obu_cmd_list(const uint32_t index,
                               legionclaw::interface::ObuCmd &obu_cmd_list) {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list_[index] = obu_cmd_list;
  }

  inline void add_obu_cmd_list(const legionclaw::interface::ObuCmd &obu_cmd_list) {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list_.emplace_back(obu_cmd_list);
  }

  inline const legionclaw::interface::ObuCmd &obu_cmd_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    return obu_cmd_list_[index];
  }

  inline std::vector<legionclaw::interface::ObuCmd> *mutable_obu_cmd_list() {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    return &obu_cmd_list_;
  }

  inline void
  obu_cmd_list(std::vector<legionclaw::interface::ObuCmd> &obu_cmd_list) const {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list.assign(obu_cmd_list_.begin(), obu_cmd_list_.end());
  }

  inline const std::vector<legionclaw::interface::ObuCmd> &obu_cmd_list() const {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    return obu_cmd_list_;
  }

  inline uint32_t obu_cmd_list_size() const {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    return obu_cmd_list_.size();
  }

  inline void clear_obu_cmd_list() {
    std::lock_guard<std::mutex> lock(*obu_cmd_list_mutex_);
    obu_cmd_list_.clear();
    obu_cmd_list_.shrink_to_fit();
  }

  inline bool has_obu_cmd_list() { return (obu_cmd_list_size() != 0); }

  void operator=(const ObuCmdMsg &obu_cmd_msg) { CopyFrom(obu_cmd_msg); }

  void CopyFrom(const ObuCmdMsg &obu_cmd_msg) {
    header_ = obu_cmd_msg.header();
    id_ = obu_cmd_msg.id();
    name_ = obu_cmd_msg.name();
    obu_cmd_list_ = obu_cmd_msg.obu_cmd_list();
  }

protected:
  std::shared_ptr<std::mutex> obu_cmd_list_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // obu的id身份
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  // obu的名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //命令列表
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list_;
};
} // namespace interface
} // namespace legionclaw
