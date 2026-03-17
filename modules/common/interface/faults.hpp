/**
 * @file    faults.hpp
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
#include <vector>

#include "modules/common/interface/fault.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Faults {
public:
  Faults() {
    faults_mutex_ = std::make_shared<std::mutex>();

    version_ = 0;
    app_id_ = 0;
    is_active_ = false;
    clear_faults();
  }
  ~Faults() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_version(const int32_t &version) {
    version_ = version;
    version_ptr_ = &version_;
  }

  inline const int32_t &version() const { return version_; }

  inline int32_t *mutable_version() { return &version_; }

  inline bool has_version() { return (version_ptr_ != nullptr); }

  inline void set_app_id(const int32_t &app_id) {
    app_id_ = app_id;
    app_id_ptr_ = &app_id_;
  }

  inline const int32_t &app_id() const { return app_id_; }

  inline int32_t *mutable_app_id() { return &app_id_; }

  inline bool has_app_id() { return (app_id_ptr_ != nullptr); }

  inline void set_is_active(const bool &is_active) {
    is_active_ = is_active;
    is_active_ptr_ = &is_active_;
  }

  inline const bool &is_active() const { return is_active_; }

  inline bool *mutable_is_active() { return &is_active_; }

  inline bool has_is_active() { return (is_active_ptr_ != nullptr); }

  inline void set_faults(std::vector<legionclaw::interface::Fault> *faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_.assign(faults->begin(), faults->end());
  }

  inline void set_faults(const std::vector<legionclaw::interface::Fault> &faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_ = faults;
  }

  inline void set_faults(const uint32_t index,
                         legionclaw::interface::Fault &faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_[index] = faults;
  }

  inline void add_faults(const legionclaw::interface::Fault &faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_.emplace_back(faults);
  }

  inline const legionclaw::interface::Fault &faults(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    return faults_[index];
  }

  inline std::vector<legionclaw::interface::Fault> *mutable_faults() {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    return &faults_;
  }

  inline void faults(std::vector<legionclaw::interface::Fault> &faults) const {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults.assign(faults_.begin(), faults_.end());
  }

  inline const std::vector<legionclaw::interface::Fault> &faults() const {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    return faults_;
  }

  inline uint32_t faults_size() const {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    return faults_.size();
  }

  inline void clear_faults() {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_.clear();
    faults_.shrink_to_fit();
  }

  inline bool has_faults() { return (faults_size() != 0); }

  void operator=(const Faults &faults) { CopyFrom(faults); }

  void CopyFrom(const Faults &faults) {
    header_ = faults.header();
    version_ = faults.version();
    app_id_ = faults.app_id();
    is_active_ = faults.is_active();
    faults_ = faults.faults();
  }

protected:
  std::shared_ptr<std::mutex> faults_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //版本号
  int32_t version_;
  int32_t *version_ptr_ = nullptr;
  //应用app所分配的id
  int32_t app_id_;
  int32_t *app_id_ptr_ = nullptr;
  //应用是否激活，激活为true，未激活为false
  bool is_active_;
  bool *is_active_ptr_ = nullptr;
  //故障集
  std::vector<legionclaw::interface::Fault> faults_;
};
} // namespace interface
} // namespace legionclaw
