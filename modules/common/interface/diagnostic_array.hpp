/**
 * @file    diagnostic_array.hpp
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

#include "modules/common/interface/diagnostic_status.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class DiagnosticArray {
public:
  DiagnosticArray() {
    status_mutex_ = std::make_shared<std::mutex>();

    clear_status();
  }
  ~DiagnosticArray() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_status(std::vector<legionclaw::interface::DiagnosticStatus> *status) {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status_.assign(status->begin(), status->end());
  }

  inline void
  set_status(const std::vector<legionclaw::interface::DiagnosticStatus> &status) {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status_ = status;
  }

  inline void set_status(const uint32_t index,
                         legionclaw::interface::DiagnosticStatus &status) {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status_[index] = status;
  }

  inline void add_status(const legionclaw::interface::DiagnosticStatus &status) {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status_.emplace_back(status);
  }

  inline const legionclaw::interface::DiagnosticStatus &
  status(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    return status_[index];
  }

  inline std::vector<legionclaw::interface::DiagnosticStatus> *mutable_status() {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    return &status_;
  }

  inline void
  status(std::vector<legionclaw::interface::DiagnosticStatus> &status) const {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status.assign(status_.begin(), status_.end());
  }

  inline const std::vector<legionclaw::interface::DiagnosticStatus> &status() const {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    return status_;
  }

  inline uint32_t status_size() const {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    return status_.size();
  }

  inline void clear_status() {
    std::lock_guard<std::mutex> lock(*status_mutex_);
    status_.clear();
    status_.shrink_to_fit();
  }

  inline bool has_status() { return (status_size() != 0); }

  void operator=(const DiagnosticArray &diagnostic_array) {
    CopyFrom(diagnostic_array);
  }

  void CopyFrom(const DiagnosticArray &diagnostic_array) {
    header_ = diagnostic_array.header();
    status_ = diagnostic_array.status();
  }

protected:
  std::shared_ptr<std::mutex> status_mutex_;
  // timestamp is  included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //诊断状态
  std::vector<legionclaw::interface::DiagnosticStatus> status_;
};
} // namespace interface
} // namespace legionclaw
