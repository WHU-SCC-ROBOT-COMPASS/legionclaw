/**
 * @file    stop_info.hpp
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

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/stop_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class StopInfo {
public:
  StopInfo() {
    stop_points_mutex_ = std::make_shared<std::mutex>();

    clear_stop_points();
  }
  ~StopInfo() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_stop_points(std::vector<legionclaw::interface::StopPoint> *stop_points) {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points_.assign(stop_points->begin(), stop_points->end());
  }

  inline void
  set_stop_points(const std::vector<legionclaw::interface::StopPoint> &stop_points) {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points_ = stop_points;
  }

  inline void set_stop_points(const uint32_t index,
                              legionclaw::interface::StopPoint &stop_points) {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points_[index] = stop_points;
  }

  inline void add_stop_points(const legionclaw::interface::StopPoint &stop_points) {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points_.emplace_back(stop_points);
  }

  inline const legionclaw::interface::StopPoint &stop_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    return stop_points_[index];
  }

  inline std::vector<legionclaw::interface::StopPoint> *mutable_stop_points() {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    return &stop_points_;
  }

  inline void
  stop_points(std::vector<legionclaw::interface::StopPoint> &stop_points) const {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points.assign(stop_points_.begin(), stop_points_.end());
  }

  inline const std::vector<legionclaw::interface::StopPoint> &stop_points() const {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    return stop_points_;
  }

  inline uint32_t stop_points_size() const {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    return stop_points_.size();
  }

  inline void clear_stop_points() {
    std::lock_guard<std::mutex> lock(*stop_points_mutex_);
    stop_points_.clear();
    stop_points_.shrink_to_fit();
  }

  inline bool has_stop_points() { return (stop_points_size() != 0); }

  void operator=(const StopInfo &stop_info) { CopyFrom(stop_info); }

  void CopyFrom(const StopInfo &stop_info) {
    header_ = stop_info.header();
    stop_points_ = stop_info.stop_points();
  }

protected:
  std::shared_ptr<std::mutex> stop_points_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //停车点列表
  std::vector<legionclaw::interface::StopPoint> stop_points_;
};
} // namespace interface
} // namespace legionclaw
