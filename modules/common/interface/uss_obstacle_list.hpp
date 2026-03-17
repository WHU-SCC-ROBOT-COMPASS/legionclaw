/**
 * @file    uss_obstacle_list.hpp
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

#include "modules/common/enum/enum.h"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/uss_obstacle.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class UssObstacleList {
public:
  UssObstacleList() {
    uss_obstacles_mutex_ = std::make_shared<std::mutex>();
    uss_ranges_mutex_ = std::make_shared<std::mutex>();

    clear_uss_obstacles();
    clear_uss_ranges();
    error_code_ = legionclaw::common::ErrorCode::LOCALIZATION_ERROR;
    is_valid_ = false;
  }
  ~UssObstacleList() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_uss_obstacles(std::vector<legionclaw::interface::UssObstacle> *uss_obstacles) {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles_.assign(uss_obstacles->begin(), uss_obstacles->end());
  }

  inline void set_uss_obstacles(
      const std::vector<legionclaw::interface::UssObstacle> &uss_obstacles) {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles_ = uss_obstacles;
  }

  inline void set_uss_obstacles(const uint32_t index,
                                legionclaw::interface::UssObstacle &uss_obstacles) {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles_[index] = uss_obstacles;
  }

  inline void
  add_uss_obstacles(const legionclaw::interface::UssObstacle &uss_obstacles) {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles_.emplace_back(uss_obstacles);
  }

  inline const legionclaw::interface::UssObstacle &
  uss_obstacles(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    return uss_obstacles_[index];
  }

  inline std::vector<legionclaw::interface::UssObstacle> *mutable_uss_obstacles() {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    return &uss_obstacles_;
  }

  inline void uss_obstacles(
      std::vector<legionclaw::interface::UssObstacle> &uss_obstacles) const {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles.assign(uss_obstacles_.begin(), uss_obstacles_.end());
  }

  inline const std::vector<legionclaw::interface::UssObstacle> &
  uss_obstacles() const {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    return uss_obstacles_;
  }

  inline uint32_t uss_obstacles_size() const {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    return uss_obstacles_.size();
  }

  inline void clear_uss_obstacles() {
    std::lock_guard<std::mutex> lock(*uss_obstacles_mutex_);
    uss_obstacles_.clear();
    uss_obstacles_.shrink_to_fit();
  }

  inline bool has_uss_obstacles() { return (uss_obstacles_size() != 0); }

  inline void set_uss_ranges(std::vector<double> *uss_ranges) {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges_.assign(uss_ranges->begin(), uss_ranges->end());
  }

  inline void set_uss_ranges(const std::vector<double> &uss_ranges) {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges_ = uss_ranges;
  }

  inline void set_uss_ranges(const uint32_t index, double &uss_ranges) {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges_[index] = uss_ranges;
  }

  inline void add_uss_ranges(const double &uss_ranges) {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges_.emplace_back(uss_ranges);
  }

  inline const double &uss_ranges(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    return uss_ranges_[index];
  }

  inline std::vector<double> *mutable_uss_ranges() {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    return &uss_ranges_;
  }

  inline void uss_ranges(std::vector<double> &uss_ranges) const {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges.assign(uss_ranges_.begin(), uss_ranges_.end());
  }

  inline const std::vector<double> &uss_ranges() const {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    return uss_ranges_;
  }

  inline uint32_t uss_ranges_size() const {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    return uss_ranges_.size();
  }

  inline void clear_uss_ranges() {
    std::lock_guard<std::mutex> lock(*uss_ranges_mutex_);
    uss_ranges_.clear();
    uss_ranges_.shrink_to_fit();
  }

  inline bool has_uss_ranges() { return (uss_ranges_size() != 0); }

  inline void set_error_code(const legionclaw::common::ErrorCode &error_code) {
    error_code_ = error_code;
    error_code_ptr_ = &error_code_;
  }

  inline const legionclaw::common::ErrorCode &error_code() const {
    return error_code_;
  }

  inline legionclaw::common::ErrorCode *mutable_error_code() { return &error_code_; }

  inline bool has_error_code() { return (error_code_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  void operator=(const UssObstacleList &uss_obstacle_list) {
    CopyFrom(uss_obstacle_list);
  }

  void CopyFrom(const UssObstacleList &uss_obstacle_list) {
    header_ = uss_obstacle_list.header();
    uss_obstacles_ = uss_obstacle_list.uss_obstacles();
    uss_ranges_ = uss_obstacle_list.uss_ranges();
    error_code_ = uss_obstacle_list.error_code();
    is_valid_ = uss_obstacle_list.is_valid();
  }

protected:
  std::shared_ptr<std::mutex> uss_obstacles_mutex_;
  std::shared_ptr<std::mutex> uss_ranges_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //检测出的超声波障碍物数组
  std::vector<legionclaw::interface::UssObstacle> uss_obstacles_;
  //超声波检测的距离数组，存放顺序按照FA,FB,FC,FD,FE,FF,RA,RB,RC,RD,RE,RF
  std::vector<double> uss_ranges_;
  //错误码（default = OK）
  legionclaw::common::ErrorCode error_code_;
  legionclaw::common::ErrorCode *error_code_ptr_ = nullptr;
  //超声波障碍物数据是否合法
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
