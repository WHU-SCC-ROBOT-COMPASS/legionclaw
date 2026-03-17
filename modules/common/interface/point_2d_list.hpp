/**
 * @file    point_2d_list.hpp
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

#include "modules/common/interface/point_2d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Point2dList {
public:
  Point2dList() {
    point2d_list_mutex_ = std::make_shared<std::mutex>();

    clear_point2d_list();
  }
  ~Point2dList() = default;

  inline void
  set_point2d_list(std::vector<legionclaw::interface::Point2D> *point2d_list) {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list_.assign(point2d_list->begin(), point2d_list->end());
  }

  inline void
  set_point2d_list(const std::vector<legionclaw::interface::Point2D> &point2d_list) {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list_ = point2d_list;
  }

  inline void set_point2d_list(const uint32_t index,
                               legionclaw::interface::Point2D &point2d_list) {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list_[index] = point2d_list;
  }

  inline void add_point2d_list(const legionclaw::interface::Point2D &point2d_list) {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list_.emplace_back(point2d_list);
  }

  inline const legionclaw::interface::Point2D &point2d_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    return point2d_list_[index];
  }

  inline std::vector<legionclaw::interface::Point2D> *mutable_point2d_list() {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    return &point2d_list_;
  }

  inline void
  point2d_list(std::vector<legionclaw::interface::Point2D> &point2d_list) const {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list.assign(point2d_list_.begin(), point2d_list_.end());
  }

  inline const std::vector<legionclaw::interface::Point2D> &point2d_list() const {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    return point2d_list_;
  }

  inline uint32_t point2d_list_size() const {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    return point2d_list_.size();
  }

  inline void clear_point2d_list() {
    std::lock_guard<std::mutex> lock(*point2d_list_mutex_);
    point2d_list_.clear();
    point2d_list_.shrink_to_fit();
  }

  inline bool has_point2d_list() { return (point2d_list_size() != 0); }

  void operator=(const Point2dList &point_2d_list) { CopyFrom(point_2d_list); }

  void CopyFrom(const Point2dList &point_2d_list) {
    point2d_list_ = point_2d_list.point2d_list();
  }

protected:
  std::shared_ptr<std::mutex> point2d_list_mutex_;
  std::vector<legionclaw::interface::Point2D> point2d_list_;
};
} // namespace interface
} // namespace legionclaw
