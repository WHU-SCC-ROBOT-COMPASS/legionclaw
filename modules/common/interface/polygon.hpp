/**
 * @file    polygon.hpp
 * @author  zdhy
 * @date    2021-07-25
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

#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Polygon {
public:
  Polygon() { points_mutex_ = std::make_shared<std::mutex>(); }
  ~Polygon() = default;

  inline void set_points(std::vector<legionclaw::interface::Point3D> *points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.assign(points->begin(), points->end());
  }

  inline void set_points(uint32_t index, legionclaw::interface::Point3D points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_[index] = points;
  }

  inline void add_points(legionclaw::interface::Point3D points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.emplace_back(points);
  }

  inline legionclaw::interface::Point3D points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return &points_;
  }

  inline void points(std::vector<legionclaw::interface::Point3D> &points) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points.assign(points_.begin(), points_.end());
  }

  inline uint32_t points_size() const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_.size();
  }

  inline void clear_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.clear();
  }

protected:
  std::shared_ptr<std::mutex> points_mutex_;
  std::vector<legionclaw::interface::Point3D> points_;
};
} // namespace interface
} // namespace legionclaw
