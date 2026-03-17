/**
 * @file    polygon_2d.hpp
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
#include "modules/common/interface/point_2d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Polygon2D {
public:
  Polygon2D() {
    points_mutex_ = std::make_shared<std::mutex>();

    coordinate_system_ =
        legionclaw::common::CoordinateSystem::INVALID_COORDINATE_SYSTEM;
    clear_points();
  }
  ~Polygon2D() = default;

  inline void set_coordinate_system(
      const legionclaw::common::CoordinateSystem &coordinate_system) {
    coordinate_system_ = coordinate_system;
    coordinate_system_ptr_ = &coordinate_system_;
  }

  inline const legionclaw::common::CoordinateSystem &coordinate_system() const {
    return coordinate_system_;
  }

  inline legionclaw::common::CoordinateSystem *mutable_coordinate_system() {
    return &coordinate_system_;
  }

  inline bool has_coordinate_system() {
    return (coordinate_system_ptr_ != nullptr);
  }

  inline void set_points(std::vector<legionclaw::interface::Point2D> *points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.assign(points->begin(), points->end());
  }

  inline void set_points(const std::vector<legionclaw::interface::Point2D> &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_ = points;
  }

  inline void set_points(const uint32_t index,
                         legionclaw::interface::Point2D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_[index] = points;
  }

  inline void add_points(const legionclaw::interface::Point2D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.emplace_back(points);
  }

  inline const legionclaw::interface::Point2D &points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_[index];
  }

  inline std::vector<legionclaw::interface::Point2D> *mutable_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return &points_;
  }

  inline void points(std::vector<legionclaw::interface::Point2D> &points) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points.assign(points_.begin(), points_.end());
  }

  inline const std::vector<legionclaw::interface::Point2D> &points() const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_;
  }

  inline uint32_t points_size() const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_.size();
  }

  inline void clear_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.clear();
    points_.shrink_to_fit();
  }

  inline bool has_points() { return (points_size() != 0); }

  void operator=(const Polygon2D &polygon_2d) { CopyFrom(polygon_2d); }

  void CopyFrom(const Polygon2D &polygon_2d) {
    coordinate_system_ = polygon_2d.coordinate_system();
    points_ = polygon_2d.points();
  }

protected:
  std::shared_ptr<std::mutex> points_mutex_;
  //坐标系
  legionclaw::common::CoordinateSystem coordinate_system_;
  legionclaw::common::CoordinateSystem *coordinate_system_ptr_ = nullptr;
  //二维点集
  std::vector<legionclaw::interface::Point2D> points_;
};
} // namespace interface
} // namespace legionclaw
