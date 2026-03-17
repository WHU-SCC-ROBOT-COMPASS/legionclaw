/**
 * @file    end_points.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/point_2d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class EndPoints {
public:
  EndPoints() = default;
  ~EndPoints() = default;

  inline void set_start(const legionclaw::interface::Point2D &start) {
    start_ = start;
    start_ptr_ = &start_;
  }

  inline const legionclaw::interface::Point2D &start() const { return start_; }

  inline legionclaw::interface::Point2D *mutable_start() { return &start_; }

  inline bool has_start() { return (start_ptr_ != nullptr); }

  inline void set_end(const legionclaw::interface::Point2D &end) {
    end_ = end;
    end_ptr_ = &end_;
  }

  inline const legionclaw::interface::Point2D &end() const { return end_; }

  inline legionclaw::interface::Point2D *mutable_end() { return &end_; }

  inline bool has_end() { return (end_ptr_ != nullptr); }

  void operator=(const EndPoints &end_points) { CopyFrom(end_points); }

  void CopyFrom(const EndPoints &end_points) {
    start_ = end_points.start();
    end_ = end_points.end();
  }

protected:
  //车道线上顶点
  legionclaw::interface::Point2D start_;
  legionclaw::interface::Point2D *start_ptr_ = nullptr;
  //车道线下顶点
  legionclaw::interface::Point2D end_;
  legionclaw::interface::Point2D *end_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
