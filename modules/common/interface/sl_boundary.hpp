/**
 * @file    sl_boundary.hpp
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

#include "modules/common/interface/sl_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class SLBoundary {
public:
  SLBoundary() {
    boundary_point_mutex_ = std::make_shared<std::mutex>();

    start_s_ = 0.0;
    end_s_ = 0.0;
    start_l_ = 0.0;
    end_l_ = 0.0;
    clear_boundary_point();
  }
  ~SLBoundary() = default;

  inline void set_start_s(const double &start_s) {
    start_s_ = start_s;
    start_s_ptr_ = &start_s_;
  }

  inline const double &start_s() const { return start_s_; }

  inline double *mutable_start_s() { return &start_s_; }

  inline bool has_start_s() { return (start_s_ptr_ != nullptr); }

  inline void set_end_s(const double &end_s) {
    end_s_ = end_s;
    end_s_ptr_ = &end_s_;
  }

  inline const double &end_s() const { return end_s_; }

  inline double *mutable_end_s() { return &end_s_; }

  inline bool has_end_s() { return (end_s_ptr_ != nullptr); }

  inline void set_start_l(const double &start_l) {
    start_l_ = start_l;
    start_l_ptr_ = &start_l_;
  }

  inline const double &start_l() const { return start_l_; }

  inline double *mutable_start_l() { return &start_l_; }

  inline bool has_start_l() { return (start_l_ptr_ != nullptr); }

  inline void set_end_l(const double &end_l) {
    end_l_ = end_l;
    end_l_ptr_ = &end_l_;
  }

  inline const double &end_l() const { return end_l_; }

  inline double *mutable_end_l() { return &end_l_; }

  inline bool has_end_l() { return (end_l_ptr_ != nullptr); }

  inline void
  set_boundary_point(std::vector<legionclaw::interface::SLPoint> *boundary_point) {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point_.assign(boundary_point->begin(), boundary_point->end());
  }

  inline void set_boundary_point(
      const std::vector<legionclaw::interface::SLPoint> &boundary_point) {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point_ = boundary_point;
  }

  inline void set_boundary_point(const uint32_t index,
                                 legionclaw::interface::SLPoint &boundary_point) {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point_[index] = boundary_point;
  }

  inline void
  add_boundary_point(const legionclaw::interface::SLPoint &boundary_point) {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point_.emplace_back(boundary_point);
  }

  inline const legionclaw::interface::SLPoint &boundary_point(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    return boundary_point_[index];
  }

  inline std::vector<legionclaw::interface::SLPoint> *mutable_boundary_point() {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    return &boundary_point_;
  }

  inline void
  boundary_point(std::vector<legionclaw::interface::SLPoint> &boundary_point) const {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point.assign(boundary_point_.begin(), boundary_point_.end());
  }

  inline const std::vector<legionclaw::interface::SLPoint> &boundary_point() const {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    return boundary_point_;
  }

  inline uint32_t boundary_point_size() const {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    return boundary_point_.size();
  }

  inline void clear_boundary_point() {
    std::lock_guard<std::mutex> lock(*boundary_point_mutex_);
    boundary_point_.clear();
    boundary_point_.shrink_to_fit();
  }

  inline bool has_boundary_point() { return (boundary_point_size() != 0); }

  void operator=(const SLBoundary &sl_boundary) { CopyFrom(sl_boundary); }

  void CopyFrom(const SLBoundary &sl_boundary) {
    start_s_ = sl_boundary.start_s();
    end_s_ = sl_boundary.end_s();
    start_l_ = sl_boundary.start_l();
    end_l_ = sl_boundary.end_l();
    boundary_point_ = sl_boundary.boundary_point();
  }

protected:
  std::shared_ptr<std::mutex> boundary_point_mutex_;
  //起始点纵向位移
  double start_s_;
  double *start_s_ptr_ = nullptr;
  //终止点纵向位移
  double end_s_;
  double *end_s_ptr_ = nullptr;
  //起始点横向位移
  double start_l_;
  double *start_l_ptr_ = nullptr;
  //终止点横向位移
  double end_l_;
  double *end_l_ptr_ = nullptr;
  // sl点集
  std::vector<legionclaw::interface::SLPoint> boundary_point_;
};
} // namespace interface
} // namespace legionclaw
