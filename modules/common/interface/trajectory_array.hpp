/**
 * @file    trajectory_array.hpp
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
#include "modules/common/interface/path.hpp"
#include "modules/common/interface/trajectory.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrajectoryArray {
public:
  TrajectoryArray() {
    spline_s_mutex_ = std::make_shared<std::mutex>();
    qp_smooth_mutex_ = std::make_shared<std::mutex>();
    trajectory_list_mutex_ = std::make_shared<std::mutex>();

    clear_spline_s();
    clear_qp_smooth();
    clear_trajectory_list();
  }
  ~TrajectoryArray() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_spline_s(std::vector<legionclaw::interface::Path> *spline_s) {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s_.assign(spline_s->begin(), spline_s->end());
  }

  inline void
  set_spline_s(const std::vector<legionclaw::interface::Path> &spline_s) {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s_ = spline_s;
  }

  inline void set_spline_s(const uint32_t index,
                           legionclaw::interface::Path &spline_s) {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s_[index] = spline_s;
  }

  inline void add_spline_s(const legionclaw::interface::Path &spline_s) {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s_.emplace_back(spline_s);
  }

  inline const legionclaw::interface::Path &spline_s(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    return spline_s_[index];
  }

  inline std::vector<legionclaw::interface::Path> *mutable_spline_s() {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    return &spline_s_;
  }

  inline void spline_s(std::vector<legionclaw::interface::Path> &spline_s) const {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s.assign(spline_s_.begin(), spline_s_.end());
  }

  inline const std::vector<legionclaw::interface::Path> &spline_s() const {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    return spline_s_;
  }

  inline uint32_t spline_s_size() const {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    return spline_s_.size();
  }

  inline void clear_spline_s() {
    std::lock_guard<std::mutex> lock(*spline_s_mutex_);
    spline_s_.clear();
    spline_s_.shrink_to_fit();
  }

  inline bool has_spline_s() { return (spline_s_size() != 0); }

  inline void set_qp_smooth(std::vector<legionclaw::interface::Path> *qp_smooth) {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth_.assign(qp_smooth->begin(), qp_smooth->end());
  }

  inline void
  set_qp_smooth(const std::vector<legionclaw::interface::Path> &qp_smooth) {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth_ = qp_smooth;
  }

  inline void set_qp_smooth(const uint32_t index,
                            legionclaw::interface::Path &qp_smooth) {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth_[index] = qp_smooth;
  }

  inline void add_qp_smooth(const legionclaw::interface::Path &qp_smooth) {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth_.emplace_back(qp_smooth);
  }

  inline const legionclaw::interface::Path &qp_smooth(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    return qp_smooth_[index];
  }

  inline std::vector<legionclaw::interface::Path> *mutable_qp_smooth() {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    return &qp_smooth_;
  }

  inline void qp_smooth(std::vector<legionclaw::interface::Path> &qp_smooth) const {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth.assign(qp_smooth_.begin(), qp_smooth_.end());
  }

  inline const std::vector<legionclaw::interface::Path> &qp_smooth() const {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    return qp_smooth_;
  }

  inline uint32_t qp_smooth_size() const {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    return qp_smooth_.size();
  }

  inline void clear_qp_smooth() {
    std::lock_guard<std::mutex> lock(*qp_smooth_mutex_);
    qp_smooth_.clear();
    qp_smooth_.shrink_to_fit();
  }

  inline bool has_qp_smooth() { return (qp_smooth_size() != 0); }

  inline void set_trajectory_list(
      std::vector<legionclaw::interface::Trajectory> *trajectory_list) {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list_.assign(trajectory_list->begin(), trajectory_list->end());
  }

  inline void set_trajectory_list(
      const std::vector<legionclaw::interface::Trajectory> &trajectory_list) {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list_ = trajectory_list;
  }

  inline void
  set_trajectory_list(const uint32_t index,
                      legionclaw::interface::Trajectory &trajectory_list) {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list_[index] = trajectory_list;
  }

  inline void
  add_trajectory_list(const legionclaw::interface::Trajectory &trajectory_list) {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list_.emplace_back(trajectory_list);
  }

  inline const legionclaw::interface::Trajectory &
  trajectory_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    return trajectory_list_[index];
  }

  inline std::vector<legionclaw::interface::Trajectory> *mutable_trajectory_list() {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    return &trajectory_list_;
  }

  inline void trajectory_list(
      std::vector<legionclaw::interface::Trajectory> &trajectory_list) const {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list.assign(trajectory_list_.begin(), trajectory_list_.end());
  }

  inline const std::vector<legionclaw::interface::Trajectory> &
  trajectory_list() const {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    return trajectory_list_;
  }

  inline uint32_t trajectory_list_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    return trajectory_list_.size();
  }

  inline void clear_trajectory_list() {
    std::lock_guard<std::mutex> lock(*trajectory_list_mutex_);
    trajectory_list_.clear();
    trajectory_list_.shrink_to_fit();
  }

  inline bool has_trajectory_list() { return (trajectory_list_size() != 0); }

  void operator=(const TrajectoryArray &trajectory_array) {
    CopyFrom(trajectory_array);
  }

  void CopyFrom(const TrajectoryArray &trajectory_array) {
    header_ = trajectory_array.header();
    spline_s_ = trajectory_array.spline_s();
    qp_smooth_ = trajectory_array.qp_smooth();
    trajectory_list_ = trajectory_array.trajectory_list();
  }

protected:
  std::shared_ptr<std::mutex> spline_s_mutex_;
  std::shared_ptr<std::mutex> qp_smooth_mutex_;
  std::shared_ptr<std::mutex> trajectory_list_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //样条拟合曲线
  std::vector<legionclaw::interface::Path> spline_s_;
  // qp参考线拟合
  std::vector<legionclaw::interface::Path> qp_smooth_;
  //轨迹簇
  std::vector<legionclaw::interface::Trajectory> trajectory_list_;
};
} // namespace interface
} // namespace legionclaw
