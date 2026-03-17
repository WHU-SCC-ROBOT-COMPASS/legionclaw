/**
 * @file    hmi_trajectory.hpp
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
#include "modules/common/interface/hmi_trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HMITrajectory {
public:
  HMITrajectory() {
    trajectory_points_mutex_ = std::make_shared<std::mutex>();

    clear_trajectory_points();
  }
  ~HMITrajectory() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_trajectory_points(
      std::vector<legionclaw::interface::HMITrajectoryPoint> *trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.assign(trajectory_points->begin(),
                              trajectory_points->end());
  }

  inline void
  set_trajectory_points(const std::vector<legionclaw::interface::HMITrajectoryPoint>
                            &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_ = trajectory_points;
  }

  inline void set_trajectory_points(
      const uint32_t index,
      legionclaw::interface::HMITrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_[index] = trajectory_points;
  }

  inline void add_trajectory_points(
      const legionclaw::interface::HMITrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.emplace_back(trajectory_points);
  }

  inline const legionclaw::interface::HMITrajectoryPoint &
  trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::HMITrajectoryPoint> *
  mutable_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return &trajectory_points_;
  }

  inline void trajectory_points(
      std::vector<legionclaw::interface::HMITrajectoryPoint> &trajectory_points)
      const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points.assign(trajectory_points_.begin(),
                             trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::HMITrajectoryPoint> &
  trajectory_points() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_;
  }

  inline uint32_t trajectory_points_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_.size();
  }

  inline void clear_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.clear();
    trajectory_points_.shrink_to_fit();
  }

  inline bool has_trajectory_points() {
    return (trajectory_points_size() != 0);
  }

  void operator=(const HMITrajectory &hmi_trajectory) {
    CopyFrom(hmi_trajectory);
  }

  void CopyFrom(const HMITrajectory &hmi_trajectory) {
    header_ = hmi_trajectory.header();
    trajectory_points_ = hmi_trajectory.trajectory_points();
  }

protected:
  std::shared_ptr<std::mutex> trajectory_points_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //抽稀用来显示的轨迹点数组
  std::vector<legionclaw::interface::HMITrajectoryPoint> trajectory_points_;
};
} // namespace interface
} // namespace legionclaw
