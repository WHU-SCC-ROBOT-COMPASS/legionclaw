/**
 * @file    visualize_hybrid.hpp
 * @author  zdhy
 * @date    2022-05-06
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

#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VisualizeHybrid {
public:
  VisualizeHybrid() {
    points_mutex_ = std::make_shared<std::mutex>();
    s_pathpoints_mutex_ = std::make_shared<std::mutex>();
    trajectory_points_mutex_ = std::make_shared<std::mutex>();

    clear_points();
    clear_s_pathpoints();
    f_num_ = 0;
    s_num_ = 0;
    clear_trajectory_points();
  }
  ~VisualizeHybrid() = default;

  inline void set_points(std::vector<legionclaw::interface::Point3D> *points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.assign(points->begin(), points->end());
  }

  inline void set_points(const std::vector<legionclaw::interface::Point3D> &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_ = points;
  }

  inline void set_points(const uint32_t index,
                         legionclaw::interface::Point3D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_[index] = points;
  }

  inline void add_points(const legionclaw::interface::Point3D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.emplace_back(points);
  }

  inline const legionclaw::interface::Point3D &points(uint32_t index) const {
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

  inline const std::vector<legionclaw::interface::Point3D> &points() const {
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
  }

  inline bool has_points() { return (points_size() != 0); }

  inline void
  set_s_pathpoints(std::vector<legionclaw::interface::PathPoint> *s_pathpoints) {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints_.assign(s_pathpoints->begin(), s_pathpoints->end());
  }

  inline void set_s_pathpoints(
      const std::vector<legionclaw::interface::PathPoint> &s_pathpoints) {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints_ = s_pathpoints;
  }

  inline void set_s_pathpoints(const uint32_t index,
                               legionclaw::interface::PathPoint &s_pathpoints) {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints_[index] = s_pathpoints;
  }

  inline void
  add_s_pathpoints(const legionclaw::interface::PathPoint &s_pathpoints) {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints_.emplace_back(s_pathpoints);
  }

  inline const legionclaw::interface::PathPoint &s_pathpoints(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    return s_pathpoints_[index];
  }

  inline std::vector<legionclaw::interface::PathPoint> *mutable_s_pathpoints() {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    return &s_pathpoints_;
  }

  inline void
  s_pathpoints(std::vector<legionclaw::interface::PathPoint> &s_pathpoints) const {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints.assign(s_pathpoints_.begin(), s_pathpoints_.end());
  }

  inline const std::vector<legionclaw::interface::PathPoint> &s_pathpoints() const {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    return s_pathpoints_;
  }

  inline uint32_t s_pathpoints_size() const {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    return s_pathpoints_.size();
  }

  inline void clear_s_pathpoints() {
    std::lock_guard<std::mutex> lock(*s_pathpoints_mutex_);
    s_pathpoints_.clear();
  }

  inline bool has_s_pathpoints() { return (s_pathpoints_size() != 0); }

  inline void set_f_num(const int32_t &f_num) {
    f_num_ = f_num;
    f_num_ptr_ = &f_num_;
  }

  inline const int32_t &f_num() const { return f_num_; }

  inline int32_t *mutable_f_num() { return &f_num_; }

  inline bool has_f_num() { return (f_num_ptr_ != nullptr); }

  inline void set_s_num(const int32_t &s_num) {
    s_num_ = s_num;
    s_num_ptr_ = &s_num_;
  }

  inline const int32_t &s_num() const { return s_num_; }

  inline int32_t *mutable_s_num() { return &s_num_; }

  inline bool has_s_num() { return (s_num_ptr_ != nullptr); }

  inline void set_trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> *trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.assign(trajectory_points->begin(),
                              trajectory_points->end());
  }

  inline void set_trajectory_points(
      const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_ = trajectory_points;
  }

  inline void
  set_trajectory_points(const uint32_t index,
                        legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_[index] = trajectory_points;
  }

  inline void add_trajectory_points(
      const legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.emplace_back(trajectory_points);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return &trajectory_points_;
  }

  inline void trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points.assign(trajectory_points_.begin(),
                             trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
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
  }

  inline bool has_trajectory_points() {
    return (trajectory_points_size() != 0);
  }

  void operator=(const VisualizeHybrid &visualize_hybrid) {
    CopyFrom(visualize_hybrid);
  }

  void CopyFrom(const VisualizeHybrid &visualize_hybrid) {
    points_ = visualize_hybrid.points();
    s_pathpoints_ = visualize_hybrid.s_pathpoints();
    f_num_ = visualize_hybrid.f_num();
    s_num_ = visualize_hybrid.s_num();
    trajectory_points_ = visualize_hybrid.trajectory_points();
  }

protected:
  std::shared_ptr<std::mutex> points_mutex_;
  std::shared_ptr<std::mutex> s_pathpoints_mutex_;
  std::shared_ptr<std::mutex> trajectory_points_mutex_;
  //反向搜索点pose
  std::vector<legionclaw::interface::Point3D> points_;
  //二次搜索点pose
  std::vector<legionclaw::interface::PathPoint> s_pathpoints_;
  //初次点数目
  int32_t f_num_;
  int32_t *f_num_ptr_ = nullptr;
  //初次点搜索次数
  int32_t s_num_;
  int32_t *s_num_ptr_ = nullptr;
  //轨迹点列
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points_;
};
} // namespace interface
} // namespace legionclaw
