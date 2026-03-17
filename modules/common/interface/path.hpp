/**
 * @file    path.hpp
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
#include <string>
#include <vector>

#include "modules/common/interface/path_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Path {
public:
  Path() {
    path_points_mutex_ = std::make_shared<std::mutex>();

    name_.clear();
    clear_path_points();
  }
  ~Path() = default;

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void
  set_path_points(std::vector<legionclaw::interface::PathPoint> *path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.assign(path_points->begin(), path_points->end());
  }

  inline void
  set_path_points(const std::vector<legionclaw::interface::PathPoint> &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_ = path_points;
  }

  inline void set_path_points(const uint32_t index,
                              legionclaw::interface::PathPoint &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_[index] = path_points;
  }

  inline void add_path_points(const legionclaw::interface::PathPoint &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.emplace_back(path_points);
  }

  inline const legionclaw::interface::PathPoint &path_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_[index];
  }

  inline std::vector<legionclaw::interface::PathPoint> *mutable_path_points() {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return &path_points_;
  }

  inline void
  path_points(std::vector<legionclaw::interface::PathPoint> &path_points) const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points.assign(path_points_.begin(), path_points_.end());
  }

  inline const std::vector<legionclaw::interface::PathPoint> &path_points() const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_;
  }

  inline uint32_t path_points_size() const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_.size();
  }

  inline void clear_path_points() {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.clear();
    path_points_.shrink_to_fit();
  }

  inline bool has_path_points() { return (path_points_size() != 0); }

  void operator=(const Path &path) { CopyFrom(path); }

  void CopyFrom(const Path &path) {
    name_ = path.name();
    path_points_ = path.path_points();
  }

protected:
  std::shared_ptr<std::mutex> path_points_mutex_;
  //名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //路径点集
  std::vector<legionclaw::interface::PathPoint> path_points_;
};
} // namespace interface
} // namespace legionclaw
