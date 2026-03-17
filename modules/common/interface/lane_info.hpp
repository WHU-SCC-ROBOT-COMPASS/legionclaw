/**
 * @file    lane_info.hpp
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

#include "modules/common/interface/lane_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LaneInfo {
public:
  LaneInfo() {
    lane_points_mutex_ = std::make_shared<std::mutex>();

    priority_ = 0;
    global_id_ = 0;
    predecessor_id_ = 0;
    successor_id_ = 0;
    left_neighbor_id_ = 0;
    right_neighbor_id_ = 0;
    type_ = 0;
    clear_lane_points();
  }
  ~LaneInfo() = default;

  inline void set_priority(const int8_t &priority) {
    priority_ = priority;
    priority_ptr_ = &priority_;
  }

  inline const int8_t &priority() const { return priority_; }

  inline int8_t *mutable_priority() { return &priority_; }

  inline bool has_priority() { return (priority_ptr_ != nullptr); }

  inline void set_global_id(const int8_t &global_id) {
    global_id_ = global_id;
    global_id_ptr_ = &global_id_;
  }

  inline const int8_t &global_id() const { return global_id_; }

  inline int8_t *mutable_global_id() { return &global_id_; }

  inline bool has_global_id() { return (global_id_ptr_ != nullptr); }

  inline void set_predecessor_id(const int8_t &predecessor_id) {
    predecessor_id_ = predecessor_id;
    predecessor_id_ptr_ = &predecessor_id_;
  }

  inline const int8_t &predecessor_id() const { return predecessor_id_; }

  inline int8_t *mutable_predecessor_id() { return &predecessor_id_; }

  inline bool has_predecessor_id() { return (predecessor_id_ptr_ != nullptr); }

  inline void set_successor_id(const int8_t &successor_id) {
    successor_id_ = successor_id;
    successor_id_ptr_ = &successor_id_;
  }

  inline const int8_t &successor_id() const { return successor_id_; }

  inline int8_t *mutable_successor_id() { return &successor_id_; }

  inline bool has_successor_id() { return (successor_id_ptr_ != nullptr); }

  inline void set_left_neighbor_id(const int8_t &left_neighbor_id) {
    left_neighbor_id_ = left_neighbor_id;
    left_neighbor_id_ptr_ = &left_neighbor_id_;
  }

  inline const int8_t &left_neighbor_id() const { return left_neighbor_id_; }

  inline int8_t *mutable_left_neighbor_id() { return &left_neighbor_id_; }

  inline bool has_left_neighbor_id() {
    return (left_neighbor_id_ptr_ != nullptr);
  }

  inline void set_right_neighbor_id(const int8_t &right_neighbor_id) {
    right_neighbor_id_ = right_neighbor_id;
    right_neighbor_id_ptr_ = &right_neighbor_id_;
  }

  inline const int8_t &right_neighbor_id() const { return right_neighbor_id_; }

  inline int8_t *mutable_right_neighbor_id() { return &right_neighbor_id_; }

  inline bool has_right_neighbor_id() {
    return (right_neighbor_id_ptr_ != nullptr);
  }

  inline void set_type(const int8_t &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const int8_t &type() const { return type_; }

  inline int8_t *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void
  set_lane_points(std::vector<legionclaw::interface::LanePoint> *lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_.assign(lane_points->begin(), lane_points->end());
  }

  inline void
  set_lane_points(const std::vector<legionclaw::interface::LanePoint> &lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_ = lane_points;
  }

  inline void set_lane_points(const uint32_t index,
                              legionclaw::interface::LanePoint &lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_[index] = lane_points;
  }

  inline void add_lane_points(const legionclaw::interface::LanePoint &lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_.emplace_back(lane_points);
  }

  inline const legionclaw::interface::LanePoint &lane_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return lane_points_[index];
  }

  inline std::vector<legionclaw::interface::LanePoint> *mutable_lane_points() {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return &lane_points_;
  }

  inline void
  lane_points(std::vector<legionclaw::interface::LanePoint> &lane_points) const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points.assign(lane_points_.begin(), lane_points_.end());
  }

  inline const std::vector<legionclaw::interface::LanePoint> &lane_points() const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return lane_points_;
  }

  inline uint32_t lane_points_size() const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return lane_points_.size();
  }

  inline void clear_lane_points() {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_.clear();
    lane_points_.shrink_to_fit();
  }

  inline bool has_lane_points() { return (lane_points_size() != 0); }

  void operator=(const LaneInfo &lane_info) { CopyFrom(lane_info); }

  void CopyFrom(const LaneInfo &lane_info) {
    priority_ = lane_info.priority();
    global_id_ = lane_info.global_id();
    predecessor_id_ = lane_info.predecessor_id();
    successor_id_ = lane_info.successor_id();
    left_neighbor_id_ = lane_info.left_neighbor_id();
    right_neighbor_id_ = lane_info.right_neighbor_id();
    type_ = lane_info.type();
    lane_points_ = lane_info.lane_points();
  }

protected:
  std::shared_ptr<std::mutex> lane_points_mutex_;
  //道路优先级：0（非推荐车道）1（推荐车道）
  int8_t priority_;
  int8_t *priority_ptr_ = nullptr;
  //道路全局id（从左至右，从0开始）
  int8_t global_id_;
  int8_t *global_id_ptr_ = nullptr;
  //上一车道id
  int8_t predecessor_id_;
  int8_t *predecessor_id_ptr_ = nullptr;
  //下一车道id
  int8_t successor_id_;
  int8_t *successor_id_ptr_ = nullptr;
  //左边相邻车道id
  int8_t left_neighbor_id_;
  int8_t *left_neighbor_id_ptr_ = nullptr;
  //右边相邻车道id
  int8_t right_neighbor_id_;
  int8_t *right_neighbor_id_ptr_ = nullptr;
  // 0：未知 1：左转道不可掉头 2：直行道 3：右转道 4：直行且可左转道
  // 5：直行且可右转道 6：左转道且可掉头 7：不可左转仅可掉头
  int8_t type_;
  int8_t *type_ptr_ = nullptr;
  //道路参考线（z）
  std::vector<legionclaw::interface::LanePoint> lane_points_;
};
} // namespace interface
} // namespace legionclaw
