/**
 * @file    lane_info_extend.hpp
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
class LaneInfoExtend {
public:
  LaneInfoExtend() {
    predecessor_ids_mutex_ = std::make_shared<std::mutex>();
    successor_ids_mutex_ = std::make_shared<std::mutex>();
    lane_points_mutex_ = std::make_shared<std::mutex>();

    priority_ = 0;
    global_id_ = 0;
    clear_predecessor_ids();
    clear_successor_ids();
    left_neighbor_id_ = 0;
    right_neighbor_id_ = 0;
    type_ = 0;
    clear_lane_points();
  }
  ~LaneInfoExtend() = default;

  inline void set_priority(const int8_t &priority) {
    priority_ = priority;
    priority_ptr_ = &priority_;
  }

  inline const int8_t &priority() const { return priority_; }

  inline int8_t *mutable_priority() { return &priority_; }

  inline bool has_priority() { return (priority_ptr_ != nullptr); }

  inline void set_global_id(const int64_t &global_id) {
    global_id_ = global_id;
    global_id_ptr_ = &global_id_;
  }

  inline const int64_t &global_id() const { return global_id_; }

  inline int64_t *mutable_global_id() { return &global_id_; }

  inline bool has_global_id() { return (global_id_ptr_ != nullptr); }

  inline void set_predecessor_ids(std::vector<int64_t> *predecessor_ids) {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids_.assign(predecessor_ids->begin(), predecessor_ids->end());
  }

  inline void set_predecessor_ids(const std::vector<int64_t> &predecessor_ids) {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids_ = predecessor_ids;
  }

  inline void set_predecessor_ids(const uint32_t index,
                                  int64_t &predecessor_ids) {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids_[index] = predecessor_ids;
  }

  inline void add_predecessor_ids(const int64_t &predecessor_ids) {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids_.emplace_back(predecessor_ids);
  }

  inline const int64_t &predecessor_ids(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    return predecessor_ids_[index];
  }

  inline std::vector<int64_t> *mutable_predecessor_ids() {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    return &predecessor_ids_;
  }

  inline void predecessor_ids(std::vector<int64_t> &predecessor_ids) const {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids.assign(predecessor_ids_.begin(), predecessor_ids_.end());
  }

  inline const std::vector<int64_t> &predecessor_ids() const {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    return predecessor_ids_;
  }

  inline uint32_t predecessor_ids_size() const {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    return predecessor_ids_.size();
  }

  inline void clear_predecessor_ids() {
    std::lock_guard<std::mutex> lock(*predecessor_ids_mutex_);
    predecessor_ids_.clear();
    predecessor_ids_.shrink_to_fit();
  }

  inline bool has_predecessor_ids() { return (predecessor_ids_size() != 0); }

  inline void set_successor_ids(std::vector<int64_t> *successor_ids) {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids_.assign(successor_ids->begin(), successor_ids->end());
  }

  inline void set_successor_ids(const std::vector<int64_t> &successor_ids) {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids_ = successor_ids;
  }

  inline void set_successor_ids(const uint32_t index, int64_t &successor_ids) {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids_[index] = successor_ids;
  }

  inline void add_successor_ids(const int64_t &successor_ids) {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids_.emplace_back(successor_ids);
  }

  inline const int64_t &successor_ids(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    return successor_ids_[index];
  }

  inline std::vector<int64_t> *mutable_successor_ids() {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    return &successor_ids_;
  }

  inline void successor_ids(std::vector<int64_t> &successor_ids) const {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids.assign(successor_ids_.begin(), successor_ids_.end());
  }

  inline const std::vector<int64_t> &successor_ids() const {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    return successor_ids_;
  }

  inline uint32_t successor_ids_size() const {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    return successor_ids_.size();
  }

  inline void clear_successor_ids() {
    std::lock_guard<std::mutex> lock(*successor_ids_mutex_);
    successor_ids_.clear();
    successor_ids_.shrink_to_fit();
  }

  inline bool has_successor_ids() { return (successor_ids_size() != 0); }

  inline void set_left_neighbor_id(const int64_t &left_neighbor_id) {
    left_neighbor_id_ = left_neighbor_id;
    left_neighbor_id_ptr_ = &left_neighbor_id_;
  }

  inline const int64_t &left_neighbor_id() const { return left_neighbor_id_; }

  inline int64_t *mutable_left_neighbor_id() { return &left_neighbor_id_; }

  inline bool has_left_neighbor_id() {
    return (left_neighbor_id_ptr_ != nullptr);
  }

  inline void set_right_neighbor_id(const int64_t &right_neighbor_id) {
    right_neighbor_id_ = right_neighbor_id;
    right_neighbor_id_ptr_ = &right_neighbor_id_;
  }

  inline const int64_t &right_neighbor_id() const { return right_neighbor_id_; }

  inline int64_t *mutable_right_neighbor_id() { return &right_neighbor_id_; }

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

  void operator=(const LaneInfoExtend &lane_info_extend) {
    CopyFrom(lane_info_extend);
  }

  void CopyFrom(const LaneInfoExtend &lane_info_extend) {
    priority_ = lane_info_extend.priority();
    global_id_ = lane_info_extend.global_id();
    predecessor_ids_ = lane_info_extend.predecessor_ids();
    successor_ids_ = lane_info_extend.successor_ids();
    left_neighbor_id_ = lane_info_extend.left_neighbor_id();
    right_neighbor_id_ = lane_info_extend.right_neighbor_id();
    type_ = lane_info_extend.type();
    lane_points_ = lane_info_extend.lane_points();
  }

protected:
  std::shared_ptr<std::mutex> predecessor_ids_mutex_;
  std::shared_ptr<std::mutex> successor_ids_mutex_;
  std::shared_ptr<std::mutex> lane_points_mutex_;
  //道路优先级
  int8_t priority_;
  int8_t *priority_ptr_ = nullptr;
  //道路全局id（从左至右，从0开始）
  int64_t global_id_;
  int64_t *global_id_ptr_ = nullptr;
  //所有的前继车道id数组
  std::vector<int64_t> predecessor_ids_;
  //所有的后续车道id数组
  std::vector<int64_t> successor_ids_;
  //左边相邻车道id
  int64_t left_neighbor_id_;
  int64_t *left_neighbor_id_ptr_ = nullptr;
  //右边相邻车道id
  int64_t right_neighbor_id_;
  int64_t *right_neighbor_id_ptr_ = nullptr;
  //类型（预留）
  int8_t type_;
  int8_t *type_ptr_ = nullptr;
  //道路参考线（z）
  std::vector<legionclaw::interface::LanePoint> lane_points_;
};
} // namespace interface
} // namespace legionclaw
