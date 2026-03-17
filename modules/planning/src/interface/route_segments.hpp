/**
 * @file    route_segments.hpp
 * @author  zdhy
 * @date    2021-11-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <vector>
#include <memory>
#include <iostream>
#include <stdint.h>

#include "modules/planning/src/interface/lane_point_inner.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
class RouteSegments {
public:
  RouteSegments() {
    lane_points_mutex_ = std::make_shared<std::mutex>();
    priority_ = 0;
    global_id_ = 0;
    type_ = 0;
    vehicle_close_index_ = -1;
    start_index_ = -1;
    end_index_ = -1;
    remain_mileage_ = DBL_MIN;
    clear_lane_points();
  }
  ~RouteSegments() = default;

    inline void set_priority(const int8_t& priority) {
    priority_ = priority;
    priority_ptr_ = &priority_;
  }

  inline const int8_t& priority() const { return priority_; }

  inline int8_t* mutable_priority() { return &priority_; }

  inline bool has_priority() { return (priority_ptr_ != nullptr); }

  inline void set_global_id(const int8_t& global_id) {
    global_id_ = global_id;
    global_id_ptr_ = &global_id_;
  }

  inline const int8_t& global_id() const { return global_id_; }

  inline int8_t* mutable_global_id() { return &global_id_; }

  inline bool has_global_id() { return (global_id_ptr_ != nullptr); }

  inline void set_vehicle_close_index(const int& vehicle_close_index) {
    vehicle_close_index_ = vehicle_close_index;
  }

  inline const int& vehicle_close_index() const { return vehicle_close_index_; }

  inline int* mutable_vehicle_close_index() { return &vehicle_close_index_; }

  inline void set_start_index(const int& start_index) {
    start_index_ = start_index;
  }

  inline const int& start_index() const { return start_index_; }

  inline int* mutable_start_index() { return &start_index_; }

  inline void set_end_index(const int& end_index) { end_index_ = end_index; }

  inline const int& end_index() const { return end_index_; }

  inline int* mutable_end_index() { return &end_index_; }

  inline void set_remain_mileage(const double& remain_mileage) {
    remain_mileage_ = remain_mileage;
  }

  inline const double& remain_mileage() const { return remain_mileage_; }

  inline double* mutable_remain_mileage() { return &remain_mileage_; }

  inline void set_type(const int8_t& type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const int8_t& type() const { return type_; }

  inline int8_t* mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void
  set_lane_points(std::vector<legionclaw::planning::LanePointInner>* lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_.assign(lane_points->begin(), lane_points->end());
  }

  inline void set_lane_points(
      const std::vector<legionclaw::planning::LanePointInner>& lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_ = lane_points;
  }

  inline void set_lane_points(const uint32_t index,
                              legionclaw::planning::LanePointInner& lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_[index] = lane_points;
  }

  inline void add_lane_points(const legionclaw::planning::LanePointInner& lane_points) {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points_.emplace_back(lane_points);
  }

  inline const legionclaw::planning::LanePointInner& lane_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return lane_points_[index];
  }

  inline std::vector<legionclaw::planning::LanePointInner>* mutable_lane_points() {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    return &lane_points_;
  }

  inline void
  lane_points(std::vector<legionclaw::planning::LanePointInner>& lane_points) const {
    std::lock_guard<std::mutex> lock(*lane_points_mutex_);
    lane_points.assign(lane_points_.begin(), lane_points_.end());
  }

  inline const std::vector<legionclaw::planning::LanePointInner>& lane_points() const {
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
  }

  inline bool has_lane_points() { return (lane_points_size() != 0); }

  void operator=(const RouteSegments& lane_info) { CopyFrom(lane_info); }

  void CopyFrom(const RouteSegments& lane_info) {
    priority_ = lane_info.priority();
    global_id_ = lane_info.global_id();
    type_ = lane_info.type();
    vehicle_close_index_ = lane_info.vehicle_close_index();
    start_index_ = lane_info.start_index();
    end_index_ = lane_info.end_index();
    remain_mileage_ = lane_info.remain_mileage();
    lane_points_ = lane_info.lane_points();
    id_ = lane_info.id();
  }

  const std::string &id() const { return id_; }

  void set_id(const std::string &id) { id_ = id; }

  bool is_on_segment() const { return is_on_segment_; }

  void set_is_on_segment(bool on_segment) { is_on_segment_ = on_segment; }

 protected:
  std::shared_ptr<std::mutex> lane_points_mutex_;
  //道路优先级
  int8_t priority_;
  int8_t* priority_ptr_ = nullptr;
  //道路全局id（从左至右，从0开始）
  int8_t global_id_;
  int8_t* global_id_ptr_ = nullptr;
  //类型（预留）
  int8_t type_;
  int8_t* type_ptr_ = nullptr;
  //车在当前参考线的close_index
  int vehicle_close_index_;
  int start_index_;
  int end_index_;
  double remain_mileage_;
  //道路参考线（z）
  std::vector<legionclaw::planning::LanePointInner> lane_points_;
  std::string id_;

  //Indicates whether the vehicle is on current RouteSegment.
  bool is_on_segment_ = false;
};
} // namespace planning
} // namespace legionclaw
