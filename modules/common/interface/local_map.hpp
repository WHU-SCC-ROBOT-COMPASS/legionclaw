/**
 * @file    local_map.hpp
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
#include "modules/common/interface/lane_info_extend.hpp"
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LocalMap {
public:
  LocalMap() {
    lane_list_mutex_ = std::make_shared<std::mutex>();

    seq_ = 0;
    range_ = 0.0;
    clear_lane_list();
  }
  ~LocalMap() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_seq(const int32_t &seq) {
    seq_ = seq;
    seq_ptr_ = &seq_;
  }

  inline const int32_t &seq() const { return seq_; }

  inline int32_t *mutable_seq() { return &seq_; }

  inline bool has_seq() { return (seq_ptr_ != nullptr); }

  inline void set_ego_point(const legionclaw::interface::Point3D &ego_point) {
    ego_point_ = ego_point;
    ego_point_ptr_ = &ego_point_;
  }

  inline const legionclaw::interface::Point3D &ego_point() const {
    return ego_point_;
  }

  inline legionclaw::interface::Point3D *mutable_ego_point() { return &ego_point_; }

  inline bool has_ego_point() { return (ego_point_ptr_ != nullptr); }

  inline void set_range(const double &range) {
    range_ = range;
    range_ptr_ = &range_;
  }

  inline const double &range() const { return range_; }

  inline double *mutable_range() { return &range_; }

  inline bool has_range() { return (range_ptr_ != nullptr); }

  inline void
  set_lane_list(std::vector<legionclaw::interface::LaneInfoExtend> *lane_list) {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list_.assign(lane_list->begin(), lane_list->end());
  }

  inline void set_lane_list(
      const std::vector<legionclaw::interface::LaneInfoExtend> &lane_list) {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list_ = lane_list;
  }

  inline void set_lane_list(const uint32_t index,
                            legionclaw::interface::LaneInfoExtend &lane_list) {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list_[index] = lane_list;
  }

  inline void add_lane_list(const legionclaw::interface::LaneInfoExtend &lane_list) {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list_.emplace_back(lane_list);
  }

  inline const legionclaw::interface::LaneInfoExtend &
  lane_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    return lane_list_[index];
  }

  inline std::vector<legionclaw::interface::LaneInfoExtend> *mutable_lane_list() {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    return &lane_list_;
  }

  inline void
  lane_list(std::vector<legionclaw::interface::LaneInfoExtend> &lane_list) const {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list.assign(lane_list_.begin(), lane_list_.end());
  }

  inline const std::vector<legionclaw::interface::LaneInfoExtend> &
  lane_list() const {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    return lane_list_;
  }

  inline uint32_t lane_list_size() const {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    return lane_list_.size();
  }

  inline void clear_lane_list() {
    std::lock_guard<std::mutex> lock(*lane_list_mutex_);
    lane_list_.clear();
    lane_list_.shrink_to_fit();
  }

  inline bool has_lane_list() { return (lane_list_size() != 0); }

  void operator=(const LocalMap &local_map) { CopyFrom(local_map); }

  void CopyFrom(const LocalMap &local_map) {
    header_ = local_map.header();
    seq_ = local_map.seq();
    ego_point_ = local_map.ego_point();
    range_ = local_map.range();
    lane_list_ = local_map.lane_list();
  }

protected:
  std::shared_ptr<std::mutex> lane_list_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // Sequence number，lane_list数据有更新则加1，否则保持不变
  int32_t seq_;
  int32_t *seq_ptr_ = nullptr;
  //生成该LocalMap局部地图消息时的自车位置
  legionclaw::interface::Point3D ego_point_;
  legionclaw::interface::Point3D *ego_point_ptr_ = nullptr;
  //生成该LocalMap局部地图消息的范围，以ego_point为中心，range为边长的正方形，单位：m
  double range_;
  double *range_ptr_ = nullptr;
  //自车周边LocalMap范围内的车道信息列表
  std::vector<legionclaw::interface::LaneInfoExtend> lane_list_;
};
} // namespace interface
} // namespace legionclaw
