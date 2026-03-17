/**
 * @file    road_mark_list.hpp
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
#include "modules/common/interface/road_mark.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RoadMarkList {
public:
  RoadMarkList() {
    roadmarks_mutex_ = std::make_shared<std::mutex>();

    clear_roadmarks();
  }
  ~RoadMarkList() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_roadmarks(std::vector<legionclaw::interface::RoadMark> *roadmarks) {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks_.assign(roadmarks->begin(), roadmarks->end());
  }

  inline void
  set_roadmarks(const std::vector<legionclaw::interface::RoadMark> &roadmarks) {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks_ = roadmarks;
  }

  inline void set_roadmarks(const uint32_t index,
                            legionclaw::interface::RoadMark &roadmarks) {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks_[index] = roadmarks;
  }

  inline void add_roadmarks(const legionclaw::interface::RoadMark &roadmarks) {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks_.emplace_back(roadmarks);
  }

  inline const legionclaw::interface::RoadMark &roadmarks(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    return roadmarks_[index];
  }

  inline std::vector<legionclaw::interface::RoadMark> *mutable_roadmarks() {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    return &roadmarks_;
  }

  inline void
  roadmarks(std::vector<legionclaw::interface::RoadMark> &roadmarks) const {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks.assign(roadmarks_.begin(), roadmarks_.end());
  }

  inline const std::vector<legionclaw::interface::RoadMark> &roadmarks() const {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    return roadmarks_;
  }

  inline uint32_t roadmarks_size() const {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    return roadmarks_.size();
  }

  inline void clear_roadmarks() {
    std::lock_guard<std::mutex> lock(*roadmarks_mutex_);
    roadmarks_.clear();
    roadmarks_.shrink_to_fit();
  }

  inline bool has_roadmarks() { return (roadmarks_size() != 0); }

  void operator=(const RoadMarkList &road_mark_list) {
    CopyFrom(road_mark_list);
  }

  void CopyFrom(const RoadMarkList &road_mark_list) {
    header_ = road_mark_list.header();
    roadmarks_ = road_mark_list.roadmarks();
  }

protected:
  std::shared_ptr<std::mutex> roadmarks_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //交通标识列表
  std::vector<legionclaw::interface::RoadMark> roadmarks_;
};
} // namespace interface
} // namespace legionclaw
