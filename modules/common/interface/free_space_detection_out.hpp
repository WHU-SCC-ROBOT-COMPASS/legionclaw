/**
 * @file    free_space_detection_out.hpp
 * @author  zdhy
 * @date    2021-12-07
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
#include "modules/common/interface/polygon_2d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class FreeSpaceDetectionOut {
public:
  FreeSpaceDetectionOut() {
    freespace_region_mutex_ = std::make_shared<std::mutex>();

    clear_freespace_region();
  }
  ~FreeSpaceDetectionOut() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline void set_freespace_region(
      std::vector<legionclaw::interface::Polygon2D> *freespace_region) {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region_.assign(freespace_region->begin(),
                             freespace_region->end());
  }

  inline void set_freespace_region(
      const std::vector<legionclaw::interface::Polygon2D> &freespace_region) {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region_ = freespace_region;
  }

  inline void
  set_freespace_region(const uint32_t index,
                       legionclaw::interface::Polygon2D &freespace_region) {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region_[index] = freespace_region;
  }

  inline void
  add_freespace_region(const legionclaw::interface::Polygon2D &freespace_region) {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region_.emplace_back(freespace_region);
  }

  inline const legionclaw::interface::Polygon2D &
  freespace_region(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    return freespace_region_[index];
  }

  inline std::vector<legionclaw::interface::Polygon2D> *mutable_freespace_region() {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    return &freespace_region_;
  }

  inline void freespace_region(
      std::vector<legionclaw::interface::Polygon2D> &freespace_region) const {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region.assign(freespace_region_.begin(), freespace_region_.end());
  }

  inline const std::vector<legionclaw::interface::Polygon2D> &
  freespace_region() const {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    return freespace_region_;
  }

  inline uint32_t freespace_region_size() const {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    return freespace_region_.size();
  }

  inline void clear_freespace_region() {
    std::lock_guard<std::mutex> lock(*freespace_region_mutex_);
    freespace_region_.clear();
  }

protected:
  std::shared_ptr<std::mutex> freespace_region_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  // freespace轮廓图像坐标
  std::vector<legionclaw::interface::Polygon2D> freespace_region_;
};
} // namespace interface
} // namespace legionclaw
