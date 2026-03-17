/**
 * @file    free_space.hpp
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

#include "modules/common/enum/enum.h"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/polygon_2d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class FreeSpace {
public:
  FreeSpace() {
    freespace_region_mutex_ = std::make_shared<std::mutex>();

    sensor_id_ = legionclaw::common::SensorID::CAMERA_FRONT_CENTER;
    clear_freespace_region();
    error_code_ = legionclaw::common::ErrorCode::LOCALIZATION_ERROR;
    is_valid_ = 1;
  }
  ~FreeSpace() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_sensor_id(const legionclaw::common::SensorID &sensor_id) {
    sensor_id_ = sensor_id;
    sensor_id_ptr_ = &sensor_id_;
  }

  inline const legionclaw::common::SensorID &sensor_id() const { return sensor_id_; }

  inline legionclaw::common::SensorID *mutable_sensor_id() { return &sensor_id_; }

  inline bool has_sensor_id() { return (sensor_id_ptr_ != nullptr); }

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
    freespace_region_.shrink_to_fit();
  }

  inline bool has_freespace_region() { return (freespace_region_size() != 0); }

  inline void set_error_code(const legionclaw::common::ErrorCode &error_code) {
    error_code_ = error_code;
    error_code_ptr_ = &error_code_;
  }

  inline const legionclaw::common::ErrorCode &error_code() const {
    return error_code_;
  }

  inline legionclaw::common::ErrorCode *mutable_error_code() { return &error_code_; }

  inline bool has_error_code() { return (error_code_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  void operator=(const FreeSpace &free_space) { CopyFrom(free_space); }

  void CopyFrom(const FreeSpace &free_space) {
    header_ = free_space.header();
    sensor_id_ = free_space.sensor_id();
    freespace_region_ = free_space.freespace_region();
    error_code_ = free_space.error_code();
    is_valid_ = free_space.is_valid();
  }

protected:
  std::shared_ptr<std::mutex> freespace_region_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机
  //1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机
  //5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
  legionclaw::common::SensorID sensor_id_;
  legionclaw::common::SensorID *sensor_id_ptr_ = nullptr;
  // freespace轮廓图像坐标
  std::vector<legionclaw::interface::Polygon2D> freespace_region_;
  //错误码（default = OK）
  legionclaw::common::ErrorCode error_code_;
  legionclaw::common::ErrorCode *error_code_ptr_ = nullptr;
  //障碍物数据是否合法
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
