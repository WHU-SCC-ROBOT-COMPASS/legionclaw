/**
 * @file    point_cloud.hpp
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

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/point_xyzirt.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PointCloud {
public:
  PointCloud() {
    point_mutex_ = std::make_shared<std::mutex>();

    frame_id_.clear();
    is_dense_ = false;
    clear_point();
    measurement_time_ = 0.0;
    width_ = 0;
    height_ = 0;
  }
  ~PointCloud() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_frame_id(const std::string &frame_id) {
    frame_id_ = frame_id;
    frame_id_ptr_ = &frame_id_;
  }

  inline const std::string &frame_id() const { return frame_id_; }

  inline std::string *mutable_frame_id() { return &frame_id_; }

  inline bool has_frame_id() { return (frame_id_ptr_ != nullptr); }

  inline void set_is_dense(const bool &is_dense) {
    is_dense_ = is_dense;
    is_dense_ptr_ = &is_dense_;
  }

  inline const bool &is_dense() const { return is_dense_; }

  inline bool *mutable_is_dense() { return &is_dense_; }

  inline bool has_is_dense() { return (is_dense_ptr_ != nullptr); }

  inline void set_point(std::vector<legionclaw::interface::PointXYZIRT> *point) {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point_.assign(point->begin(), point->end());
  }

  inline void
  set_point(const std::vector<legionclaw::interface::PointXYZIRT> &point) {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point_ = point;
  }

  inline void set_point(const uint32_t index,
                        legionclaw::interface::PointXYZIRT &point) {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point_[index] = point;
  }

  inline void add_point(const legionclaw::interface::PointXYZIRT &point) {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point_.emplace_back(point);
  }

  inline const legionclaw::interface::PointXYZIRT &point(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    return point_[index];
  }

  inline std::vector<legionclaw::interface::PointXYZIRT> *mutable_point() {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    return &point_;
  }

  inline void point(std::vector<legionclaw::interface::PointXYZIRT> &point) const {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point.assign(point_.begin(), point_.end());
  }

  inline const std::vector<legionclaw::interface::PointXYZIRT> &point() const {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    return point_;
  }

  inline uint32_t point_size() const {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    return point_.size();
  }

  inline void clear_point() {
    std::lock_guard<std::mutex> lock(*point_mutex_);
    point_.clear();
    point_.shrink_to_fit();
  }

  inline bool has_point() { return (point_size() != 0); }

  inline void set_measurement_time(const double &measurement_time) {
    measurement_time_ = measurement_time;
    measurement_time_ptr_ = &measurement_time_;
  }

  inline const double &measurement_time() const { return measurement_time_; }

  inline double *mutable_measurement_time() { return &measurement_time_; }

  inline bool has_measurement_time() {
    return (measurement_time_ptr_ != nullptr);
  }

  inline void set_width(const uint32_t &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const uint32_t &width() const { return width_; }

  inline uint32_t *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_height(const uint32_t &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const uint32_t &height() const { return height_; }

  inline uint32_t *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  void operator=(const PointCloud &point_cloud) { CopyFrom(point_cloud); }

  void CopyFrom(const PointCloud &point_cloud) {
    header_ = point_cloud.header();
    frame_id_ = point_cloud.frame_id();
    is_dense_ = point_cloud.is_dense();
    point_ = point_cloud.point();
    measurement_time_ = point_cloud.measurement_time();
    width_ = point_cloud.width();
    height_ = point_cloud.height();
  }

protected:
  std::shared_ptr<std::mutex> point_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //帧id
  std::string frame_id_;
  std::string *frame_id_ptr_ = nullptr;
  // True if there are no invalid points
  bool is_dense_;
  bool *is_dense_ptr_ = nullptr;
  //激光点结构体
  std::vector<legionclaw::interface::PointXYZIRT> point_;
  //测量时间
  double measurement_time_;
  double *measurement_time_ptr_ = nullptr;
  //激光点宽度
  uint32_t width_;
  uint32_t *width_ptr_ = nullptr;
  //激光点高度
  uint32_t height_;
  uint32_t *height_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
