/**
 * @file    odometry.hpp
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
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/quaternion.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Odometry {
public:
  Odometry() {
    covariance_mutex_ = std::make_shared<std::mutex>();

    clear_covariance();
  }
  ~Odometry() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::PointENU &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::PointENU &position() const {
    return position_;
  }

  inline legionclaw::interface::PointENU *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_orientation(const legionclaw::interface::Quaternion &orientation) {
    orientation_ = orientation;
    orientation_ptr_ = &orientation_;
  }

  inline const legionclaw::interface::Quaternion &orientation() const {
    return orientation_;
  }

  inline legionclaw::interface::Quaternion *mutable_orientation() {
    return &orientation_;
  }

  inline bool has_orientation() { return (orientation_ptr_ != nullptr); }

  inline void set_covariance(std::vector<double> *covariance) {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance_.assign(covariance->begin(), covariance->end());
  }

  inline void set_covariance(const std::vector<double> &covariance) {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance_ = covariance;
  }

  inline void set_covariance(const uint32_t index, double &covariance) {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance_[index] = covariance;
  }

  inline void add_covariance(const double &covariance) {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance_.emplace_back(covariance);
  }

  inline const double &covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    return covariance_[index];
  }

  inline std::vector<double> *mutable_covariance() {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    return &covariance_;
  }

  inline void covariance(std::vector<double> &covariance) const {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance.assign(covariance_.begin(), covariance_.end());
  }

  inline const std::vector<double> &covariance() const {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    return covariance_;
  }

  inline uint32_t covariance_size() const {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    return covariance_.size();
  }

  inline void clear_covariance() {
    std::lock_guard<std::mutex> lock(*covariance_mutex_);
    covariance_.clear();
    covariance_.shrink_to_fit();
  }

  inline bool has_covariance() { return (covariance_size() != 0); }

  void operator=(const Odometry &odometry) { CopyFrom(odometry); }

  void CopyFrom(const Odometry &odometry) {
    header_ = odometry.header();
    position_ = odometry.position();
    orientation_ = odometry.orientation();
    covariance_ = odometry.covariance();
  }

protected:
  std::shared_ptr<std::mutex> covariance_mutex_;
  //含有时间戳信息的结构体
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //位置
  legionclaw::interface::PointENU position_;
  legionclaw::interface::PointENU *position_ptr_ = nullptr;
  //角度
  legionclaw::interface::Quaternion orientation_;
  legionclaw::interface::Quaternion *orientation_ptr_ = nullptr;
  //方差
  std::vector<double> covariance_;
};
} // namespace interface
} // namespace legionclaw
