/**
 * @file    imu.hpp
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
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/quaternion.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Imu {
public:
  Imu() {
    orientation_covariance_mutex_ = std::make_shared<std::mutex>();
    angular_velocity_covariance_mutex_ = std::make_shared<std::mutex>();
    linear_acceleration_covariance_mutex_ = std::make_shared<std::mutex>();

    is_valid_ = false;
    clear_orientation_covariance();
    clear_angular_velocity_covariance();
    clear_linear_acceleration_covariance();
  }
  ~Imu() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

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

  inline void
  set_orientation_covariance(std::vector<double> *orientation_covariance) {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance_.assign(orientation_covariance->begin(),
                                   orientation_covariance->end());
  }

  inline void set_orientation_covariance(
      const std::vector<double> &orientation_covariance) {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance_ = orientation_covariance;
  }

  inline void set_orientation_covariance(const uint32_t index,
                                         double &orientation_covariance) {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance_[index] = orientation_covariance;
  }

  inline void add_orientation_covariance(const double &orientation_covariance) {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance_.emplace_back(orientation_covariance);
  }

  inline const double &orientation_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    return orientation_covariance_[index];
  }

  inline std::vector<double> *mutable_orientation_covariance() {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    return &orientation_covariance_;
  }

  inline void
  orientation_covariance(std::vector<double> &orientation_covariance) const {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance.assign(orientation_covariance_.begin(),
                                  orientation_covariance_.end());
  }

  inline const std::vector<double> &orientation_covariance() const {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    return orientation_covariance_;
  }

  inline uint32_t orientation_covariance_size() const {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    return orientation_covariance_.size();
  }

  inline void clear_orientation_covariance() {
    std::lock_guard<std::mutex> lock(*orientation_covariance_mutex_);
    orientation_covariance_.clear();
    orientation_covariance_.shrink_to_fit();
  }

  inline bool has_orientation_covariance() {
    return (orientation_covariance_size() != 0);
  }

  inline void
  set_angular_velocity(const legionclaw::interface::Point3D &angular_velocity) {
    angular_velocity_ = angular_velocity;
    angular_velocity_ptr_ = &angular_velocity_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity() const {
    return angular_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity() {
    return &angular_velocity_;
  }

  inline bool has_angular_velocity() {
    return (angular_velocity_ptr_ != nullptr);
  }

  inline void set_angular_velocity_covariance(
      std::vector<double> *angular_velocity_covariance) {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance_.assign(angular_velocity_covariance->begin(),
                                        angular_velocity_covariance->end());
  }

  inline void set_angular_velocity_covariance(
      const std::vector<double> &angular_velocity_covariance) {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance_ = angular_velocity_covariance;
  }

  inline void
  set_angular_velocity_covariance(const uint32_t index,
                                  double &angular_velocity_covariance) {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance_[index] = angular_velocity_covariance;
  }

  inline void
  add_angular_velocity_covariance(const double &angular_velocity_covariance) {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance_.emplace_back(angular_velocity_covariance);
  }

  inline const double &angular_velocity_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    return angular_velocity_covariance_[index];
  }

  inline std::vector<double> *mutable_angular_velocity_covariance() {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    return &angular_velocity_covariance_;
  }

  inline void angular_velocity_covariance(
      std::vector<double> &angular_velocity_covariance) const {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance.assign(angular_velocity_covariance_.begin(),
                                       angular_velocity_covariance_.end());
  }

  inline const std::vector<double> &angular_velocity_covariance() const {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    return angular_velocity_covariance_;
  }

  inline uint32_t angular_velocity_covariance_size() const {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    return angular_velocity_covariance_.size();
  }

  inline void clear_angular_velocity_covariance() {
    std::lock_guard<std::mutex> lock(*angular_velocity_covariance_mutex_);
    angular_velocity_covariance_.clear();
    angular_velocity_covariance_.shrink_to_fit();
  }

  inline bool has_angular_velocity_covariance() {
    return (angular_velocity_covariance_size() != 0);
  }

  inline void set_linear_acceleration(
      const legionclaw::interface::Point3D &linear_acceleration) {
    linear_acceleration_ = linear_acceleration;
    linear_acceleration_ptr_ = &linear_acceleration_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration() const {
    return linear_acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration() {
    return &linear_acceleration_;
  }

  inline bool has_linear_acceleration() {
    return (linear_acceleration_ptr_ != nullptr);
  }

  inline void set_linear_acceleration_covariance(
      std::vector<double> *linear_acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance_.assign(
        linear_acceleration_covariance->begin(),
        linear_acceleration_covariance->end());
  }

  inline void set_linear_acceleration_covariance(
      const std::vector<double> &linear_acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance_ = linear_acceleration_covariance;
  }

  inline void
  set_linear_acceleration_covariance(const uint32_t index,
                                     double &linear_acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance_[index] = linear_acceleration_covariance;
  }

  inline void add_linear_acceleration_covariance(
      const double &linear_acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance_.emplace_back(
        linear_acceleration_covariance);
  }

  inline const double &linear_acceleration_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    return linear_acceleration_covariance_[index];
  }

  inline std::vector<double> *mutable_linear_acceleration_covariance() {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    return &linear_acceleration_covariance_;
  }

  inline void linear_acceleration_covariance(
      std::vector<double> &linear_acceleration_covariance) const {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance.assign(
        linear_acceleration_covariance_.begin(),
        linear_acceleration_covariance_.end());
  }

  inline const std::vector<double> &linear_acceleration_covariance() const {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    return linear_acceleration_covariance_;
  }

  inline uint32_t linear_acceleration_covariance_size() const {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    return linear_acceleration_covariance_.size();
  }

  inline void clear_linear_acceleration_covariance() {
    std::lock_guard<std::mutex> lock(*linear_acceleration_covariance_mutex_);
    linear_acceleration_covariance_.clear();
    linear_acceleration_covariance_.shrink_to_fit();
  }

  inline bool has_linear_acceleration_covariance() {
    return (linear_acceleration_covariance_size() != 0);
  }

  void operator=(const Imu &imu) { CopyFrom(imu); }

  void CopyFrom(const Imu &imu) {
    header_ = imu.header();
    is_valid_ = imu.is_valid();
    orientation_ = imu.orientation();
    orientation_covariance_ = imu.orientation_covariance();
    angular_velocity_ = imu.angular_velocity();
    angular_velocity_covariance_ = imu.angular_velocity_covariance();
    linear_acceleration_ = imu.linear_acceleration();
    linear_acceleration_covariance_ = imu.linear_acceleration_covariance();
  }

protected:
  std::shared_ptr<std::mutex> orientation_covariance_mutex_;
  std::shared_ptr<std::mutex> angular_velocity_covariance_mutex_;
  std::shared_ptr<std::mutex> linear_acceleration_covariance_mutex_;
  //头部信息
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //数据有效性标志位
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
  //角度（四元素）
  legionclaw::interface::Quaternion orientation_;
  legionclaw::interface::Quaternion *orientation_ptr_ = nullptr;
  //角度方差
  std::vector<double> orientation_covariance_;
  //角速度信息angularVelocity.x：x方向角速度信息angularVelocity.y:
  //y方向角速度信息angularVelocity.z：z方向角速度信息
  legionclaw::interface::Point3D angular_velocity_;
  legionclaw::interface::Point3D *angular_velocity_ptr_ = nullptr;
  //速度方差
  std::vector<double> angular_velocity_covariance_;
  //线加速度信息linearAcceleration.x：x方向线加速度信息linearAcceleration.y:
  //y方向线加速度信息linearAcceleration.z：z方向线加速度信息
  legionclaw::interface::Point3D linear_acceleration_;
  legionclaw::interface::Point3D *linear_acceleration_ptr_ = nullptr;
  //线性加速度方差
  std::vector<double> linear_acceleration_covariance_;
};
} // namespace interface
} // namespace legionclaw
