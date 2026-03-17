/**
 * @file    camera_parking_info.hpp
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
#include "modules/common/interface/image_key_point.hpp"
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CameraParkingInfo {
public:
  CameraParkingInfo() {
    parking_points_in_image_mutex_ = std::make_shared<std::mutex>();

    parking_space_id_ = -1.0;
    parking_type_ = legionclaw::common::ParkingType::INVALID_PARKING;
    is_parking_enable_ = 0.0;
    confidence_ = 0.0;
    theta_ = 0.0;
    width_ = 0.0;
    length_ = 0.0;
    yaw_offset_ = 0.0;
    clear_parking_points_in_image();
  }
  ~CameraParkingInfo() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_parking_space_id(const int32_t &parking_space_id) {
    parking_space_id_ = parking_space_id;
    parking_space_id_ptr_ = &parking_space_id_;
  }

  inline const int32_t &parking_space_id() const { return parking_space_id_; }

  inline int32_t *mutable_parking_space_id() { return &parking_space_id_; }

  inline bool has_parking_space_id() {
    return (parking_space_id_ptr_ != nullptr);
  }

  inline void set_parking_type(const legionclaw::common::ParkingType &parking_type) {
    parking_type_ = parking_type;
    parking_type_ptr_ = &parking_type_;
  }

  inline const legionclaw::common::ParkingType &parking_type() const {
    return parking_type_;
  }

  inline legionclaw::common::ParkingType *mutable_parking_type() {
    return &parking_type_;
  }

  inline bool has_parking_type() { return (parking_type_ptr_ != nullptr); }

  inline void set_is_parking_enable(const bool &is_parking_enable) {
    is_parking_enable_ = is_parking_enable;
    is_parking_enable_ptr_ = &is_parking_enable_;
  }

  inline const bool &is_parking_enable() const { return is_parking_enable_; }

  inline bool *mutable_is_parking_enable() { return &is_parking_enable_; }

  inline bool has_is_parking_enable() {
    return (is_parking_enable_ptr_ != nullptr);
  }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_center_point_of_parking(
      const legionclaw::interface::Point3D &center_point_of_parking) {
    center_point_of_parking_ = center_point_of_parking;
    center_point_of_parking_ptr_ = &center_point_of_parking_;
  }

  inline const legionclaw::interface::Point3D &center_point_of_parking() const {
    return center_point_of_parking_;
  }

  inline legionclaw::interface::Point3D *mutable_center_point_of_parking() {
    return &center_point_of_parking_;
  }

  inline bool has_center_point_of_parking() {
    return (center_point_of_parking_ptr_ != nullptr);
  }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

  inline void set_width(const double &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const double &width() const { return width_; }

  inline double *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_length(const double &length) {
    length_ = length;
    length_ptr_ = &length_;
  }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline bool has_length() { return (length_ptr_ != nullptr); }

  inline void set_yaw_offset(const double &yaw_offset) {
    yaw_offset_ = yaw_offset;
    yaw_offset_ptr_ = &yaw_offset_;
  }

  inline const double &yaw_offset() const { return yaw_offset_; }

  inline double *mutable_yaw_offset() { return &yaw_offset_; }

  inline bool has_yaw_offset() { return (yaw_offset_ptr_ != nullptr); }

  inline void set_parking_points_in_image(
      std::vector<legionclaw::interface::ImageKeyPoint> *parking_points_in_image) {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image_.assign(parking_points_in_image->begin(),
                                    parking_points_in_image->end());
  }

  inline void
  set_parking_points_in_image(const std::vector<legionclaw::interface::ImageKeyPoint>
                                  &parking_points_in_image) {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image_ = parking_points_in_image;
  }

  inline void set_parking_points_in_image(
      const uint32_t index,
      legionclaw::interface::ImageKeyPoint &parking_points_in_image) {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image_[index] = parking_points_in_image;
  }

  inline void add_parking_points_in_image(
      const legionclaw::interface::ImageKeyPoint &parking_points_in_image) {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image_.emplace_back(parking_points_in_image);
  }

  inline const legionclaw::interface::ImageKeyPoint &
  parking_points_in_image(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    return parking_points_in_image_[index];
  }

  inline std::vector<legionclaw::interface::ImageKeyPoint> *
  mutable_parking_points_in_image() {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    return &parking_points_in_image_;
  }

  inline void parking_points_in_image(
      std::vector<legionclaw::interface::ImageKeyPoint> &parking_points_in_image)
      const {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image.assign(parking_points_in_image_.begin(),
                                   parking_points_in_image_.end());
  }

  inline const std::vector<legionclaw::interface::ImageKeyPoint> &
  parking_points_in_image() const {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    return parking_points_in_image_;
  }

  inline uint32_t parking_points_in_image_size() const {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    return parking_points_in_image_.size();
  }

  inline void clear_parking_points_in_image() {
    std::lock_guard<std::mutex> lock(*parking_points_in_image_mutex_);
    parking_points_in_image_.clear();
    parking_points_in_image_.shrink_to_fit();
  }

  inline bool has_parking_points_in_image() {
    return (parking_points_in_image_size() != 0);
  }

  void operator=(const CameraParkingInfo &camera_parking_info) {
    CopyFrom(camera_parking_info);
  }

  void CopyFrom(const CameraParkingInfo &camera_parking_info) {
    header_ = camera_parking_info.header();
    parking_space_id_ = camera_parking_info.parking_space_id();
    parking_type_ = camera_parking_info.parking_type();
    is_parking_enable_ = camera_parking_info.is_parking_enable();
    confidence_ = camera_parking_info.confidence();
    center_point_of_parking_ = camera_parking_info.center_point_of_parking();
    theta_ = camera_parking_info.theta();
    width_ = camera_parking_info.width();
    length_ = camera_parking_info.length();
    yaw_offset_ = camera_parking_info.yaw_offset();
    parking_points_in_image_ = camera_parking_info.parking_points_in_image();
  }

protected:
  std::shared_ptr<std::mutex> parking_points_in_image_mutex_;
  // timestamp is  included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //停车位id
  int32_t parking_space_id_;
  int32_t *parking_space_id_ptr_ = nullptr;
  // PARKING_TYPE=0//无效泊车 PARKING_TYPE=1//垂直泊车 PARKING_TYPE=2//水平泊车
  // PARKING_TYPE=3//倾斜泊车
  legionclaw::common::ParkingType parking_type_;
  legionclaw::common::ParkingType *parking_type_ptr_ = nullptr;
  // PARKING_ENABLE=0//不可泊入 PARKING_ENABLE=1//可泊入
  bool is_parking_enable_;
  bool *is_parking_enable_ptr_ = nullptr;
  //停车位置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //车辆坐标系的停车位中心点
  legionclaw::interface::Point3D center_point_of_parking_;
  legionclaw::interface::Point3D *center_point_of_parking_ptr_ = nullptr;
  //车辆坐标系下停车位的朝向
  double theta_;
  double *theta_ptr_ = nullptr;
  //停车位宽
  double width_;
  double *width_ptr_ = nullptr;
  //停车位长
  double length_;
  double *length_ptr_ = nullptr;
  //停车位角度偏移量（倾斜车位），倾斜车位与道路边线夹角
  double yaw_offset_;
  double *yaw_offset_ptr_ = nullptr;
  //图像坐标系下停车位关键点，顺时针方向
  std::vector<legionclaw::interface::ImageKeyPoint> parking_points_in_image_;
};
} // namespace interface
} // namespace legionclaw
