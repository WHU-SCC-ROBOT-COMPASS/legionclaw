/**
 * @file    parking_info.hpp
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
#include "modules/common/interface/parking_stopper.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/polygon_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ParkingInfo {
public:
  ParkingInfo() {
    parking_stoppers_mutex_ = std::make_shared<std::mutex>();

    parking_space_id_ = 0;
    parking_type_ = legionclaw::common::ParkingType::INVALID_PARKING;
    parking_status_ = legionclaw::common::ParkingStatus::PARKING_ENABLE;
    confidence_ = 0.0;
    theta_ = 0.0;
    width_ = 0.0;
    length_ = 0.0;
    yaw_offset_ = 0.0;
    sensor_id_ = legionclaw::common::SensorID::CAMERA_FRONT_CENTER;
    is_lane_width_valid_ = false;
    lane_width_ = 0.0;
    clear_parking_stoppers();
    parking_direction_type_ = legionclaw::common::Direction::DIR_INVALID;
    left_occupied_status_ =
        legionclaw::common::OccupiedStatus::UNKNOWN_OCCUPIED_STATUS;
    right_occupied_status_ =
        legionclaw::common::OccupiedStatus::UNKNOWN_OCCUPIED_STATUS;
    parking_source_type_ = legionclaw::common::ParkingSourceType::LINE_PARKING;
  }
  ~ParkingInfo() = default;

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

  inline void
  set_parking_status(const legionclaw::common::ParkingStatus &parking_status) {
    parking_status_ = parking_status;
    parking_status_ptr_ = &parking_status_;
  }

  inline const legionclaw::common::ParkingStatus &parking_status() const {
    return parking_status_;
  }

  inline legionclaw::common::ParkingStatus *mutable_parking_status() {
    return &parking_status_;
  }

  inline bool has_parking_status() { return (parking_status_ptr_ != nullptr); }

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

  inline void set_polygon(const legionclaw::interface::Polygon3D &polygon) {
    polygon_ = polygon;
    polygon_ptr_ = &polygon_;
  }

  inline const legionclaw::interface::Polygon3D &polygon() const { return polygon_; }

  inline legionclaw::interface::Polygon3D *mutable_polygon() { return &polygon_; }

  inline bool has_polygon() { return (polygon_ptr_ != nullptr); }

  inline void set_sensor_id(const legionclaw::common::SensorID &sensor_id) {
    sensor_id_ = sensor_id;
    sensor_id_ptr_ = &sensor_id_;
  }

  inline const legionclaw::common::SensorID &sensor_id() const { return sensor_id_; }

  inline legionclaw::common::SensorID *mutable_sensor_id() { return &sensor_id_; }

  inline bool has_sensor_id() { return (sensor_id_ptr_ != nullptr); }

  inline void set_is_lane_width_valid(const bool &is_lane_width_valid) {
    is_lane_width_valid_ = is_lane_width_valid;
    is_lane_width_valid_ptr_ = &is_lane_width_valid_;
  }

  inline const bool &is_lane_width_valid() const {
    return is_lane_width_valid_;
  }

  inline bool *mutable_is_lane_width_valid() { return &is_lane_width_valid_; }

  inline bool has_is_lane_width_valid() {
    return (is_lane_width_valid_ptr_ != nullptr);
  }

  inline void set_lane_width(const double &lane_width) {
    lane_width_ = lane_width;
    lane_width_ptr_ = &lane_width_;
  }

  inline const double &lane_width() const { return lane_width_; }

  inline double *mutable_lane_width() { return &lane_width_; }

  inline bool has_lane_width() { return (lane_width_ptr_ != nullptr); }

  inline void set_parking_stoppers(
      std::vector<legionclaw::interface::ParkingStopper> *parking_stoppers) {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers_.assign(parking_stoppers->begin(),
                             parking_stoppers->end());
  }

  inline void set_parking_stoppers(
      const std::vector<legionclaw::interface::ParkingStopper> &parking_stoppers) {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers_ = parking_stoppers;
  }

  inline void
  set_parking_stoppers(const uint32_t index,
                       legionclaw::interface::ParkingStopper &parking_stoppers) {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers_[index] = parking_stoppers;
  }

  inline void add_parking_stoppers(
      const legionclaw::interface::ParkingStopper &parking_stoppers) {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers_.emplace_back(parking_stoppers);
  }

  inline const legionclaw::interface::ParkingStopper &
  parking_stoppers(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    return parking_stoppers_[index];
  }

  inline std::vector<legionclaw::interface::ParkingStopper> *
  mutable_parking_stoppers() {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    return &parking_stoppers_;
  }

  inline void parking_stoppers(
      std::vector<legionclaw::interface::ParkingStopper> &parking_stoppers) const {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers.assign(parking_stoppers_.begin(), parking_stoppers_.end());
  }

  inline const std::vector<legionclaw::interface::ParkingStopper> &
  parking_stoppers() const {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    return parking_stoppers_;
  }

  inline uint32_t parking_stoppers_size() const {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    return parking_stoppers_.size();
  }

  inline void clear_parking_stoppers() {
    std::lock_guard<std::mutex> lock(*parking_stoppers_mutex_);
    parking_stoppers_.clear();
    parking_stoppers_.shrink_to_fit();
  }

  inline bool has_parking_stoppers() { return (parking_stoppers_size() != 0); }

  inline void set_parking_direction_type(
      const legionclaw::common::Direction &parking_direction_type) {
    parking_direction_type_ = parking_direction_type;
    parking_direction_type_ptr_ = &parking_direction_type_;
  }

  inline const legionclaw::common::Direction &parking_direction_type() const {
    return parking_direction_type_;
  }

  inline legionclaw::common::Direction *mutable_parking_direction_type() {
    return &parking_direction_type_;
  }

  inline bool has_parking_direction_type() {
    return (parking_direction_type_ptr_ != nullptr);
  }

  inline void set_left_occupied_status(
      const legionclaw::common::OccupiedStatus &left_occupied_status) {
    left_occupied_status_ = left_occupied_status;
    left_occupied_status_ptr_ = &left_occupied_status_;
  }

  inline const legionclaw::common::OccupiedStatus &left_occupied_status() const {
    return left_occupied_status_;
  }

  inline legionclaw::common::OccupiedStatus *mutable_left_occupied_status() {
    return &left_occupied_status_;
  }

  inline bool has_left_occupied_status() {
    return (left_occupied_status_ptr_ != nullptr);
  }

  inline void set_right_occupied_status(
      const legionclaw::common::OccupiedStatus &right_occupied_status) {
    right_occupied_status_ = right_occupied_status;
    right_occupied_status_ptr_ = &right_occupied_status_;
  }

  inline const legionclaw::common::OccupiedStatus &right_occupied_status() const {
    return right_occupied_status_;
  }

  inline legionclaw::common::OccupiedStatus *mutable_right_occupied_status() {
    return &right_occupied_status_;
  }

  inline bool has_right_occupied_status() {
    return (right_occupied_status_ptr_ != nullptr);
  }

  inline void set_parking_source_type(
      const legionclaw::common::ParkingSourceType &parking_source_type) {
    parking_source_type_ = parking_source_type;
    parking_source_type_ptr_ = &parking_source_type_;
  }

  inline const legionclaw::common::ParkingSourceType &parking_source_type() const {
    return parking_source_type_;
  }

  inline legionclaw::common::ParkingSourceType *mutable_parking_source_type() {
    return &parking_source_type_;
  }

  inline bool has_parking_source_type() {
    return (parking_source_type_ptr_ != nullptr);
  }

  void operator=(const ParkingInfo &parking_info) { CopyFrom(parking_info); }

  void CopyFrom(const ParkingInfo &parking_info) {
    header_ = parking_info.header();
    parking_space_id_ = parking_info.parking_space_id();
    parking_type_ = parking_info.parking_type();
    parking_status_ = parking_info.parking_status();
    confidence_ = parking_info.confidence();
    center_point_of_parking_ = parking_info.center_point_of_parking();
    theta_ = parking_info.theta();
    width_ = parking_info.width();
    length_ = parking_info.length();
    yaw_offset_ = parking_info.yaw_offset();
    polygon_ = parking_info.polygon();
    sensor_id_ = parking_info.sensor_id();
    is_lane_width_valid_ = parking_info.is_lane_width_valid();
    lane_width_ = parking_info.lane_width();
    parking_stoppers_ = parking_info.parking_stoppers();
    parking_direction_type_ = parking_info.parking_direction_type();
    left_occupied_status_ = parking_info.left_occupied_status();
    right_occupied_status_ = parking_info.right_occupied_status();
    parking_source_type_ = parking_info.parking_source_type();
  }

protected:
  std::shared_ptr<std::mutex> parking_stoppers_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //停车位id
  int32_t parking_space_id_;
  int32_t *parking_space_id_ptr_ = nullptr;
  // PARKING_TYPE=0//无效泊车 PARKING_TYPE=1//水平泊车 PARKING_TYPE=2//垂直泊车
  // PARKING_TYPE=3//倾斜泊车
  legionclaw::common::ParkingType parking_type_;
  legionclaw::common::ParkingType *parking_type_ptr_ = nullptr;
  // PARKING_ENABLE=0//可泊 PARKING_DISENABLE=1//不可泊
  // PARKING_NONOPTIONAL=2//不可选
  legionclaw::common::ParkingStatus parking_status_;
  legionclaw::common::ParkingStatus *parking_status_ptr_ = nullptr;
  //车位置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //世界坐标系下停车位中心点
  legionclaw::interface::Point3D center_point_of_parking_;
  legionclaw::interface::Point3D *center_point_of_parking_ptr_ = nullptr;
  //世界坐标系下的夹角（单位rad），车位出口与正东方向夹角
  double theta_;
  double *theta_ptr_ = nullptr;
  //停车位宽（单位m）
  double width_;
  double *width_ptr_ = nullptr;
  //停车位长单位m）
  double length_;
  double *length_ptr_ = nullptr;
  //停车位角度偏移量（倾斜车位），倾斜车位与道路边线夹角,(单位m）
  double yaw_offset_;
  double *yaw_offset_ptr_ = nullptr;
  //世界坐标系下的车位多边形
  legionclaw::interface::Polygon3D polygon_;
  legionclaw::interface::Polygon3D *polygon_ptr_ = nullptr;
  //车位数据来自相机id 0-front_center相机 1-front_left相机 2-front_right相机
  //3-left_front相机 4-left_back右后相机 5-right_front相机 6-right_back相机
  //7-back相机 8-相机融合
  legionclaw::common::SensorID sensor_id_;
  legionclaw::common::SensorID *sensor_id_ptr_ = nullptr;
  //车道宽度是否有效
  bool is_lane_width_valid_;
  bool *is_lane_width_valid_ptr_ = nullptr;
  //泊车可用(双)车道宽度
  double lane_width_;
  double *lane_width_ptr_ = nullptr;
  //车位内部的限位器数组
  std::vector<legionclaw::interface::ParkingStopper> parking_stoppers_;
  //  DIR_INVALID = 0,   LEFT = 1,   UP = 2,   RIGHT = 3,   DOWN = 4,
  legionclaw::common::Direction parking_direction_type_;
  legionclaw::common::Direction *parking_direction_type_ptr_ = nullptr;
  // 0-unknown 未知，1-empty 空闲，2-occupied 被占用
  legionclaw::common::OccupiedStatus left_occupied_status_;
  legionclaw::common::OccupiedStatus *left_occupied_status_ptr_ = nullptr;
  // 0-unknown 未知，1-empty 空闲，2-occupied 被占用
  legionclaw::common::OccupiedStatus right_occupied_status_;
  legionclaw::common::OccupiedStatus *right_occupied_status_ptr_ = nullptr;
  // 0-线车位， 1-空间车位， 2-融合车位
  legionclaw::common::ParkingSourceType parking_source_type_;
  legionclaw::common::ParkingSourceType *parking_source_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
