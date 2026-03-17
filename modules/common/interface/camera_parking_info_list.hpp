/**
 * @file    camera_parking_info_list.hpp
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
#include "modules/common/interface/camera_parking_info.hpp"
#include "modules/common/interface/camera_parking_stopper.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CameraParkingInfoList {
public:
  CameraParkingInfoList() {
    camera_parking_mutex_ = std::make_shared<std::mutex>();
    camera_parking_stoppers_mutex_ = std::make_shared<std::mutex>();

    sensor_id_ = legionclaw::common::SensorID::CAMERA_FRONT_CENTER;
    clear_camera_parking();
    clear_camera_parking_stoppers();
    error_code_ = legionclaw::common::ErrorCode::LOCALIZATION_ERROR;
    is_valid_ = 1;
  }
  ~CameraParkingInfoList() = default;

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

  inline void set_camera_parking(
      std::vector<legionclaw::interface::CameraParkingInfo> *camera_parking) {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking_.assign(camera_parking->begin(), camera_parking->end());
  }

  inline void set_camera_parking(
      const std::vector<legionclaw::interface::CameraParkingInfo> &camera_parking) {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking_ = camera_parking;
  }

  inline void
  set_camera_parking(const uint32_t index,
                     legionclaw::interface::CameraParkingInfo &camera_parking) {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking_[index] = camera_parking;
  }

  inline void add_camera_parking(
      const legionclaw::interface::CameraParkingInfo &camera_parking) {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking_.emplace_back(camera_parking);
  }

  inline const legionclaw::interface::CameraParkingInfo &
  camera_parking(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    return camera_parking_[index];
  }

  inline std::vector<legionclaw::interface::CameraParkingInfo> *
  mutable_camera_parking() {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    return &camera_parking_;
  }

  inline void camera_parking(
      std::vector<legionclaw::interface::CameraParkingInfo> &camera_parking) const {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking.assign(camera_parking_.begin(), camera_parking_.end());
  }

  inline const std::vector<legionclaw::interface::CameraParkingInfo> &
  camera_parking() const {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    return camera_parking_;
  }

  inline uint32_t camera_parking_size() const {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    return camera_parking_.size();
  }

  inline void clear_camera_parking() {
    std::lock_guard<std::mutex> lock(*camera_parking_mutex_);
    camera_parking_.clear();
    camera_parking_.shrink_to_fit();
  }

  inline bool has_camera_parking() { return (camera_parking_size() != 0); }

  inline void set_camera_parking_stoppers(
      std::vector<legionclaw::interface::CameraParkingStopper>
          *camera_parking_stoppers) {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers_.assign(camera_parking_stoppers->begin(),
                                    camera_parking_stoppers->end());
  }

  inline void set_camera_parking_stoppers(
      const std::vector<legionclaw::interface::CameraParkingStopper>
          &camera_parking_stoppers) {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers_ = camera_parking_stoppers;
  }

  inline void set_camera_parking_stoppers(
      const uint32_t index,
      legionclaw::interface::CameraParkingStopper &camera_parking_stoppers) {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers_[index] = camera_parking_stoppers;
  }

  inline void add_camera_parking_stoppers(
      const legionclaw::interface::CameraParkingStopper &camera_parking_stoppers) {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers_.emplace_back(camera_parking_stoppers);
  }

  inline const legionclaw::interface::CameraParkingStopper &
  camera_parking_stoppers(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    return camera_parking_stoppers_[index];
  }

  inline std::vector<legionclaw::interface::CameraParkingStopper> *
  mutable_camera_parking_stoppers() {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    return &camera_parking_stoppers_;
  }

  inline void
  camera_parking_stoppers(std::vector<legionclaw::interface::CameraParkingStopper>
                              &camera_parking_stoppers) const {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers.assign(camera_parking_stoppers_.begin(),
                                   camera_parking_stoppers_.end());
  }

  inline const std::vector<legionclaw::interface::CameraParkingStopper> &
  camera_parking_stoppers() const {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    return camera_parking_stoppers_;
  }

  inline uint32_t camera_parking_stoppers_size() const {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    return camera_parking_stoppers_.size();
  }

  inline void clear_camera_parking_stoppers() {
    std::lock_guard<std::mutex> lock(*camera_parking_stoppers_mutex_);
    camera_parking_stoppers_.clear();
    camera_parking_stoppers_.shrink_to_fit();
  }

  inline bool has_camera_parking_stoppers() {
    return (camera_parking_stoppers_size() != 0);
  }

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

  void operator=(const CameraParkingInfoList &camera_parking_info_list) {
    CopyFrom(camera_parking_info_list);
  }

  void CopyFrom(const CameraParkingInfoList &camera_parking_info_list) {
    header_ = camera_parking_info_list.header();
    sensor_id_ = camera_parking_info_list.sensor_id();
    camera_parking_ = camera_parking_info_list.camera_parking();
    camera_parking_stoppers_ =
        camera_parking_info_list.camera_parking_stoppers();
    error_code_ = camera_parking_info_list.error_code();
    is_valid_ = camera_parking_info_list.is_valid();
  }

protected:
  std::shared_ptr<std::mutex> camera_parking_mutex_;
  std::shared_ptr<std::mutex> camera_parking_stoppers_mutex_;
  //消息头，时间戳使用接收的图像的时间戳
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机
  //1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机
  //5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
  legionclaw::common::SensorID sensor_id_;
  legionclaw::common::SensorID *sensor_id_ptr_ = nullptr;
  //检测出的障碍物数组
  std::vector<legionclaw::interface::CameraParkingInfo> camera_parking_;
  //检测出的限位器数组
  std::vector<legionclaw::interface::CameraParkingStopper> camera_parking_stoppers_;
  //错误码（default = OK）
  legionclaw::common::ErrorCode error_code_;
  legionclaw::common::ErrorCode *error_code_ptr_ = nullptr;
  //障碍物数据是否合法
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
