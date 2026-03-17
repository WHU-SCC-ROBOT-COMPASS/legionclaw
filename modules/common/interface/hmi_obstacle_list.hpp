/**
 * @file    hmi_obstacle_list.hpp
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
#include "modules/common/interface/hmi_obstacle.hpp"
#include "modules/common/interface/location.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HMIObstacleList {
public:
  HMIObstacleList() {
    obstacle_mutex_ = std::make_shared<std::mutex>();

    sensor_id_ = legionclaw::common::SensorID::CAMERA_FRONT_CENTER;
    clear_obstacle();
    error_code_ = legionclaw::common::ErrorCode::LOCALIZATION_ERROR;
    is_valid_ = false;
    change_origin_flag_ =
        legionclaw::interface::Location::ChangeOriginFlag::CHANGE_NULL;
  }
  ~HMIObstacleList() = default;

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

  inline void
  set_obstacle(std::vector<legionclaw::interface::HMIObstacle> *obstacle) {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle_.assign(obstacle->begin(), obstacle->end());
  }

  inline void
  set_obstacle(const std::vector<legionclaw::interface::HMIObstacle> &obstacle) {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle_ = obstacle;
  }

  inline void set_obstacle(const uint32_t index,
                           legionclaw::interface::HMIObstacle &obstacle) {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle_[index] = obstacle;
  }

  inline void add_obstacle(const legionclaw::interface::HMIObstacle &obstacle) {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle_.emplace_back(obstacle);
  }

  inline const legionclaw::interface::HMIObstacle &obstacle(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    return obstacle_[index];
  }

  inline std::vector<legionclaw::interface::HMIObstacle> *mutable_obstacle() {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    return &obstacle_;
  }

  inline void
  obstacle(std::vector<legionclaw::interface::HMIObstacle> &obstacle) const {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle.assign(obstacle_.begin(), obstacle_.end());
  }

  inline const std::vector<legionclaw::interface::HMIObstacle> &obstacle() const {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    return obstacle_;
  }

  inline uint32_t obstacle_size() const {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    return obstacle_.size();
  }

  inline void clear_obstacle() {
    std::lock_guard<std::mutex> lock(*obstacle_mutex_);
    obstacle_.clear();
    obstacle_.shrink_to_fit();
  }

  inline bool has_obstacle() { return (obstacle_size() != 0); }

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

  inline void set_change_origin_flag(
      const legionclaw::interface::Location::ChangeOriginFlag &change_origin_flag) {
    change_origin_flag_ = change_origin_flag;
    change_origin_flag_ptr_ = &change_origin_flag_;
  }

  inline const legionclaw::interface::Location::ChangeOriginFlag &
  change_origin_flag() const {
    return change_origin_flag_;
  }

  inline legionclaw::interface::Location::ChangeOriginFlag *
  mutable_change_origin_flag() {
    return &change_origin_flag_;
  }

  inline bool has_change_origin_flag() {
    return (change_origin_flag_ptr_ != nullptr);
  }

  void operator=(const HMIObstacleList &hmi_obstacle_list) {
    CopyFrom(hmi_obstacle_list);
  }

  void CopyFrom(const HMIObstacleList &hmi_obstacle_list) {
    header_ = hmi_obstacle_list.header();
    sensor_id_ = hmi_obstacle_list.sensor_id();
    obstacle_ = hmi_obstacle_list.obstacle();
    error_code_ = hmi_obstacle_list.error_code();
    is_valid_ = hmi_obstacle_list.is_valid();
    change_origin_flag_ = hmi_obstacle_list.change_origin_flag();
  }

protected:
  std::shared_ptr<std::mutex> obstacle_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机
  //1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机
  //5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
  legionclaw::common::SensorID sensor_id_;
  legionclaw::common::SensorID *sensor_id_ptr_ = nullptr;
  //检测出的障碍物数组
  std::vector<legionclaw::interface::HMIObstacle> obstacle_;
  //错误码（default = OK）
  legionclaw::common::ErrorCode error_code_;
  legionclaw::common::ErrorCode *error_code_ptr_ = nullptr;
  //障碍物数据是否合法
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
  //坐标切换状态（0-坐标系切换成功 1-坐标系切换中 2-坐标系切换故障）
  legionclaw::interface::Location::ChangeOriginFlag change_origin_flag_;
  legionclaw::interface::Location::ChangeOriginFlag *change_origin_flag_ptr_ =
      nullptr;
};
} // namespace interface
} // namespace legionclaw
