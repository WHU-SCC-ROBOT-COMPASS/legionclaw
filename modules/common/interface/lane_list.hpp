/**
 * @file    lane_list.hpp
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
#include "modules/common/interface/holistic_path_prediction.hpp"
#include "modules/common/interface/lane_line.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/road_mark.hpp"
#include "modules/common/interface/sensor_calibrator.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LaneList {
public:
  LaneList() {
    camera_laneline_mutex_ = std::make_shared<std::mutex>();
    road_marks_mutex_ = std::make_shared<std::mutex>();

    sensor_id_ = legionclaw::common::SensorID::CAMERA_FRONT_CENTER;
    error_code_ = 0;
    sensor_status_ = 0;
    change_origin_flag_ =
        legionclaw::interface::Location::ChangeOriginFlag::CHANGE_NULL;
    is_valid_ = false;
    clear_camera_laneline();
    clear_road_marks();
  }
  ~LaneList() = default;

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

  inline void set_error_code(const int8_t &error_code) {
    error_code_ = error_code;
    error_code_ptr_ = &error_code_;
  }

  inline const int8_t &error_code() const { return error_code_; }

  inline int8_t *mutable_error_code() { return &error_code_; }

  inline bool has_error_code() { return (error_code_ptr_ != nullptr); }

  inline void set_sensor_status(const uint8_t &sensor_status) {
    sensor_status_ = sensor_status;
    sensor_status_ptr_ = &sensor_status_;
  }

  inline const uint8_t &sensor_status() const { return sensor_status_; }

  inline uint8_t *mutable_sensor_status() { return &sensor_status_; }

  inline bool has_sensor_status() { return (sensor_status_ptr_ != nullptr); }

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

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  inline void set_sensor_calibrator(
      const legionclaw::interface::SensorCalibrator &sensor_calibrator) {
    sensor_calibrator_ = sensor_calibrator;
    sensor_calibrator_ptr_ = &sensor_calibrator_;
  }

  inline const legionclaw::interface::SensorCalibrator &sensor_calibrator() const {
    return sensor_calibrator_;
  }

  inline legionclaw::interface::SensorCalibrator *mutable_sensor_calibrator() {
    return &sensor_calibrator_;
  }

  inline bool has_sensor_calibrator() {
    return (sensor_calibrator_ptr_ != nullptr);
  }

  inline void set_camera_laneline(
      std::vector<legionclaw::interface::LaneLine> *camera_laneline) {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline_.assign(camera_laneline->begin(), camera_laneline->end());
  }

  inline void set_camera_laneline(
      const std::vector<legionclaw::interface::LaneLine> &camera_laneline) {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline_ = camera_laneline;
  }

  inline void set_camera_laneline(const uint32_t index,
                                  legionclaw::interface::LaneLine &camera_laneline) {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline_[index] = camera_laneline;
  }

  inline void
  add_camera_laneline(const legionclaw::interface::LaneLine &camera_laneline) {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline_.emplace_back(camera_laneline);
  }

  inline const legionclaw::interface::LaneLine &
  camera_laneline(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    return camera_laneline_[index];
  }

  inline std::vector<legionclaw::interface::LaneLine> *mutable_camera_laneline() {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    return &camera_laneline_;
  }

  inline void camera_laneline(
      std::vector<legionclaw::interface::LaneLine> &camera_laneline) const {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline.assign(camera_laneline_.begin(), camera_laneline_.end());
  }

  inline const std::vector<legionclaw::interface::LaneLine> &
  camera_laneline() const {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    return camera_laneline_;
  }

  inline uint32_t camera_laneline_size() const {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    return camera_laneline_.size();
  }

  inline void clear_camera_laneline() {
    std::lock_guard<std::mutex> lock(*camera_laneline_mutex_);
    camera_laneline_.clear();
    camera_laneline_.shrink_to_fit();
  }

  inline bool has_camera_laneline() { return (camera_laneline_size() != 0); }

  inline void set_hpp(const legionclaw::interface::HolisticPathPrediction &hpp) {
    hpp_ = hpp;
    hpp_ptr_ = &hpp_;
  }

  inline const legionclaw::interface::HolisticPathPrediction &hpp() const {
    return hpp_;
  }

  inline legionclaw::interface::HolisticPathPrediction *mutable_hpp() {
    return &hpp_;
  }

  inline bool has_hpp() { return (hpp_ptr_ != nullptr); }

  inline void
  set_road_marks(std::vector<legionclaw::interface::RoadMark> *road_marks) {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks_.assign(road_marks->begin(), road_marks->end());
  }

  inline void
  set_road_marks(const std::vector<legionclaw::interface::RoadMark> &road_marks) {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks_ = road_marks;
  }

  inline void set_road_marks(const uint32_t index,
                             legionclaw::interface::RoadMark &road_marks) {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks_[index] = road_marks;
  }

  inline void add_road_marks(const legionclaw::interface::RoadMark &road_marks) {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks_.emplace_back(road_marks);
  }

  inline const legionclaw::interface::RoadMark &road_marks(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    return road_marks_[index];
  }

  inline std::vector<legionclaw::interface::RoadMark> *mutable_road_marks() {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    return &road_marks_;
  }

  inline void
  road_marks(std::vector<legionclaw::interface::RoadMark> &road_marks) const {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks.assign(road_marks_.begin(), road_marks_.end());
  }

  inline const std::vector<legionclaw::interface::RoadMark> &road_marks() const {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    return road_marks_;
  }

  inline uint32_t road_marks_size() const {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    return road_marks_.size();
  }

  inline void clear_road_marks() {
    std::lock_guard<std::mutex> lock(*road_marks_mutex_);
    road_marks_.clear();
    road_marks_.shrink_to_fit();
  }

  inline bool has_road_marks() { return (road_marks_size() != 0); }

  void operator=(const LaneList &lane_list) { CopyFrom(lane_list); }

  void CopyFrom(const LaneList &lane_list) {
    header_ = lane_list.header();
    sensor_id_ = lane_list.sensor_id();
    error_code_ = lane_list.error_code();
    sensor_status_ = lane_list.sensor_status();
    change_origin_flag_ = lane_list.change_origin_flag();
    is_valid_ = lane_list.is_valid();
    sensor_calibrator_ = lane_list.sensor_calibrator();
    camera_laneline_ = lane_list.camera_laneline();
    hpp_ = lane_list.hpp();
    road_marks_ = lane_list.road_marks();
  }

protected:
  std::shared_ptr<std::mutex> camera_laneline_mutex_;
  std::shared_ptr<std::mutex> road_marks_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机
  //1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机
  //5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
  legionclaw::common::SensorID sensor_id_;
  legionclaw::common::SensorID *sensor_id_ptr_ = nullptr;
  //错误码： ERROR_NONE = 0; ERROR_UNKNOWN = 1;
  int8_t error_code_;
  int8_t *error_code_ptr_ = nullptr;
  //传感器状态： NORMAL=0； ABNORMAL=1；
  uint8_t sensor_status_;
  uint8_t *sensor_status_ptr_ = nullptr;
  //坐标切换状态（0-坐标系切换成功 1-坐标系切换中 2-坐标系切换故障）
  legionclaw::interface::Location::ChangeOriginFlag change_origin_flag_;
  legionclaw::interface::Location::ChangeOriginFlag *change_origin_flag_ptr_ =
      nullptr;
  //车道线数据是否合法
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
  //传感器标定参数
  legionclaw::interface::SensorCalibrator sensor_calibrator_;
  legionclaw::interface::SensorCalibrator *sensor_calibrator_ptr_ = nullptr;
  //车道线检测结果数组
  std::vector<legionclaw::interface::LaneLine> camera_laneline_;
  //行驶预测线
  legionclaw::interface::HolisticPathPrediction hpp_;
  legionclaw::interface::HolisticPathPrediction *hpp_ptr_ = nullptr;
  //路面标识
  std::vector<legionclaw::interface::RoadMark> road_marks_;
};
} // namespace interface
} // namespace legionclaw
