/**
 * @file    radar_obstacle_list_msg.hpp
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
#include "modules/common/interface/radar_obstacle.hpp"
#include "modules/common/interface/radar_state.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RadarObstacleListMsg {
public:
  RadarObstacleListMsg() {
    radar_obstacle_mutex_ = std::make_shared<std::mutex>();

    clear_radar_obstacle();
    sensor_id_ = 0;
    is_valid_ = false;
  }
  ~RadarObstacleListMsg() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_radar_obstacle(
      std::vector<legionclaw::interface::RadarObstacle> *radar_obstacle) {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle_.assign(radar_obstacle->begin(), radar_obstacle->end());
  }

  inline void set_radar_obstacle(
      const std::vector<legionclaw::interface::RadarObstacle> &radar_obstacle) {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle_ = radar_obstacle;
  }

  inline void
  set_radar_obstacle(const uint32_t index,
                     legionclaw::interface::RadarObstacle &radar_obstacle) {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle_[index] = radar_obstacle;
  }

  inline void
  add_radar_obstacle(const legionclaw::interface::RadarObstacle &radar_obstacle) {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle_.emplace_back(radar_obstacle);
  }

  inline const legionclaw::interface::RadarObstacle &
  radar_obstacle(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    return radar_obstacle_[index];
  }

  inline std::vector<legionclaw::interface::RadarObstacle> *
  mutable_radar_obstacle() {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    return &radar_obstacle_;
  }

  inline void radar_obstacle(
      std::vector<legionclaw::interface::RadarObstacle> &radar_obstacle) const {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle.assign(radar_obstacle_.begin(), radar_obstacle_.end());
  }

  inline const std::vector<legionclaw::interface::RadarObstacle> &
  radar_obstacle() const {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    return radar_obstacle_;
  }

  inline uint32_t radar_obstacle_size() const {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    return radar_obstacle_.size();
  }

  inline void clear_radar_obstacle() {
    std::lock_guard<std::mutex> lock(*radar_obstacle_mutex_);
    radar_obstacle_.clear();
    radar_obstacle_.shrink_to_fit();
  }

  inline bool has_radar_obstacle() { return (radar_obstacle_size() != 0); }

  inline void set_sensor_id(const uint8_t &sensor_id) {
    sensor_id_ = sensor_id;
    sensor_id_ptr_ = &sensor_id_;
  }

  inline const uint8_t &sensor_id() const { return sensor_id_; }

  inline uint8_t *mutable_sensor_id() { return &sensor_id_; }

  inline bool has_sensor_id() { return (sensor_id_ptr_ != nullptr); }

  inline void set_radar_state(const legionclaw::interface::RadarState &radar_state) {
    radar_state_ = radar_state;
    radar_state_ptr_ = &radar_state_;
  }

  inline const legionclaw::interface::RadarState &radar_state() const {
    return radar_state_;
  }

  inline legionclaw::interface::RadarState *mutable_radar_state() {
    return &radar_state_;
  }

  inline bool has_radar_state() { return (radar_state_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  void operator=(const RadarObstacleListMsg &radar_obstacle_list_msg) {
    CopyFrom(radar_obstacle_list_msg);
  }

  void CopyFrom(const RadarObstacleListMsg &radar_obstacle_list_msg) {
    header_ = radar_obstacle_list_msg.header();
    radar_obstacle_ = radar_obstacle_list_msg.radar_obstacle();
    sensor_id_ = radar_obstacle_list_msg.sensor_id();
    radar_state_ = radar_obstacle_list_msg.radar_state();
    is_valid_ = radar_obstacle_list_msg.is_valid();
  }

protected:
  std::shared_ptr<std::mutex> radar_obstacle_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //毫米波雷达障碍物数组
  std::vector<legionclaw::interface::RadarObstacle> radar_obstacle_;
  //毫米波雷达 ID.
  uint8_t sensor_id_;
  uint8_t *sensor_id_ptr_ = nullptr;
  // radar state.
  legionclaw::interface::RadarState radar_state_;
  legionclaw::interface::RadarState *radar_state_ptr_ = nullptr;
  // true: valid  false: invalid
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
