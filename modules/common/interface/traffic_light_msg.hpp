/**
 * @file    traffic_light_msg.hpp
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
#include "modules/common/interface/traffic_light.hpp"
#include "modules/common/interface/traffic_light_debug.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrafficLightMsg {
public:
  TrafficLightMsg() {
    traffic_light_mutex_ = std::make_shared<std::mutex>();

    clear_traffic_light();
    contain_lights_ = false;
    camera_id_ = legionclaw::interface::TrafficLightMsg::CameraID::CAMERA_FRONT_LONG;
    is_valid_ = false;
  }
  ~TrafficLightMsg() = default;

  enum CameraID {
    CAMERA_FRONT_LONG = 0,
    CAMERA_FRONT_NARROW = 1,
    CAMERA_FRONT_SHORT = 2,
    CAMERA_FRONT_WIDE = 3,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_traffic_light(
      std::vector<legionclaw::interface::TrafficLight> *traffic_light) {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light_.assign(traffic_light->begin(), traffic_light->end());
  }

  inline void set_traffic_light(
      const std::vector<legionclaw::interface::TrafficLight> &traffic_light) {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light_ = traffic_light;
  }

  inline void set_traffic_light(const uint32_t index,
                                legionclaw::interface::TrafficLight &traffic_light) {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light_[index] = traffic_light;
  }

  inline void
  add_traffic_light(const legionclaw::interface::TrafficLight &traffic_light) {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light_.emplace_back(traffic_light);
  }

  inline const legionclaw::interface::TrafficLight &
  traffic_light(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    return traffic_light_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLight> *mutable_traffic_light() {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    return &traffic_light_;
  }

  inline void traffic_light(
      std::vector<legionclaw::interface::TrafficLight> &traffic_light) const {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light.assign(traffic_light_.begin(), traffic_light_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLight> &
  traffic_light() const {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    return traffic_light_;
  }

  inline uint32_t traffic_light_size() const {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    return traffic_light_.size();
  }

  inline void clear_traffic_light() {
    std::lock_guard<std::mutex> lock(*traffic_light_mutex_);
    traffic_light_.clear();
    traffic_light_.shrink_to_fit();
  }

  inline bool has_traffic_light() { return (traffic_light_size() != 0); }

  inline void set_traffic_light_debug(
      const legionclaw::interface::TrafficLightDebug &traffic_light_debug) {
    traffic_light_debug_ = traffic_light_debug;
    traffic_light_debug_ptr_ = &traffic_light_debug_;
  }

  inline const legionclaw::interface::TrafficLightDebug &
  traffic_light_debug() const {
    return traffic_light_debug_;
  }

  inline legionclaw::interface::TrafficLightDebug *mutable_traffic_light_debug() {
    return &traffic_light_debug_;
  }

  inline bool has_traffic_light_debug() {
    return (traffic_light_debug_ptr_ != nullptr);
  }

  inline void set_contain_lights(const bool &contain_lights) {
    contain_lights_ = contain_lights;
    contain_lights_ptr_ = &contain_lights_;
  }

  inline const bool &contain_lights() const { return contain_lights_; }

  inline bool *mutable_contain_lights() { return &contain_lights_; }

  inline bool has_contain_lights() { return (contain_lights_ptr_ != nullptr); }

  inline void
  set_camera_id(const legionclaw::interface::TrafficLightMsg::CameraID &camera_id) {
    camera_id_ = camera_id;
    camera_id_ptr_ = &camera_id_;
  }

  inline const legionclaw::interface::TrafficLightMsg::CameraID &camera_id() const {
    return camera_id_;
  }

  inline legionclaw::interface::TrafficLightMsg::CameraID *mutable_camera_id() {
    return &camera_id_;
  }

  inline bool has_camera_id() { return (camera_id_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  void operator=(const TrafficLightMsg &traffic_light_msg) {
    CopyFrom(traffic_light_msg);
  }

  void CopyFrom(const TrafficLightMsg &traffic_light_msg) {
    header_ = traffic_light_msg.header();
    traffic_light_ = traffic_light_msg.traffic_light();
    traffic_light_debug_ = traffic_light_msg.traffic_light_debug();
    contain_lights_ = traffic_light_msg.contain_lights();
    camera_id_ = traffic_light_msg.camera_id();
    is_valid_ = traffic_light_msg.is_valid();
  }

protected:
  std::shared_ptr<std::mutex> traffic_light_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //检出的交通灯数组
  std::vector<legionclaw::interface::TrafficLight> traffic_light_;
  //交通灯数组调试
  legionclaw::interface::TrafficLightDebug traffic_light_debug_;
  legionclaw::interface::TrafficLightDebug *traffic_light_debug_ptr_ = nullptr;
  //是否包含交通灯
  bool contain_lights_;
  bool *contain_lights_ptr_ = nullptr;
  // CAMERA_FRONT_LONG = 0; CAMERA_FRONT_NARROW = 1; CAMERA_FRONT_SHORT = 2;
  // CAMERA_FRONT_WIDE = 3;
  legionclaw::interface::TrafficLightMsg::CameraID camera_id_;
  legionclaw::interface::TrafficLightMsg::CameraID *camera_id_ptr_ = nullptr;
  // true: valid  false: invalid
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
