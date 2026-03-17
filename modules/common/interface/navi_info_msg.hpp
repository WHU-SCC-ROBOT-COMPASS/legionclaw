/**
 * @file    navi_info_msg.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class NaviInfoMsg {
public:
  NaviInfoMsg() {
    lane_type_ = 0;
    lane_count_ = 0;
    lane_index_ = 0;
    lane_target_ = 0;
    road_speed_ = 0.0;
    turning_speed_ = 0.0;
    turning_deriction_ = 0.0;
    distance_to_cross_ = 0.0;
    traffic_light_stop_ = false;
    crossing_behavior_ = 0;
    distance_to_stop_ = 0.0;
  }
  ~NaviInfoMsg() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_lane_type(const uint32_t &lane_type) {
    lane_type_ = lane_type;
    lane_type_ptr_ = &lane_type_;
  }

  inline const uint32_t &lane_type() const { return lane_type_; }

  inline uint32_t *mutable_lane_type() { return &lane_type_; }

  inline bool has_lane_type() { return (lane_type_ptr_ != nullptr); }

  inline void set_lane_count(const uint32_t &lane_count) {
    lane_count_ = lane_count;
    lane_count_ptr_ = &lane_count_;
  }

  inline const uint32_t &lane_count() const { return lane_count_; }

  inline uint32_t *mutable_lane_count() { return &lane_count_; }

  inline bool has_lane_count() { return (lane_count_ptr_ != nullptr); }

  inline void set_lane_index(const uint32_t &lane_index) {
    lane_index_ = lane_index;
    lane_index_ptr_ = &lane_index_;
  }

  inline const uint32_t &lane_index() const { return lane_index_; }

  inline uint32_t *mutable_lane_index() { return &lane_index_; }

  inline bool has_lane_index() { return (lane_index_ptr_ != nullptr); }

  inline void set_lane_target(const uint32_t &lane_target) {
    lane_target_ = lane_target;
    lane_target_ptr_ = &lane_target_;
  }

  inline const uint32_t &lane_target() const { return lane_target_; }

  inline uint32_t *mutable_lane_target() { return &lane_target_; }

  inline bool has_lane_target() { return (lane_target_ptr_ != nullptr); }

  inline void set_road_speed(const float &road_speed) {
    road_speed_ = road_speed;
    road_speed_ptr_ = &road_speed_;
  }

  inline const float &road_speed() const { return road_speed_; }

  inline float *mutable_road_speed() { return &road_speed_; }

  inline bool has_road_speed() { return (road_speed_ptr_ != nullptr); }

  inline void set_turning_speed(const float &turning_speed) {
    turning_speed_ = turning_speed;
    turning_speed_ptr_ = &turning_speed_;
  }

  inline const float &turning_speed() const { return turning_speed_; }

  inline float *mutable_turning_speed() { return &turning_speed_; }

  inline bool has_turning_speed() { return (turning_speed_ptr_ != nullptr); }

  inline void set_turning_deriction(const float &turning_deriction) {
    turning_deriction_ = turning_deriction;
    turning_deriction_ptr_ = &turning_deriction_;
  }

  inline const float &turning_deriction() const { return turning_deriction_; }

  inline float *mutable_turning_deriction() { return &turning_deriction_; }

  inline bool has_turning_deriction() {
    return (turning_deriction_ptr_ != nullptr);
  }

  inline void set_distance_to_cross(const float &distance_to_cross) {
    distance_to_cross_ = distance_to_cross;
    distance_to_cross_ptr_ = &distance_to_cross_;
  }

  inline const float &distance_to_cross() const { return distance_to_cross_; }

  inline float *mutable_distance_to_cross() { return &distance_to_cross_; }

  inline bool has_distance_to_cross() {
    return (distance_to_cross_ptr_ != nullptr);
  }

  inline void set_traffic_light_stop(const bool &traffic_light_stop) {
    traffic_light_stop_ = traffic_light_stop;
    traffic_light_stop_ptr_ = &traffic_light_stop_;
  }

  inline const bool &traffic_light_stop() const { return traffic_light_stop_; }

  inline bool *mutable_traffic_light_stop() { return &traffic_light_stop_; }

  inline bool has_traffic_light_stop() {
    return (traffic_light_stop_ptr_ != nullptr);
  }

  inline void set_crossing_behavior(const uint32_t &crossing_behavior) {
    crossing_behavior_ = crossing_behavior;
    crossing_behavior_ptr_ = &crossing_behavior_;
  }

  inline const uint32_t &crossing_behavior() const {
    return crossing_behavior_;
  }

  inline uint32_t *mutable_crossing_behavior() { return &crossing_behavior_; }

  inline bool has_crossing_behavior() {
    return (crossing_behavior_ptr_ != nullptr);
  }

  inline void set_distance_to_stop(const float &distance_to_stop) {
    distance_to_stop_ = distance_to_stop;
    distance_to_stop_ptr_ = &distance_to_stop_;
  }

  inline const float &distance_to_stop() const { return distance_to_stop_; }

  inline float *mutable_distance_to_stop() { return &distance_to_stop_; }

  inline bool has_distance_to_stop() {
    return (distance_to_stop_ptr_ != nullptr);
  }

  void operator=(const NaviInfoMsg &navi_info_msg) { CopyFrom(navi_info_msg); }

  void CopyFrom(const NaviInfoMsg &navi_info_msg) {
    header_ = navi_info_msg.header();
    lane_type_ = navi_info_msg.lane_type();
    lane_count_ = navi_info_msg.lane_count();
    lane_index_ = navi_info_msg.lane_index();
    lane_target_ = navi_info_msg.lane_target();
    road_speed_ = navi_info_msg.road_speed();
    turning_speed_ = navi_info_msg.turning_speed();
    turning_deriction_ = navi_info_msg.turning_deriction();
    distance_to_cross_ = navi_info_msg.distance_to_cross();
    traffic_light_stop_ = navi_info_msg.traffic_light_stop();
    crossing_behavior_ = navi_info_msg.crossing_behavior();
    distance_to_stop_ = navi_info_msg.distance_to_stop();
  }

protected:
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //道路类型0: 常规道路; 1: 路口；（如果没有，给0）
  uint32_t lane_type_;
  uint32_t *lane_type_ptr_ = nullptr;
  //当前道路切面车道个数
  uint32_t lane_count_;
  uint32_t *lane_count_ptr_ = nullptr;
  //当前车所在道路索引，最左侧为0
  uint32_t lane_index_;
  uint32_t *lane_index_ptr_ = nullptr;
  //目标行驶车道索引，最左侧为0
  uint32_t lane_target_;
  uint32_t *lane_target_ptr_ = nullptr;
  //地图限速，单位：米/秒
  float road_speed_;
  float *road_speed_ptr_ = nullptr;
  //转弯建议速度，根据曲率计算，单位：米/秒（如果没有，给极大值）
  float turning_speed_;
  float *turning_speed_ptr_ = nullptr;
  //转弯完成后，道路朝向，单位：角度（如果没有，给0）
  float turning_deriction_;
  float *turning_deriction_ptr_ = nullptr;
  //距离路口的距离，单位：米
  float distance_to_cross_;
  float *distance_to_cross_ptr_ = nullptr;
  //路口是否需要红绿灯停车
  bool traffic_light_stop_;
  bool *traffic_light_stop_ptr_ = nullptr;
  //路口决策行为：0: 直行; 1: 左转; 2: 右转; 3: 掉头
  uint32_t crossing_behavior_;
  uint32_t *crossing_behavior_ptr_ = nullptr;
  //全局剩余里程，单位：米
  float distance_to_stop_;
  float *distance_to_stop_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
