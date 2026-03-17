/**
 * @file    traffic_light.hpp
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

#include "modules/common/interface/image_rect.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/time.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrafficLight {
public:
  TrafficLight() {
    light_lanes_mutex_ = std::make_shared<std::mutex>();

    color_ = legionclaw::common::TrafficLightColor::COLOR_UNKNOWN;
    id_ = 0;
    type_ = legionclaw::common::TrafficLightType::STRAIGHT;
    confidence_ = 0.0;
    distance_ = 0.0;
    clear_light_lanes();
    tracking_time_ = 0.0;
    blink_ = false;
    blinking_time_ = 0.0;
    remaining_time_ = 0.0;
  }
  ~TrafficLight() = default;

  inline void set_color(const legionclaw::common::TrafficLightColor &color) {
    color_ = color;
    color_ptr_ = &color_;
  }

  inline const legionclaw::common::TrafficLightColor &color() const {
    return color_;
  }

  inline legionclaw::common::TrafficLightColor *mutable_color() { return &color_; }

  inline bool has_color() { return (color_ptr_ != nullptr); }

  inline void set_id(const uint32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const uint32_t &id() const { return id_; }

  inline uint32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_type(const legionclaw::common::TrafficLightType &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::common::TrafficLightType &type() const { return type_; }

  inline legionclaw::common::TrafficLightType *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_light_rect(const legionclaw::interface::ImageRect &light_rect) {
    light_rect_ = light_rect;
    light_rect_ptr_ = &light_rect_;
  }

  inline const legionclaw::interface::ImageRect &light_rect() const {
    return light_rect_;
  }

  inline legionclaw::interface::ImageRect *mutable_light_rect() {
    return &light_rect_;
  }

  inline bool has_light_rect() { return (light_rect_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::Point3D &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::Point3D &position() const { return position_; }

  inline legionclaw::interface::Point3D *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_distance(const double &distance) {
    distance_ = distance;
    distance_ptr_ = &distance_;
  }

  inline const double &distance() const { return distance_; }

  inline double *mutable_distance() { return &distance_; }

  inline bool has_distance() { return (distance_ptr_ != nullptr); }

  inline void set_light_lanes(std::vector<int32_t> *light_lanes) {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes_.assign(light_lanes->begin(), light_lanes->end());
  }

  inline void set_light_lanes(const std::vector<int32_t> &light_lanes) {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes_ = light_lanes;
  }

  inline void set_light_lanes(const uint32_t index, int32_t &light_lanes) {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes_[index] = light_lanes;
  }

  inline void add_light_lanes(const int32_t &light_lanes) {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes_.emplace_back(light_lanes);
  }

  inline const int32_t &light_lanes(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    return light_lanes_[index];
  }

  inline std::vector<int32_t> *mutable_light_lanes() {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    return &light_lanes_;
  }

  inline void light_lanes(std::vector<int32_t> &light_lanes) const {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes.assign(light_lanes_.begin(), light_lanes_.end());
  }

  inline const std::vector<int32_t> &light_lanes() const {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    return light_lanes_;
  }

  inline uint32_t light_lanes_size() const {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    return light_lanes_.size();
  }

  inline void clear_light_lanes() {
    std::lock_guard<std::mutex> lock(*light_lanes_mutex_);
    light_lanes_.clear();
    light_lanes_.shrink_to_fit();
  }

  inline bool has_light_lanes() { return (light_lanes_size() != 0); }

  inline void set_tracking_time(const double &tracking_time) {
    tracking_time_ = tracking_time;
    tracking_time_ptr_ = &tracking_time_;
  }

  inline const double &tracking_time() const { return tracking_time_; }

  inline double *mutable_tracking_time() { return &tracking_time_; }

  inline bool has_tracking_time() { return (tracking_time_ptr_ != nullptr); }

  inline void set_blink(const bool &blink) {
    blink_ = blink;
    blink_ptr_ = &blink_;
  }

  inline const bool &blink() const { return blink_; }

  inline bool *mutable_blink() { return &blink_; }

  inline bool has_blink() { return (blink_ptr_ != nullptr); }

  inline void set_blinking_time(const double &blinking_time) {
    blinking_time_ = blinking_time;
    blinking_time_ptr_ = &blinking_time_;
  }

  inline const double &blinking_time() const { return blinking_time_; }

  inline double *mutable_blinking_time() { return &blinking_time_; }

  inline bool has_blinking_time() { return (blinking_time_ptr_ != nullptr); }

  inline void set_remaining_time(const double &remaining_time) {
    remaining_time_ = remaining_time;
    remaining_time_ptr_ = &remaining_time_;
  }

  inline const double &remaining_time() const { return remaining_time_; }

  inline double *mutable_remaining_time() { return &remaining_time_; }

  inline bool has_remaining_time() { return (remaining_time_ptr_ != nullptr); }

  inline void set_create_time(const legionclaw::interface::Time &create_time) {
    create_time_ = create_time;
    create_time_ptr_ = &create_time_;
  }

  inline const legionclaw::interface::Time &create_time() const {
    return create_time_;
  }

  inline legionclaw::interface::Time *mutable_create_time() { return &create_time_; }

  inline bool has_create_time() { return (create_time_ptr_ != nullptr); }

  void operator=(const TrafficLight &traffic_light) { CopyFrom(traffic_light); }

  void CopyFrom(const TrafficLight &traffic_light) {
    color_ = traffic_light.color();
    id_ = traffic_light.id();
    type_ = traffic_light.type();
    confidence_ = traffic_light.confidence();
    light_rect_ = traffic_light.light_rect();
    position_ = traffic_light.position();
    distance_ = traffic_light.distance();
    light_lanes_ = traffic_light.light_lanes();
    tracking_time_ = traffic_light.tracking_time();
    blink_ = traffic_light.blink();
    blinking_time_ = traffic_light.blinking_time();
    remaining_time_ = traffic_light.remaining_time();
    create_time_ = traffic_light.create_time();
  }

protected:
  std::shared_ptr<std::mutex> light_lanes_mutex_;
  // UNKNOWN = 0; RED = 1; YELLOW = 2; GREEN = 3; BLACK = 4;
  legionclaw::common::TrafficLightColor color_;
  legionclaw::common::TrafficLightColor *color_ptr_ = nullptr;
  // light id
  uint32_t id_;
  uint32_t *id_ptr_ = nullptr;
  // STRAIGHT = 0; TURN_LEFT = 1; TURN_RIGHT = 2; STRAIGHT_TURN_LEFT = 3;
  // STRAIGHT_TURN_RIGHT =4; CIRCULAR = 5; PEDESTRIAN = 6; CYCLIST = 7; UNKNOWN =
  // 8;
  legionclaw::common::TrafficLightType type_;
  legionclaw::common::TrafficLightType *type_ptr_ = nullptr;
  //置信度confdence: [0-1]
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //交通灯矩形框light rect
  legionclaw::interface::ImageRect light_rect_;
  legionclaw::interface::ImageRect *light_rect_ptr_ = nullptr;
  //交通灯位置position of light
  legionclaw::interface::Point3D position_;
  legionclaw::interface::Point3D *position_ptr_ = nullptr;
  //距离distance between light and stop line
  double distance_;
  double *distance_ptr_ = nullptr;
  // corresponding lanes of light
  std::vector<int32_t> light_lanes_;
  // duration of a light since track
  double tracking_time_;
  double *tracking_time_ptr_ = nullptr;
  // true:
  bool blink_;
  bool *blink_ptr_ = nullptr;
  // duration of a light blinking
  double blinking_time_;
  double *blinking_time_ptr_ = nullptr;
  // V2X time of light
  double remaining_time_;
  double *remaining_time_ptr_ = nullptr;
  // time of light detected
  legionclaw::interface::Time create_time_;
  legionclaw::interface::Time *create_time_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
