/**
 * @file    traffic_light_box.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>
#include <string>

#include "modules/common/enum/enum.h"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrafficLightBox {
public:
  TrafficLightBox() {
    x_ = 0;
    y_ = 0;
    width_ = 0;
    height_ = 0;
    color_ = legionclaw::common::TrafficLightColor::COLOR_UNKNOWN;
    selected_ = false;
    camera_name_.clear();
  }
  ~TrafficLightBox() = default;

  inline void set_x(const int32_t &x) {
    x_ = x;
    x_ptr_ = &x_;
  }

  inline const int32_t &x() const { return x_; }

  inline int32_t *mutable_x() { return &x_; }

  inline bool has_x() { return (x_ptr_ != nullptr); }

  inline void set_y(const int32_t &y) {
    y_ = y;
    y_ptr_ = &y_;
  }

  inline const int32_t &y() const { return y_; }

  inline int32_t *mutable_y() { return &y_; }

  inline bool has_y() { return (y_ptr_ != nullptr); }

  inline void set_width(const int32_t &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const int32_t &width() const { return width_; }

  inline int32_t *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_height(const int32_t &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const int32_t &height() const { return height_; }

  inline int32_t *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  inline void set_color(const legionclaw::common::TrafficLightColor &color) {
    color_ = color;
    color_ptr_ = &color_;
  }

  inline const legionclaw::common::TrafficLightColor &color() const {
    return color_;
  }

  inline legionclaw::common::TrafficLightColor *mutable_color() { return &color_; }

  inline bool has_color() { return (color_ptr_ != nullptr); }

  inline void set_selected(const bool &selected) {
    selected_ = selected;
    selected_ptr_ = &selected_;
  }

  inline const bool &selected() const { return selected_; }

  inline bool *mutable_selected() { return &selected_; }

  inline bool has_selected() { return (selected_ptr_ != nullptr); }

  inline void set_camera_name(const std::string &camera_name) {
    camera_name_ = camera_name;
    camera_name_ptr_ = &camera_name_;
  }

  inline const std::string &camera_name() const { return camera_name_; }

  inline std::string *mutable_camera_name() { return &camera_name_; }

  inline bool has_camera_name() { return (camera_name_ptr_ != nullptr); }

  void operator=(const TrafficLightBox &traffic_light_box) {
    CopyFrom(traffic_light_box);
  }

  void CopyFrom(const TrafficLightBox &traffic_light_box) {
    x_ = traffic_light_box.x();
    y_ = traffic_light_box.y();
    width_ = traffic_light_box.width();
    height_ = traffic_light_box.height();
    color_ = traffic_light_box.color();
    selected_ = traffic_light_box.selected();
    camera_name_ = traffic_light_box.camera_name();
  }

protected:
  //交通灯目标在图像上中心点横向方向像素位置
  int32_t x_;
  int32_t *x_ptr_ = nullptr;
  //交通灯目标在图像上中心点纵向方向像素位置
  int32_t y_;
  int32_t *y_ptr_ = nullptr;
  //交通灯目标在图像上宽度
  int32_t width_;
  int32_t *width_ptr_ = nullptr;
  //交通灯目标在图像上高度
  int32_t height_;
  int32_t *height_ptr_ = nullptr;
  // UNKNOWN = 0; RED = 1; YELLOW = 2; GREEN = 3; BLACK = 4;
  legionclaw::common::TrafficLightColor color_;
  legionclaw::common::TrafficLightColor *color_ptr_ = nullptr;
  //是否选择
  bool selected_;
  bool *selected_ptr_ = nullptr;
  //相机名称
  std::string camera_name_;
  std::string *camera_name_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
