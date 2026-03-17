/**
 * @file    key_point.hpp
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

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class KeyPoint {
public:
  KeyPoint() {
    id_ = 0;
    latitude_ = 0.0;
    longitude_ = 0.0;
    ele_ = 0.0;
    heading_ = 0.0;
    name_.clear();
  }
  ~KeyPoint() = default;

  inline void set_id(const int16_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int16_t &id() const { return id_; }

  inline int16_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_latitude(const double &latitude) {
    latitude_ = latitude;
    latitude_ptr_ = &latitude_;
  }

  inline const double &latitude() const { return latitude_; }

  inline double *mutable_latitude() { return &latitude_; }

  inline bool has_latitude() { return (latitude_ptr_ != nullptr); }

  inline void set_longitude(const double &longitude) {
    longitude_ = longitude;
    longitude_ptr_ = &longitude_;
  }

  inline const double &longitude() const { return longitude_; }

  inline double *mutable_longitude() { return &longitude_; }

  inline bool has_longitude() { return (longitude_ptr_ != nullptr); }

  inline void set_ele(const double &ele) {
    ele_ = ele;
    ele_ptr_ = &ele_;
  }

  inline const double &ele() const { return ele_; }

  inline double *mutable_ele() { return &ele_; }

  inline bool has_ele() { return (ele_ptr_ != nullptr); }

  inline void set_heading(const double &heading) {
    heading_ = heading;
    heading_ptr_ = &heading_;
  }

  inline const double &heading() const { return heading_; }

  inline double *mutable_heading() { return &heading_; }

  inline bool has_heading() { return (heading_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  void operator=(const KeyPoint &key_point) { CopyFrom(key_point); }

  void CopyFrom(const KeyPoint &key_point) {
    id_ = key_point.id();
    latitude_ = key_point.latitude();
    longitude_ = key_point.longitude();
    ele_ = key_point.ele();
    heading_ = key_point.heading();
    name_ = key_point.name();
  }

protected:
  //关键点id
  int16_t id_;
  int16_t *id_ptr_ = nullptr;
  //关键点纬度
  double latitude_;
  double *latitude_ptr_ = nullptr;
  //关键点经度
  double longitude_;
  double *longitude_ptr_ = nullptr;
  //关键点海拔
  double ele_;
  double *ele_ptr_ = nullptr;
  //关键点航向角（与北方向夹角，顺时针方向为正，单位为°）
  double heading_;
  double *heading_ptr_ = nullptr;
  //关键点名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
