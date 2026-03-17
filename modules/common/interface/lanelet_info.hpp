/**
 * @file    lanelet_info.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LaneletInfo {
public:
  LaneletInfo() {
    lanelet_id_ = 0;
    length_ = 0.0;
  }
  ~LaneletInfo() = default;

  inline void set_lanelet_id(const int32_t &lanelet_id) {
    lanelet_id_ = lanelet_id;
    lanelet_id_ptr_ = &lanelet_id_;
  }

  inline const int32_t &lanelet_id() const { return lanelet_id_; }

  inline int32_t *mutable_lanelet_id() { return &lanelet_id_; }

  inline bool has_lanelet_id() { return (lanelet_id_ptr_ != nullptr); }

  inline void set_length(const double &length) {
    length_ = length;
    length_ptr_ = &length_;
  }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline bool has_length() { return (length_ptr_ != nullptr); }

  void operator=(const LaneletInfo &lanelet_info) { CopyFrom(lanelet_info); }

  void CopyFrom(const LaneletInfo &lanelet_info) {
    lanelet_id_ = lanelet_info.lanelet_id();
    length_ = lanelet_info.length();
  }

protected:
  //道路id，与地图文件对应
  int32_t lanelet_id_;
  int32_t *lanelet_id_ptr_ = nullptr;
  //道路长度
  double length_;
  double *length_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
