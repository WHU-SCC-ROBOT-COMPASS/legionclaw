/**
 * @file    curvature_info.hpp
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
class CurvatureInfo {
public:
  CurvatureInfo() {
    offset_ = 0.0;
    value_ = 0.0;
  }
  ~CurvatureInfo() = default;

  inline void set_offset(const double &offset) {
    offset_ = offset;
    offset_ptr_ = &offset_;
  }

  inline const double &offset() const { return offset_; }

  inline double *mutable_offset() { return &offset_; }

  inline bool has_offset() { return (offset_ptr_ != nullptr); }

  inline void set_value(const double &value) {
    value_ = value;
    value_ptr_ = &value_;
  }

  inline const double &value() const { return value_; }

  inline double *mutable_value() { return &value_; }

  inline bool has_value() { return (value_ptr_ != nullptr); }

  void operator=(const CurvatureInfo &curvature_info) {
    CopyFrom(curvature_info);
  }

  void CopyFrom(const CurvatureInfo &curvature_info) {
    offset_ = curvature_info.offset();
    value_ = curvature_info.value();
  }

protected:
  //道路中心线上点位于该道路的位置（与道路起点的距离）
  double offset_;
  double *offset_ptr_ = nullptr;
  //道路中心线上点的曲率值
  double value_;
  double *value_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
