/**
 * @file    bbox_2d.hpp
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
class BBox2D {
public:
  BBox2D() {
    xmin_ = 0;
    ymin_ = 0;
    xmax_ = 0;
    ymax_ = 0;
  }
  ~BBox2D() = default;

  inline void set_xmin(const int16_t &xmin) {
    xmin_ = xmin;
    xmin_ptr_ = &xmin_;
  }

  inline const int16_t &xmin() const { return xmin_; }

  inline int16_t *mutable_xmin() { return &xmin_; }

  inline bool has_xmin() { return (xmin_ptr_ != nullptr); }

  inline void set_ymin(const int16_t &ymin) {
    ymin_ = ymin;
    ymin_ptr_ = &ymin_;
  }

  inline const int16_t &ymin() const { return ymin_; }

  inline int16_t *mutable_ymin() { return &ymin_; }

  inline bool has_ymin() { return (ymin_ptr_ != nullptr); }

  inline void set_xmax(const int16_t &xmax) {
    xmax_ = xmax;
    xmax_ptr_ = &xmax_;
  }

  inline const int16_t &xmax() const { return xmax_; }

  inline int16_t *mutable_xmax() { return &xmax_; }

  inline bool has_xmax() { return (xmax_ptr_ != nullptr); }

  inline void set_ymax(const int16_t &ymax) {
    ymax_ = ymax;
    ymax_ptr_ = &ymax_;
  }

  inline const int16_t &ymax() const { return ymax_; }

  inline int16_t *mutable_ymax() { return &ymax_; }

  inline bool has_ymax() { return (ymax_ptr_ != nullptr); }

  void operator=(const BBox2D &bbox_2d) { CopyFrom(bbox_2d); }

  void CopyFrom(const BBox2D &bbox_2d) {
    xmin_ = bbox_2d.xmin();
    ymin_ = bbox_2d.ymin();
    xmax_ = bbox_2d.xmax();
    ymax_ = bbox_2d.ymax();
  }

protected:
  //图像框左上角的x坐标
  int16_t xmin_;
  int16_t *xmin_ptr_ = nullptr;
  //图像框左上角的y坐标
  int16_t ymin_;
  int16_t *ymin_ptr_ = nullptr;
  //图像框右下角的x坐标
  int16_t xmax_;
  int16_t *xmax_ptr_ = nullptr;
  //图像框右下角的y坐标
  int16_t ymax_;
  int16_t *ymax_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
