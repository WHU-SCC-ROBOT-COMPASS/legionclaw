/**
 * @file    image_key_point.hpp
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
class ImageKeyPoint {
public:
  ImageKeyPoint() {
    x_ = 0.0;
    y_ = 0.0;
    confidence_ = 0.0;
  }
  ~ImageKeyPoint() = default;

  inline void set_x(const double &x) {
    x_ = x;
    x_ptr_ = &x_;
  }

  inline const double &x() const { return x_; }

  inline double *mutable_x() { return &x_; }

  inline bool has_x() { return (x_ptr_ != nullptr); }

  inline void set_y(const double &y) {
    y_ = y;
    y_ptr_ = &y_;
  }

  inline const double &y() const { return y_; }

  inline double *mutable_y() { return &y_; }

  inline bool has_y() { return (y_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  void operator=(const ImageKeyPoint &image_key_point) {
    CopyFrom(image_key_point);
  }

  void CopyFrom(const ImageKeyPoint &image_key_point) {
    x_ = image_key_point.x();
    y_ = image_key_point.y();
    confidence_ = image_key_point.confidence();
  }

protected:
  //车位图像关键点x坐标
  double x_;
  double *x_ptr_ = nullptr;
  //车位图像关键点y坐标
  double y_;
  double *y_ptr_ = nullptr;
  //置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
