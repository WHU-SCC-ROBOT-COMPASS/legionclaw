/**
 * @file    image_rect.hpp
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
class ImageRect {
public:
  ImageRect() {
    x_ = 0;
    y_ = 0;
    width_ = 0;
    height_ = 0;
  }
  ~ImageRect() = default;

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

  void operator=(const ImageRect &image_rect) { CopyFrom(image_rect); }

  void CopyFrom(const ImageRect &image_rect) {
    x_ = image_rect.x();
    y_ = image_rect.y();
    width_ = image_rect.width();
    height_ = image_rect.height();
  }

protected:
  // coordinate x of left-top point of image rect
  int32_t x_;
  int32_t *x_ptr_ = nullptr;
  // coordinate y of left-top point of image rect
  int32_t y_;
  int32_t *y_ptr_ = nullptr;
  // width of image rect
  int32_t width_;
  int32_t *width_ptr_ = nullptr;
  // height of image rect
  int32_t height_;
  int32_t *height_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
