/**
 * @file    point_basic.hpp
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
class PointBasic {
public:
  PointBasic() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    theta_ = 0.0;
  }
  ~PointBasic() = default;

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

  inline void set_z(const double &z) {
    z_ = z;
    z_ptr_ = &z_;
  }

  inline const double &z() const { return z_; }

  inline double *mutable_z() { return &z_; }

  inline bool has_z() { return (z_ptr_ != nullptr); }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

  void operator=(const PointBasic &point_basic) { CopyFrom(point_basic); }

  void CopyFrom(const PointBasic &point_basic) {
    x_ = point_basic.x();
    y_ = point_basic.y();
    z_ = point_basic.z();
    theta_ = point_basic.theta();
  }

protected:
  //位置坐标x
  double x_;
  double *x_ptr_ = nullptr;
  //位置坐标y
  double y_;
  double *y_ptr_ = nullptr;
  //位置坐标z
  double z_;
  double *z_ptr_ = nullptr;
  // heading(rad)
  double theta_;
  double *theta_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
