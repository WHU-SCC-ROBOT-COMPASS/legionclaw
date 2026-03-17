/**
 * @file    point_enu.hpp
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
class PointENU {
public:
  PointENU() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
  }
  ~PointENU() = default;

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

  void operator=(const PointENU &point_enu) { CopyFrom(point_enu); }

  void CopyFrom(const PointENU &point_enu) {
    x_ = point_enu.x();
    y_ = point_enu.y();
    z_ = point_enu.z();
  }

protected:
  // East from the origin, in meters.
  double x_;
  double *x_ptr_ = nullptr;
  // North from the origin, in meters.
  double y_;
  double *y_ptr_ = nullptr;
  // Up from the WGS-84 ellipsoid, in meters.
  double z_;
  double *z_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
