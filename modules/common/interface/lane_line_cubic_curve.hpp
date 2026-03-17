/**
 * @file    lane_line_cubic_curve.hpp
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
class LaneLineCubicCurve {
public:
  LaneLineCubicCurve() {
    start_x_ = 0.0;
    end_x_ = 0.0;
    a_ = 0.0;
    b_ = 0.0;
    c_ = 0.0;
    d_ = 0.0;
  }
  ~LaneLineCubicCurve() = default;

  inline void set_start_x(const double &start_x) {
    start_x_ = start_x;
    start_x_ptr_ = &start_x_;
  }

  inline const double &start_x() const { return start_x_; }

  inline double *mutable_start_x() { return &start_x_; }

  inline bool has_start_x() { return (start_x_ptr_ != nullptr); }

  inline void set_end_x(const double &end_x) {
    end_x_ = end_x;
    end_x_ptr_ = &end_x_;
  }

  inline const double &end_x() const { return end_x_; }

  inline double *mutable_end_x() { return &end_x_; }

  inline bool has_end_x() { return (end_x_ptr_ != nullptr); }

  inline void set_a(const double &a) {
    a_ = a;
    a_ptr_ = &a_;
  }

  inline const double &a() const { return a_; }

  inline double *mutable_a() { return &a_; }

  inline bool has_a() { return (a_ptr_ != nullptr); }

  inline void set_b(const double &b) {
    b_ = b;
    b_ptr_ = &b_;
  }

  inline const double &b() const { return b_; }

  inline double *mutable_b() { return &b_; }

  inline bool has_b() { return (b_ptr_ != nullptr); }

  inline void set_c(const double &c) {
    c_ = c;
    c_ptr_ = &c_;
  }

  inline const double &c() const { return c_; }

  inline double *mutable_c() { return &c_; }

  inline bool has_c() { return (c_ptr_ != nullptr); }

  inline void set_d(const double &d) {
    d_ = d;
    d_ptr_ = &d_;
  }

  inline const double &d() const { return d_; }

  inline double *mutable_d() { return &d_; }

  inline bool has_d() { return (d_ptr_ != nullptr); }

  void operator=(const LaneLineCubicCurve &lane_line_cubic_curve) {
    CopyFrom(lane_line_cubic_curve);
  }

  void CopyFrom(const LaneLineCubicCurve &lane_line_cubic_curve) {
    start_x_ = lane_line_cubic_curve.start_x();
    end_x_ = lane_line_cubic_curve.end_x();
    a_ = lane_line_cubic_curve.a();
    b_ = lane_line_cubic_curve.b();
    c_ = lane_line_cubic_curve.c();
    d_ = lane_line_cubic_curve.d();
  }

protected:
  //车道线起始位置
  double start_x_;
  double *start_x_ptr_ = nullptr;
  //车道线结束位置
  double end_x_;
  double *end_x_ptr_ = nullptr;
  //三次曲线系数
  double a_;
  double *a_ptr_ = nullptr;
  double b_;
  double *b_ptr_ = nullptr;
  double c_;
  double *c_ptr_ = nullptr;
  double d_;
  double *d_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
