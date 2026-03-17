/**
 * @file    speed_point.hpp
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
class SpeedPoint {
public:
  SpeedPoint() {
    s_ = 0.0;
    t_ = 0.0;
    v_ = 0.0;
    a_ = 0.0;
    da_ = 0.0;
  }
  ~SpeedPoint() = default;

  inline void set_s(const double &s) {
    s_ = s;
    s_ptr_ = &s_;
  }

  inline const double &s() const { return s_; }

  inline double *mutable_s() { return &s_; }

  inline bool has_s() { return (s_ptr_ != nullptr); }

  inline void set_t(const double &t) {
    t_ = t;
    t_ptr_ = &t_;
  }

  inline const double &t() const { return t_; }

  inline double *mutable_t() { return &t_; }

  inline bool has_t() { return (t_ptr_ != nullptr); }

  inline void set_v(const double &v) {
    v_ = v;
    v_ptr_ = &v_;
  }

  inline const double &v() const { return v_; }

  inline double *mutable_v() { return &v_; }

  inline bool has_v() { return (v_ptr_ != nullptr); }

  inline void set_a(const double &a) {
    a_ = a;
    a_ptr_ = &a_;
  }

  inline const double &a() const { return a_; }

  inline double *mutable_a() { return &a_; }

  inline bool has_a() { return (a_ptr_ != nullptr); }

  inline void set_da(const double &da) {
    da_ = da;
    da_ptr_ = &da_;
  }

  inline const double &da() const { return da_; }

  inline double *mutable_da() { return &da_; }

  inline bool has_da() { return (da_ptr_ != nullptr); }

  void operator=(const SpeedPoint &speed_point) { CopyFrom(speed_point); }

  void CopyFrom(const SpeedPoint &speed_point) {
    s_ = speed_point.s();
    t_ = speed_point.t();
    v_ = speed_point.v();
    a_ = speed_point.a();
    da_ = speed_point.da();
  }

protected:
  //里程
  double s_;
  double *s_ptr_ = nullptr;
  //时间
  double t_;
  double *t_ptr_ = nullptr;
  // speed (m/s)
  double v_;
  double *v_ptr_ = nullptr;
  // acceleration (m/s^2)
  double a_;
  double *a_ptr_ = nullptr;
  // jerk (m/s^3)
  double da_;
  double *da_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
