/**
 * @file    sl_point.hpp
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
class SLPoint {
public:
  SLPoint() {
    s_ = 0.0;
    l_ = 0.0;
  }
  ~SLPoint() = default;

  inline void set_s(const double &s) {
    s_ = s;
    s_ptr_ = &s_;
  }

  inline const double &s() const { return s_; }

  inline double *mutable_s() { return &s_; }

  inline bool has_s() { return (s_ptr_ != nullptr); }

  inline void set_l(const double &l) {
    l_ = l;
    l_ptr_ = &l_;
  }

  inline const double &l() const { return l_; }

  inline double *mutable_l() { return &l_; }

  inline bool has_l() { return (l_ptr_ != nullptr); }

  void operator=(const SLPoint &sl_point) { CopyFrom(sl_point); }

  void CopyFrom(const SLPoint &sl_point) {
    s_ = sl_point.s();
    l_ = sl_point.l();
  }

protected:
  //纵向位移
  double s_;
  double *s_ptr_ = nullptr;
  //横向位移
  double l_;
  double *l_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
