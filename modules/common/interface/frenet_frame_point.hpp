/**
 * @file    frenet_frame_point.hpp
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
class FrenetFramePoint {
public:
  FrenetFramePoint() {
    s_ = 0.0;
    l_ = 0.0;
    dl_ = 0.0;
    ddl_ = 0.0;
  }
  ~FrenetFramePoint() = default;

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

  inline void set_dl(const double &dl) {
    dl_ = dl;
    dl_ptr_ = &dl_;
  }

  inline const double &dl() const { return dl_; }

  inline double *mutable_dl() { return &dl_; }

  inline bool has_dl() { return (dl_ptr_ != nullptr); }

  inline void set_ddl(const double &ddl) {
    ddl_ = ddl;
    ddl_ptr_ = &ddl_;
  }

  inline const double &ddl() const { return ddl_; }

  inline double *mutable_ddl() { return &ddl_; }

  inline bool has_ddl() { return (ddl_ptr_ != nullptr); }

  void operator=(const FrenetFramePoint &frenet_frame_point) {
    CopyFrom(frenet_frame_point);
  }

  void CopyFrom(const FrenetFramePoint &frenet_frame_point) {
    s_ = frenet_frame_point.s();
    l_ = frenet_frame_point.l();
    dl_ = frenet_frame_point.dl();
    ddl_ = frenet_frame_point.ddl();
  }

protected:
  //纵向位移
  double s_;
  double *s_ptr_ = nullptr;
  //横向位移
  double l_;
  double *l_ptr_ = nullptr;
  //横向位移变化率
  double dl_;
  double *dl_ptr_ = nullptr;
  //横向位移变化率的变化率
  double ddl_;
  double *ddl_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
