/**
 * @file    quaternion.hpp
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
class Quaternion {
public:
  Quaternion() {
    qx_ = 0.0;
    qy_ = 0.0;
    qz_ = 0.0;
    qw_ = 0.0;
  }
  ~Quaternion() = default;

  inline void set_qx(const double &qx) {
    qx_ = qx;
    qx_ptr_ = &qx_;
  }

  inline const double &qx() const { return qx_; }

  inline double *mutable_qx() { return &qx_; }

  inline bool has_qx() { return (qx_ptr_ != nullptr); }

  inline void set_qy(const double &qy) {
    qy_ = qy;
    qy_ptr_ = &qy_;
  }

  inline const double &qy() const { return qy_; }

  inline double *mutable_qy() { return &qy_; }

  inline bool has_qy() { return (qy_ptr_ != nullptr); }

  inline void set_qz(const double &qz) {
    qz_ = qz;
    qz_ptr_ = &qz_;
  }

  inline const double &qz() const { return qz_; }

  inline double *mutable_qz() { return &qz_; }

  inline bool has_qz() { return (qz_ptr_ != nullptr); }

  inline void set_qw(const double &qw) {
    qw_ = qw;
    qw_ptr_ = &qw_;
  }

  inline const double &qw() const { return qw_; }

  inline double *mutable_qw() { return &qw_; }

  inline bool has_qw() { return (qw_ptr_ != nullptr); }

  void operator=(const Quaternion &quaternion) { CopyFrom(quaternion); }

  void CopyFrom(const Quaternion &quaternion) {
    qx_ = quaternion.qx();
    qy_ = quaternion.qy();
    qz_ = quaternion.qz();
    qw_ = quaternion.qw();
  }

protected:
  //四维空间x
  double qx_;
  double *qx_ptr_ = nullptr;
  //四维空间y
  double qy_;
  double *qy_ptr_ = nullptr;
  //四维空间z
  double qz_;
  double *qz_ptr_ = nullptr;
  //四维空间w
  double qw_;
  double *qw_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
