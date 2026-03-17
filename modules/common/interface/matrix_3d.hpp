/**
 * @file    matrix_3d.hpp
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
class Matrix3D {
public:
  Matrix3D() {
    a00_ = 0.0;
    a01_ = 0.0;
    a02_ = 0.0;
    a10_ = 0.0;
    a11_ = 0.0;
    a12_ = 0.0;
    a20_ = 0.0;
    a21_ = 0.0;
    a22_ = 0.0;
  }
  ~Matrix3D() = default;

  inline void set_a00(const double &a00) {
    a00_ = a00;
    a00_ptr_ = &a00_;
  }

  inline const double &a00() const { return a00_; }

  inline double *mutable_a00() { return &a00_; }

  inline bool has_a00() { return (a00_ptr_ != nullptr); }

  inline void set_a01(const double &a01) {
    a01_ = a01;
    a01_ptr_ = &a01_;
  }

  inline const double &a01() const { return a01_; }

  inline double *mutable_a01() { return &a01_; }

  inline bool has_a01() { return (a01_ptr_ != nullptr); }

  inline void set_a02(const double &a02) {
    a02_ = a02;
    a02_ptr_ = &a02_;
  }

  inline const double &a02() const { return a02_; }

  inline double *mutable_a02() { return &a02_; }

  inline bool has_a02() { return (a02_ptr_ != nullptr); }

  inline void set_a10(const double &a10) {
    a10_ = a10;
    a10_ptr_ = &a10_;
  }

  inline const double &a10() const { return a10_; }

  inline double *mutable_a10() { return &a10_; }

  inline bool has_a10() { return (a10_ptr_ != nullptr); }

  inline void set_a11(const double &a11) {
    a11_ = a11;
    a11_ptr_ = &a11_;
  }

  inline const double &a11() const { return a11_; }

  inline double *mutable_a11() { return &a11_; }

  inline bool has_a11() { return (a11_ptr_ != nullptr); }

  inline void set_a12(const double &a12) {
    a12_ = a12;
    a12_ptr_ = &a12_;
  }

  inline const double &a12() const { return a12_; }

  inline double *mutable_a12() { return &a12_; }

  inline bool has_a12() { return (a12_ptr_ != nullptr); }

  inline void set_a20(const double &a20) {
    a20_ = a20;
    a20_ptr_ = &a20_;
  }

  inline const double &a20() const { return a20_; }

  inline double *mutable_a20() { return &a20_; }

  inline bool has_a20() { return (a20_ptr_ != nullptr); }

  inline void set_a21(const double &a21) {
    a21_ = a21;
    a21_ptr_ = &a21_;
  }

  inline const double &a21() const { return a21_; }

  inline double *mutable_a21() { return &a21_; }

  inline bool has_a21() { return (a21_ptr_ != nullptr); }

  inline void set_a22(const double &a22) {
    a22_ = a22;
    a22_ptr_ = &a22_;
  }

  inline const double &a22() const { return a22_; }

  inline double *mutable_a22() { return &a22_; }

  inline bool has_a22() { return (a22_ptr_ != nullptr); }

  void operator=(const Matrix3D &matrix_3d) { CopyFrom(matrix_3d); }

  void CopyFrom(const Matrix3D &matrix_3d) {
    a00_ = matrix_3d.a00();
    a01_ = matrix_3d.a01();
    a02_ = matrix_3d.a02();
    a10_ = matrix_3d.a10();
    a11_ = matrix_3d.a11();
    a12_ = matrix_3d.a12();
    a20_ = matrix_3d.a20();
    a21_ = matrix_3d.a21();
    a22_ = matrix_3d.a22();
  }

protected:
  //第一行第一列元素
  double a00_;
  double *a00_ptr_ = nullptr;
  //第一行第二列元素
  double a01_;
  double *a01_ptr_ = nullptr;
  //第一行第三列元素
  double a02_;
  double *a02_ptr_ = nullptr;
  //第二行第一列元素
  double a10_;
  double *a10_ptr_ = nullptr;
  //第二行第二列元素
  double a11_;
  double *a11_ptr_ = nullptr;
  //第二行第三列元素
  double a12_;
  double *a12_ptr_ = nullptr;
  //第三行第一列元素
  double a20_;
  double *a20_ptr_ = nullptr;
  //第三行第二列元素
  double a21_;
  double *a21_ptr_ = nullptr;
  //第三行第三列元素
  double a22_;
  double *a22_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
