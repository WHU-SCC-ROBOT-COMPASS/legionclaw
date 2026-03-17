/**
 * @file    path_point.hpp
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
class PathPoint {
public:
  PathPoint() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    theta_ = 0.0;
    kappa_ = 0.0;
    s_ = 0.0;
    dkappa_ = 0.0;
    ddkappa_ = 0.0;
    lane_id_ = 0.0;
    x_derivative_ = 0.0;
    y_derivative_ = 0.0;
  }
  ~PathPoint() = default;

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

  inline void set_kappa(const double &kappa) {
    kappa_ = kappa;
    kappa_ptr_ = &kappa_;
  }

  inline const double &kappa() const { return kappa_; }

  inline double *mutable_kappa() { return &kappa_; }

  inline bool has_kappa() { return (kappa_ptr_ != nullptr); }

  inline void set_s(const double &s) {
    s_ = s;
    s_ptr_ = &s_;
  }

  inline const double &s() const { return s_; }

  inline double *mutable_s() { return &s_; }

  inline bool has_s() { return (s_ptr_ != nullptr); }

  inline void set_dkappa(const double &dkappa) {
    dkappa_ = dkappa;
    dkappa_ptr_ = &dkappa_;
  }

  inline const double &dkappa() const { return dkappa_; }

  inline double *mutable_dkappa() { return &dkappa_; }

  inline bool has_dkappa() { return (dkappa_ptr_ != nullptr); }

  inline void set_ddkappa(const double &ddkappa) {
    ddkappa_ = ddkappa;
    ddkappa_ptr_ = &ddkappa_;
  }

  inline const double &ddkappa() const { return ddkappa_; }

  inline double *mutable_ddkappa() { return &ddkappa_; }

  inline bool has_ddkappa() { return (ddkappa_ptr_ != nullptr); }

  inline void set_lane_id(const double &lane_id) {
    lane_id_ = lane_id;
    lane_id_ptr_ = &lane_id_;
  }

  inline const double &lane_id() const { return lane_id_; }

  inline double *mutable_lane_id() { return &lane_id_; }

  inline bool has_lane_id() { return (lane_id_ptr_ != nullptr); }

  inline void set_x_derivative(const double &x_derivative) {
    x_derivative_ = x_derivative;
    x_derivative_ptr_ = &x_derivative_;
  }

  inline const double &x_derivative() const { return x_derivative_; }

  inline double *mutable_x_derivative() { return &x_derivative_; }

  inline bool has_x_derivative() { return (x_derivative_ptr_ != nullptr); }

  inline void set_y_derivative(const double &y_derivative) {
    y_derivative_ = y_derivative;
    y_derivative_ptr_ = &y_derivative_;
  }

  inline const double &y_derivative() const { return y_derivative_; }

  inline double *mutable_y_derivative() { return &y_derivative_; }

  inline bool has_y_derivative() { return (y_derivative_ptr_ != nullptr); }

  void operator=(const PathPoint &path_point) { CopyFrom(path_point); }

  void CopyFrom(const PathPoint &path_point) {
    x_ = path_point.x();
    y_ = path_point.y();
    z_ = path_point.z();
    theta_ = path_point.theta();
    kappa_ = path_point.kappa();
    s_ = path_point.s();
    dkappa_ = path_point.dkappa();
    ddkappa_ = path_point.ddkappa();
    lane_id_ = path_point.lane_id();
    x_derivative_ = path_point.x_derivative();
    y_derivative_ = path_point.y_derivative();
  }

protected:
  // coordinates x
  double x_;
  double *x_ptr_ = nullptr;
  // coordinates y
  double y_;
  double *y_ptr_ = nullptr;
  // coordinates z
  double z_;
  double *z_ptr_ = nullptr;
  // direction on the x-y plane
  double theta_;
  double *theta_ptr_ = nullptr;
  // curvature on the x-y planning
  double kappa_;
  double *kappa_ptr_ = nullptr;
  // accumulated distance from beginning of the path
  double s_;
  double *s_ptr_ = nullptr;
  // derivative of kappa w.r.t s.
  double dkappa_;
  double *dkappa_ptr_ = nullptr;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa_;
  double *ddkappa_ptr_ = nullptr;
  // The lane ID where the path point is on
  double lane_id_;
  double *lane_id_ptr_ = nullptr;
  // derivative of x w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative_;
  double *x_derivative_ptr_ = nullptr;
  // derivative of y w.r.t parametric parameter t in CosThetareferenceline
  double y_derivative_;
  double *y_derivative_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
