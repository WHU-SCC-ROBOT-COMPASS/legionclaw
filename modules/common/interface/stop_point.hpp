/**
 * @file    stop_point.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class StopPoint {
public:
  StopPoint() {
    theta_ = 0.0;
    type_ = 0;
    stop_distance_ = 0.0;
  }
  ~StopPoint() = default;

  inline void set_point(const legionclaw::interface::Point3D &point) {
    point_ = point;
    point_ptr_ = &point_;
  }

  inline const legionclaw::interface::Point3D &point() const { return point_; }

  inline legionclaw::interface::Point3D *mutable_point() { return &point_; }

  inline bool has_point() { return (point_ptr_ != nullptr); }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

  inline void set_type(const int8_t &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const int8_t &type() const { return type_; }

  inline int8_t *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void set_stop_distance(const double &stop_distance) {
    stop_distance_ = stop_distance;
    stop_distance_ptr_ = &stop_distance_;
  }

  inline const double &stop_distance() const { return stop_distance_; }

  inline double *mutable_stop_distance() { return &stop_distance_; }

  inline bool has_stop_distance() { return (stop_distance_ptr_ != nullptr); }

  void operator=(const StopPoint &stop_point) { CopyFrom(stop_point); }

  void CopyFrom(const StopPoint &stop_point) {
    point_ = stop_point.point();
    theta_ = stop_point.theta();
    type_ = stop_point.type();
    stop_distance_ = stop_point.stop_distance();
  }

protected:
  //点xyz
  legionclaw::interface::Point3D point_;
  legionclaw::interface::Point3D *point_ptr_ = nullptr;
  //方向（与东方向夹角，逆时针方向为正，单位为弧度）
  double theta_;
  double *theta_ptr_ = nullptr;
  // STOP_TYPE_TERMINAL=0 //目的地终点 STOP_TYPE_JUNCTION=1 //路口停止线
  int8_t type_;
  int8_t *type_ptr_ = nullptr;
  //距停止点多少米停车
  double stop_distance_;
  double *stop_distance_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
