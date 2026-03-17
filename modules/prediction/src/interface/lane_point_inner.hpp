/**
 * @file    lane_point_inner.hpp
 * @author  legionclaw
 * @date    2021-11-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/enum/enum.h"
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */
namespace legionclaw {
namespace prediction {
class LanePointInner {
public:
  LanePointInner() {
    theta_ = 0.0;
    kappa_ = 0.0;
    mileage_ = 0.0;
    limit_speed_ = 0.0;
    left_road_width_ = 0.0;
    right_road_width_ = 0.0;
    left_line_type_ = legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
    right_line_type_ = legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
  }
  ~LanePointInner() = default;

  inline void set_point(const legionclaw::interface::Point3D &point) {
    point_ = point;
  }

  inline const legionclaw::interface::Point3D &point() const { return point_; }

  inline legionclaw::interface::Point3D *mutable_point() { return &point_; }

  inline void set_theta(const double &theta) { theta_ = theta; }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline void set_kappa(const double &kappa) { kappa_ = kappa; }

  inline const double &kappa() const { return kappa_; }

  inline double *mutable_kappa() { return &kappa_; }

  inline void set_mileage(const double &mileage) { mileage_ = mileage; }

  inline const double &mileage() const { return mileage_; }

  inline double *mutable_mileage() { return &mileage_; }

  inline void set_limit_speed(const double &limit_speed) {
    limit_speed_ = limit_speed;
  }

  inline const double &limit_speed() const { return limit_speed_; }

  inline double *mutable_limit_speed() { return &limit_speed_; }

  inline void set_left_road_width(const double &left_road_width) {
    left_road_width_ = left_road_width;
  }

  inline const double &left_road_width() const { return left_road_width_; }

  inline double *mutable_left_road_width() { return &left_road_width_; }

  inline void set_right_road_width(const double &right_road_width) {
    right_road_width_ = right_road_width;
  }

  inline const double &right_road_width() const { return right_road_width_; }

  inline double *mutable_right_road_width() { return &right_road_width_; }

  inline void
  set_left_line_type(const legionclaw::common::LaneLineType &left_line_type) {
    left_line_type_ = left_line_type;
  }

  inline const legionclaw::common::LaneLineType &left_line_type() const {
    return left_line_type_;
  }

  inline legionclaw::common::LaneLineType *mutable_left_line_type() {
    return &left_line_type_;
  }

  inline void
  set_right_line_type(const legionclaw::common::LaneLineType &right_line_type) {
    right_line_type_ = right_line_type;
  }

  inline const legionclaw::common::LaneLineType &right_line_type() const {
    return right_line_type_;
  }

  inline legionclaw::common::LaneLineType *mutable_right_line_type() {
    return &right_line_type_;
  }

protected:
  //点xyz
  legionclaw::interface::Point3D point_;
  //方向（与东方向夹角，逆时针方向为正，单位为弧度）
  double theta_;
  //曲率
  double kappa_;
  //总体里程
  double mileage_;
  //地图限速（单位m/s）
  double limit_speed_;
  //左边车道宽
  double left_road_width_;
  //右边车道宽
  double right_road_width_;
  legionclaw::common::LaneLineType left_line_type_;
  legionclaw::common::LaneLineType right_line_type_;
};
} // namespace prediction
} // namespace legionclaw
