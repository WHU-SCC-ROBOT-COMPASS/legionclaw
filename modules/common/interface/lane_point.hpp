/**
 * @file    lane_point.hpp
 * @author  zdhy
 * @date    2024-02-21
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
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class LanePoint {
public:
  LanePoint() {
    theta_ = 0.0;
    mileage_ = 0.0;
    limit_speed_ = 0.0;
    left_road_width_ = 0.0;
    right_road_width_ = 0.0;
    left_line_type_ = legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
    right_line_type_ = legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
  }
  ~LanePoint() = default;

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

  inline void set_mileage(const double &mileage) {
    mileage_ = mileage;
    mileage_ptr_ = &mileage_;
  }

  inline const double &mileage() const { return mileage_; }

  inline double *mutable_mileage() { return &mileage_; }

  inline bool has_mileage() { return (mileage_ptr_ != nullptr); }

  inline void set_limit_speed(const double &limit_speed) {
    limit_speed_ = limit_speed;
    limit_speed_ptr_ = &limit_speed_;
  }

  inline const double &limit_speed() const { return limit_speed_; }

  inline double *mutable_limit_speed() { return &limit_speed_; }

  inline bool has_limit_speed() { return (limit_speed_ptr_ != nullptr); }

  inline void set_left_road_width(const double &left_road_width) {
    left_road_width_ = left_road_width;
    left_road_width_ptr_ = &left_road_width_;
  }

  inline const double &left_road_width() const { return left_road_width_; }

  inline double *mutable_left_road_width() { return &left_road_width_; }

  inline bool has_left_road_width() {
    return (left_road_width_ptr_ != nullptr);
  }

  inline void set_right_road_width(const double &right_road_width) {
    right_road_width_ = right_road_width;
    right_road_width_ptr_ = &right_road_width_;
  }

  inline const double &right_road_width() const { return right_road_width_; }

  inline double *mutable_right_road_width() { return &right_road_width_; }

  inline bool has_right_road_width() {
    return (right_road_width_ptr_ != nullptr);
  }

  inline void
  set_left_line_type(const legionclaw::common::LaneLineType &left_line_type) {
    left_line_type_ = left_line_type;
    left_line_type_ptr_ = &left_line_type_;
  }

  inline const legionclaw::common::LaneLineType &left_line_type() const {
    return left_line_type_;
  }

  inline legionclaw::common::LaneLineType *mutable_left_line_type() {
    return &left_line_type_;
  }

  inline bool has_left_line_type() { return (left_line_type_ptr_ != nullptr); }

  inline void
  set_right_line_type(const legionclaw::common::LaneLineType &right_line_type) {
    right_line_type_ = right_line_type;
    right_line_type_ptr_ = &right_line_type_;
  }

  inline const legionclaw::common::LaneLineType &right_line_type() const {
    return right_line_type_;
  }

  inline legionclaw::common::LaneLineType *mutable_right_line_type() {
    return &right_line_type_;
  }

  inline bool has_right_line_type() {
    return (right_line_type_ptr_ != nullptr);
  }

  void operator=(const LanePoint &lane_point) { CopyFrom(lane_point); }

  void CopyFrom(const LanePoint &lane_point) {
    point_ = lane_point.point();
    theta_ = lane_point.theta();
    mileage_ = lane_point.mileage();
    limit_speed_ = lane_point.limit_speed();
    left_road_width_ = lane_point.left_road_width();
    right_road_width_ = lane_point.right_road_width();
    left_line_type_ = lane_point.left_line_type();
    right_line_type_ = lane_point.right_line_type();
  }

protected:
  //点xyz
  legionclaw::interface::Point3D point_;
  legionclaw::interface::Point3D *point_ptr_ = nullptr;
  //方向（与东方向夹角，逆时针方向为正，单位为弧度）
  double theta_;
  double *theta_ptr_ = nullptr;
  //总体里程
  double mileage_;
  double *mileage_ptr_ = nullptr;
  //地图限速（单位m/s）
  double limit_speed_;
  double *limit_speed_ptr_ = nullptr;
  //左边车道宽
  double left_road_width_;
  double *left_road_width_ptr_ = nullptr;
  //右边车道宽
  double right_road_width_;
  double *right_road_width_ptr_ = nullptr;
  //左边车线线型
  legionclaw::common::LaneLineType left_line_type_;
  legionclaw::common::LaneLineType *left_line_type_ptr_ = nullptr;
  //右边车线线型
  legionclaw::common::LaneLineType right_line_type_;
  legionclaw::common::LaneLineType *right_line_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
