/**
 * @file    road_mark.hpp
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
class RoadMark {
public:
  RoadMark() {
    longitude_dist_ = 0.0;
    lateral_dist_ = 0.0;
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    confidence_ = 0.0;
    type_ = legionclaw::common::RoadMarkType::STOP_LINE;
  }
  ~RoadMark() = default;

  inline void set_longitude_dist(const double &longitude_dist) {
    longitude_dist_ = longitude_dist;
    longitude_dist_ptr_ = &longitude_dist_;
  }

  inline const double &longitude_dist() const { return longitude_dist_; }

  inline double *mutable_longitude_dist() { return &longitude_dist_; }

  inline bool has_longitude_dist() { return (longitude_dist_ptr_ != nullptr); }

  inline void set_lateral_dist(const double &lateral_dist) {
    lateral_dist_ = lateral_dist;
    lateral_dist_ptr_ = &lateral_dist_;
  }

  inline const double &lateral_dist() const { return lateral_dist_; }

  inline double *mutable_lateral_dist() { return &lateral_dist_; }

  inline bool has_lateral_dist() { return (lateral_dist_ptr_ != nullptr); }

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

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_type(const legionclaw::common::RoadMarkType &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::common::RoadMarkType &type() const { return type_; }

  inline legionclaw::common::RoadMarkType *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  void operator=(const RoadMark &road_mark) { CopyFrom(road_mark); }

  void CopyFrom(const RoadMark &road_mark) {
    longitude_dist_ = road_mark.longitude_dist();
    lateral_dist_ = road_mark.lateral_dist();
    x_ = road_mark.x();
    y_ = road_mark.y();
    z_ = road_mark.z();
    confidence_ = road_mark.confidence();
    type_ = road_mark.type();
  }

protected:
  //相对于车身坐标系原点的纵向距离,单位m
  double longitude_dist_;
  double *longitude_dist_ptr_ = nullptr;
  //相对于车身坐标系原点的横向距离,单位m
  double lateral_dist_;
  double *lateral_dist_ptr_ = nullptr;
  //质心坐标x
  double x_;
  double *x_ptr_ = nullptr;
  //质心坐标y
  double y_;
  double *y_ptr_ = nullptr;
  //质心坐标z
  double z_;
  double *z_ptr_ = nullptr;
  //置信度[0 - 1]
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //标识类型 1 - 停止线 2 - 斑马线 3 - 禁停区 4 - 直行箭头 5 - 左转箭头 6 -
  //右转箭头 7 - 直行+左转 8 - 直行+右转 9 - 直行+左转+右转 10 - 掉头 11 -
  //左掉头 12 - 菱形(前方有人行横道) 13 - 倒三角 (减速让行) 14 - 地面文字
  legionclaw::common::RoadMarkType type_;
  legionclaw::common::RoadMarkType *type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
