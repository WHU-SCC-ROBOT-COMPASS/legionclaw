/**
 * @file    grid.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/region.hpp"
#include "modules/common/interface/sl_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Grid {
public:
  Grid() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    potential_ = 0.0;
    region_id_ = legionclaw::interface::Region::RegionID::REGION_ID_UNKNOWN;
  }
  ~Grid() = default;

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

  inline void set_sl_point(const legionclaw::interface::SLPoint &sl_point) {
    sl_point_ = sl_point;
    sl_point_ptr_ = &sl_point_;
  }

  inline const legionclaw::interface::SLPoint &sl_point() const { return sl_point_; }

  inline legionclaw::interface::SLPoint *mutable_sl_point() { return &sl_point_; }

  inline bool has_sl_point() { return (sl_point_ptr_ != nullptr); }

  inline void set_yaw(const double &yaw) {
    yaw_ = yaw;
    yaw_ptr_ = &yaw_;
  }

  inline const double &yaw() const { return yaw_; }

  inline double *mutable_yaw() { return &yaw_; }

  inline bool has_yaw() { return (yaw_ptr_ != nullptr); }

  inline void set_potential(const double &potential) {
    potential_ = potential;
    potential_ptr_ = &potential_;
  }

  inline const double &potential() const { return potential_; }

  inline double *mutable_potential() { return &potential_; }

  inline bool has_potential() { return (potential_ptr_ != nullptr); }

  inline void
  set_region_id(const legionclaw::interface::Region::RegionID &region_id) {
    region_id_ = region_id;
    region_id_ptr_ = &region_id_;
  }

  inline const legionclaw::interface::Region::RegionID &region_id() const {
    return region_id_;
  }

  inline legionclaw::interface::Region::RegionID *mutable_region_id() {
    return &region_id_;
  }

  inline bool has_region_id() { return (region_id_ptr_ != nullptr); }

  void operator=(const Grid &grid) { CopyFrom(grid); }

  void CopyFrom(const Grid &grid) {
    x_ = grid.x();
    y_ = grid.y();
    sl_point_ = grid.sl_point();
    yaw_ = grid.yaw();
    potential_ = grid.potential();
    region_id_ = grid.region_id();
  }

protected:
  //世界坐标系下的x
  double x_;
  double *x_ptr_ = nullptr;
  //世界坐标系下的y
  double y_;
  double *y_ptr_ = nullptr;
  // Frenet坐标系下的x
  legionclaw::interface::SLPoint sl_point_;
  legionclaw::interface::SLPoint *sl_point_ptr_ = nullptr;
  double yaw_;
  double *yaw_ptr_ = nullptr;
  //势场值
  double potential_;
  double *potential_ptr_ = nullptr;
  //区域编号
  legionclaw::interface::Region::RegionID region_id_;
  legionclaw::interface::Region::RegionID *region_id_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
