/**
 * @file    uss_obstacle.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class UssObstacle {
public:
  UssObstacle() {
    uss_obstacle_region_ = legionclaw::common::UssObstacleRegion::USS_BACK;
    mx_ = 0.0;
    my_ = 0.0;
  }
  ~UssObstacle() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_uss_obstacle_region(
      const legionclaw::common::UssObstacleRegion &uss_obstacle_region) {
    uss_obstacle_region_ = uss_obstacle_region;
    uss_obstacle_region_ptr_ = &uss_obstacle_region_;
  }

  inline const legionclaw::common::UssObstacleRegion &uss_obstacle_region() const {
    return uss_obstacle_region_;
  }

  inline legionclaw::common::UssObstacleRegion *mutable_uss_obstacle_region() {
    return &uss_obstacle_region_;
  }

  inline bool has_uss_obstacle_region() {
    return (uss_obstacle_region_ptr_ != nullptr);
  }

  inline void set_mx(const double &mx) {
    mx_ = mx;
    mx_ptr_ = &mx_;
  }

  inline const double &mx() const { return mx_; }

  inline double *mutable_mx() { return &mx_; }

  inline bool has_mx() { return (mx_ptr_ != nullptr); }

  inline void set_my(const double &my) {
    my_ = my;
    my_ptr_ = &my_;
  }

  inline const double &my() const { return my_; }

  inline double *mutable_my() { return &my_; }

  inline bool has_my() { return (my_ptr_ != nullptr); }

  void operator=(const UssObstacle &uss_obstacle) { CopyFrom(uss_obstacle); }

  void CopyFrom(const UssObstacle &uss_obstacle) {
    header_ = uss_obstacle.header();
    uss_obstacle_region_ = uss_obstacle.uss_obstacle_region();
    mx_ = uss_obstacle.mx();
    my_ = uss_obstacle.my();
  }

protected:
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //超声波障碍物点区域 0-车辆后方区域 1-车辆前方区域 2-车辆右方区域
  //3-车辆左方区域
  legionclaw::common::UssObstacleRegion uss_obstacle_region_;
  legionclaw::common::UssObstacleRegion *uss_obstacle_region_ptr_ = nullptr;
  //车体中心坐标系下，超声波障碍物点x坐标
  double mx_;
  double *mx_ptr_ = nullptr;
  //车体中心坐标系下，超声波障碍物点y坐标
  double my_;
  double *my_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
