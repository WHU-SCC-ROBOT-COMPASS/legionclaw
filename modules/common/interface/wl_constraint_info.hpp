/**
 * @file    wl_constraint_info.hpp
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
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class WLConstraintInfo {
public:
  WLConstraintInfo() { id_ = 0; }
  ~WLConstraintInfo() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void
  set_vehicle_point1(const legionclaw::interface::Point3D &vehicle_point1) {
    vehicle_point1_ = vehicle_point1;
    vehicle_point1_ptr_ = &vehicle_point1_;
  }

  inline const legionclaw::interface::Point3D &vehicle_point1() const {
    return vehicle_point1_;
  }

  inline legionclaw::interface::Point3D *mutable_vehicle_point1() {
    return &vehicle_point1_;
  }

  inline bool has_vehicle_point1() { return (vehicle_point1_ptr_ != nullptr); }

  inline void
  set_vehicle_point2(const legionclaw::interface::Point3D &vehicle_point2) {
    vehicle_point2_ = vehicle_point2;
    vehicle_point2_ptr_ = &vehicle_point2_;
  }

  inline const legionclaw::interface::Point3D &vehicle_point2() const {
    return vehicle_point2_;
  }

  inline legionclaw::interface::Point3D *mutable_vehicle_point2() {
    return &vehicle_point2_;
  }

  inline bool has_vehicle_point2() { return (vehicle_point2_ptr_ != nullptr); }

  inline void set_abs_point1(const legionclaw::interface::Point3D &abs_point1) {
    abs_point1_ = abs_point1;
    abs_point1_ptr_ = &abs_point1_;
  }

  inline const legionclaw::interface::Point3D &abs_point1() const {
    return abs_point1_;
  }

  inline legionclaw::interface::Point3D *mutable_abs_point1() {
    return &abs_point1_;
  }

  inline bool has_abs_point1() { return (abs_point1_ptr_ != nullptr); }

  inline void set_abs_point2(const legionclaw::interface::Point3D &abs_point2) {
    abs_point2_ = abs_point2;
    abs_point2_ptr_ = &abs_point2_;
  }

  inline const legionclaw::interface::Point3D &abs_point2() const {
    return abs_point2_;
  }

  inline legionclaw::interface::Point3D *mutable_abs_point2() {
    return &abs_point2_;
  }

  inline bool has_abs_point2() { return (abs_point2_ptr_ != nullptr); }

  void operator=(const WLConstraintInfo &wl_constraint_info) {
    CopyFrom(wl_constraint_info);
  }

  void CopyFrom(const WLConstraintInfo &wl_constraint_info) {
    header_ = wl_constraint_info.header();
    id_ = wl_constraint_info.id();
    vehicle_point1_ = wl_constraint_info.vehicle_point1();
    vehicle_point2_ = wl_constraint_info.vehicle_point2();
    abs_point1_ = wl_constraint_info.abs_point1();
    abs_point2_ = wl_constraint_info.abs_point2();
  }

protected:
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // id
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //车辆坐标系下约束点1
  legionclaw::interface::Point3D vehicle_point1_;
  legionclaw::interface::Point3D *vehicle_point1_ptr_ = nullptr;
  //车辆坐标系下约束点2
  legionclaw::interface::Point3D vehicle_point2_;
  legionclaw::interface::Point3D *vehicle_point2_ptr_ = nullptr;
  //绝对坐标系下约束点1
  legionclaw::interface::Point3D abs_point1_;
  legionclaw::interface::Point3D *abs_point1_ptr_ = nullptr;
  //绝对坐标系下约束点2
  legionclaw::interface::Point3D abs_point2_;
  legionclaw::interface::Point3D *abs_point2_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
