/**
 * @file    vehicle_motion.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

#include "modules/common/interface/vehicle_motion_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleMotion {
public:
  VehicleMotion() {
    vehicle_motion_point_mutex_ = std::make_shared<std::mutex>();

    name_.clear();
    clear_vehicle_motion_point();
  }
  ~VehicleMotion() = default;

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_vehicle_motion_point(
      std::vector<legionclaw::interface::VehicleMotionPoint> *vehicle_motion_point) {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point_.assign(vehicle_motion_point->begin(),
                                 vehicle_motion_point->end());
  }

  inline void set_vehicle_motion_point(
      const std::vector<legionclaw::interface::VehicleMotionPoint>
          &vehicle_motion_point) {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point_ = vehicle_motion_point;
  }

  inline void set_vehicle_motion_point(
      const uint32_t index,
      legionclaw::interface::VehicleMotionPoint &vehicle_motion_point) {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point_[index] = vehicle_motion_point;
  }

  inline void add_vehicle_motion_point(
      const legionclaw::interface::VehicleMotionPoint &vehicle_motion_point) {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point_.emplace_back(vehicle_motion_point);
  }

  inline const legionclaw::interface::VehicleMotionPoint &
  vehicle_motion_point(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    return vehicle_motion_point_[index];
  }

  inline std::vector<legionclaw::interface::VehicleMotionPoint> *
  mutable_vehicle_motion_point() {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    return &vehicle_motion_point_;
  }

  inline void vehicle_motion_point(
      std::vector<legionclaw::interface::VehicleMotionPoint> &vehicle_motion_point)
      const {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point.assign(vehicle_motion_point_.begin(),
                                vehicle_motion_point_.end());
  }

  inline const std::vector<legionclaw::interface::VehicleMotionPoint> &
  vehicle_motion_point() const {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    return vehicle_motion_point_;
  }

  inline uint32_t vehicle_motion_point_size() const {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    return vehicle_motion_point_.size();
  }

  inline void clear_vehicle_motion_point() {
    std::lock_guard<std::mutex> lock(*vehicle_motion_point_mutex_);
    vehicle_motion_point_.clear();
    vehicle_motion_point_.shrink_to_fit();
  }

  inline bool has_vehicle_motion_point() {
    return (vehicle_motion_point_size() != 0);
  }

  void operator=(const VehicleMotion &vehicle_motion) {
    CopyFrom(vehicle_motion);
  }

  void CopyFrom(const VehicleMotion &vehicle_motion) {
    name_ = vehicle_motion.name();
    vehicle_motion_point_ = vehicle_motion.vehicle_motion_point();
  }

protected:
  std::shared_ptr<std::mutex> vehicle_motion_point_mutex_;
  //名称
  std::string name_;
  std::string *name_ptr_ = nullptr;
  std::vector<legionclaw::interface::VehicleMotionPoint> vehicle_motion_point_;
};
} // namespace interface
} // namespace legionclaw
