/**
 * @file    vehicle_config.hpp
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
#include "modules/common/data/vehicle_param/proto/vehicle_param.pb.h"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class VehicleConfig {
public:
  VehicleConfig() = default;
  ~VehicleConfig() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void
  set_vehicle_param(const legionclaw::interface::VehicleParam &vehicle_param) {
    vehicle_param_ = vehicle_param;
    vehicle_param_ptr_ = &vehicle_param_;
  }

  inline const legionclaw::interface::VehicleParam &vehicle_param() const {
    return vehicle_param_;
  }

  inline legionclaw::interface::VehicleParam *mutable_vehicle_param() {
    return &vehicle_param_;
  }

  inline bool has_vehicle_param() { return (vehicle_param_ptr_ != nullptr); }

  void operator=(const VehicleConfig &vehicle_config) {
    CopyFrom(vehicle_config);
  }

  void CopyFrom(const VehicleConfig &vehicle_config) {
    header_ = vehicle_config.header();
    vehicle_param_ = vehicle_config.vehicle_param();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //车辆参数表
  legionclaw::interface::VehicleParam vehicle_param_;
  legionclaw::interface::VehicleParam *vehicle_param_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
