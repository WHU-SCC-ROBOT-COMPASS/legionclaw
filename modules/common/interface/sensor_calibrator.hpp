/**
 * @file    sensor_calibrator.hpp
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
class SensorCalibrator {
public:
  SensorCalibrator() = default;
  ~SensorCalibrator() = default;

  inline void set_pose(const legionclaw::interface::Point3D &pose) {
    pose_ = pose;
    pose_ptr_ = &pose_;
  }

  inline const legionclaw::interface::Point3D &pose() const { return pose_; }

  inline legionclaw::interface::Point3D *mutable_pose() { return &pose_; }

  inline bool has_pose() { return (pose_ptr_ != nullptr); }

  inline void set_angle(const legionclaw::interface::Point3D &angle) {
    angle_ = angle;
    angle_ptr_ = &angle_;
  }

  inline const legionclaw::interface::Point3D &angle() const { return angle_; }

  inline legionclaw::interface::Point3D *mutable_angle() { return &angle_; }

  inline bool has_angle() { return (angle_ptr_ != nullptr); }

  void operator=(const SensorCalibrator &sensor_calibrator) {
    CopyFrom(sensor_calibrator);
  }

  void CopyFrom(const SensorCalibrator &sensor_calibrator) {
    pose_ = sensor_calibrator.pose();
    angle_ = sensor_calibrator.angle();
  }

protected:
  //传感器安装位置(相对于后轴中心点)
  legionclaw::interface::Point3D pose_;
  legionclaw::interface::Point3D *pose_ptr_ = nullptr;
  //传感器安装角度(车体坐标系)
  legionclaw::interface::Point3D angle_;
  legionclaw::interface::Point3D *angle_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
