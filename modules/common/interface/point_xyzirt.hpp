/**
 * @file    point_xyzirt.hpp
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
class PointXYZIRT {
public:
  PointXYZIRT() {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    intensity_ = 0;
    ring_id_ = 0;
    timestamp_ = 0;
  }
  ~PointXYZIRT() = default;

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

  inline void set_intensity(const uint32_t &intensity) {
    intensity_ = intensity;
    intensity_ptr_ = &intensity_;
  }

  inline const uint32_t &intensity() const { return intensity_; }

  inline uint32_t *mutable_intensity() { return &intensity_; }

  inline bool has_intensity() { return (intensity_ptr_ != nullptr); }

  inline void set_ring_id(const uint8_t &ring_id) {
    ring_id_ = ring_id;
    ring_id_ptr_ = &ring_id_;
  }

  inline const uint8_t &ring_id() const { return ring_id_; }

  inline uint8_t *mutable_ring_id() { return &ring_id_; }

  inline bool has_ring_id() { return (ring_id_ptr_ != nullptr); }

  inline void set_timestamp(const uint64_t &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const uint64_t &timestamp() const { return timestamp_; }

  inline uint64_t *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  void operator=(const PointXYZIRT &point_xyzirt) { CopyFrom(point_xyzirt); }

  void CopyFrom(const PointXYZIRT &point_xyzirt) {
    x_ = point_xyzirt.x();
    y_ = point_xyzirt.y();
    z_ = point_xyzirt.z();
    intensity_ = point_xyzirt.intensity();
    ring_id_ = point_xyzirt.ring_id();
    timestamp_ = point_xyzirt.timestamp();
  }

protected:
  // x//坐标x
  double x_;
  double *x_ptr_ = nullptr;
  // y//坐标y
  double y_;
  double *y_ptr_ = nullptr;
  // z//坐标z
  double z_;
  double *z_ptr_ = nullptr;
  //反射强度
  uint32_t intensity_;
  uint32_t *intensity_ptr_ = nullptr;
  //线束ID
  uint8_t ring_id_;
  uint8_t *ring_id_ptr_ = nullptr;
  //时间戳
  uint64_t timestamp_;
  uint64_t *timestamp_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
