/**
 * @file    point_llh.hpp
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
class PointLLH {
public:
  PointLLH() {
    lon_ = 0.0;
    lat_ = 0.0;
    height_ = 0.0;
  }
  ~PointLLH() = default;

  inline void set_lon(const double &lon) {
    lon_ = lon;
    lon_ptr_ = &lon_;
  }

  inline const double &lon() const { return lon_; }

  inline double *mutable_lon() { return &lon_; }

  inline bool has_lon() { return (lon_ptr_ != nullptr); }

  inline void set_lat(const double &lat) {
    lat_ = lat;
    lat_ptr_ = &lat_;
  }

  inline const double &lat() const { return lat_; }

  inline double *mutable_lat() { return &lat_; }

  inline bool has_lat() { return (lat_ptr_ != nullptr); }

  inline void set_height(const double &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const double &height() const { return height_; }

  inline double *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  void operator=(const PointLLH &point_llh) { CopyFrom(point_llh); }

  void CopyFrom(const PointLLH &point_llh) {
    lon_ = point_llh.lon();
    lat_ = point_llh.lat();
    height_ = point_llh.height();
  }

protected:
  // Longitude in degrees, ranging from -180 to 180.
  double lon_;
  double *lon_ptr_ = nullptr;
  // Latitude in degrees, ranging from -90 to 90.
  double lat_;
  double *lat_ptr_ = nullptr;
  // WGS-84 ellipsoid height in meters.
  double height_;
  double *height_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
