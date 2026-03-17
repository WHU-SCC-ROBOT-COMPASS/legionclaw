/**
 * @file    twist.hpp
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
class Twist {
public:
  Twist() = default;
  ~Twist() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_linear(const legionclaw::interface::Point3D &linear) {
    linear_ = linear;
    linear_ptr_ = &linear_;
  }

  inline const legionclaw::interface::Point3D &linear() const { return linear_; }

  inline legionclaw::interface::Point3D *mutable_linear() { return &linear_; }

  inline bool has_linear() { return (linear_ptr_ != nullptr); }

  inline void set_angular(const legionclaw::interface::Point3D &angular) {
    angular_ = angular;
    angular_ptr_ = &angular_;
  }

  inline const legionclaw::interface::Point3D &angular() const { return angular_; }

  inline legionclaw::interface::Point3D *mutable_angular() { return &angular_; }

  inline bool has_angular() { return (angular_ptr_ != nullptr); }

  void operator=(const Twist &twist) { CopyFrom(twist); }

  void CopyFrom(const Twist &twist) {
    header_ = twist.header();
    linear_ = twist.linear();
    angular_ = twist.angular();
  }

protected:
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //线速度 m/s
  legionclaw::interface::Point3D linear_;
  legionclaw::interface::Point3D *linear_ptr_ = nullptr;
  //角速度 deg/s
  legionclaw::interface::Point3D angular_;
  legionclaw::interface::Point3D *angular_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
