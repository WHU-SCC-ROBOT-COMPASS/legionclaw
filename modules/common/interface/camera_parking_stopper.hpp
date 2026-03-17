/**
 * @file    camera_parking_stopper.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/bbox_2d.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CameraParkingStopper {
public:
  CameraParkingStopper() = default;
  ~CameraParkingStopper() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_bbox2d(const legionclaw::interface::BBox2D &bbox2d) {
    bbox2d_ = bbox2d;
    bbox2d_ptr_ = &bbox2d_;
  }

  inline const legionclaw::interface::BBox2D &bbox2d() const { return bbox2d_; }

  inline legionclaw::interface::BBox2D *mutable_bbox2d() { return &bbox2d_; }

  inline bool has_bbox2d() { return (bbox2d_ptr_ != nullptr); }

  void operator=(const CameraParkingStopper &camera_parking_stopper) {
    CopyFrom(camera_parking_stopper);
  }

  void CopyFrom(const CameraParkingStopper &camera_parking_stopper) {
    header_ = camera_parking_stopper.header();
    bbox2d_ = camera_parking_stopper.bbox2d();
  }

protected:
  // timestamp is  included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //限位器图像框
  legionclaw::interface::BBox2D bbox2d_;
  legionclaw::interface::BBox2D *bbox2d_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
