/**
 * @file    map_boundary.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-30
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <vector>

#include "header.hpp"
#include "polygon_2d.hpp"

/**
 * @namespace motion_manager::interface
 * @brief motion_manager::interface
 */
namespace motion_manager {
namespace interface {
class MapBoundary {
public:
  MapBoundary() {
    boundary_mutex_ = std::make_shared<std::mutex>();

    clear_boundary();
  }
  ~MapBoundary() = default;

  inline void set_header(const motion_manager::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const motion_manager::interface::Header &header() const { return header_; }

  inline motion_manager::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_boundary(std::vector<motion_manager::interface::Polygon2D> *boundary) {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary_.assign(boundary->begin(), boundary->end());
  }

  inline void
  set_boundary(const std::vector<motion_manager::interface::Polygon2D> &boundary) {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary_ = boundary;
  }

  inline void set_boundary(const uint32_t index,
                           motion_manager::interface::Polygon2D &boundary) {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary_[index] = boundary;
  }

  inline void add_boundary(const motion_manager::interface::Polygon2D &boundary) {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary_.emplace_back(boundary);
  }

  inline const motion_manager::interface::Polygon2D &boundary(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    return boundary_[index];
  }

  inline std::vector<motion_manager::interface::Polygon2D> *mutable_boundary() {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    return &boundary_;
  }

  inline void
  boundary(std::vector<motion_manager::interface::Polygon2D> &boundary) const {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary.assign(boundary_.begin(), boundary_.end());
  }

  inline const std::vector<motion_manager::interface::Polygon2D> &boundary() const {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    return boundary_;
  }

  inline uint32_t boundary_size() const {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    return boundary_.size();
  }

  inline void clear_boundary() {
    std::lock_guard<std::mutex> lock(*boundary_mutex_);
    boundary_.clear();
    boundary_.shrink_to_fit();
  }

  inline bool has_boundary() { return (boundary_size() != 0); }

  void operator=(const MapBoundary &map_boundary) { CopyFrom(map_boundary); }

  void CopyFrom(const MapBoundary &map_boundary) {
    header_ = map_boundary.header();
    boundary_ = map_boundary.boundary();
  }

protected:
  std::shared_ptr<std::mutex> boundary_mutex_;
  // timestamp is  included in header
  motion_manager::interface::Header header_;
  motion_manager::interface::Header *header_ptr_ = nullptr;
  //地图边界
  std::vector<motion_manager::interface::Polygon2D> boundary_;
};
} // namespace interface
} // namespace motion_manager
