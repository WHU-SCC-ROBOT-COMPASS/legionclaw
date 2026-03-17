/**
 * @file    rss_info.hpp
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
class RSSInfo {
public:
  RSSInfo() {
    is_rss_safe_ = false;
    cur_dist_lon_ = 0.0;
    rss_safe_dist_lon_ = 0.0;
    acc_lon_range_minimum_ = 0.0;
    acc_lon_range_maximum_ = 0.0;
    acc_lat_left_range_minimum_ = 0.0;
    acc_lat_left_range_maximum_ = 0.0;
    acc_lat_right_range_minimum_ = 0.0;
    acc_lat_right_range_maximum_ = 0.0;
  }
  ~RSSInfo() = default;

  inline void set_is_rss_safe(const bool &is_rss_safe) {
    is_rss_safe_ = is_rss_safe;
    is_rss_safe_ptr_ = &is_rss_safe_;
  }

  inline const bool &is_rss_safe() const { return is_rss_safe_; }

  inline bool *mutable_is_rss_safe() { return &is_rss_safe_; }

  inline bool has_is_rss_safe() { return (is_rss_safe_ptr_ != nullptr); }

  inline void set_cur_dist_lon(const double &cur_dist_lon) {
    cur_dist_lon_ = cur_dist_lon;
    cur_dist_lon_ptr_ = &cur_dist_lon_;
  }

  inline const double &cur_dist_lon() const { return cur_dist_lon_; }

  inline double *mutable_cur_dist_lon() { return &cur_dist_lon_; }

  inline bool has_cur_dist_lon() { return (cur_dist_lon_ptr_ != nullptr); }

  inline void set_rss_safe_dist_lon(const double &rss_safe_dist_lon) {
    rss_safe_dist_lon_ = rss_safe_dist_lon;
    rss_safe_dist_lon_ptr_ = &rss_safe_dist_lon_;
  }

  inline const double &rss_safe_dist_lon() const { return rss_safe_dist_lon_; }

  inline double *mutable_rss_safe_dist_lon() { return &rss_safe_dist_lon_; }

  inline bool has_rss_safe_dist_lon() {
    return (rss_safe_dist_lon_ptr_ != nullptr);
  }

  inline void set_acc_lon_range_minimum(const double &acc_lon_range_minimum) {
    acc_lon_range_minimum_ = acc_lon_range_minimum;
    acc_lon_range_minimum_ptr_ = &acc_lon_range_minimum_;
  }

  inline const double &acc_lon_range_minimum() const {
    return acc_lon_range_minimum_;
  }

  inline double *mutable_acc_lon_range_minimum() {
    return &acc_lon_range_minimum_;
  }

  inline bool has_acc_lon_range_minimum() {
    return (acc_lon_range_minimum_ptr_ != nullptr);
  }

  inline void set_acc_lon_range_maximum(const double &acc_lon_range_maximum) {
    acc_lon_range_maximum_ = acc_lon_range_maximum;
    acc_lon_range_maximum_ptr_ = &acc_lon_range_maximum_;
  }

  inline const double &acc_lon_range_maximum() const {
    return acc_lon_range_maximum_;
  }

  inline double *mutable_acc_lon_range_maximum() {
    return &acc_lon_range_maximum_;
  }

  inline bool has_acc_lon_range_maximum() {
    return (acc_lon_range_maximum_ptr_ != nullptr);
  }

  inline void
  set_acc_lat_left_range_minimum(const double &acc_lat_left_range_minimum) {
    acc_lat_left_range_minimum_ = acc_lat_left_range_minimum;
    acc_lat_left_range_minimum_ptr_ = &acc_lat_left_range_minimum_;
  }

  inline const double &acc_lat_left_range_minimum() const {
    return acc_lat_left_range_minimum_;
  }

  inline double *mutable_acc_lat_left_range_minimum() {
    return &acc_lat_left_range_minimum_;
  }

  inline bool has_acc_lat_left_range_minimum() {
    return (acc_lat_left_range_minimum_ptr_ != nullptr);
  }

  inline void
  set_acc_lat_left_range_maximum(const double &acc_lat_left_range_maximum) {
    acc_lat_left_range_maximum_ = acc_lat_left_range_maximum;
    acc_lat_left_range_maximum_ptr_ = &acc_lat_left_range_maximum_;
  }

  inline const double &acc_lat_left_range_maximum() const {
    return acc_lat_left_range_maximum_;
  }

  inline double *mutable_acc_lat_left_range_maximum() {
    return &acc_lat_left_range_maximum_;
  }

  inline bool has_acc_lat_left_range_maximum() {
    return (acc_lat_left_range_maximum_ptr_ != nullptr);
  }

  inline void
  set_acc_lat_right_range_minimum(const double &acc_lat_right_range_minimum) {
    acc_lat_right_range_minimum_ = acc_lat_right_range_minimum;
    acc_lat_right_range_minimum_ptr_ = &acc_lat_right_range_minimum_;
  }

  inline const double &acc_lat_right_range_minimum() const {
    return acc_lat_right_range_minimum_;
  }

  inline double *mutable_acc_lat_right_range_minimum() {
    return &acc_lat_right_range_minimum_;
  }

  inline bool has_acc_lat_right_range_minimum() {
    return (acc_lat_right_range_minimum_ptr_ != nullptr);
  }

  inline void
  set_acc_lat_right_range_maximum(const double &acc_lat_right_range_maximum) {
    acc_lat_right_range_maximum_ = acc_lat_right_range_maximum;
    acc_lat_right_range_maximum_ptr_ = &acc_lat_right_range_maximum_;
  }

  inline const double &acc_lat_right_range_maximum() const {
    return acc_lat_right_range_maximum_;
  }

  inline double *mutable_acc_lat_right_range_maximum() {
    return &acc_lat_right_range_maximum_;
  }

  inline bool has_acc_lat_right_range_maximum() {
    return (acc_lat_right_range_maximum_ptr_ != nullptr);
  }

  void operator=(const RSSInfo &rss_info) { CopyFrom(rss_info); }

  void CopyFrom(const RSSInfo &rss_info) {
    is_rss_safe_ = rss_info.is_rss_safe();
    cur_dist_lon_ = rss_info.cur_dist_lon();
    rss_safe_dist_lon_ = rss_info.rss_safe_dist_lon();
    acc_lon_range_minimum_ = rss_info.acc_lon_range_minimum();
    acc_lon_range_maximum_ = rss_info.acc_lon_range_maximum();
    acc_lat_left_range_minimum_ = rss_info.acc_lat_left_range_minimum();
    acc_lat_left_range_maximum_ = rss_info.acc_lat_left_range_maximum();
    acc_lat_right_range_minimum_ = rss_info.acc_lat_right_range_minimum();
    acc_lat_right_range_maximum_ = rss_info.acc_lat_right_range_maximum();
  }

protected:
  bool is_rss_safe_;
  bool *is_rss_safe_ptr_ = nullptr;
  // current longitudinal distance
  double cur_dist_lon_;
  double *cur_dist_lon_ptr_ = nullptr;
  // longitudinal safe distance of rss
  double rss_safe_dist_lon_;
  double *rss_safe_dist_lon_ptr_ = nullptr;
  // Minimum range of longitudinal acceleration
  double acc_lon_range_minimum_;
  double *acc_lon_range_minimum_ptr_ = nullptr;
  // Maximum range of longitudinal acceleration
  double acc_lon_range_maximum_;
  double *acc_lon_range_maximum_ptr_ = nullptr;
  // Minimum left range of lateral acceleration
  double acc_lat_left_range_minimum_;
  double *acc_lat_left_range_minimum_ptr_ = nullptr;
  // Maximum left range of lateral acceleration
  double acc_lat_left_range_maximum_;
  double *acc_lat_left_range_maximum_ptr_ = nullptr;
  // Minimum right range of lateral acceleration
  double acc_lat_right_range_minimum_;
  double *acc_lat_right_range_minimum_ptr_ = nullptr;
  // Maximum right range of lateral acceleration
  double acc_lat_right_range_maximum_;
  double *acc_lat_right_range_maximum_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
