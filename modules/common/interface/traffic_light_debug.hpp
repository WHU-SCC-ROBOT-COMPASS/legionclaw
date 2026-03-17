/**
 * @file    traffic_light_debug.hpp
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
#include <vector>

#include "modules/common/interface/traffic_light_box.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class TrafficLightDebug {
public:
  TrafficLightDebug() {
    box_mutex_ = std::make_shared<std::mutex>();
    crop_roi_mutex_ = std::make_shared<std::mutex>();
    projected_roi_mutex_ = std::make_shared<std::mutex>();
    rectified_roi_mutex_ = std::make_shared<std::mutex>();
    debug_roi_mutex_ = std::make_shared<std::mutex>();

    clear_box();
    signal_num_ = 0;
    valid_pos_ = 0;
    ts_diff_pos_ = 0.0;
    ts_diff_sys_ = 0.0;
    project_error_ = 0;
    distance_to_stop_line_ = 0.0;
    camera_id_ = 0;
    clear_crop_roi();
    clear_projected_roi();
    clear_rectified_roi();
    clear_debug_roi();
  }
  ~TrafficLightDebug() = default;

  inline void set_cropbox(const legionclaw::interface::TrafficLightBox &cropbox) {
    cropbox_ = cropbox;
    cropbox_ptr_ = &cropbox_;
  }

  inline const legionclaw::interface::TrafficLightBox &cropbox() const {
    return cropbox_;
  }

  inline legionclaw::interface::TrafficLightBox *mutable_cropbox() {
    return &cropbox_;
  }

  inline bool has_cropbox() { return (cropbox_ptr_ != nullptr); }

  inline void set_box(std::vector<legionclaw::interface::TrafficLightBox> *box) {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box_.assign(box->begin(), box->end());
  }

  inline void
  set_box(const std::vector<legionclaw::interface::TrafficLightBox> &box) {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box_ = box;
  }

  inline void set_box(const uint32_t index,
                      legionclaw::interface::TrafficLightBox &box) {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box_[index] = box;
  }

  inline void add_box(const legionclaw::interface::TrafficLightBox &box) {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box_.emplace_back(box);
  }

  inline const legionclaw::interface::TrafficLightBox &box(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    return box_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLightBox> *mutable_box() {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    return &box_;
  }

  inline void box(std::vector<legionclaw::interface::TrafficLightBox> &box) const {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box.assign(box_.begin(), box_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLightBox> &box() const {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    return box_;
  }

  inline uint32_t box_size() const {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    return box_.size();
  }

  inline void clear_box() {
    std::lock_guard<std::mutex> lock(*box_mutex_);
    box_.clear();
    box_.shrink_to_fit();
  }

  inline bool has_box() { return (box_size() != 0); }

  inline void set_signal_num(const int32_t &signal_num) {
    signal_num_ = signal_num;
    signal_num_ptr_ = &signal_num_;
  }

  inline const int32_t &signal_num() const { return signal_num_; }

  inline int32_t *mutable_signal_num() { return &signal_num_; }

  inline bool has_signal_num() { return (signal_num_ptr_ != nullptr); }

  inline void set_valid_pos(const int32_t &valid_pos) {
    valid_pos_ = valid_pos;
    valid_pos_ptr_ = &valid_pos_;
  }

  inline const int32_t &valid_pos() const { return valid_pos_; }

  inline int32_t *mutable_valid_pos() { return &valid_pos_; }

  inline bool has_valid_pos() { return (valid_pos_ptr_ != nullptr); }

  inline void set_ts_diff_pos(const double &ts_diff_pos) {
    ts_diff_pos_ = ts_diff_pos;
    ts_diff_pos_ptr_ = &ts_diff_pos_;
  }

  inline const double &ts_diff_pos() const { return ts_diff_pos_; }

  inline double *mutable_ts_diff_pos() { return &ts_diff_pos_; }

  inline bool has_ts_diff_pos() { return (ts_diff_pos_ptr_ != nullptr); }

  inline void set_ts_diff_sys(const double &ts_diff_sys) {
    ts_diff_sys_ = ts_diff_sys;
    ts_diff_sys_ptr_ = &ts_diff_sys_;
  }

  inline const double &ts_diff_sys() const { return ts_diff_sys_; }

  inline double *mutable_ts_diff_sys() { return &ts_diff_sys_; }

  inline bool has_ts_diff_sys() { return (ts_diff_sys_ptr_ != nullptr); }

  inline void set_project_error(const int32_t &project_error) {
    project_error_ = project_error;
    project_error_ptr_ = &project_error_;
  }

  inline const int32_t &project_error() const { return project_error_; }

  inline int32_t *mutable_project_error() { return &project_error_; }

  inline bool has_project_error() { return (project_error_ptr_ != nullptr); }

  inline void set_distance_to_stop_line(const double &distance_to_stop_line) {
    distance_to_stop_line_ = distance_to_stop_line;
    distance_to_stop_line_ptr_ = &distance_to_stop_line_;
  }

  inline const double &distance_to_stop_line() const {
    return distance_to_stop_line_;
  }

  inline double *mutable_distance_to_stop_line() {
    return &distance_to_stop_line_;
  }

  inline bool has_distance_to_stop_line() {
    return (distance_to_stop_line_ptr_ != nullptr);
  }

  inline void set_camera_id(const int32_t &camera_id) {
    camera_id_ = camera_id;
    camera_id_ptr_ = &camera_id_;
  }

  inline const int32_t &camera_id() const { return camera_id_; }

  inline int32_t *mutable_camera_id() { return &camera_id_; }

  inline bool has_camera_id() { return (camera_id_ptr_ != nullptr); }

  inline void
  set_crop_roi(std::vector<legionclaw::interface::TrafficLightBox> *crop_roi) {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi_.assign(crop_roi->begin(), crop_roi->end());
  }

  inline void
  set_crop_roi(const std::vector<legionclaw::interface::TrafficLightBox> &crop_roi) {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi_ = crop_roi;
  }

  inline void set_crop_roi(const uint32_t index,
                           legionclaw::interface::TrafficLightBox &crop_roi) {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi_[index] = crop_roi;
  }

  inline void add_crop_roi(const legionclaw::interface::TrafficLightBox &crop_roi) {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi_.emplace_back(crop_roi);
  }

  inline const legionclaw::interface::TrafficLightBox &
  crop_roi(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    return crop_roi_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLightBox> *mutable_crop_roi() {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    return &crop_roi_;
  }

  inline void
  crop_roi(std::vector<legionclaw::interface::TrafficLightBox> &crop_roi) const {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi.assign(crop_roi_.begin(), crop_roi_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLightBox> &
  crop_roi() const {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    return crop_roi_;
  }

  inline uint32_t crop_roi_size() const {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    return crop_roi_.size();
  }

  inline void clear_crop_roi() {
    std::lock_guard<std::mutex> lock(*crop_roi_mutex_);
    crop_roi_.clear();
    crop_roi_.shrink_to_fit();
  }

  inline bool has_crop_roi() { return (crop_roi_size() != 0); }

  inline void set_projected_roi(
      std::vector<legionclaw::interface::TrafficLightBox> *projected_roi) {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi_.assign(projected_roi->begin(), projected_roi->end());
  }

  inline void set_projected_roi(
      const std::vector<legionclaw::interface::TrafficLightBox> &projected_roi) {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi_ = projected_roi;
  }

  inline void
  set_projected_roi(const uint32_t index,
                    legionclaw::interface::TrafficLightBox &projected_roi) {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi_[index] = projected_roi;
  }

  inline void
  add_projected_roi(const legionclaw::interface::TrafficLightBox &projected_roi) {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi_.emplace_back(projected_roi);
  }

  inline const legionclaw::interface::TrafficLightBox &
  projected_roi(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    return projected_roi_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLightBox> *
  mutable_projected_roi() {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    return &projected_roi_;
  }

  inline void projected_roi(
      std::vector<legionclaw::interface::TrafficLightBox> &projected_roi) const {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi.assign(projected_roi_.begin(), projected_roi_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLightBox> &
  projected_roi() const {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    return projected_roi_;
  }

  inline uint32_t projected_roi_size() const {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    return projected_roi_.size();
  }

  inline void clear_projected_roi() {
    std::lock_guard<std::mutex> lock(*projected_roi_mutex_);
    projected_roi_.clear();
    projected_roi_.shrink_to_fit();
  }

  inline bool has_projected_roi() { return (projected_roi_size() != 0); }

  inline void set_rectified_roi(
      std::vector<legionclaw::interface::TrafficLightBox> *rectified_roi) {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi_.assign(rectified_roi->begin(), rectified_roi->end());
  }

  inline void set_rectified_roi(
      const std::vector<legionclaw::interface::TrafficLightBox> &rectified_roi) {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi_ = rectified_roi;
  }

  inline void
  set_rectified_roi(const uint32_t index,
                    legionclaw::interface::TrafficLightBox &rectified_roi) {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi_[index] = rectified_roi;
  }

  inline void
  add_rectified_roi(const legionclaw::interface::TrafficLightBox &rectified_roi) {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi_.emplace_back(rectified_roi);
  }

  inline const legionclaw::interface::TrafficLightBox &
  rectified_roi(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    return rectified_roi_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLightBox> *
  mutable_rectified_roi() {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    return &rectified_roi_;
  }

  inline void rectified_roi(
      std::vector<legionclaw::interface::TrafficLightBox> &rectified_roi) const {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi.assign(rectified_roi_.begin(), rectified_roi_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLightBox> &
  rectified_roi() const {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    return rectified_roi_;
  }

  inline uint32_t rectified_roi_size() const {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    return rectified_roi_.size();
  }

  inline void clear_rectified_roi() {
    std::lock_guard<std::mutex> lock(*rectified_roi_mutex_);
    rectified_roi_.clear();
    rectified_roi_.shrink_to_fit();
  }

  inline bool has_rectified_roi() { return (rectified_roi_size() != 0); }

  inline void
  set_debug_roi(std::vector<legionclaw::interface::TrafficLightBox> *debug_roi) {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi_.assign(debug_roi->begin(), debug_roi->end());
  }

  inline void set_debug_roi(
      const std::vector<legionclaw::interface::TrafficLightBox> &debug_roi) {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi_ = debug_roi;
  }

  inline void set_debug_roi(const uint32_t index,
                            legionclaw::interface::TrafficLightBox &debug_roi) {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi_[index] = debug_roi;
  }

  inline void
  add_debug_roi(const legionclaw::interface::TrafficLightBox &debug_roi) {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi_.emplace_back(debug_roi);
  }

  inline const legionclaw::interface::TrafficLightBox &
  debug_roi(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    return debug_roi_[index];
  }

  inline std::vector<legionclaw::interface::TrafficLightBox> *mutable_debug_roi() {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    return &debug_roi_;
  }

  inline void
  debug_roi(std::vector<legionclaw::interface::TrafficLightBox> &debug_roi) const {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi.assign(debug_roi_.begin(), debug_roi_.end());
  }

  inline const std::vector<legionclaw::interface::TrafficLightBox> &
  debug_roi() const {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    return debug_roi_;
  }

  inline uint32_t debug_roi_size() const {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    return debug_roi_.size();
  }

  inline void clear_debug_roi() {
    std::lock_guard<std::mutex> lock(*debug_roi_mutex_);
    debug_roi_.clear();
    debug_roi_.shrink_to_fit();
  }

  inline bool has_debug_roi() { return (debug_roi_size() != 0); }

  void operator=(const TrafficLightDebug &traffic_light_debug) {
    CopyFrom(traffic_light_debug);
  }

  void CopyFrom(const TrafficLightDebug &traffic_light_debug) {
    cropbox_ = traffic_light_debug.cropbox();
    box_ = traffic_light_debug.box();
    signal_num_ = traffic_light_debug.signal_num();
    valid_pos_ = traffic_light_debug.valid_pos();
    ts_diff_pos_ = traffic_light_debug.ts_diff_pos();
    ts_diff_sys_ = traffic_light_debug.ts_diff_sys();
    project_error_ = traffic_light_debug.project_error();
    distance_to_stop_line_ = traffic_light_debug.distance_to_stop_line();
    camera_id_ = traffic_light_debug.camera_id();
    crop_roi_ = traffic_light_debug.crop_roi();
    projected_roi_ = traffic_light_debug.projected_roi();
    rectified_roi_ = traffic_light_debug.rectified_roi();
    debug_roi_ = traffic_light_debug.debug_roi();
  }

protected:
  std::shared_ptr<std::mutex> box_mutex_;
  std::shared_ptr<std::mutex> crop_roi_mutex_;
  std::shared_ptr<std::mutex> projected_roi_mutex_;
  std::shared_ptr<std::mutex> rectified_roi_mutex_;
  std::shared_ptr<std::mutex> debug_roi_mutex_;
  legionclaw::interface::TrafficLightBox cropbox_;
  legionclaw::interface::TrafficLightBox *cropbox_ptr_ = nullptr;
  std::vector<legionclaw::interface::TrafficLightBox> box_;
  int32_t signal_num_;
  int32_t *signal_num_ptr_ = nullptr;
  int32_t valid_pos_;
  int32_t *valid_pos_ptr_ = nullptr;
  double ts_diff_pos_;
  double *ts_diff_pos_ptr_ = nullptr;
  double ts_diff_sys_;
  double *ts_diff_sys_ptr_ = nullptr;
  int32_t project_error_;
  int32_t *project_error_ptr_ = nullptr;
  double distance_to_stop_line_;
  double *distance_to_stop_line_ptr_ = nullptr;
  int32_t camera_id_;
  int32_t *camera_id_ptr_ = nullptr;
  std::vector<legionclaw::interface::TrafficLightBox> crop_roi_;
  std::vector<legionclaw::interface::TrafficLightBox> projected_roi_;
  std::vector<legionclaw::interface::TrafficLightBox> rectified_roi_;
  std::vector<legionclaw::interface::TrafficLightBox> debug_roi_;
};
} // namespace interface
} // namespace legionclaw
