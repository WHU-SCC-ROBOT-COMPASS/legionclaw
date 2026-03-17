/**
 * @file    hmi_parking_info_list.hpp
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

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/hmi_parking_info.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HMIParkingInfoList {
public:
  HMIParkingInfoList() {
    parking_info_mutex_ = std::make_shared<std::mutex>();

    clear_parking_info();
    parking_out_enable_ = false;
    parking_out_direction_ = 0;
    parking_out_id_ = 0;
  }
  ~HMIParkingInfoList() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_parking_info(
      std::vector<legionclaw::interface::HMIParkingInfo> *parking_info) {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info_.assign(parking_info->begin(), parking_info->end());
  }

  inline void set_parking_info(
      const std::vector<legionclaw::interface::HMIParkingInfo> &parking_info) {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info_ = parking_info;
  }

  inline void set_parking_info(const uint32_t index,
                               legionclaw::interface::HMIParkingInfo &parking_info) {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info_[index] = parking_info;
  }

  inline void
  add_parking_info(const legionclaw::interface::HMIParkingInfo &parking_info) {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info_.emplace_back(parking_info);
  }

  inline const legionclaw::interface::HMIParkingInfo &
  parking_info(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    return parking_info_[index];
  }

  inline std::vector<legionclaw::interface::HMIParkingInfo> *mutable_parking_info() {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    return &parking_info_;
  }

  inline void parking_info(
      std::vector<legionclaw::interface::HMIParkingInfo> &parking_info) const {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info.assign(parking_info_.begin(), parking_info_.end());
  }

  inline const std::vector<legionclaw::interface::HMIParkingInfo> &
  parking_info() const {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    return parking_info_;
  }

  inline uint32_t parking_info_size() const {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    return parking_info_.size();
  }

  inline void clear_parking_info() {
    std::lock_guard<std::mutex> lock(*parking_info_mutex_);
    parking_info_.clear();
    parking_info_.shrink_to_fit();
  }

  inline bool has_parking_info() { return (parking_info_size() != 0); }

  inline void set_parking_out_enable(const bool &parking_out_enable) {
    parking_out_enable_ = parking_out_enable;
    parking_out_enable_ptr_ = &parking_out_enable_;
  }

  inline const bool &parking_out_enable() const { return parking_out_enable_; }

  inline bool *mutable_parking_out_enable() { return &parking_out_enable_; }

  inline bool has_parking_out_enable() {
    return (parking_out_enable_ptr_ != nullptr);
  }

  inline void set_parking_out_direction(const int32_t &parking_out_direction) {
    parking_out_direction_ = parking_out_direction;
    parking_out_direction_ptr_ = &parking_out_direction_;
  }

  inline const int32_t &parking_out_direction() const {
    return parking_out_direction_;
  }

  inline int32_t *mutable_parking_out_direction() {
    return &parking_out_direction_;
  }

  inline bool has_parking_out_direction() {
    return (parking_out_direction_ptr_ != nullptr);
  }

  inline void set_parking_out_id(const int32_t &parking_out_id) {
    parking_out_id_ = parking_out_id;
    parking_out_id_ptr_ = &parking_out_id_;
  }

  inline const int32_t &parking_out_id() const { return parking_out_id_; }

  inline int32_t *mutable_parking_out_id() { return &parking_out_id_; }

  inline bool has_parking_out_id() { return (parking_out_id_ptr_ != nullptr); }

  void operator=(const HMIParkingInfoList &hmi_parking_info_list) {
    CopyFrom(hmi_parking_info_list);
  }

  void CopyFrom(const HMIParkingInfoList &hmi_parking_info_list) {
    header_ = hmi_parking_info_list.header();
    parking_info_ = hmi_parking_info_list.parking_info();
    parking_out_enable_ = hmi_parking_info_list.parking_out_enable();
    parking_out_direction_ = hmi_parking_info_list.parking_out_direction();
    parking_out_id_ = hmi_parking_info_list.parking_out_id();
  }

protected:
  std::shared_ptr<std::mutex> parking_info_mutex_;
  // timestamp is  included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  std::vector<legionclaw::interface::HMIParkingInfo> parking_info_;
  bool parking_out_enable_;
  bool *parking_out_enable_ptr_ = nullptr;
  int32_t parking_out_direction_;
  int32_t *parking_out_direction_ptr_ = nullptr;
  int32_t parking_out_id_;
  int32_t *parking_out_id_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
