/**
 * @file    example.h
 * @author  editor
 * @date    2020-05-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>

#include "modules/common/interface/header.hpp"

using namespace std;

namespace legionclaw {
namespace localization {
/**
 * @class LocalizationStatus
 *
 * @brief 消息的描述.
 */
class LocalizationStatus {
 public:
  LocalizationStatus() = default;
  ~LocalizationStatus() = default;

  void set_header(legionclaw::interface::Header header) { header_ = header; }

  legionclaw::interface::Header header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  void set_fusion_status(int8_t fusion_status) {
    fusion_status_ = fusion_status;
  }

  int8_t fusion_status() const { return fusion_status_; }

  inline int8_t *mutable_fusion_status() { return &fusion_status_; }

  void set_gnss_status(int8_t gnss_status) { gnss_status_ = gnss_status; }

  int8_t gnss_status() const { return gnss_status_; }

  inline int8_t *mutable_gnss_status() { return &gnss_status_; }

  void set_lidar_status(int8_t lidar_status) { lidar_status_ = lidar_status; }

  int8_t lidar_status() const { return lidar_status_; }

  inline int8_t *mutable_lidar_status() { return &lidar_status_; }

  void set_measurement_time(double measurement_time) {
    measurement_time_ = measurement_time;
  }

  double measurement_time() const { return measurement_time_; }

  inline double *mutable_measurement_time() { return &measurement_time_; }

  void set_state_message(string state_message) {
    state_message_ = state_message;
  }

  string state_message() const { return state_message_; }

  inline string *mutable_state_message() { return &state_message_; }

 protected:
  legionclaw::interface::Header header_;
  // OK = 0;
  // WARNNING = 1;
  // ERROR = 2;
  // CRITICAL_ERROR = 3;
  // FATAL_ERROR = 4;
  int8_t fusion_status_;
  // OK = 0;
  // WARNNING = 1;
  // ERROR = 2;
  // CRITICAL_ERROR = 3;
  // FATAL_ERROR = 4;
  int8_t gnss_status_;
  // OK = 0;
  // WARNNING = 1;
  // ERROR = 2;
  // CRITICAL_ERROR = 3;
  // FATAL_ERROR = 4;
  int8_t lidar_status_;
  // The time of pose measurement, seconds since 1970-1-1 (UNIX time)
  double measurement_time_;
  string state_message_;
};
}  // namespace localization
}  // namespace legionclaw
