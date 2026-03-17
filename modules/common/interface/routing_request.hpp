/**
 * @file    routing_request.hpp
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
#include <string>
#include <vector>

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/key_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RoutingRequest {
public:
  RoutingRequest() {
    key_point_list_mutex_ = std::make_shared<std::mutex>();

    request_source_.clear();
    request_type_ = 0;
    num_of_kp_ = 0;
    clear_key_point_list();
  }
  ~RoutingRequest() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_request_source(const std::string &request_source) {
    request_source_ = request_source;
    request_source_ptr_ = &request_source_;
  }

  inline const std::string &request_source() const { return request_source_; }

  inline std::string *mutable_request_source() { return &request_source_; }

  inline bool has_request_source() { return (request_source_ptr_ != nullptr); }

  inline void set_request_type(const int8_t &request_type) {
    request_type_ = request_type;
    request_type_ptr_ = &request_type_;
  }

  inline const int8_t &request_type() const { return request_type_; }

  inline int8_t *mutable_request_type() { return &request_type_; }

  inline bool has_request_type() { return (request_type_ptr_ != nullptr); }

  inline void set_num_of_kp(const int16_t &num_of_kp) {
    num_of_kp_ = num_of_kp;
    num_of_kp_ptr_ = &num_of_kp_;
  }

  inline const int16_t &num_of_kp() const { return num_of_kp_; }

  inline int16_t *mutable_num_of_kp() { return &num_of_kp_; }

  inline bool has_num_of_kp() { return (num_of_kp_ptr_ != nullptr); }

  inline void
  set_key_point_list(std::vector<legionclaw::interface::KeyPoint> *key_point_list) {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list_.assign(key_point_list->begin(), key_point_list->end());
  }

  inline void set_key_point_list(
      const std::vector<legionclaw::interface::KeyPoint> &key_point_list) {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list_ = key_point_list;
  }

  inline void set_key_point_list(const uint32_t index,
                                 legionclaw::interface::KeyPoint &key_point_list) {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list_[index] = key_point_list;
  }

  inline void
  add_key_point_list(const legionclaw::interface::KeyPoint &key_point_list) {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list_.emplace_back(key_point_list);
  }

  inline const legionclaw::interface::KeyPoint &
  key_point_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    return key_point_list_[index];
  }

  inline std::vector<legionclaw::interface::KeyPoint> *mutable_key_point_list() {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    return &key_point_list_;
  }

  inline void key_point_list(
      std::vector<legionclaw::interface::KeyPoint> &key_point_list) const {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list.assign(key_point_list_.begin(), key_point_list_.end());
  }

  inline const std::vector<legionclaw::interface::KeyPoint> &key_point_list() const {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    return key_point_list_;
  }

  inline uint32_t key_point_list_size() const {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    return key_point_list_.size();
  }

  inline void clear_key_point_list() {
    std::lock_guard<std::mutex> lock(*key_point_list_mutex_);
    key_point_list_.clear();
    key_point_list_.shrink_to_fit();
  }

  inline bool has_key_point_list() { return (key_point_list_size() != 0); }

  void operator=(const RoutingRequest &routing_request) {
    CopyFrom(routing_request);
  }

  void CopyFrom(const RoutingRequest &routing_request) {
    header_ = routing_request.header();
    request_source_ = routing_request.request_source();
    request_type_ = routing_request.request_type();
    num_of_kp_ = routing_request.num_of_kp();
    key_point_list_ = routing_request.key_point_list();
  }

protected:
  std::shared_ptr<std::mutex> key_point_list_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //发起请求的来源名称
  std::string request_source_;
  std::string *request_source_ptr_ = nullptr;
  // REQUEST_TYPE=0//无效 REQUEST_TYPE=1//正常路由 REQUEST_TYPE=2//泊车路由
  int8_t request_type_;
  int8_t *request_type_ptr_ = nullptr;
  //关键点数量
  int16_t num_of_kp_;
  int16_t *num_of_kp_ptr_ = nullptr;
  //关键点列表，第一个为起点，最后一个为终点，中间为途经点
  std::vector<legionclaw::interface::KeyPoint> key_point_list_;
};
} // namespace interface
} // namespace legionclaw
