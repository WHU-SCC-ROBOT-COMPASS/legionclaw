/**
 * @file    compressed_image.hpp
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

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class CompressedImage {
public:
  CompressedImage() {
    data_mutex_ = std::make_shared<std::mutex>();

    frame_id_.clear();
    format_.clear();
    clear_data();
    measurement_time_ = 0.0;
    frame_type_ = 0;
  }
  ~CompressedImage() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_frame_id(const std::string &frame_id) {
    frame_id_ = frame_id;
    frame_id_ptr_ = &frame_id_;
  }

  inline const std::string &frame_id() const { return frame_id_; }

  inline std::string *mutable_frame_id() { return &frame_id_; }

  inline bool has_frame_id() { return (frame_id_ptr_ != nullptr); }

  inline void set_format(const std::string &format) {
    format_ = format;
    format_ptr_ = &format_;
  }

  inline const std::string &format() const { return format_; }

  inline std::string *mutable_format() { return &format_; }

  inline bool has_format() { return (format_ptr_ != nullptr); }

  inline void set_data(std::vector<int8_t> *data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_.assign(data->begin(), data->end());
  }

  inline void set_data(const std::vector<int8_t> &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_ = data;
  }

  inline void set_data(const uint32_t index, int8_t &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_[index] = data;
  }

  inline void add_data(const int8_t &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_.emplace_back(data);
  }

  inline const int8_t &data(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return data_[index];
  }

  inline std::vector<int8_t> *mutable_data() {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return &data_;
  }

  inline void data(std::vector<int8_t> &data) const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data.assign(data_.begin(), data_.end());
  }

  inline const std::vector<int8_t> &data() const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return data_;
  }

  inline uint32_t data_size() const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return data_.size();
  }

  inline void clear_data() {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_.clear();
    data_.shrink_to_fit();
  }

  inline bool has_data() { return (data_size() != 0); }

  inline void set_measurement_time(const double &measurement_time) {
    measurement_time_ = measurement_time;
    measurement_time_ptr_ = &measurement_time_;
  }

  inline const double &measurement_time() const { return measurement_time_; }

  inline double *mutable_measurement_time() { return &measurement_time_; }

  inline bool has_measurement_time() {
    return (measurement_time_ptr_ != nullptr);
  }

  inline void set_frame_type(const uint32_t &frame_type) {
    frame_type_ = frame_type;
    frame_type_ptr_ = &frame_type_;
  }

  inline const uint32_t &frame_type() const { return frame_type_; }

  inline uint32_t *mutable_frame_type() { return &frame_type_; }

  inline bool has_frame_type() { return (frame_type_ptr_ != nullptr); }

  void operator=(const CompressedImage &compressed_image) {
    CopyFrom(compressed_image);
  }

  void CopyFrom(const CompressedImage &compressed_image) {
    header_ = compressed_image.header();
    frame_id_ = compressed_image.frame_id();
    format_ = compressed_image.format();
    data_ = compressed_image.data();
    measurement_time_ = compressed_image.measurement_time();
    frame_type_ = compressed_image.frame_type();
  }

protected:
  std::shared_ptr<std::mutex> data_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //图像帧id
  std::string frame_id_;
  std::string *frame_id_ptr_ = nullptr;
  // Specifies the format of the data Acceptable values: jpeg, png
  std::string format_;
  std::string *format_ptr_ = nullptr;
  // Compressed image buffer
  std::vector<int8_t> data_;
  //测量时间
  double measurement_time_;
  double *measurement_time_ptr_ = nullptr;
  //图像帧类型
  uint32_t frame_type_;
  uint32_t *frame_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
