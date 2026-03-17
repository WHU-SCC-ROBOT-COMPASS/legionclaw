/**
 * @file    image.hpp
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
class Image {
public:
  Image() {
    data_mutex_ = std::make_shared<std::mutex>();

    height_ = 0;
    width_ = 0;
    encoding_.clear();
    is_bigendian_ = 0;
    step_ = 0;
    clear_data();
  }
  ~Image() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_height(const uint32_t &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const uint32_t &height() const { return height_; }

  inline uint32_t *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  inline void set_width(const uint32_t &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const uint32_t &width() const { return width_; }

  inline uint32_t *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_encoding(const std::string &encoding) {
    encoding_ = encoding;
    encoding_ptr_ = &encoding_;
  }

  inline const std::string &encoding() const { return encoding_; }

  inline std::string *mutable_encoding() { return &encoding_; }

  inline bool has_encoding() { return (encoding_ptr_ != nullptr); }

  inline void set_is_bigendian(const uint8_t &is_bigendian) {
    is_bigendian_ = is_bigendian;
    is_bigendian_ptr_ = &is_bigendian_;
  }

  inline const uint8_t &is_bigendian() const { return is_bigendian_; }

  inline uint8_t *mutable_is_bigendian() { return &is_bigendian_; }

  inline bool has_is_bigendian() { return (is_bigendian_ptr_ != nullptr); }

  inline void set_step(const uint32_t &step) {
    step_ = step;
    step_ptr_ = &step_;
  }

  inline const uint32_t &step() const { return step_; }

  inline uint32_t *mutable_step() { return &step_; }

  inline bool has_step() { return (step_ptr_ != nullptr); }

  inline void set_data(std::vector<uint8_t> *data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_.assign(data->begin(), data->end());
  }

  inline void set_data(const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_ = data;
  }

  inline void set_data(const uint32_t index, uint8_t &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_[index] = data;
  }

  inline void add_data(const uint8_t &data) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data_.emplace_back(data);
  }

  inline const uint8_t &data(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return data_[index];
  }

  inline std::vector<uint8_t> *mutable_data() {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    return &data_;
  }

  inline void data(std::vector<uint8_t> &data) const {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    data.assign(data_.begin(), data_.end());
  }

  inline const std::vector<uint8_t> &data() const {
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

  void operator=(const Image &image) { CopyFrom(image); }

  void CopyFrom(const Image &image) {
    header_ = image.header();
    height_ = image.height();
    width_ = image.width();
    encoding_ = image.encoding();
    is_bigendian_ = image.is_bigendian();
    step_ = image.step();
    data_ = image.data();
  }

protected:
  std::shared_ptr<std::mutex> data_mutex_;
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // image height, that is, number of rows
  uint32_t height_;
  uint32_t *height_ptr_ = nullptr;
  // image width, that is, number of columns
  uint32_t width_;
  uint32_t *width_ptr_ = nullptr;
  //图像编码方式
  std::string encoding_;
  std::string *encoding_ptr_ = nullptr;
  // is this data bigendian?
  uint8_t is_bigendian_;
  uint8_t *is_bigendian_ptr_ = nullptr;
  // Full row length in bytes
  uint32_t step_;
  uint32_t *step_ptr_ = nullptr;
  // actual matrix data, size is (step * rows)
  std::vector<uint8_t> data_;
};
} // namespace interface
} // namespace legionclaw
