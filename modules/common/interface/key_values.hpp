/**
 * @file    key_values.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>
#include <string>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class KeyValues {
public:
  KeyValues() {
    key_.clear();
    value_.clear();
  }
  ~KeyValues() = default;

  inline void set_key(const std::string &key) {
    key_ = key;
    key_ptr_ = &key_;
  }

  inline const std::string &key() const { return key_; }

  inline std::string *mutable_key() { return &key_; }

  inline bool has_key() { return (key_ptr_ != nullptr); }

  inline void set_value(const std::string &value) {
    value_ = value;
    value_ptr_ = &value_;
  }

  inline const std::string &value() const { return value_; }

  inline std::string *mutable_value() { return &value_; }

  inline bool has_value() { return (value_ptr_ != nullptr); }

  void operator=(const KeyValues &key_values) { CopyFrom(key_values); }

  void CopyFrom(const KeyValues &key_values) {
    key_ = key_values.key();
    value_ = key_values.value();
  }

protected:
  std::string key_;
  std::string *key_ptr_ = nullptr;
  std::string value_;
  std::string *value_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
