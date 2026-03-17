/**
 * @file    time_consume.hpp
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
class TimeConsume {
public:
  TimeConsume() {
    name_.clear();
    time_consume_ = 0.0;
  }
  ~TimeConsume() = default;

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_time_consume(const double &time_consume) {
    time_consume_ = time_consume;
    time_consume_ptr_ = &time_consume_;
  }

  inline const double &time_consume() const { return time_consume_; }

  inline double *mutable_time_consume() { return &time_consume_; }

  inline bool has_time_consume() { return (time_consume_ptr_ != nullptr); }

  void operator=(const TimeConsume &time_consume) { CopyFrom(time_consume); }

  void CopyFrom(const TimeConsume &time_consume) {
    name_ = time_consume.name();
    time_consume_ = time_consume.time_consume();
  }

protected:
  std::string name_;
  std::string *name_ptr_ = nullptr;
  double time_consume_;
  double *time_consume_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
