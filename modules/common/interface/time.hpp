/**
 * @file    time.hpp
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
class Time {
public:
  Time() {
    sec_ = 0;
    nsec_ = 0;
  }
  ~Time() = default;

  inline void set_sec(const uint32_t &sec) {
    sec_ = sec;
    sec_ptr_ = &sec_;
  }

  inline const uint32_t &sec() const { return sec_; }

  inline uint32_t *mutable_sec() { return &sec_; }

  inline bool has_sec() { return (sec_ptr_ != nullptr); }

  inline void set_nsec(const uint32_t &nsec) {
    nsec_ = nsec;
    nsec_ptr_ = &nsec_;
  }

  inline const uint32_t &nsec() const { return nsec_; }

  inline uint32_t *mutable_nsec() { return &nsec_; }

  inline bool has_nsec() { return (nsec_ptr_ != nullptr); }

  void operator=(const Time &time) { CopyFrom(time); }

  void CopyFrom(const Time &time) {
    sec_ = time.sec();
    nsec_ = time.nsec();
  }

protected:
  //秒
  uint32_t sec_;
  uint32_t *sec_ptr_ = nullptr;
  //纳秒
  uint32_t nsec_;
  uint32_t *nsec_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
