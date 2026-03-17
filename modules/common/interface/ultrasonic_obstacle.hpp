/**
 * @file    ultrasonic_obstacle.hpp
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
class UltrasonicObstacle {
public:
  UltrasonicObstacle() {
    id_ = 0;
    range_ = 0.0;
  }
  ~UltrasonicObstacle() = default;

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_range(const double &range) {
    range_ = range;
    range_ptr_ = &range_;
  }

  inline const double &range() const { return range_; }

  inline double *mutable_range() { return &range_; }

  inline bool has_range() { return (range_ptr_ != nullptr); }

  void operator=(const UltrasonicObstacle &ultrasonic_obstacle) {
    CopyFrom(ultrasonic_obstacle);
  }

  void CopyFrom(const UltrasonicObstacle &ultrasonic_obstacle) {
    id_ = ultrasonic_obstacle.id();
    range_ = ultrasonic_obstacle.range();
  }

protected:
  //障碍物id
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //障碍物距离
  double range_;
  double *range_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
