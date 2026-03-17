/**
 * @file    trajectory_limit_command.hpp
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
class TrajectoryLimitCommand {
public:
  TrajectoryLimitCommand() {
    speed_limit_enable_ = false;
    speed_limit_ = 0.0;
    kappa_limit_enable_ = false;
    kappa_limit_ = 0.0;
  }
  ~TrajectoryLimitCommand() = default;

  inline void set_speed_limit_enable(const bool &speed_limit_enable) {
    speed_limit_enable_ = speed_limit_enable;
    speed_limit_enable_ptr_ = &speed_limit_enable_;
  }

  inline const bool &speed_limit_enable() const { return speed_limit_enable_; }

  inline bool *mutable_speed_limit_enable() { return &speed_limit_enable_; }

  inline bool has_speed_limit_enable() {
    return (speed_limit_enable_ptr_ != nullptr);
  }

  inline void set_speed_limit(const double &speed_limit) {
    speed_limit_ = speed_limit;
    speed_limit_ptr_ = &speed_limit_;
  }

  inline const double &speed_limit() const { return speed_limit_; }

  inline double *mutable_speed_limit() { return &speed_limit_; }

  inline bool has_speed_limit() { return (speed_limit_ptr_ != nullptr); }

  inline void set_kappa_limit_enable(const bool &kappa_limit_enable) {
    kappa_limit_enable_ = kappa_limit_enable;
    kappa_limit_enable_ptr_ = &kappa_limit_enable_;
  }

  inline const bool &kappa_limit_enable() const { return kappa_limit_enable_; }

  inline bool *mutable_kappa_limit_enable() { return &kappa_limit_enable_; }

  inline bool has_kappa_limit_enable() {
    return (kappa_limit_enable_ptr_ != nullptr);
  }

  inline void set_kappa_limit(const double &kappa_limit) {
    kappa_limit_ = kappa_limit;
    kappa_limit_ptr_ = &kappa_limit_;
  }

  inline const double &kappa_limit() const { return kappa_limit_; }

  inline double *mutable_kappa_limit() { return &kappa_limit_; }

  inline bool has_kappa_limit() { return (kappa_limit_ptr_ != nullptr); }

  void operator=(const TrajectoryLimitCommand &trajectory_limit_command) {
    CopyFrom(trajectory_limit_command);
  }

  void CopyFrom(const TrajectoryLimitCommand &trajectory_limit_command) {
    speed_limit_enable_ = trajectory_limit_command.speed_limit_enable();
    speed_limit_ = trajectory_limit_command.speed_limit();
    kappa_limit_enable_ = trajectory_limit_command.kappa_limit_enable();
    kappa_limit_ = trajectory_limit_command.kappa_limit();
  }

protected:
  bool speed_limit_enable_;
  bool *speed_limit_enable_ptr_ = nullptr;
  // m/s
  double speed_limit_;
  double *speed_limit_ptr_ = nullptr;
  bool kappa_limit_enable_;
  bool *kappa_limit_enable_ptr_ = nullptr;
  // deg/m
  double kappa_limit_;
  double *kappa_limit_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
