/**
 * @file    message_status.hpp
 * @author  jiang <jiangchengjie@indrv.cn>
 * @date    2021-05-25
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */

#pragma once

#include <map>
#include <string>

#include "modules/common/logging/logging.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */
namespace legionclaw {
namespace prediction {

/**
 * @class MessageStatus
 * @brief MessageStatus.
 */
class MessageStatus {
 public:
  typedef enum {
    REQUIRED = 0,
    PROVIDED = 1,
  } Type;

  typedef enum {
    PERIODIC = 0,
    TRIGGER = 1,
  } Mode;

  typedef enum {
    INITIALIZATION = 0,
    TIMEOUT = 2,
    NORMAL = 1,
    INTERRUPTED = 3,
  } Status;
  /**
   * @brief constructor
   */
  MessageStatus() = default;

  /**
   * @brief destructor
   */
  ~MessageStatus() = default;
  
  void set_name(std::string name) { name_ = name; }

  std::string name() const { return name_; }

  void set_type(Type type) { type_ = type; }

  Type type() const { return type_; }

  void set_mode(Mode mode) { mode_ = mode; }

  Mode mode() const { return mode_; }

  void set_period(double period) { period_ = period; }

  double period() const { return period_; }

  void set_init_timeout(double init_timeout) { init_timeout_ = init_timeout; }

  double init_timeout() const { return init_timeout_; }

  void set_max_miss_num(uint32_t max_miss_num) { max_miss_num_ = max_miss_num; }

  uint32_t max_miss_num() const { return max_miss_num_; }

  void set_last_time(int64_t last_time) { last_time_ = last_time; }

  int64_t last_time() const { return last_time_; }

  void set_status(Status status) { status_ = status; }

  Status status() const { return status_; }

 protected:
  std::string name_;
  Type type_;
  Mode mode_;
  double period_;
  double init_timeout_;
  uint32_t max_miss_num_;
  int64_t last_time_;
  Status status_;
};
}  // namespace prediction
}  // namespace legionclaw
