/**
 * @file    log.hpp
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

#include "modules/common/interface/time.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Log {
public:
  Log() {
    level_ = 0;
    name_.clear();
    msg_.clear();
    file_.clear();
    function_.clear();
    line_ = 0;
  }
  ~Log() = default;

  inline void set_stamp(const legionclaw::interface::Time &stamp) {
    stamp_ = stamp;
    stamp_ptr_ = &stamp_;
  }

  inline const legionclaw::interface::Time &stamp() const { return stamp_; }

  inline legionclaw::interface::Time *mutable_stamp() { return &stamp_; }

  inline bool has_stamp() { return (stamp_ptr_ != nullptr); }

  inline void set_level(const uint8_t &level) {
    level_ = level;
    level_ptr_ = &level_;
  }

  inline const uint8_t &level() const { return level_; }

  inline uint8_t *mutable_level() { return &level_; }

  inline bool has_level() { return (level_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_msg(const std::string &msg) {
    msg_ = msg;
    msg_ptr_ = &msg_;
  }

  inline const std::string &msg() const { return msg_; }

  inline std::string *mutable_msg() { return &msg_; }

  inline bool has_msg() { return (msg_ptr_ != nullptr); }

  inline void set_file(const std::string &file) {
    file_ = file;
    file_ptr_ = &file_;
  }

  inline const std::string &file() const { return file_; }

  inline std::string *mutable_file() { return &file_; }

  inline bool has_file() { return (file_ptr_ != nullptr); }

  inline void set_function(const std::string &function) {
    function_ = function;
    function_ptr_ = &function_;
  }

  inline const std::string &function() const { return function_; }

  inline std::string *mutable_function() { return &function_; }

  inline bool has_function() { return (function_ptr_ != nullptr); }

  inline void set_line(const uint32_t &line) {
    line_ = line;
    line_ptr_ = &line_;
  }

  inline const uint32_t &line() const { return line_; }

  inline uint32_t *mutable_line() { return &line_; }

  inline bool has_line() { return (line_ptr_ != nullptr); }

  void operator=(const Log &log) { CopyFrom(log); }

  void CopyFrom(const Log &log) {
    stamp_ = log.stamp();
    level_ = log.level();
    name_ = log.name();
    msg_ = log.msg();
    file_ = log.file();
    function_ = log.function();
    line_ = log.line();
  }

protected:
  //时间戳
  legionclaw::interface::Time stamp_;
  legionclaw::interface::Time *stamp_ptr_ = nullptr;
  // log等级 DEBUG   : 10  INFO       : 20   WARN     : 30   ERROR   : 40  FATAL
  // : 50
  uint8_t level_;
  uint8_t *level_ptr_ = nullptr;
  //节点名
  std::string name_;
  std::string *name_ptr_ = nullptr;
  // log消息
  std::string msg_;
  std::string *msg_ptr_ = nullptr;
  //发送log文件路径
  std::string file_;
  std::string *file_ptr_ = nullptr;
  //模块名
  std::string function_;
  std::string *function_ptr_ = nullptr;
  //消息行
  uint32_t line_;
  uint32_t *line_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
