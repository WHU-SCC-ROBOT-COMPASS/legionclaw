/**
 * @file    diagnostic_status.hpp
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
#include "modules/common/interface/key_values.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class DiagnosticStatus {
public:
  DiagnosticStatus() {
    values_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    level_ = legionclaw::interface::DiagnosticStatus::Level::OK;
    name_.clear();
    message_.clear();
    hardware_id_.clear();
    clear_values();
  }
  ~DiagnosticStatus() = default;

  enum Level {
    OK = 0,
    WARN = 1,
    ERROR = 2,
    STALE = 3,
    RECOVER = 4,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void
  set_level(const legionclaw::interface::DiagnosticStatus::Level &level) {
    level_ = level;
    level_ptr_ = &level_;
  }

  inline const legionclaw::interface::DiagnosticStatus::Level &level() const {
    return level_;
  }

  inline legionclaw::interface::DiagnosticStatus::Level *mutable_level() {
    return &level_;
  }

  inline bool has_level() { return (level_ptr_ != nullptr); }

  inline void set_name(const std::string &name) {
    name_ = name;
    name_ptr_ = &name_;
  }

  inline const std::string &name() const { return name_; }

  inline std::string *mutable_name() { return &name_; }

  inline bool has_name() { return (name_ptr_ != nullptr); }

  inline void set_message(const std::string &message) {
    message_ = message;
    message_ptr_ = &message_;
  }

  inline const std::string &message() const { return message_; }

  inline std::string *mutable_message() { return &message_; }

  inline bool has_message() { return (message_ptr_ != nullptr); }

  inline void set_hardware_id(const std::string &hardware_id) {
    hardware_id_ = hardware_id;
    hardware_id_ptr_ = &hardware_id_;
  }

  inline const std::string &hardware_id() const { return hardware_id_; }

  inline std::string *mutable_hardware_id() { return &hardware_id_; }

  inline bool has_hardware_id() { return (hardware_id_ptr_ != nullptr); }

  inline void set_values(std::vector<legionclaw::interface::KeyValues> *values) {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values_.assign(values->begin(), values->end());
  }

  inline void
  set_values(const std::vector<legionclaw::interface::KeyValues> &values) {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values_ = values;
  }

  inline void set_values(const uint32_t index,
                         legionclaw::interface::KeyValues &values) {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values_[index] = values;
  }

  inline void add_values(const legionclaw::interface::KeyValues &values) {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values_.emplace_back(values);
  }

  inline const legionclaw::interface::KeyValues &values(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    return values_[index];
  }

  inline std::vector<legionclaw::interface::KeyValues> *mutable_values() {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    return &values_;
  }

  inline void values(std::vector<legionclaw::interface::KeyValues> &values) const {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values.assign(values_.begin(), values_.end());
  }

  inline const std::vector<legionclaw::interface::KeyValues> &values() const {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    return values_;
  }

  inline uint32_t values_size() const {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    return values_.size();
  }

  inline void clear_values() {
    std::lock_guard<std::mutex> lock(*values_mutex_);
    values_.clear();
    values_.shrink_to_fit();
  }

  inline bool has_values() { return (values_size() != 0); }

  void operator=(const DiagnosticStatus &diagnostic_status) {
    CopyFrom(diagnostic_status);
  }

  void CopyFrom(const DiagnosticStatus &diagnostic_status) {
    header_ = diagnostic_status.header();
    id_ = diagnostic_status.id();
    level_ = diagnostic_status.level();
    name_ = diagnostic_status.name();
    message_ = diagnostic_status.message();
    hardware_id_ = diagnostic_status.hardware_id();
    values_ = diagnostic_status.values();
  }

protected:
  std::shared_ptr<std::mutex> values_mutex_;
  // timestamp is  included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // level*100000 + 序号（RECOVER 400000） WARN 车运动状态（静止）100001
  // 左前门状态（关闭）100002 右前门状态（关闭）100003 左后门状态（关闭）100004
  // 右后门状态（关闭）100005 安全带状态（关闭）100006 后视镜状态（打开）100007
  // 充电枪状态 100008 油门状态 （正常）100009 智驾系统（正常）100010
  // 动力系统（正常）100011 左前轮胎压（正常）100012 右前轮胎压（正常）100013
  // 左后轮胎压（正常）100014 右后轮胎压（正常）100015 转向系统（正常）100016
  // 制动系统（正常）100017 档位系统（正常）100018 EPB系统（正常）100019
  // 气囊状态（正常）100020 油箱盖状态（正常）100021 后备箱状态（正常）100022
  // 底盘信息（正常）100023 定位信息（正常）100024 方向盘人工接管 100025
  // 电量表（正常）100026 暂停超时 100027 暂停次数过多 100028 泊车超时 100029
  // 人为刹车 100030 人为挂P挡 100031 障碍物碰撞 100032 EPB状态(释放) 100033
  // ERROR 规划失败 200000
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //诊断状态
  legionclaw::interface::DiagnosticStatus::Level level_;
  legionclaw::interface::DiagnosticStatus::Level *level_ptr_ = nullptr;
  //测试/组件报告的描述
  std::string name_;
  std::string *name_ptr_ = nullptr;
  //状态描述
  std::string message_;
  std::string *message_ptr_ = nullptr;
  //硬件的唯一描述
  std::string hardware_id_;
  std::string *hardware_id_ptr_ = nullptr;
  //与状态关联的key-value组
  std::vector<legionclaw::interface::KeyValues> values_;
};
} // namespace interface
} // namespace legionclaw
