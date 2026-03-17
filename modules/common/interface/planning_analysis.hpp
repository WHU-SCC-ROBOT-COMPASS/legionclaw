/**
 * @file    planning_analysis.hpp
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
#include <vector>

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/planning_parking_debug.hpp"
#include "modules/common/interface/time_consume.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PlanningAnalysis {
public:
  PlanningAnalysis() {
    evaluator_time_mutex_ = std::make_shared<std::mutex>();

    frame_update_time_ = 0.0;
    generator_time_ = 0.0;
    clear_evaluator_time();
    selector_time_ = 0.0;
    dis_to_center_line_ = 0.0;
    diff_to_velocity_ = 0.0;
    referencline_kappa_ = 0.0;
  }
  ~PlanningAnalysis() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_frame_update_time(const double &frame_update_time) {
    frame_update_time_ = frame_update_time;
    frame_update_time_ptr_ = &frame_update_time_;
  }

  inline const double &frame_update_time() const { return frame_update_time_; }

  inline double *mutable_frame_update_time() { return &frame_update_time_; }

  inline bool has_frame_update_time() {
    return (frame_update_time_ptr_ != nullptr);
  }

  inline void set_generator_time(const double &generator_time) {
    generator_time_ = generator_time;
    generator_time_ptr_ = &generator_time_;
  }

  inline const double &generator_time() const { return generator_time_; }

  inline double *mutable_generator_time() { return &generator_time_; }

  inline bool has_generator_time() { return (generator_time_ptr_ != nullptr); }

  inline void set_evaluator_time(
      std::vector<legionclaw::interface::TimeConsume> *evaluator_time) {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time_.assign(evaluator_time->begin(), evaluator_time->end());
  }

  inline void set_evaluator_time(
      const std::vector<legionclaw::interface::TimeConsume> &evaluator_time) {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time_ = evaluator_time;
  }

  inline void
  set_evaluator_time(const uint32_t index,
                     legionclaw::interface::TimeConsume &evaluator_time) {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time_[index] = evaluator_time;
  }

  inline void
  add_evaluator_time(const legionclaw::interface::TimeConsume &evaluator_time) {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time_.emplace_back(evaluator_time);
  }

  inline const legionclaw::interface::TimeConsume &
  evaluator_time(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    return evaluator_time_[index];
  }

  inline std::vector<legionclaw::interface::TimeConsume> *mutable_evaluator_time() {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    return &evaluator_time_;
  }

  inline void evaluator_time(
      std::vector<legionclaw::interface::TimeConsume> &evaluator_time) const {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time.assign(evaluator_time_.begin(), evaluator_time_.end());
  }

  inline const std::vector<legionclaw::interface::TimeConsume> &
  evaluator_time() const {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    return evaluator_time_;
  }

  inline uint32_t evaluator_time_size() const {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    return evaluator_time_.size();
  }

  inline void clear_evaluator_time() {
    std::lock_guard<std::mutex> lock(*evaluator_time_mutex_);
    evaluator_time_.clear();
    evaluator_time_.shrink_to_fit();
  }

  inline bool has_evaluator_time() { return (evaluator_time_size() != 0); }

  inline void set_selector_time(const double &selector_time) {
    selector_time_ = selector_time;
    selector_time_ptr_ = &selector_time_;
  }

  inline const double &selector_time() const { return selector_time_; }

  inline double *mutable_selector_time() { return &selector_time_; }

  inline bool has_selector_time() { return (selector_time_ptr_ != nullptr); }

  inline void set_planning_parking_debug(
      const legionclaw::interface::PlanningParkingDebug &planning_parking_debug) {
    planning_parking_debug_ = planning_parking_debug;
    planning_parking_debug_ptr_ = &planning_parking_debug_;
  }

  inline const legionclaw::interface::PlanningParkingDebug &
  planning_parking_debug() const {
    return planning_parking_debug_;
  }

  inline legionclaw::interface::PlanningParkingDebug *
  mutable_planning_parking_debug() {
    return &planning_parking_debug_;
  }

  inline bool has_planning_parking_debug() {
    return (planning_parking_debug_ptr_ != nullptr);
  }

  inline void set_dis_to_center_line(const double &dis_to_center_line) {
    dis_to_center_line_ = dis_to_center_line;
    dis_to_center_line_ptr_ = &dis_to_center_line_;
  }

  inline const double &dis_to_center_line() const {
    return dis_to_center_line_;
  }

  inline double *mutable_dis_to_center_line() { return &dis_to_center_line_; }

  inline bool has_dis_to_center_line() {
    return (dis_to_center_line_ptr_ != nullptr);
  }

  inline void set_diff_to_velocity(const double &diff_to_velocity) {
    diff_to_velocity_ = diff_to_velocity;
    diff_to_velocity_ptr_ = &diff_to_velocity_;
  }

  inline const double &diff_to_velocity() const { return diff_to_velocity_; }

  inline double *mutable_diff_to_velocity() { return &diff_to_velocity_; }

  inline bool has_diff_to_velocity() {
    return (diff_to_velocity_ptr_ != nullptr);
  }

  inline void set_referencline_kappa(const double &referencline_kappa) {
    referencline_kappa_ = referencline_kappa;
    referencline_kappa_ptr_ = &referencline_kappa_;
  }

  inline const double &referencline_kappa() const {
    return referencline_kappa_;
  }

  inline double *mutable_referencline_kappa() { return &referencline_kappa_; }

  inline bool has_referencline_kappa() {
    return (referencline_kappa_ptr_ != nullptr);
  }

  void operator=(const PlanningAnalysis &planning_analysis) {
    CopyFrom(planning_analysis);
  }

  void CopyFrom(const PlanningAnalysis &planning_analysis) {
    header_ = planning_analysis.header();
    frame_update_time_ = planning_analysis.frame_update_time();
    generator_time_ = planning_analysis.generator_time();
    evaluator_time_ = planning_analysis.evaluator_time();
    selector_time_ = planning_analysis.selector_time();
    planning_parking_debug_ = planning_analysis.planning_parking_debug();
    dis_to_center_line_ = planning_analysis.dis_to_center_line();
    diff_to_velocity_ = planning_analysis.diff_to_velocity();
    referencline_kappa_ = planning_analysis.referencline_kappa();
  }

protected:
  std::shared_ptr<std::mutex> evaluator_time_mutex_;
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // frame update计算时间
  double frame_update_time_;
  double *frame_update_time_ptr_ = nullptr;
  //轨迹生成 计算时间
  double generator_time_;
  double *generator_time_ptr_ = nullptr;
  //轨迹评估计算时间
  std::vector<legionclaw::interface::TimeConsume> evaluator_time_;
  //轨迹选择计算时间
  double selector_time_;
  double *selector_time_ptr_ = nullptr;
  // planning泊车调试消息
  legionclaw::interface::PlanningParkingDebug planning_parking_debug_;
  legionclaw::interface::PlanningParkingDebug *planning_parking_debug_ptr_ = nullptr;
  //车辆距离车道中心线的垂直距离
  double dis_to_center_line_;
  double *dis_to_center_line_ptr_ = nullptr;
  //目标速度与规划速度之差
  double diff_to_velocity_;
  double *diff_to_velocity_ptr_ = nullptr;
  //参考线曲率
  double referencline_kappa_;
  double *referencline_kappa_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
