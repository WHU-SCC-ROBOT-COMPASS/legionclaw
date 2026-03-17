/**
 * @file    guide_road.hpp
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

#include "modules/common/interface/curvature_info.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class GuideRoad {
public:
  GuideRoad() {
    curvature_mutex_ = std::make_shared<std::mutex>();

    road_id_ = 0;
    road_type_ = 0;
    turn_type_ = 0;
    avg_curvature_ = 0.0;
    curvature_size_ = 0;
    clear_curvature();
  }
  ~GuideRoad() = default;

  inline void set_road_id(const int64_t &road_id) {
    road_id_ = road_id;
    road_id_ptr_ = &road_id_;
  }

  inline const int64_t &road_id() const { return road_id_; }

  inline int64_t *mutable_road_id() { return &road_id_; }

  inline bool has_road_id() { return (road_id_ptr_ != nullptr); }

  inline void set_road_type(const int8_t &road_type) {
    road_type_ = road_type;
    road_type_ptr_ = &road_type_;
  }

  inline const int8_t &road_type() const { return road_type_; }

  inline int8_t *mutable_road_type() { return &road_type_; }

  inline bool has_road_type() { return (road_type_ptr_ != nullptr); }

  inline void set_turn_type(const int8_t &turn_type) {
    turn_type_ = turn_type;
    turn_type_ptr_ = &turn_type_;
  }

  inline const int8_t &turn_type() const { return turn_type_; }

  inline int8_t *mutable_turn_type() { return &turn_type_; }

  inline bool has_turn_type() { return (turn_type_ptr_ != nullptr); }

  inline void set_avg_curvature(const double &avg_curvature) {
    avg_curvature_ = avg_curvature;
    avg_curvature_ptr_ = &avg_curvature_;
  }

  inline const double &avg_curvature() const { return avg_curvature_; }

  inline double *mutable_avg_curvature() { return &avg_curvature_; }

  inline bool has_avg_curvature() { return (avg_curvature_ptr_ != nullptr); }

  inline void
  set_curvature(std::vector<legionclaw::interface::CurvatureInfo> *curvature) {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature_.assign(curvature->begin(), curvature->end());
  }

  inline void
  set_curvature(const std::vector<legionclaw::interface::CurvatureInfo> &curvature) {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature_ = curvature;
  }

  inline void set_curvature(const uint32_t index,
                            legionclaw::interface::CurvatureInfo &curvature) {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature_[index] = curvature;
  }

  inline void add_curvature(const legionclaw::interface::CurvatureInfo &curvature) {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature_.emplace_back(curvature);
  }

  inline const legionclaw::interface::CurvatureInfo &
  curvature(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    return curvature_[index];
  }

  inline std::vector<legionclaw::interface::CurvatureInfo> *mutable_curvature() {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    return &curvature_;
  }

  inline void
  curvature(std::vector<legionclaw::interface::CurvatureInfo> &curvature) const {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature.assign(curvature_.begin(), curvature_.end());
  }

  inline const std::vector<legionclaw::interface::CurvatureInfo> &curvature() const {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    return curvature_;
  }

  inline uint32_t curvature_size() const {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    return curvature_.size();
  }

  inline void clear_curvature() {
    std::lock_guard<std::mutex> lock(*curvature_mutex_);
    curvature_.clear();
    curvature_.shrink_to_fit();
  }

  inline bool has_curvature() { return (curvature_size() != 0); }

  void operator=(const GuideRoad &guide_road) { CopyFrom(guide_road); }

  void CopyFrom(const GuideRoad &guide_road) {
    road_id_ = guide_road.road_id();
    road_type_ = guide_road.road_type();
    turn_type_ = guide_road.turn_type();
    avg_curvature_ = guide_road.avg_curvature();
    curvature_size_ = guide_road.curvature_size();
    curvature_ = guide_road.curvature();
  }

protected:
  std::shared_ptr<std::mutex> curvature_mutex_;
  //道路的id
  int64_t road_id_;
  int64_t *road_id_ptr_ = nullptr;
  //道路的类型: 0：未知或一般道路  1：环岛 2：交叉路口 3：主路 4：辅路
  int8_t road_type_;
  int8_t *road_type_ptr_ = nullptr;
  //规划路线上道路的转向信息 1：左转 2：直行 3：右转
  int8_t turn_type_;
  int8_t *turn_type_ptr_ = nullptr;
  //道路的平均曲率
  double avg_curvature_;
  double *avg_curvature_ptr_ = nullptr;
  //道路中心线上点(曲率)数量
  int32_t curvature_size_;
  int32_t *curvature_size_ptr_ = nullptr;
  //道路曲率信息数组
  std::vector<legionclaw::interface::CurvatureInfo> curvature_;
};
} // namespace interface
} // namespace legionclaw
