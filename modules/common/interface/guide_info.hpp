/**
 * @file    guide_info.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/guide_road.hpp"
#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class GuideInfo {
public:
  GuideInfo() {
    next_dis_ = 0.0;
    round_status_ = 0;
    intersection_status_ = 0;
    roads_status_ = 0;
  }
  ~GuideInfo() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_next_dis(const double &next_dis) {
    next_dis_ = next_dis;
    next_dis_ptr_ = &next_dis_;
  }

  inline const double &next_dis() const { return next_dis_; }

  inline double *mutable_next_dis() { return &next_dis_; }

  inline bool has_next_dis() { return (next_dis_ptr_ != nullptr); }

  inline void
  set_current_road(const legionclaw::interface::GuideRoad &current_road) {
    current_road_ = current_road;
    current_road_ptr_ = &current_road_;
  }

  inline const legionclaw::interface::GuideRoad &current_road() const {
    return current_road_;
  }

  inline legionclaw::interface::GuideRoad *mutable_current_road() {
    return &current_road_;
  }

  inline bool has_current_road() { return (current_road_ptr_ != nullptr); }

  inline void set_next_road(const legionclaw::interface::GuideRoad &next_road) {
    next_road_ = next_road;
    next_road_ptr_ = &next_road_;
  }

  inline const legionclaw::interface::GuideRoad &next_road() const {
    return next_road_;
  }

  inline legionclaw::interface::GuideRoad *mutable_next_road() {
    return &next_road_;
  }

  inline bool has_next_road() { return (next_road_ptr_ != nullptr); }

  inline void set_round_status(const int8_t &round_status) {
    round_status_ = round_status;
    round_status_ptr_ = &round_status_;
  }

  inline const int8_t &round_status() const { return round_status_; }

  inline int8_t *mutable_round_status() { return &round_status_; }

  inline bool has_round_status() { return (round_status_ptr_ != nullptr); }

  inline void set_intersection_status(const int8_t &intersection_status) {
    intersection_status_ = intersection_status;
    intersection_status_ptr_ = &intersection_status_;
  }

  inline const int8_t &intersection_status() const {
    return intersection_status_;
  }

  inline int8_t *mutable_intersection_status() { return &intersection_status_; }

  inline bool has_intersection_status() {
    return (intersection_status_ptr_ != nullptr);
  }

  inline void set_roads_status(const int8_t &roads_status) {
    roads_status_ = roads_status;
    roads_status_ptr_ = &roads_status_;
  }

  inline const int8_t &roads_status() const { return roads_status_; }

  inline int8_t *mutable_roads_status() { return &roads_status_; }

  inline bool has_roads_status() { return (roads_status_ptr_ != nullptr); }

  void operator=(const GuideInfo &guide_info) { CopyFrom(guide_info); }

  void CopyFrom(const GuideInfo &guide_info) {
    header_ = guide_info.header();
    next_dis_ = guide_info.next_dis();
    current_road_ = guide_info.current_road();
    next_road_ = guide_info.next_road();
    round_status_ = guide_info.round_status();
    intersection_status_ = guide_info.intersection_status();
    roads_status_ = guide_info.roads_status();
  }

protected:
  //头部信息
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //下一条规划路线上道路的距离
  double next_dis_;
  double *next_dis_ptr_ = nullptr;
  //当前道路的引导信息
  legionclaw::interface::GuideRoad current_road_;
  legionclaw::interface::GuideRoad *current_road_ptr_ = nullptr;
  //下一条规划道路的引导信息
  legionclaw::interface::GuideRoad next_road_;
  legionclaw::interface::GuideRoad *next_road_ptr_ = nullptr;
  //环岛状态: 0：未知或非环岛状态
  //1：当前为非环岛道路并且规划路线即将驶入环岛主路
  //2：当前道路类型是环岛并且规划路线还是在环岛主路中行驶
  //3：当前道路类型是环岛并且规划路线即将驶出环岛主路
  int8_t round_status_;
  int8_t *round_status_ptr_ = nullptr;
  //交叉路口状态: 0：未知 1：非交叉路口状态 2：交叉路口通行状态
  int8_t intersection_status_;
  int8_t *intersection_status_ptr_ = nullptr;
  //当前行车推荐状态引导：(跨路口)
  //0：1km内（1km配置文件可配置）无需要左转、右转路口
  //1：需要左转路口（1km配置文件可配置） 2：需要右转路口  3：车道级规划失败
  int8_t roads_status_;
  int8_t *roads_status_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
