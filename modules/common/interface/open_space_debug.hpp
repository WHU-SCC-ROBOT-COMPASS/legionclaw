/**
 * @file    open_space_debug.hpp
 * @author  zdhy
 * @date    2022-04-11
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

#include "modules/common/interface/point_2d_list.hpp"
#include "modules/common/interface/polygon_2d.hpp"
#include "modules/common/interface/trajectory.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class OpenSpaceDebug {
public:
  OpenSpaceDebug() {
    obstacle_list_mutex_ = std::make_shared<std::mutex>();
    obstacles_vec_mutex_ = std::make_shared<std::mutex>();
    vehicle_preiew_polygon_mutex_ = std::make_shared<std::mutex>();

    lat_error_ = 0.0;
    lon_error_ = 0.0;
    yaw_error_ = 0.0;
    pos_error_ = 0.0;
    clear_obstacle_list();
    clear_obstacles_vec();
    clear_vehicle_preiew_polygon();
  }
  ~OpenSpaceDebug() = default;

  inline void set_lat_error(const double &lat_error) {
    lat_error_ = lat_error;
    lat_error_ptr_ = &lat_error_;
  }

  inline const double &lat_error() const { return lat_error_; }

  inline double *mutable_lat_error() { return &lat_error_; }

  inline bool has_lat_error() { return (lat_error_ptr_ != nullptr); }

  inline void set_lon_error(const double &lon_error) {
    lon_error_ = lon_error;
    lon_error_ptr_ = &lon_error_;
  }

  inline const double &lon_error() const { return lon_error_; }

  inline double *mutable_lon_error() { return &lon_error_; }

  inline bool has_lon_error() { return (lon_error_ptr_ != nullptr); }

  inline void set_yaw_error(const double &yaw_error) {
    yaw_error_ = yaw_error;
    yaw_error_ptr_ = &yaw_error_;
  }

  inline const double &yaw_error() const { return yaw_error_; }

  inline double *mutable_yaw_error() { return &yaw_error_; }

  inline bool has_yaw_error() { return (yaw_error_ptr_ != nullptr); }

  inline void set_pos_error(const double &pos_error) {
    pos_error_ = pos_error;
    pos_error_ptr_ = &pos_error_;
  }

  inline const double &pos_error() const { return pos_error_; }

  inline double *mutable_pos_error() { return &pos_error_; }

  inline bool has_pos_error() { return (pos_error_ptr_ != nullptr); }

  inline void
  set_warm_start_path(const legionclaw::interface::Trajectory &warm_start_path) {
    warm_start_path_ = warm_start_path;
    warm_start_path_ptr_ = &warm_start_path_;
  }

  inline const legionclaw::interface::Trajectory &warm_start_path() const {
    return warm_start_path_;
  }

  inline legionclaw::interface::Trajectory *mutable_warm_start_path() {
    return &warm_start_path_;
  }

  inline bool has_warm_start_path() {
    return (warm_start_path_ptr_ != nullptr);
  }

  inline void
  set_smoothed_path(const legionclaw::interface::Trajectory &smoothed_path) {
    smoothed_path_ = smoothed_path;
    smoothed_path_ptr_ = &smoothed_path_;
  }

  inline const legionclaw::interface::Trajectory &smoothed_path() const {
    return smoothed_path_;
  }

  inline legionclaw::interface::Trajectory *mutable_smoothed_path() {
    return &smoothed_path_;
  }

  inline bool has_smoothed_path() { return (smoothed_path_ptr_ != nullptr); }

  inline void
  set_obstacle_list(std::vector<legionclaw::interface::Polygon2D> *obstacle_list) {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list_.assign(obstacle_list->begin(), obstacle_list->end());
  }

  inline void set_obstacle_list(
      const std::vector<legionclaw::interface::Polygon2D> &obstacle_list) {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list_ = obstacle_list;
  }

  inline void set_obstacle_list(const uint32_t index,
                                legionclaw::interface::Polygon2D &obstacle_list) {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list_[index] = obstacle_list;
  }

  inline void
  add_obstacle_list(const legionclaw::interface::Polygon2D &obstacle_list) {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list_.emplace_back(obstacle_list);
  }

  inline const legionclaw::interface::Polygon2D &
  obstacle_list(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    return obstacle_list_[index];
  }

  inline std::vector<legionclaw::interface::Polygon2D> *mutable_obstacle_list() {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    return &obstacle_list_;
  }

  inline void
  obstacle_list(std::vector<legionclaw::interface::Polygon2D> &obstacle_list) const {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list.assign(obstacle_list_.begin(), obstacle_list_.end());
  }

  inline const std::vector<legionclaw::interface::Polygon2D> &obstacle_list() const {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    return obstacle_list_;
  }

  inline uint32_t obstacle_list_size() const {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    return obstacle_list_.size();
  }

  inline void clear_obstacle_list() {
    std::lock_guard<std::mutex> lock(*obstacle_list_mutex_);
    obstacle_list_.clear();
  }

  inline bool has_obstacle_list() { return (obstacle_list_size() != 0); }

  inline void
  set_obstacles_vec(std::vector<legionclaw::interface::Point2dList> *obstacles_vec) {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec_.assign(obstacles_vec->begin(), obstacles_vec->end());
  }

  inline void set_obstacles_vec(
      const std::vector<legionclaw::interface::Point2dList> &obstacles_vec) {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec_ = obstacles_vec;
  }

  inline void set_obstacles_vec(const uint32_t index,
                                legionclaw::interface::Point2dList &obstacles_vec) {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec_[index] = obstacles_vec;
  }

  inline void
  add_obstacles_vec(const legionclaw::interface::Point2dList &obstacles_vec) {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec_.emplace_back(obstacles_vec);
  }

  inline const legionclaw::interface::Point2dList &
  obstacles_vec(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    return obstacles_vec_[index];
  }

  inline std::vector<legionclaw::interface::Point2dList> *mutable_obstacles_vec() {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    return &obstacles_vec_;
  }

  inline void obstacles_vec(
      std::vector<legionclaw::interface::Point2dList> &obstacles_vec) const {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec.assign(obstacles_vec_.begin(), obstacles_vec_.end());
  }

  inline const std::vector<legionclaw::interface::Point2dList> &
  obstacles_vec() const {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    return obstacles_vec_;
  }

  inline uint32_t obstacles_vec_size() const {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    return obstacles_vec_.size();
  }

  inline void clear_obstacles_vec() {
    std::lock_guard<std::mutex> lock(*obstacles_vec_mutex_);
    obstacles_vec_.clear();
  }

  inline bool has_obstacles_vec() { return (obstacles_vec_size() != 0); }

  inline void set_vehicle_preiew_polygon(
      std::vector<legionclaw::interface::Polygon2D> *vehicle_preiew_polygon) {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon_.assign(vehicle_preiew_polygon->begin(),
                                   vehicle_preiew_polygon->end());
  }

  inline void set_vehicle_preiew_polygon(
      const std::vector<legionclaw::interface::Polygon2D> &vehicle_preiew_polygon) {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon_ = vehicle_preiew_polygon;
  }

  inline void set_vehicle_preiew_polygon(
      const uint32_t index,
      legionclaw::interface::Polygon2D &vehicle_preiew_polygon) {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon_[index] = vehicle_preiew_polygon;
  }

  inline void add_vehicle_preiew_polygon(
      const legionclaw::interface::Polygon2D &vehicle_preiew_polygon) {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon_.emplace_back(vehicle_preiew_polygon);
  }

  inline const legionclaw::interface::Polygon2D &
  vehicle_preiew_polygon(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    return vehicle_preiew_polygon_[index];
  }

  inline std::vector<legionclaw::interface::Polygon2D> *
  mutable_vehicle_preiew_polygon() {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    return &vehicle_preiew_polygon_;
  }

  inline void vehicle_preiew_polygon(
      std::vector<legionclaw::interface::Polygon2D> &vehicle_preiew_polygon) const {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon.assign(vehicle_preiew_polygon_.begin(),
                                  vehicle_preiew_polygon_.end());
  }

  inline const std::vector<legionclaw::interface::Polygon2D> &
  vehicle_preiew_polygon() const {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    return vehicle_preiew_polygon_;
  }

  inline uint32_t vehicle_preiew_polygon_size() const {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    return vehicle_preiew_polygon_.size();
  }

  inline void clear_vehicle_preiew_polygon() {
    std::lock_guard<std::mutex> lock(*vehicle_preiew_polygon_mutex_);
    vehicle_preiew_polygon_.clear();
  }

  inline bool has_vehicle_preiew_polygon() {
    return (vehicle_preiew_polygon_size() != 0);
  }

  void operator=(const OpenSpaceDebug &open_space_debug) {
    CopyFrom(open_space_debug);
  }

  void CopyFrom(const OpenSpaceDebug &open_space_debug) {
    lat_error_ = open_space_debug.lat_error();
    lon_error_ = open_space_debug.lon_error();
    yaw_error_ = open_space_debug.yaw_error();
    pos_error_ = open_space_debug.pos_error();
    warm_start_path_ = open_space_debug.warm_start_path();
    smoothed_path_ = open_space_debug.smoothed_path();
    obstacle_list_ = open_space_debug.obstacle_list();
    obstacles_vec_ = open_space_debug.obstacles_vec();
    vehicle_preiew_polygon_ = open_space_debug.vehicle_preiew_polygon();
  }

protected:
  std::shared_ptr<std::mutex> obstacle_list_mutex_;
  std::shared_ptr<std::mutex> obstacles_vec_mutex_;
  std::shared_ptr<std::mutex> vehicle_preiew_polygon_mutex_;
  //横向误差
  double lat_error_;
  double *lat_error_ptr_ = nullptr;
  //纵向误差
  double lon_error_;
  double *lon_error_ptr_ = nullptr;
  //角度误差(度)
  double yaw_error_;
  double *yaw_error_ptr_ = nullptr;
  //位置误差
  double pos_error_;
  double *pos_error_ptr_ = nullptr;
  //粗轨迹
  legionclaw::interface::Trajectory warm_start_path_;
  legionclaw::interface::Trajectory *warm_start_path_ptr_ = nullptr;
  //平滑轨迹
  legionclaw::interface::Trajectory smoothed_path_;
  legionclaw::interface::Trajectory *smoothed_path_ptr_ = nullptr;
  //动态障碍物
  std::vector<legionclaw::interface::Polygon2D> obstacle_list_;
  //静态障碍物
  std::vector<legionclaw::interface::Point2dList> obstacles_vec_;
  //车辆预测多边形
  std::vector<legionclaw::interface::Polygon2D> vehicle_preiew_polygon_;
};
} // namespace interface
} // namespace legionclaw
