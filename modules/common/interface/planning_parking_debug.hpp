/**
 * @file    planning_parking_debug.hpp
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

#include "modules/common/enum/enum.h"
#include "modules/common/interface/parking_state_display.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/point_2d_list.hpp"
#include "modules/common/interface/polygon_2d.hpp"
#include "modules/common/interface/trajectory.hpp"
#include "modules/common/interface/trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PlanningParkingDebug {
public:
  PlanningParkingDebug() {
    vehicle_preiew_polygon_mutex_ = std::make_shared<std::mutex>();
    obstacles_polygon_mutex_ = std::make_shared<std::mutex>();
    path_points_mutex_ = std::make_shared<std::mutex>();
    obstacles_vec_mutex_ = std::make_shared<std::mutex>();
    warm_start_traj_mutex_ = std::make_shared<std::mutex>();
    smoothed_traj_stage1_mutex_ = std::make_shared<std::mutex>();
    smoothed_traj_stage2_mutex_ = std::make_shared<std::mutex>();
    trajectory_array_mutex_ = std::make_shared<std::mutex>();

    clear_vehicle_preiew_polygon();
    clear_obstacles_polygon();
    clear_path_points();
    lat_error_ = 0.0;
    lon_error_ = 0.0;
    yaw_error_ = 0.0;
    clear_obstacles_vec();
    clear_warm_start_traj();
    clear_smoothed_traj_stage1();
    clear_smoothed_traj_stage2();
    clear_trajectory_array();
    hybrid_a_star_map_time_ = 0.0;
    hybrid_a_star_heuristic_time_ = 0.0;
    hybrid_a_star_rs_time_ = 0.0;
    hybrid_a_star_total_time_ = 0.0;
    ias_collision_avoidance_time_ = 0.0;
    ias_path_smooth_time_ = 0.0;
    ias_speed_smooth_time_ = 0.0;
    ias_total_time_ = 0.0;
    samping_trajectory_time_ = 0.0;
    is_replan_ = false;
    replan_reason_.clear();
    replan_time_ = 0.0;
    replan_num_ = 0;
    optimizer_thread_counter_ = 0;
    replan_by_context_update_counter_ = 0;
    replan_by_large_error_counter_ = 0;
    parking_type_ = legionclaw::common::ParkingType::INVALID_PARKING;
    moves_counter_ = 0;
    remain_distance_ = 0.0;
    distance_to_leader_obj_ = 0.0;
    state_ = legionclaw::interface::ParkingStateDisplay::ParkingState::
        PARKING_STATE_NORMAL;
  }
  ~PlanningParkingDebug() = default;

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
    vehicle_preiew_polygon_.shrink_to_fit();
  }

  inline bool has_vehicle_preiew_polygon() {
    return (vehicle_preiew_polygon_size() != 0);
  }

  inline void set_obstacles_polygon(
      std::vector<legionclaw::interface::Polygon2D> *obstacles_polygon) {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon_.assign(obstacles_polygon->begin(),
                              obstacles_polygon->end());
  }

  inline void set_obstacles_polygon(
      const std::vector<legionclaw::interface::Polygon2D> &obstacles_polygon) {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon_ = obstacles_polygon;
  }

  inline void
  set_obstacles_polygon(const uint32_t index,
                        legionclaw::interface::Polygon2D &obstacles_polygon) {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon_[index] = obstacles_polygon;
  }

  inline void
  add_obstacles_polygon(const legionclaw::interface::Polygon2D &obstacles_polygon) {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon_.emplace_back(obstacles_polygon);
  }

  inline const legionclaw::interface::Polygon2D &
  obstacles_polygon(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    return obstacles_polygon_[index];
  }

  inline std::vector<legionclaw::interface::Polygon2D> *mutable_obstacles_polygon() {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    return &obstacles_polygon_;
  }

  inline void obstacles_polygon(
      std::vector<legionclaw::interface::Polygon2D> &obstacles_polygon) const {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon.assign(obstacles_polygon_.begin(),
                             obstacles_polygon_.end());
  }

  inline const std::vector<legionclaw::interface::Polygon2D> &
  obstacles_polygon() const {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    return obstacles_polygon_;
  }

  inline uint32_t obstacles_polygon_size() const {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    return obstacles_polygon_.size();
  }

  inline void clear_obstacles_polygon() {
    std::lock_guard<std::mutex> lock(*obstacles_polygon_mutex_);
    obstacles_polygon_.clear();
    obstacles_polygon_.shrink_to_fit();
  }

  inline bool has_obstacles_polygon() {
    return (obstacles_polygon_size() != 0);
  }

  inline void
  set_path_points(std::vector<legionclaw::interface::PathPoint> *path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.assign(path_points->begin(), path_points->end());
  }

  inline void
  set_path_points(const std::vector<legionclaw::interface::PathPoint> &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_ = path_points;
  }

  inline void set_path_points(const uint32_t index,
                              legionclaw::interface::PathPoint &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_[index] = path_points;
  }

  inline void add_path_points(const legionclaw::interface::PathPoint &path_points) {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.emplace_back(path_points);
  }

  inline const legionclaw::interface::PathPoint &path_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_[index];
  }

  inline std::vector<legionclaw::interface::PathPoint> *mutable_path_points() {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return &path_points_;
  }

  inline void
  path_points(std::vector<legionclaw::interface::PathPoint> &path_points) const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points.assign(path_points_.begin(), path_points_.end());
  }

  inline const std::vector<legionclaw::interface::PathPoint> &path_points() const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_;
  }

  inline uint32_t path_points_size() const {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    return path_points_.size();
  }

  inline void clear_path_points() {
    std::lock_guard<std::mutex> lock(*path_points_mutex_);
    path_points_.clear();
    path_points_.shrink_to_fit();
  }

  inline bool has_path_points() { return (path_points_size() != 0); }

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
    obstacles_vec_.shrink_to_fit();
  }

  inline bool has_obstacles_vec() { return (obstacles_vec_size() != 0); }

  inline void set_warm_start_traj(
      std::vector<legionclaw::interface::TrajectoryPoint> *warm_start_traj) {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj_.assign(warm_start_traj->begin(), warm_start_traj->end());
  }

  inline void set_warm_start_traj(
      const std::vector<legionclaw::interface::TrajectoryPoint> &warm_start_traj) {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj_ = warm_start_traj;
  }

  inline void
  set_warm_start_traj(const uint32_t index,
                      legionclaw::interface::TrajectoryPoint &warm_start_traj) {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj_[index] = warm_start_traj;
  }

  inline void add_warm_start_traj(
      const legionclaw::interface::TrajectoryPoint &warm_start_traj) {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj_.emplace_back(warm_start_traj);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  warm_start_traj(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    return warm_start_traj_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_warm_start_traj() {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    return &warm_start_traj_;
  }

  inline void warm_start_traj(
      std::vector<legionclaw::interface::TrajectoryPoint> &warm_start_traj) const {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj.assign(warm_start_traj_.begin(), warm_start_traj_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
  warm_start_traj() const {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    return warm_start_traj_;
  }

  inline uint32_t warm_start_traj_size() const {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    return warm_start_traj_.size();
  }

  inline void clear_warm_start_traj() {
    std::lock_guard<std::mutex> lock(*warm_start_traj_mutex_);
    warm_start_traj_.clear();
    warm_start_traj_.shrink_to_fit();
  }

  inline bool has_warm_start_traj() { return (warm_start_traj_size() != 0); }

  inline void set_smoothed_traj_stage1(
      std::vector<legionclaw::interface::TrajectoryPoint> *smoothed_traj_stage1) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1_.assign(smoothed_traj_stage1->begin(),
                                 smoothed_traj_stage1->end());
  }

  inline void
  set_smoothed_traj_stage1(const std::vector<legionclaw::interface::TrajectoryPoint>
                               &smoothed_traj_stage1) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1_ = smoothed_traj_stage1;
  }

  inline void set_smoothed_traj_stage1(
      const uint32_t index,
      legionclaw::interface::TrajectoryPoint &smoothed_traj_stage1) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1_[index] = smoothed_traj_stage1;
  }

  inline void add_smoothed_traj_stage1(
      const legionclaw::interface::TrajectoryPoint &smoothed_traj_stage1) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1_.emplace_back(smoothed_traj_stage1);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  smoothed_traj_stage1(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    return smoothed_traj_stage1_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_smoothed_traj_stage1() {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    return &smoothed_traj_stage1_;
  }

  inline void smoothed_traj_stage1(
      std::vector<legionclaw::interface::TrajectoryPoint> &smoothed_traj_stage1)
      const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1.assign(smoothed_traj_stage1_.begin(),
                                smoothed_traj_stage1_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
  smoothed_traj_stage1() const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    return smoothed_traj_stage1_;
  }

  inline uint32_t smoothed_traj_stage1_size() const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    return smoothed_traj_stage1_.size();
  }

  inline void clear_smoothed_traj_stage1() {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage1_mutex_);
    smoothed_traj_stage1_.clear();
    smoothed_traj_stage1_.shrink_to_fit();
  }

  inline bool has_smoothed_traj_stage1() {
    return (smoothed_traj_stage1_size() != 0);
  }

  inline void set_smoothed_traj_stage2(
      std::vector<legionclaw::interface::TrajectoryPoint> *smoothed_traj_stage2) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2_.assign(smoothed_traj_stage2->begin(),
                                 smoothed_traj_stage2->end());
  }

  inline void
  set_smoothed_traj_stage2(const std::vector<legionclaw::interface::TrajectoryPoint>
                               &smoothed_traj_stage2) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2_ = smoothed_traj_stage2;
  }

  inline void set_smoothed_traj_stage2(
      const uint32_t index,
      legionclaw::interface::TrajectoryPoint &smoothed_traj_stage2) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2_[index] = smoothed_traj_stage2;
  }

  inline void add_smoothed_traj_stage2(
      const legionclaw::interface::TrajectoryPoint &smoothed_traj_stage2) {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2_.emplace_back(smoothed_traj_stage2);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  smoothed_traj_stage2(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    return smoothed_traj_stage2_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_smoothed_traj_stage2() {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    return &smoothed_traj_stage2_;
  }

  inline void smoothed_traj_stage2(
      std::vector<legionclaw::interface::TrajectoryPoint> &smoothed_traj_stage2)
      const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2.assign(smoothed_traj_stage2_.begin(),
                                smoothed_traj_stage2_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
  smoothed_traj_stage2() const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    return smoothed_traj_stage2_;
  }

  inline uint32_t smoothed_traj_stage2_size() const {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    return smoothed_traj_stage2_.size();
  }

  inline void clear_smoothed_traj_stage2() {
    std::lock_guard<std::mutex> lock(*smoothed_traj_stage2_mutex_);
    smoothed_traj_stage2_.clear();
    smoothed_traj_stage2_.shrink_to_fit();
  }

  inline bool has_smoothed_traj_stage2() {
    return (smoothed_traj_stage2_size() != 0);
  }

  inline void
  set_reference_line(const legionclaw::interface::Trajectory &reference_line) {
    reference_line_ = reference_line;
    reference_line_ptr_ = &reference_line_;
  }

  inline const legionclaw::interface::Trajectory &reference_line() const {
    return reference_line_;
  }

  inline legionclaw::interface::Trajectory *mutable_reference_line() {
    return &reference_line_;
  }

  inline bool has_reference_line() { return (reference_line_ptr_ != nullptr); }

  inline void set_trajectory_array(
      std::vector<legionclaw::interface::Trajectory> *trajectory_array) {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array_.assign(trajectory_array->begin(),
                             trajectory_array->end());
  }

  inline void set_trajectory_array(
      const std::vector<legionclaw::interface::Trajectory> &trajectory_array) {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array_ = trajectory_array;
  }

  inline void
  set_trajectory_array(const uint32_t index,
                       legionclaw::interface::Trajectory &trajectory_array) {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array_[index] = trajectory_array;
  }

  inline void
  add_trajectory_array(const legionclaw::interface::Trajectory &trajectory_array) {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array_.emplace_back(trajectory_array);
  }

  inline const legionclaw::interface::Trajectory &
  trajectory_array(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    return trajectory_array_[index];
  }

  inline std::vector<legionclaw::interface::Trajectory> *mutable_trajectory_array() {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    return &trajectory_array_;
  }

  inline void trajectory_array(
      std::vector<legionclaw::interface::Trajectory> &trajectory_array) const {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array.assign(trajectory_array_.begin(), trajectory_array_.end());
  }

  inline const std::vector<legionclaw::interface::Trajectory> &
  trajectory_array() const {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    return trajectory_array_;
  }

  inline uint32_t trajectory_array_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    return trajectory_array_.size();
  }

  inline void clear_trajectory_array() {
    std::lock_guard<std::mutex> lock(*trajectory_array_mutex_);
    trajectory_array_.clear();
    trajectory_array_.shrink_to_fit();
  }

  inline bool has_trajectory_array() { return (trajectory_array_size() != 0); }

  inline void set_optimal_coarse_trajectory(
      const legionclaw::interface::Trajectory &optimal_coarse_trajectory) {
    optimal_coarse_trajectory_ = optimal_coarse_trajectory;
    optimal_coarse_trajectory_ptr_ = &optimal_coarse_trajectory_;
  }

  inline const legionclaw::interface::Trajectory &optimal_coarse_trajectory() const {
    return optimal_coarse_trajectory_;
  }

  inline legionclaw::interface::Trajectory *mutable_optimal_coarse_trajectory() {
    return &optimal_coarse_trajectory_;
  }

  inline bool has_optimal_coarse_trajectory() {
    return (optimal_coarse_trajectory_ptr_ != nullptr);
  }

  inline void set_optimal_smooth_trajectory(
      const legionclaw::interface::Trajectory &optimal_smooth_trajectory) {
    optimal_smooth_trajectory_ = optimal_smooth_trajectory;
    optimal_smooth_trajectory_ptr_ = &optimal_smooth_trajectory_;
  }

  inline const legionclaw::interface::Trajectory &optimal_smooth_trajectory() const {
    return optimal_smooth_trajectory_;
  }

  inline legionclaw::interface::Trajectory *mutable_optimal_smooth_trajectory() {
    return &optimal_smooth_trajectory_;
  }

  inline bool has_optimal_smooth_trajectory() {
    return (optimal_smooth_trajectory_ptr_ != nullptr);
  }

  inline void set_hybrid_a_star_map_time(const double &hybrid_a_star_map_time) {
    hybrid_a_star_map_time_ = hybrid_a_star_map_time;
    hybrid_a_star_map_time_ptr_ = &hybrid_a_star_map_time_;
  }

  inline const double &hybrid_a_star_map_time() const {
    return hybrid_a_star_map_time_;
  }

  inline double *mutable_hybrid_a_star_map_time() {
    return &hybrid_a_star_map_time_;
  }

  inline bool has_hybrid_a_star_map_time() {
    return (hybrid_a_star_map_time_ptr_ != nullptr);
  }

  inline void
  set_hybrid_a_star_heuristic_time(const double &hybrid_a_star_heuristic_time) {
    hybrid_a_star_heuristic_time_ = hybrid_a_star_heuristic_time;
    hybrid_a_star_heuristic_time_ptr_ = &hybrid_a_star_heuristic_time_;
  }

  inline const double &hybrid_a_star_heuristic_time() const {
    return hybrid_a_star_heuristic_time_;
  }

  inline double *mutable_hybrid_a_star_heuristic_time() {
    return &hybrid_a_star_heuristic_time_;
  }

  inline bool has_hybrid_a_star_heuristic_time() {
    return (hybrid_a_star_heuristic_time_ptr_ != nullptr);
  }

  inline void set_hybrid_a_star_rs_time(const double &hybrid_a_star_rs_time) {
    hybrid_a_star_rs_time_ = hybrid_a_star_rs_time;
    hybrid_a_star_rs_time_ptr_ = &hybrid_a_star_rs_time_;
  }

  inline const double &hybrid_a_star_rs_time() const {
    return hybrid_a_star_rs_time_;
  }

  inline double *mutable_hybrid_a_star_rs_time() {
    return &hybrid_a_star_rs_time_;
  }

  inline bool has_hybrid_a_star_rs_time() {
    return (hybrid_a_star_rs_time_ptr_ != nullptr);
  }

  inline void
  set_hybrid_a_star_total_time(const double &hybrid_a_star_total_time) {
    hybrid_a_star_total_time_ = hybrid_a_star_total_time;
    hybrid_a_star_total_time_ptr_ = &hybrid_a_star_total_time_;
  }

  inline const double &hybrid_a_star_total_time() const {
    return hybrid_a_star_total_time_;
  }

  inline double *mutable_hybrid_a_star_total_time() {
    return &hybrid_a_star_total_time_;
  }

  inline bool has_hybrid_a_star_total_time() {
    return (hybrid_a_star_total_time_ptr_ != nullptr);
  }

  inline void
  set_ias_collision_avoidance_time(const double &ias_collision_avoidance_time) {
    ias_collision_avoidance_time_ = ias_collision_avoidance_time;
    ias_collision_avoidance_time_ptr_ = &ias_collision_avoidance_time_;
  }

  inline const double &ias_collision_avoidance_time() const {
    return ias_collision_avoidance_time_;
  }

  inline double *mutable_ias_collision_avoidance_time() {
    return &ias_collision_avoidance_time_;
  }

  inline bool has_ias_collision_avoidance_time() {
    return (ias_collision_avoidance_time_ptr_ != nullptr);
  }

  inline void set_ias_path_smooth_time(const double &ias_path_smooth_time) {
    ias_path_smooth_time_ = ias_path_smooth_time;
    ias_path_smooth_time_ptr_ = &ias_path_smooth_time_;
  }

  inline const double &ias_path_smooth_time() const {
    return ias_path_smooth_time_;
  }

  inline double *mutable_ias_path_smooth_time() {
    return &ias_path_smooth_time_;
  }

  inline bool has_ias_path_smooth_time() {
    return (ias_path_smooth_time_ptr_ != nullptr);
  }

  inline void set_ias_speed_smooth_time(const double &ias_speed_smooth_time) {
    ias_speed_smooth_time_ = ias_speed_smooth_time;
    ias_speed_smooth_time_ptr_ = &ias_speed_smooth_time_;
  }

  inline const double &ias_speed_smooth_time() const {
    return ias_speed_smooth_time_;
  }

  inline double *mutable_ias_speed_smooth_time() {
    return &ias_speed_smooth_time_;
  }

  inline bool has_ias_speed_smooth_time() {
    return (ias_speed_smooth_time_ptr_ != nullptr);
  }

  inline void set_ias_total_time(const double &ias_total_time) {
    ias_total_time_ = ias_total_time;
    ias_total_time_ptr_ = &ias_total_time_;
  }

  inline const double &ias_total_time() const { return ias_total_time_; }

  inline double *mutable_ias_total_time() { return &ias_total_time_; }

  inline bool has_ias_total_time() { return (ias_total_time_ptr_ != nullptr); }

  inline void
  set_samping_trajectory_time(const double &samping_trajectory_time) {
    samping_trajectory_time_ = samping_trajectory_time;
    samping_trajectory_time_ptr_ = &samping_trajectory_time_;
  }

  inline const double &samping_trajectory_time() const {
    return samping_trajectory_time_;
  }

  inline double *mutable_samping_trajectory_time() {
    return &samping_trajectory_time_;
  }

  inline bool has_samping_trajectory_time() {
    return (samping_trajectory_time_ptr_ != nullptr);
  }

  inline void set_is_replan(const bool &is_replan) {
    is_replan_ = is_replan;
    is_replan_ptr_ = &is_replan_;
  }

  inline const bool &is_replan() const { return is_replan_; }

  inline bool *mutable_is_replan() { return &is_replan_; }

  inline bool has_is_replan() { return (is_replan_ptr_ != nullptr); }

  inline void set_replan_reason(const std::string &replan_reason) {
    replan_reason_ = replan_reason;
    replan_reason_ptr_ = &replan_reason_;
  }

  inline const std::string &replan_reason() const { return replan_reason_; }

  inline std::string *mutable_replan_reason() { return &replan_reason_; }

  inline bool has_replan_reason() { return (replan_reason_ptr_ != nullptr); }

  inline void set_replan_time(const double &replan_time) {
    replan_time_ = replan_time;
    replan_time_ptr_ = &replan_time_;
  }

  inline const double &replan_time() const { return replan_time_; }

  inline double *mutable_replan_time() { return &replan_time_; }

  inline bool has_replan_time() { return (replan_time_ptr_ != nullptr); }

  inline void set_replan_num(const int32_t &replan_num) {
    replan_num_ = replan_num;
    replan_num_ptr_ = &replan_num_;
  }

  inline const int32_t &replan_num() const { return replan_num_; }

  inline int32_t *mutable_replan_num() { return &replan_num_; }

  inline bool has_replan_num() { return (replan_num_ptr_ != nullptr); }

  inline void
  set_optimizer_thread_counter(const int32_t &optimizer_thread_counter) {
    optimizer_thread_counter_ = optimizer_thread_counter;
    optimizer_thread_counter_ptr_ = &optimizer_thread_counter_;
  }

  inline const int32_t &optimizer_thread_counter() const {
    return optimizer_thread_counter_;
  }

  inline int32_t *mutable_optimizer_thread_counter() {
    return &optimizer_thread_counter_;
  }

  inline bool has_optimizer_thread_counter() {
    return (optimizer_thread_counter_ptr_ != nullptr);
  }

  inline void set_replan_by_context_update_counter(
      const int32_t &replan_by_context_update_counter) {
    replan_by_context_update_counter_ = replan_by_context_update_counter;
    replan_by_context_update_counter_ptr_ = &replan_by_context_update_counter_;
  }

  inline const int32_t &replan_by_context_update_counter() const {
    return replan_by_context_update_counter_;
  }

  inline int32_t *mutable_replan_by_context_update_counter() {
    return &replan_by_context_update_counter_;
  }

  inline bool has_replan_by_context_update_counter() {
    return (replan_by_context_update_counter_ptr_ != nullptr);
  }

  inline void set_replan_by_large_error_counter(
      const int32_t &replan_by_large_error_counter) {
    replan_by_large_error_counter_ = replan_by_large_error_counter;
    replan_by_large_error_counter_ptr_ = &replan_by_large_error_counter_;
  }

  inline const int32_t &replan_by_large_error_counter() const {
    return replan_by_large_error_counter_;
  }

  inline int32_t *mutable_replan_by_large_error_counter() {
    return &replan_by_large_error_counter_;
  }

  inline bool has_replan_by_large_error_counter() {
    return (replan_by_large_error_counter_ptr_ != nullptr);
  }

  inline void set_parking_type(const legionclaw::common::ParkingType &parking_type) {
    parking_type_ = parking_type;
    parking_type_ptr_ = &parking_type_;
  }

  inline const legionclaw::common::ParkingType &parking_type() const {
    return parking_type_;
  }

  inline legionclaw::common::ParkingType *mutable_parking_type() {
    return &parking_type_;
  }

  inline bool has_parking_type() { return (parking_type_ptr_ != nullptr); }

  inline void set_moves_counter(const int32_t &moves_counter) {
    moves_counter_ = moves_counter;
    moves_counter_ptr_ = &moves_counter_;
  }

  inline const int32_t &moves_counter() const { return moves_counter_; }

  inline int32_t *mutable_moves_counter() { return &moves_counter_; }

  inline bool has_moves_counter() { return (moves_counter_ptr_ != nullptr); }

  inline void set_remain_distance(const double &remain_distance) {
    remain_distance_ = remain_distance;
    remain_distance_ptr_ = &remain_distance_;
  }

  inline const double &remain_distance() const { return remain_distance_; }

  inline double *mutable_remain_distance() { return &remain_distance_; }

  inline bool has_remain_distance() {
    return (remain_distance_ptr_ != nullptr);
  }

  inline void set_distance_to_leader_obj(const double &distance_to_leader_obj) {
    distance_to_leader_obj_ = distance_to_leader_obj;
    distance_to_leader_obj_ptr_ = &distance_to_leader_obj_;
  }

  inline const double &distance_to_leader_obj() const {
    return distance_to_leader_obj_;
  }

  inline double *mutable_distance_to_leader_obj() {
    return &distance_to_leader_obj_;
  }

  inline bool has_distance_to_leader_obj() {
    return (distance_to_leader_obj_ptr_ != nullptr);
  }

  inline void
  set_state(const legionclaw::interface::ParkingStateDisplay::ParkingState &state) {
    state_ = state;
    state_ptr_ = &state_;
  }

  inline const legionclaw::interface::ParkingStateDisplay::ParkingState &
  state() const {
    return state_;
  }

  inline legionclaw::interface::ParkingStateDisplay::ParkingState *mutable_state() {
    return &state_;
  }

  inline bool has_state() { return (state_ptr_ != nullptr); }

  void operator=(const PlanningParkingDebug &planning_parking_debug) {
    CopyFrom(planning_parking_debug);
  }

  void CopyFrom(const PlanningParkingDebug &planning_parking_debug) {
    vehicle_preiew_polygon_ = planning_parking_debug.vehicle_preiew_polygon();
    obstacles_polygon_ = planning_parking_debug.obstacles_polygon();
    path_points_ = planning_parking_debug.path_points();
    lat_error_ = planning_parking_debug.lat_error();
    lon_error_ = planning_parking_debug.lon_error();
    yaw_error_ = planning_parking_debug.yaw_error();
    obstacles_vec_ = planning_parking_debug.obstacles_vec();
    warm_start_traj_ = planning_parking_debug.warm_start_traj();
    smoothed_traj_stage1_ = planning_parking_debug.smoothed_traj_stage1();
    smoothed_traj_stage2_ = planning_parking_debug.smoothed_traj_stage2();
    reference_line_ = planning_parking_debug.reference_line();
    trajectory_array_ = planning_parking_debug.trajectory_array();
    optimal_coarse_trajectory_ =
        planning_parking_debug.optimal_coarse_trajectory();
    optimal_smooth_trajectory_ =
        planning_parking_debug.optimal_smooth_trajectory();
    hybrid_a_star_map_time_ = planning_parking_debug.hybrid_a_star_map_time();
    hybrid_a_star_heuristic_time_ =
        planning_parking_debug.hybrid_a_star_heuristic_time();
    hybrid_a_star_rs_time_ = planning_parking_debug.hybrid_a_star_rs_time();
    hybrid_a_star_total_time_ =
        planning_parking_debug.hybrid_a_star_total_time();
    ias_collision_avoidance_time_ =
        planning_parking_debug.ias_collision_avoidance_time();
    ias_path_smooth_time_ = planning_parking_debug.ias_path_smooth_time();
    ias_speed_smooth_time_ = planning_parking_debug.ias_speed_smooth_time();
    ias_total_time_ = planning_parking_debug.ias_total_time();
    samping_trajectory_time_ = planning_parking_debug.samping_trajectory_time();
    is_replan_ = planning_parking_debug.is_replan();
    replan_reason_ = planning_parking_debug.replan_reason();
    replan_time_ = planning_parking_debug.replan_time();
    replan_num_ = planning_parking_debug.replan_num();
    optimizer_thread_counter_ =
        planning_parking_debug.optimizer_thread_counter();
    replan_by_context_update_counter_ =
        planning_parking_debug.replan_by_context_update_counter();
    replan_by_large_error_counter_ =
        planning_parking_debug.replan_by_large_error_counter();
    parking_type_ = planning_parking_debug.parking_type();
    moves_counter_ = planning_parking_debug.moves_counter();
    remain_distance_ = planning_parking_debug.remain_distance();
    distance_to_leader_obj_ = planning_parking_debug.distance_to_leader_obj();
    state_ = planning_parking_debug.state();
  }

protected:
  std::shared_ptr<std::mutex> vehicle_preiew_polygon_mutex_;
  std::shared_ptr<std::mutex> obstacles_polygon_mutex_;
  std::shared_ptr<std::mutex> path_points_mutex_;
  std::shared_ptr<std::mutex> obstacles_vec_mutex_;
  std::shared_ptr<std::mutex> warm_start_traj_mutex_;
  std::shared_ptr<std::mutex> smoothed_traj_stage1_mutex_;
  std::shared_ptr<std::mutex> smoothed_traj_stage2_mutex_;
  std::shared_ptr<std::mutex> trajectory_array_mutex_;
  //车辆预测多边形
  std::vector<legionclaw::interface::Polygon2D> vehicle_preiew_polygon_;
  //障碍物多边形
  std::vector<legionclaw::interface::Polygon2D> obstacles_polygon_;
  //轨迹点
  std::vector<legionclaw::interface::PathPoint> path_points_;
  //横向误差
  double lat_error_;
  double *lat_error_ptr_ = nullptr;
  //纵向误差
  double lon_error_;
  double *lon_error_ptr_ = nullptr;
  //角度误差(度)
  double yaw_error_;
  double *yaw_error_ptr_ = nullptr;
  std::vector<legionclaw::interface::Point2dList> obstacles_vec_;
  std::vector<legionclaw::interface::TrajectoryPoint> warm_start_traj_;
  std::vector<legionclaw::interface::TrajectoryPoint> smoothed_traj_stage1_;
  std::vector<legionclaw::interface::TrajectoryPoint> smoothed_traj_stage2_;
  legionclaw::interface::Trajectory reference_line_;
  legionclaw::interface::Trajectory *reference_line_ptr_ = nullptr;
  //轨迹簇
  std::vector<legionclaw::interface::Trajectory> trajectory_array_;
  legionclaw::interface::Trajectory optimal_coarse_trajectory_;
  legionclaw::interface::Trajectory *optimal_coarse_trajectory_ptr_ = nullptr;
  legionclaw::interface::Trajectory optimal_smooth_trajectory_;
  legionclaw::interface::Trajectory *optimal_smooth_trajectory_ptr_ = nullptr;
  double hybrid_a_star_map_time_;
  double *hybrid_a_star_map_time_ptr_ = nullptr;
  double hybrid_a_star_heuristic_time_;
  double *hybrid_a_star_heuristic_time_ptr_ = nullptr;
  double hybrid_a_star_rs_time_;
  double *hybrid_a_star_rs_time_ptr_ = nullptr;
  double hybrid_a_star_total_time_;
  double *hybrid_a_star_total_time_ptr_ = nullptr;
  double ias_collision_avoidance_time_;
  double *ias_collision_avoidance_time_ptr_ = nullptr;
  double ias_path_smooth_time_;
  double *ias_path_smooth_time_ptr_ = nullptr;
  double ias_speed_smooth_time_;
  double *ias_speed_smooth_time_ptr_ = nullptr;
  double ias_total_time_;
  double *ias_total_time_ptr_ = nullptr;
  double samping_trajectory_time_;
  double *samping_trajectory_time_ptr_ = nullptr;
  bool is_replan_;
  bool *is_replan_ptr_ = nullptr;
  std::string replan_reason_;
  std::string *replan_reason_ptr_ = nullptr;
  double replan_time_;
  double *replan_time_ptr_ = nullptr;
  int32_t replan_num_;
  int32_t *replan_num_ptr_ = nullptr;
  int32_t optimizer_thread_counter_;
  int32_t *optimizer_thread_counter_ptr_ = nullptr;
  int32_t replan_by_context_update_counter_;
  int32_t *replan_by_context_update_counter_ptr_ = nullptr;
  int32_t replan_by_large_error_counter_;
  int32_t *replan_by_large_error_counter_ptr_ = nullptr;
  legionclaw::common::ParkingType parking_type_;
  legionclaw::common::ParkingType *parking_type_ptr_ = nullptr;
  //揉库次数
  int32_t moves_counter_;
  int32_t *moves_counter_ptr_ = nullptr;
  //剩余距离
  double remain_distance_;
  double *remain_distance_ptr_ = nullptr;
  //距离最近障碍物距离
  double distance_to_leader_obj_;
  double *distance_to_leader_obj_ptr_ = nullptr;
  legionclaw::interface::ParkingStateDisplay::ParkingState state_;
  legionclaw::interface::ParkingStateDisplay::ParkingState *state_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
