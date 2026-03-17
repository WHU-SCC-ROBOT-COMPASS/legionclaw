/**
 * @file    hmi_obstacle.hpp
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

#include "modules/common/enum/enum.h"
#include "modules/common/interface/obstacle.hpp"
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HMIObstacle {
public:
  HMIObstacle() {
    points_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    theta_vehicle_ = 0.0;
    theta_abs_ = 0.0;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    type_ = legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN;
    confidence_ = 0.0;
    confidence_type_ =
        legionclaw::interface::Obstacle::ConfidenceType::CONFIDENCE_UNKNOWN;
    sub_type_ = legionclaw::common::ObstacleSubType::ST_UNKNOWN;
    clear_points();
    cipv_flag_ = 0;
    fusion_type_ = 0;
  }
  ~HMIObstacle() = default;

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void
  set_center_pos_vehicle(const legionclaw::interface::Point3D &center_pos_vehicle) {
    center_pos_vehicle_ = center_pos_vehicle;
    center_pos_vehicle_ptr_ = &center_pos_vehicle_;
  }

  inline const legionclaw::interface::Point3D &center_pos_vehicle() const {
    return center_pos_vehicle_;
  }

  inline legionclaw::interface::Point3D *mutable_center_pos_vehicle() {
    return &center_pos_vehicle_;
  }

  inline bool has_center_pos_vehicle() {
    return (center_pos_vehicle_ptr_ != nullptr);
  }

  inline void
  set_center_pos_abs(const legionclaw::interface::Point3D &center_pos_abs) {
    center_pos_abs_ = center_pos_abs;
    center_pos_abs_ptr_ = &center_pos_abs_;
  }

  inline const legionclaw::interface::Point3D &center_pos_abs() const {
    return center_pos_abs_;
  }

  inline legionclaw::interface::Point3D *mutable_center_pos_abs() {
    return &center_pos_abs_;
  }

  inline bool has_center_pos_abs() { return (center_pos_abs_ptr_ != nullptr); }

  inline void set_theta_vehicle(const double &theta_vehicle) {
    theta_vehicle_ = theta_vehicle;
    theta_vehicle_ptr_ = &theta_vehicle_;
  }

  inline const double &theta_vehicle() const { return theta_vehicle_; }

  inline double *mutable_theta_vehicle() { return &theta_vehicle_; }

  inline bool has_theta_vehicle() { return (theta_vehicle_ptr_ != nullptr); }

  inline void set_theta_abs(const double &theta_abs) {
    theta_abs_ = theta_abs;
    theta_abs_ptr_ = &theta_abs_;
  }

  inline const double &theta_abs() const { return theta_abs_; }

  inline double *mutable_theta_abs() { return &theta_abs_; }

  inline bool has_theta_abs() { return (theta_abs_ptr_ != nullptr); }

  inline void set_length(const double &length) {
    length_ = length;
    length_ptr_ = &length_;
  }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline bool has_length() { return (length_ptr_ != nullptr); }

  inline void set_width(const double &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const double &width() const { return width_; }

  inline double *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_height(const double &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const double &height() const { return height_; }

  inline double *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  inline void set_type(const legionclaw::common::ObstacleType &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::common::ObstacleType &type() const { return type_; }

  inline legionclaw::common::ObstacleType *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_confidence_type(
      const legionclaw::interface::Obstacle::ConfidenceType &confidence_type) {
    confidence_type_ = confidence_type;
    confidence_type_ptr_ = &confidence_type_;
  }

  inline const legionclaw::interface::Obstacle::ConfidenceType &
  confidence_type() const {
    return confidence_type_;
  }

  inline legionclaw::interface::Obstacle::ConfidenceType *mutable_confidence_type() {
    return &confidence_type_;
  }

  inline bool has_confidence_type() {
    return (confidence_type_ptr_ != nullptr);
  }

  inline void set_sub_type(const legionclaw::common::ObstacleSubType &sub_type) {
    sub_type_ = sub_type;
    sub_type_ptr_ = &sub_type_;
  }

  inline const legionclaw::common::ObstacleSubType &sub_type() const {
    return sub_type_;
  }

  inline legionclaw::common::ObstacleSubType *mutable_sub_type() {
    return &sub_type_;
  }

  inline bool has_sub_type() { return (sub_type_ptr_ != nullptr); }

  inline void set_points(std::vector<legionclaw::interface::Point3D> *points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.assign(points->begin(), points->end());
  }

  inline void set_points(const std::vector<legionclaw::interface::Point3D> &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_ = points;
  }

  inline void set_points(const uint32_t index,
                         legionclaw::interface::Point3D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_[index] = points;
  }

  inline void add_points(const legionclaw::interface::Point3D &points) {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.emplace_back(points);
  }

  inline const legionclaw::interface::Point3D &points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return &points_;
  }

  inline void points(std::vector<legionclaw::interface::Point3D> &points) const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points.assign(points_.begin(), points_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &points() const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_;
  }

  inline uint32_t points_size() const {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    return points_.size();
  }

  inline void clear_points() {
    std::lock_guard<std::mutex> lock(*points_mutex_);
    points_.clear();
    points_.shrink_to_fit();
  }

  inline bool has_points() { return (points_size() != 0); }

  inline void set_cipv_flag(const uint8_t &cipv_flag) {
    cipv_flag_ = cipv_flag;
    cipv_flag_ptr_ = &cipv_flag_;
  }

  inline const uint8_t &cipv_flag() const { return cipv_flag_; }

  inline uint8_t *mutable_cipv_flag() { return &cipv_flag_; }

  inline bool has_cipv_flag() { return (cipv_flag_ptr_ != nullptr); }

  inline void set_fusion_type(const int32_t &fusion_type) {
    fusion_type_ = fusion_type;
    fusion_type_ptr_ = &fusion_type_;
  }

  inline const int32_t &fusion_type() const { return fusion_type_; }

  inline int32_t *mutable_fusion_type() { return &fusion_type_; }

  inline bool has_fusion_type() { return (fusion_type_ptr_ != nullptr); }

  void operator=(const HMIObstacle &hmi_obstacle) { CopyFrom(hmi_obstacle); }

  void CopyFrom(const HMIObstacle &hmi_obstacle) {
    id_ = hmi_obstacle.id();
    center_pos_vehicle_ = hmi_obstacle.center_pos_vehicle();
    center_pos_abs_ = hmi_obstacle.center_pos_abs();
    theta_vehicle_ = hmi_obstacle.theta_vehicle();
    theta_abs_ = hmi_obstacle.theta_abs();
    length_ = hmi_obstacle.length();
    width_ = hmi_obstacle.width();
    height_ = hmi_obstacle.height();
    type_ = hmi_obstacle.type();
    confidence_ = hmi_obstacle.confidence();
    confidence_type_ = hmi_obstacle.confidence_type();
    sub_type_ = hmi_obstacle.sub_type();
    points_ = hmi_obstacle.points();
    cipv_flag_ = hmi_obstacle.cipv_flag();
    fusion_type_ = hmi_obstacle.fusion_type();
  }

protected:
  std::shared_ptr<std::mutex> points_mutex_;
  //障碍物id
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //车辆坐标系下障碍物的中心位置
  legionclaw::interface::Point3D center_pos_vehicle_;
  legionclaw::interface::Point3D *center_pos_vehicle_ptr_ = nullptr;
  //世界坐标系下障碍物的中心位置
  legionclaw::interface::Point3D center_pos_abs_;
  legionclaw::interface::Point3D *center_pos_abs_ptr_ = nullptr;
  //车辆坐标系下障碍物的朝向
  double theta_vehicle_;
  double *theta_vehicle_ptr_ = nullptr;
  //世界坐标系下障碍物的朝向
  double theta_abs_;
  double *theta_abs_ptr_ = nullptr;
  //障碍物长度
  double length_;
  double *length_ptr_ = nullptr;
  //障碍物宽度
  double width_;
  double *width_ptr_ = nullptr;
  //障碍物高度
  double height_;
  double *height_ptr_ = nullptr;
  //障碍物类别： UNKNOWN = 0; UNKNOWN_MOVABLE = 1; UNKNOWN_UNMOVABLE = 2;
  //PEDESTRIAN = 3; BICYCLE = 4; VEHICLE = 5;
  legionclaw::common::ObstacleType type_;
  legionclaw::common::ObstacleType *type_ptr_ = nullptr;
  //障碍物类别置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
  //障碍物置信度类别： CONFIDENCE_UNKNOWN = 0; CONFIDENCE_CNN = 1;
  //CONFIDENCE_RADAR = 2;
  legionclaw::interface::Obstacle::ConfidenceType confidence_type_;
  legionclaw::interface::Obstacle::ConfidenceType *confidence_type_ptr_ = nullptr;
  //障碍物类别： ST_UNKNOWN = 0; ST_UNKNOWN_MOVABLE = 1; ST_UNKNOWN_UNMOVABLE =
  //2; ST_CAR = 3; ST_VAN = 4; ST_TRUCK = 5; ST_BUS = 6; ST_CYCLIST = 7;
  //ST_MOTORCYCLIST = 8; ST_TRICYCLIST = 9; ST_PEDESTRIAN = 10; ST_TRAFFICCONE =
  //11; ST_PILLAR = 12; ST_SPEED_BUMP = 13;
  legionclaw::common::ObstacleSubType sub_type_;
  legionclaw::common::ObstacleSubType *sub_type_ptr_ = nullptr;
  std::vector<legionclaw::interface::Point3D> points_;
  //障碍物状态标志（0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE）
  uint8_t cipv_flag_;
  uint8_t *cipv_flag_ptr_ = nullptr;
  int32_t fusion_type_;
  int32_t *fusion_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
