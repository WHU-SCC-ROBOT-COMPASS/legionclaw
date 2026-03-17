/**
 * @file    perception_obstacle.hpp
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
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class PerceptionObstacle {
public:
  PerceptionObstacle() {
    polygon_point_mutex_ = std::make_shared<std::mutex>();
    bounding_box_mutex_ = std::make_shared<std::mutex>();
    position_covariance_mutex_ = std::make_shared<std::mutex>();
    velocity_covariance_mutex_ = std::make_shared<std::mutex>();
    acceleration_covariance_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    theta_ = 0.0;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    clear_polygon_point();
    tracking_time_ = 0.0;
    type_ = legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN;
    lane_position_ = legionclaw::common::LanePosition::NEXT_LEFT_LANE;
    confidence_ = 0.0;
    timestamp_ = 0.0;
    confidence_type_ = 0;
    clear_bounding_box();
    sub_type_ = legionclaw::common::ObstacleSubType::ST_UNKNOWN;
    height_above_ground_ = 0.0;
    clear_position_covariance();
    clear_velocity_covariance();
    clear_acceleration_covariance();
    light_status_ = 0;
  }
  ~PerceptionObstacle() = default;

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::Point3D &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::Point3D &position() const { return position_; }

  inline legionclaw::interface::Point3D *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

  inline void set_velocity(const legionclaw::interface::Point3D &velocity) {
    velocity_ = velocity;
    velocity_ptr_ = &velocity_;
  }

  inline const legionclaw::interface::Point3D &velocity() const { return velocity_; }

  inline legionclaw::interface::Point3D *mutable_velocity() { return &velocity_; }

  inline bool has_velocity() { return (velocity_ptr_ != nullptr); }

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

  inline void
  set_polygon_point(std::vector<legionclaw::interface::Point3D> *polygon_point) {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point_.assign(polygon_point->begin(), polygon_point->end());
  }

  inline void set_polygon_point(
      const std::vector<legionclaw::interface::Point3D> &polygon_point) {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point_ = polygon_point;
  }

  inline void set_polygon_point(const uint32_t index,
                                legionclaw::interface::Point3D &polygon_point) {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point_[index] = polygon_point;
  }

  inline void
  add_polygon_point(const legionclaw::interface::Point3D &polygon_point) {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point_.emplace_back(polygon_point);
  }

  inline const legionclaw::interface::Point3D &polygon_point(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    return polygon_point_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_polygon_point() {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    return &polygon_point_;
  }

  inline void
  polygon_point(std::vector<legionclaw::interface::Point3D> &polygon_point) const {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point.assign(polygon_point_.begin(), polygon_point_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &polygon_point() const {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    return polygon_point_;
  }

  inline uint32_t polygon_point_size() const {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    return polygon_point_.size();
  }

  inline void clear_polygon_point() {
    std::lock_guard<std::mutex> lock(*polygon_point_mutex_);
    polygon_point_.clear();
    polygon_point_.shrink_to_fit();
  }

  inline bool has_polygon_point() { return (polygon_point_size() != 0); }

  inline void set_tracking_time(const double &tracking_time) {
    tracking_time_ = tracking_time;
    tracking_time_ptr_ = &tracking_time_;
  }

  inline const double &tracking_time() const { return tracking_time_; }

  inline double *mutable_tracking_time() { return &tracking_time_; }

  inline bool has_tracking_time() { return (tracking_time_ptr_ != nullptr); }

  inline void set_type(const legionclaw::common::ObstacleType &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::common::ObstacleType &type() const { return type_; }

  inline legionclaw::common::ObstacleType *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void
  set_lane_position(const legionclaw::common::LanePosition &lane_position) {
    lane_position_ = lane_position;
    lane_position_ptr_ = &lane_position_;
  }

  inline const legionclaw::common::LanePosition &lane_position() const {
    return lane_position_;
  }

  inline legionclaw::common::LanePosition *mutable_lane_position() {
    return &lane_position_;
  }

  inline bool has_lane_position() { return (lane_position_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  inline void set_timestamp(const double &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_confidence_type(const uint8_t &confidence_type) {
    confidence_type_ = confidence_type;
    confidence_type_ptr_ = &confidence_type_;
  }

  inline const uint8_t &confidence_type() const { return confidence_type_; }

  inline uint8_t *mutable_confidence_type() { return &confidence_type_; }

  inline bool has_confidence_type() {
    return (confidence_type_ptr_ != nullptr);
  }

  inline void set_drops(const legionclaw::interface::Point3D &drops) {
    drops_ = drops;
    drops_ptr_ = &drops_;
  }

  inline const legionclaw::interface::Point3D &drops() const { return drops_; }

  inline legionclaw::interface::Point3D *mutable_drops() { return &drops_; }

  inline bool has_drops() { return (drops_ptr_ != nullptr); }

  inline void set_acceleration(const legionclaw::interface::Point3D &acceleration) {
    acceleration_ = acceleration;
    acceleration_ptr_ = &acceleration_;
  }

  inline const legionclaw::interface::Point3D &acceleration() const {
    return acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_acceleration() {
    return &acceleration_;
  }

  inline bool has_acceleration() { return (acceleration_ptr_ != nullptr); }

  inline void set_anchor_point(const legionclaw::interface::Point3D &anchor_point) {
    anchor_point_ = anchor_point;
    anchor_point_ptr_ = &anchor_point_;
  }

  inline const legionclaw::interface::Point3D &anchor_point() const {
    return anchor_point_;
  }

  inline legionclaw::interface::Point3D *mutable_anchor_point() {
    return &anchor_point_;
  }

  inline bool has_anchor_point() { return (anchor_point_ptr_ != nullptr); }

  inline void
  set_bounding_box(std::vector<legionclaw::interface::Point3D> *bounding_box) {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box_.assign(bounding_box->begin(), bounding_box->end());
  }

  inline void
  set_bounding_box(const std::vector<legionclaw::interface::Point3D> &bounding_box) {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box_ = bounding_box;
  }

  inline void set_bounding_box(const uint32_t index,
                               legionclaw::interface::Point3D &bounding_box) {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box_[index] = bounding_box;
  }

  inline void add_bounding_box(const legionclaw::interface::Point3D &bounding_box) {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box_.emplace_back(bounding_box);
  }

  inline const legionclaw::interface::Point3D &bounding_box(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    return bounding_box_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_bounding_box() {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    return &bounding_box_;
  }

  inline void
  bounding_box(std::vector<legionclaw::interface::Point3D> &bounding_box) const {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box.assign(bounding_box_.begin(), bounding_box_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &bounding_box() const {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    return bounding_box_;
  }

  inline uint32_t bounding_box_size() const {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    return bounding_box_.size();
  }

  inline void clear_bounding_box() {
    std::lock_guard<std::mutex> lock(*bounding_box_mutex_);
    bounding_box_.clear();
    bounding_box_.shrink_to_fit();
  }

  inline bool has_bounding_box() { return (bounding_box_size() != 0); }

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

  inline void set_height_above_ground(const double &height_above_ground) {
    height_above_ground_ = height_above_ground;
    height_above_ground_ptr_ = &height_above_ground_;
  }

  inline const double &height_above_ground() const {
    return height_above_ground_;
  }

  inline double *mutable_height_above_ground() { return &height_above_ground_; }

  inline bool has_height_above_ground() {
    return (height_above_ground_ptr_ != nullptr);
  }

  inline void
  set_position_covariance(std::vector<double> *position_covariance) {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_.assign(position_covariance->begin(),
                                position_covariance->end());
  }

  inline void
  set_position_covariance(const std::vector<double> &position_covariance) {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_ = position_covariance;
  }

  inline void set_position_covariance(const uint32_t index,
                                      double &position_covariance) {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_[index] = position_covariance;
  }

  inline void add_position_covariance(const double &position_covariance) {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_.emplace_back(position_covariance);
  }

  inline const double &position_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    return position_covariance_[index];
  }

  inline std::vector<double> *mutable_position_covariance() {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    return &position_covariance_;
  }

  inline void
  position_covariance(std::vector<double> &position_covariance) const {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance.assign(position_covariance_.begin(),
                               position_covariance_.end());
  }

  inline const std::vector<double> &position_covariance() const {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    return position_covariance_;
  }

  inline uint32_t position_covariance_size() const {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    return position_covariance_.size();
  }

  inline void clear_position_covariance() {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_.clear();
    position_covariance_.shrink_to_fit();
  }

  inline bool has_position_covariance() {
    return (position_covariance_size() != 0);
  }

  inline void
  set_velocity_covariance(std::vector<double> *velocity_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_.assign(velocity_covariance->begin(),
                                velocity_covariance->end());
  }

  inline void
  set_velocity_covariance(const std::vector<double> &velocity_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_ = velocity_covariance;
  }

  inline void set_velocity_covariance(const uint32_t index,
                                      double &velocity_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_[index] = velocity_covariance;
  }

  inline void add_velocity_covariance(const double &velocity_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_.emplace_back(velocity_covariance);
  }

  inline const double &velocity_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    return velocity_covariance_[index];
  }

  inline std::vector<double> *mutable_velocity_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    return &velocity_covariance_;
  }

  inline void
  velocity_covariance(std::vector<double> &velocity_covariance) const {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance.assign(velocity_covariance_.begin(),
                               velocity_covariance_.end());
  }

  inline const std::vector<double> &velocity_covariance() const {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    return velocity_covariance_;
  }

  inline uint32_t velocity_covariance_size() const {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    return velocity_covariance_.size();
  }

  inline void clear_velocity_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_.clear();
    velocity_covariance_.shrink_to_fit();
  }

  inline bool has_velocity_covariance() {
    return (velocity_covariance_size() != 0);
  }

  inline void
  set_acceleration_covariance(std::vector<double> *acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance_.assign(acceleration_covariance->begin(),
                                    acceleration_covariance->end());
  }

  inline void set_acceleration_covariance(
      const std::vector<double> &acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance_ = acceleration_covariance;
  }

  inline void set_acceleration_covariance(const uint32_t index,
                                          double &acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance_[index] = acceleration_covariance;
  }

  inline void
  add_acceleration_covariance(const double &acceleration_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance_.emplace_back(acceleration_covariance);
  }

  inline const double &acceleration_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    return acceleration_covariance_[index];
  }

  inline std::vector<double> *mutable_acceleration_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    return &acceleration_covariance_;
  }

  inline void
  acceleration_covariance(std::vector<double> &acceleration_covariance) const {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance.assign(acceleration_covariance_.begin(),
                                   acceleration_covariance_.end());
  }

  inline const std::vector<double> &acceleration_covariance() const {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    return acceleration_covariance_;
  }

  inline uint32_t acceleration_covariance_size() const {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    return acceleration_covariance_.size();
  }

  inline void clear_acceleration_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_covariance_mutex_);
    acceleration_covariance_.clear();
    acceleration_covariance_.shrink_to_fit();
  }

  inline bool has_acceleration_covariance() {
    return (acceleration_covariance_size() != 0);
  }

  inline void set_light_status(const uint8_t &light_status) {
    light_status_ = light_status;
    light_status_ptr_ = &light_status_;
  }

  inline const uint8_t &light_status() const { return light_status_; }

  inline uint8_t *mutable_light_status() { return &light_status_; }

  inline bool has_light_status() { return (light_status_ptr_ != nullptr); }

  void operator=(const PerceptionObstacle &perception_obstacle) {
    CopyFrom(perception_obstacle);
  }

  void CopyFrom(const PerceptionObstacle &perception_obstacle) {
    id_ = perception_obstacle.id();
    position_ = perception_obstacle.position();
    theta_ = perception_obstacle.theta();
    velocity_ = perception_obstacle.velocity();
    length_ = perception_obstacle.length();
    width_ = perception_obstacle.width();
    height_ = perception_obstacle.height();
    polygon_point_ = perception_obstacle.polygon_point();
    tracking_time_ = perception_obstacle.tracking_time();
    type_ = perception_obstacle.type();
    lane_position_ = perception_obstacle.lane_position();
    confidence_ = perception_obstacle.confidence();
    timestamp_ = perception_obstacle.timestamp();
    confidence_type_ = perception_obstacle.confidence_type();
    drops_ = perception_obstacle.drops();
    acceleration_ = perception_obstacle.acceleration();
    anchor_point_ = perception_obstacle.anchor_point();
    bounding_box_ = perception_obstacle.bounding_box();
    sub_type_ = perception_obstacle.sub_type();
    height_above_ground_ = perception_obstacle.height_above_ground();
    position_covariance_ = perception_obstacle.position_covariance();
    velocity_covariance_ = perception_obstacle.velocity_covariance();
    acceleration_covariance_ = perception_obstacle.acceleration_covariance();
    light_status_ = perception_obstacle.light_status();
  }

protected:
  std::shared_ptr<std::mutex> polygon_point_mutex_;
  std::shared_ptr<std::mutex> bounding_box_mutex_;
  std::shared_ptr<std::mutex> position_covariance_mutex_;
  std::shared_ptr<std::mutex> velocity_covariance_mutex_;
  std::shared_ptr<std::mutex> acceleration_covariance_mutex_;
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  legionclaw::interface::Point3D position_;
  legionclaw::interface::Point3D *position_ptr_ = nullptr;
  double theta_;
  double *theta_ptr_ = nullptr;
  legionclaw::interface::Point3D velocity_;
  legionclaw::interface::Point3D *velocity_ptr_ = nullptr;
  double length_;
  double *length_ptr_ = nullptr;
  double width_;
  double *width_ptr_ = nullptr;
  double height_;
  double *height_ptr_ = nullptr;
  std::vector<legionclaw::interface::Point3D> polygon_point_;
  double tracking_time_;
  double *tracking_time_ptr_ = nullptr;
  //障碍物类型： 0-Unknown 1-Unknown_movable 2-Unknown_unmovable 3-Pedestrian
  //4-Bicycle   5-Vehicle
  legionclaw::common::ObstacleType type_;
  legionclaw::common::ObstacleType *type_ptr_ = nullptr;
  //车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE
  //2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN
  legionclaw::common::LanePosition lane_position_;
  legionclaw::common::LanePosition *lane_position_ptr_ = nullptr;
  double confidence_;
  double *confidence_ptr_ = nullptr;
  double timestamp_;
  double *timestamp_ptr_ = nullptr;
  //置信度类型                  0-CONFIDENCE_UNKNOWN, 1-CONFIDENCE_CN,
  //2-CONFIDENCE_RAD
  uint8_t confidence_type_;
  uint8_t *confidence_type_ptr_ = nullptr;
  legionclaw::interface::Point3D drops_;
  legionclaw::interface::Point3D *drops_ptr_ = nullptr;
  legionclaw::interface::Point3D acceleration_;
  legionclaw::interface::Point3D *acceleration_ptr_ = nullptr;
  legionclaw::interface::Point3D anchor_point_;
  legionclaw::interface::Point3D *anchor_point_ptr_ = nullptr;
  std::vector<legionclaw::interface::Point3D> bounding_box_;
  legionclaw::common::ObstacleSubType sub_type_;
  legionclaw::common::ObstacleSubType *sub_type_ptr_ = nullptr;
  double height_above_ground_;
  double *height_above_ground_ptr_ = nullptr;
  std::vector<double> position_covariance_;
  std::vector<double> velocity_covariance_;
  std::vector<double> acceleration_covariance_;
  uint8_t light_status_;
  uint8_t *light_status_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
