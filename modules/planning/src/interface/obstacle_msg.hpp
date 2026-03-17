/**
 * @file    obstacle_msg.hpp
 * @author  zdhy
 * @date    2021-09-09
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <stdint.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "modules/common/enum/enum.h"
// #include "modules/common/interface/vec_2d.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/polygon_2d.hpp"
#include "modules/common/math/polygon2d.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
using namespace std;
using namespace legionclaw::common::math;

namespace legionclaw {
namespace planning {
class ObstacleMsg {
 public:
  ObstacleMsg() {
    relative_position_mutex_ = std::make_shared<std::mutex>();
    position_covariance_mutex_ = std::make_shared<std::mutex>();
    velocity_covariance_mutex_ = std::make_shared<std::mutex>();
    acceleration_covariance_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    obj_lon_index_ = -1;
    theta_ = 0.0;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    has_velocity_ = false;
    clear_relative_position();
    tracking_time_ = 0.0;
    type_ = legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN;
    confidence_ = 0.0;
    timestamp_ = 0.0;
    confidence_type_ = 0;
    lane_position_ = legionclaw::common::LanePosition::LANE_POSITION_UNKNOWN;
    sub_type_ = legionclaw::common::ObstacleSubType::ST_UNKNOWN;
    height_above_ground_ = 0.0;
    clear_position_covariance();
    clear_velocity_covariance();
    clear_acceleration_covariance();
    light_status_ = 0;
  }
  ~ObstacleMsg() = default;

  inline void set_id(const int32_t &id) { id_ = id; }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline void set_position(const legionclaw::interface::Point3D &position) {
    position_ = position;
  }

  inline const legionclaw::interface::Point3D &position() const {
    return position_;
  }

  inline legionclaw::interface::Point3D *mutable_position() { return &position_; }

  inline void set_obj_lon_index(const int &obj_lon_index) {
    obj_lon_index_ = obj_lon_index;
  }

  inline const int &obj_lon_index() const { return obj_lon_index_; }

  inline int *mutable_obj_lon_index() { return &obj_lon_index_; }

  inline void set_theta(const double &theta) { theta_ = theta; }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline void set_length(const double &length) { length_ = length; }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline void set_width(const double &width) { width_ = width; }

  inline const double &width() const { return width_; }

  inline double *mutable_width() { return &width_; }

  inline void set_height(const double &height) { height_ = height; }

  inline const double &height() const { return height_; }

  inline double *mutable_height() { return &height_; }

  inline void set_velocity(const legionclaw::interface::Point3D &velocity) {
    velocity_ = velocity;
  }

  inline const legionclaw::interface::Point3D &velocity() const {
    return velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_velocity() { return &velocity_; }

  inline void set_has_velocity(const bool &has_velocity) {
    has_velocity_ = has_velocity;
  }

  inline const bool &has_velocity() const { return has_velocity_; }

  inline bool *mutable_has_velocity() { return &has_velocity_; }

  inline void set_acceleration(const legionclaw::interface::Point3D &acceleration) {
    acceleration_ = acceleration;
  }

  inline const legionclaw::interface::Point3D &acceleration() const {
    return acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_acceleration() {
    return &acceleration_;
  }

  inline void set_relative_position(std::vector<Vec2d> *relative_position) {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position_.assign(relative_position->begin(),
                              relative_position->end());
  }

  inline void set_relative_position(
      const std::vector<Vec2d> &relative_position) {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position_ = relative_position;
  }

  inline void set_relative_position(const uint32_t index,
                                    Vec2d &relative_position) {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position_[index] = relative_position;
  }

  inline void add_relative_position(const Vec2d &relative_position) {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position_.emplace_back(relative_position);
  }

  inline const Vec2d &relative_position(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    return relative_position_[index];
  }

  inline std::vector<Vec2d> *mutable_relative_position() {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    return &relative_position_;
  }

  inline void relative_position(std::vector<Vec2d> &relative_position) const {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position.assign(relative_position_.begin(),
                             relative_position_.end());
  }

  inline const std::vector<Vec2d> &relative_position() const {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    return relative_position_;
  }

  inline uint32_t relative_position_size() const {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    return relative_position_.size();
  }

  inline void clear_relative_position() {
    std::lock_guard<std::mutex> lock(*relative_position_mutex_);
    relative_position_.clear();
  }

  inline void set_polygon(const Polygon2d &polygon) { polygon_ = polygon; }

  inline const Polygon2d &polygon() const { return polygon_; }

  inline Polygon2d *mutable_polygon() { return &polygon_; }

  inline void set_tracking_time(const double &tracking_time) {
    tracking_time_ = tracking_time;
  }

  inline const double &tracking_time() const { return tracking_time_; }

  inline double *mutable_tracking_time() { return &tracking_time_; }

  inline void set_type(const legionclaw::common::ObstacleType &type) {
    type_ = type;
  }

  inline const legionclaw::common::ObstacleType &type() const { return type_; }

  inline legionclaw::common::ObstacleType *mutable_type() { return &type_; }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline void set_timestamp(const double &timestamp) { timestamp_ = timestamp; }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline void set_confidence_type(const uint8_t &confidence_type) {
    confidence_type_ = confidence_type;
  }

  inline const uint8_t &confidence_type() const { return confidence_type_; }

  inline uint8_t *mutable_confidence_type() { return &confidence_type_; }

  inline void set_lane_position(const legionclaw::common::LanePosition &lane_position) {
    lane_position_ = lane_position;
  }

  inline const legionclaw::common::LanePosition &lane_position() const {
    return lane_position_;
  }

  inline legionclaw::common::LanePosition *mutable_lane_position() {
    return &lane_position_;
  }

  inline void set_sub_type(const legionclaw::common::ObstacleSubType &sub_type) {
    sub_type_ = sub_type;
  }

  inline const legionclaw::common::ObstacleSubType &sub_type() const {
    return sub_type_;
  }

  inline legionclaw::common::ObstacleSubType *mutable_sub_type() { return &sub_type_; }

  inline void set_height_above_ground(const double &height_above_ground) {
    height_above_ground_ = height_above_ground;
  }

  inline const double &height_above_ground() const {
    return height_above_ground_;
  }

  inline double *mutable_height_above_ground() { return &height_above_ground_; }

  inline void set_position_covariance(
      std::vector<double> *position_covariance) {
    std::lock_guard<std::mutex> lock(*position_covariance_mutex_);
    position_covariance_.assign(position_covariance->begin(),
                                position_covariance->end());
  }

  inline void set_position_covariance(
      const std::vector<double> &position_covariance) {
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

  inline const void position_covariance(
      std::vector<double> &position_covariance) const {
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
  }

  inline void set_velocity_covariance(
      const std::vector<double> *velocity_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_covariance_mutex_);
    velocity_covariance_.assign(velocity_covariance->begin(),
                                velocity_covariance->end());
  }

  inline void set_velocity_covariance(
      const std::vector<double> &velocity_covariance) {
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

  inline void velocity_covariance(
      std::vector<double> &velocity_covariance) const {
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
  }

  inline void set_acceleration_covariance(
      std::vector<double> *acceleration_covariance) {
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

  inline void add_acceleration_covariance(
      const double &acceleration_covariance) {
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

  inline void acceleration_covariance(
      std::vector<double> &acceleration_covariance) const {
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
  }

  inline void set_light_status(const uint8_t &light_status) {
    light_status_ = light_status;
  }

  inline const uint8_t &light_status() const { return light_status_; }

  inline uint8_t *mutable_light_status() { return &light_status_; }

 protected:
  std::shared_ptr<std::mutex> relative_position_mutex_;
  std::shared_ptr<std::mutex> position_covariance_mutex_;
  std::shared_ptr<std::mutex> velocity_covariance_mutex_;
  std::shared_ptr<std::mutex> acceleration_covariance_mutex_;
  int32_t id_;
  legionclaw::interface::Point3D position_;
  int obj_lon_index_;
  double theta_;
  double length_;
  double width_;
  double height_;
  legionclaw::interface::Point3D velocity_;
  bool has_velocity_;
  legionclaw::interface::Point3D acceleration_;
  std::vector<Vec2d> relative_position_;
  Polygon2d polygon_;
  double tracking_time_;
  //障碍物类型： 0-Unknown 1-Unknown_movable 2-Unknown_unmovable 3-Pedestrian
  // 4-Bicycle   5-Vehicle
  legionclaw::common::ObstacleType type_;
  double confidence_;
  double timestamp_;
  //置信度类型                  0-CONFIDENCE_UNKNOWN, 1-CONFIDENCE_CN,
  // 2-CONFIDENCE_RAD
  uint8_t confidence_type_;
  legionclaw::common::LanePosition lane_position_;
  legionclaw::common::ObstacleSubType sub_type_;
  double height_above_ground_;
  std::vector<double> position_covariance_;
  std::vector<double> velocity_covariance_;
  std::vector<double> acceleration_covariance_;
  uint8_t light_status_;
};
}  // namespace planning
}  // namespace legionclaw
