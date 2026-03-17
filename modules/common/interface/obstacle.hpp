/**
 * @file    obstacle.hpp
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
#include "modules/common/interface/bbox_2d.hpp"
#include "modules/common/interface/image_key_point.hpp"
#include "modules/common/interface/point_2d.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/sensor_calibrator.hpp"
#include "modules/common/interface/time.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Obstacle {
public:
  Obstacle() {
    image_key_points_mutex_ = std::make_shared<std::mutex>();
    polygon_point_abs_mutex_ = std::make_shared<std::mutex>();
    polygon_point_vehicle_mutex_ = std::make_shared<std::mutex>();
    drops_mutex_ = std::make_shared<std::mutex>();
    position_abs_covariance_mutex_ = std::make_shared<std::mutex>();
    velocity_abs_covariance_mutex_ = std::make_shared<std::mutex>();
    acceleration_abs_covariance_mutex_ = std::make_shared<std::mutex>();
    position_vehicle_covariance_mutex_ = std::make_shared<std::mutex>();
    velocity_vehicle_covariance_mutex_ = std::make_shared<std::mutex>();
    acceleration_vehicle_covariance_mutex_ = std::make_shared<std::mutex>();

    id_ = 0;
    existence_prob_ = 0.0;
    theta_vehicle_ = 0.0;
    theta_abs_ = 0.0;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    clear_image_key_points();
    clear_polygon_point_abs();
    clear_polygon_point_vehicle();
    tracking_time_ = 0.0;
    type_ = legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN;
    confidence_ = 0.0;
    confidence_type_ =
        legionclaw::interface::Obstacle::ConfidenceType::CONFIDENCE_UNKNOWN;
    clear_drops();
    sub_type_ = legionclaw::common::ObstacleSubType::ST_UNKNOWN;
    height_above_ground_ = 0.0;
    clear_position_abs_covariance();
    clear_velocity_abs_covariance();
    clear_acceleration_abs_covariance();
    theta_abs_covariance_ = 0.0;
    clear_position_vehicle_covariance();
    clear_velocity_vehicle_covariance();
    clear_acceleration_vehicle_covariance();
    theta_vehicle_covariance_ = 0.0;
    cipv_flag_ = 0;
    lane_position_ = legionclaw::common::LanePosition::NEXT_LEFT_LANE;
    pihp_percentage_ = 0.0;
    blinker_flag_ = legionclaw::common::BlinkerFlag::BLINKER_FLAG_OFF;
    fusion_type_ = legionclaw::interface::Obstacle::FusionType::CAMERA;
  }
  ~Obstacle() = default;

  enum ConfidenceType {
    CONFIDENCE_UNKNOWN = 0,
    CONFIDENCE_CNN = 1,
    CONFIDENCE_RADAR = 2,
  };

  enum FusionType {
    CAMERA = 0,
    RADAR = 1,
    LIDAR = 2,
    ULTRASONIC = 3,
    FUSED = 4,
    FUSION_TYPE_UNKNOWN = 5,
  };

  inline void set_timestamp(const legionclaw::interface::Time &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const legionclaw::interface::Time &timestamp() const { return timestamp_; }

  inline legionclaw::interface::Time *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_existence_prob(const double &existence_prob) {
    existence_prob_ = existence_prob;
    existence_prob_ptr_ = &existence_prob_;
  }

  inline const double &existence_prob() const { return existence_prob_; }

  inline double *mutable_existence_prob() { return &existence_prob_; }

  inline bool has_existence_prob() { return (existence_prob_ptr_ != nullptr); }

  inline void set_create_time(const legionclaw::interface::Time &create_time) {
    create_time_ = create_time;
    create_time_ptr_ = &create_time_;
  }

  inline const legionclaw::interface::Time &create_time() const {
    return create_time_;
  }

  inline legionclaw::interface::Time *mutable_create_time() { return &create_time_; }

  inline bool has_create_time() { return (create_time_ptr_ != nullptr); }

  inline void
  set_last_updated_time(const legionclaw::interface::Time &last_updated_time) {
    last_updated_time_ = last_updated_time;
    last_updated_time_ptr_ = &last_updated_time_;
  }

  inline const legionclaw::interface::Time &last_updated_time() const {
    return last_updated_time_;
  }

  inline legionclaw::interface::Time *mutable_last_updated_time() {
    return &last_updated_time_;
  }

  inline bool has_last_updated_time() {
    return (last_updated_time_ptr_ != nullptr);
  }

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

  inline void
  set_velocity_vehicle(const legionclaw::interface::Point3D &velocity_vehicle) {
    velocity_vehicle_ = velocity_vehicle;
    velocity_vehicle_ptr_ = &velocity_vehicle_;
  }

  inline const legionclaw::interface::Point3D &velocity_vehicle() const {
    return velocity_vehicle_;
  }

  inline legionclaw::interface::Point3D *mutable_velocity_vehicle() {
    return &velocity_vehicle_;
  }

  inline bool has_velocity_vehicle() {
    return (velocity_vehicle_ptr_ != nullptr);
  }

  inline void set_velocity_abs(const legionclaw::interface::Point3D &velocity_abs) {
    velocity_abs_ = velocity_abs;
    velocity_abs_ptr_ = &velocity_abs_;
  }

  inline const legionclaw::interface::Point3D &velocity_abs() const {
    return velocity_abs_;
  }

  inline legionclaw::interface::Point3D *mutable_velocity_abs() {
    return &velocity_abs_;
  }

  inline bool has_velocity_abs() { return (velocity_abs_ptr_ != nullptr); }

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

  inline void set_image_key_points(
      std::vector<legionclaw::interface::ImageKeyPoint> *image_key_points) {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points_.assign(image_key_points->begin(),
                             image_key_points->end());
  }

  inline void set_image_key_points(
      const std::vector<legionclaw::interface::ImageKeyPoint> &image_key_points) {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points_ = image_key_points;
  }

  inline void
  set_image_key_points(const uint32_t index,
                       legionclaw::interface::ImageKeyPoint &image_key_points) {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points_[index] = image_key_points;
  }

  inline void add_image_key_points(
      const legionclaw::interface::ImageKeyPoint &image_key_points) {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points_.emplace_back(image_key_points);
  }

  inline const legionclaw::interface::ImageKeyPoint &
  image_key_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    return image_key_points_[index];
  }

  inline std::vector<legionclaw::interface::ImageKeyPoint> *
  mutable_image_key_points() {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    return &image_key_points_;
  }

  inline void image_key_points(
      std::vector<legionclaw::interface::ImageKeyPoint> &image_key_points) const {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points.assign(image_key_points_.begin(), image_key_points_.end());
  }

  inline const std::vector<legionclaw::interface::ImageKeyPoint> &
  image_key_points() const {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    return image_key_points_;
  }

  inline uint32_t image_key_points_size() const {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    return image_key_points_.size();
  }

  inline void clear_image_key_points() {
    std::lock_guard<std::mutex> lock(*image_key_points_mutex_);
    image_key_points_.clear();
    image_key_points_.shrink_to_fit();
  }

  inline bool has_image_key_points() { return (image_key_points_size() != 0); }

  inline void set_polygon_point_abs(
      std::vector<legionclaw::interface::Point3D> *polygon_point_abs) {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs_.assign(polygon_point_abs->begin(),
                              polygon_point_abs->end());
  }

  inline void set_polygon_point_abs(
      const std::vector<legionclaw::interface::Point3D> &polygon_point_abs) {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs_ = polygon_point_abs;
  }

  inline void
  set_polygon_point_abs(const uint32_t index,
                        legionclaw::interface::Point3D &polygon_point_abs) {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs_[index] = polygon_point_abs;
  }

  inline void
  add_polygon_point_abs(const legionclaw::interface::Point3D &polygon_point_abs) {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs_.emplace_back(polygon_point_abs);
  }

  inline const legionclaw::interface::Point3D &
  polygon_point_abs(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    return polygon_point_abs_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_polygon_point_abs() {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    return &polygon_point_abs_;
  }

  inline void polygon_point_abs(
      std::vector<legionclaw::interface::Point3D> &polygon_point_abs) const {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs.assign(polygon_point_abs_.begin(),
                             polygon_point_abs_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &
  polygon_point_abs() const {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    return polygon_point_abs_;
  }

  inline uint32_t polygon_point_abs_size() const {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    return polygon_point_abs_.size();
  }

  inline void clear_polygon_point_abs() {
    std::lock_guard<std::mutex> lock(*polygon_point_abs_mutex_);
    polygon_point_abs_.clear();
    polygon_point_abs_.shrink_to_fit();
  }

  inline bool has_polygon_point_abs() {
    return (polygon_point_abs_size() != 0);
  }

  inline void set_polygon_point_vehicle(
      std::vector<legionclaw::interface::Point3D> *polygon_point_vehicle) {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle_.assign(polygon_point_vehicle->begin(),
                                  polygon_point_vehicle->end());
  }

  inline void set_polygon_point_vehicle(
      const std::vector<legionclaw::interface::Point3D> &polygon_point_vehicle) {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle_ = polygon_point_vehicle;
  }

  inline void
  set_polygon_point_vehicle(const uint32_t index,
                            legionclaw::interface::Point3D &polygon_point_vehicle) {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle_[index] = polygon_point_vehicle;
  }

  inline void add_polygon_point_vehicle(
      const legionclaw::interface::Point3D &polygon_point_vehicle) {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle_.emplace_back(polygon_point_vehicle);
  }

  inline const legionclaw::interface::Point3D &
  polygon_point_vehicle(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    return polygon_point_vehicle_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *
  mutable_polygon_point_vehicle() {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    return &polygon_point_vehicle_;
  }

  inline void polygon_point_vehicle(
      std::vector<legionclaw::interface::Point3D> &polygon_point_vehicle) const {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle.assign(polygon_point_vehicle_.begin(),
                                 polygon_point_vehicle_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &
  polygon_point_vehicle() const {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    return polygon_point_vehicle_;
  }

  inline uint32_t polygon_point_vehicle_size() const {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    return polygon_point_vehicle_.size();
  }

  inline void clear_polygon_point_vehicle() {
    std::lock_guard<std::mutex> lock(*polygon_point_vehicle_mutex_);
    polygon_point_vehicle_.clear();
    polygon_point_vehicle_.shrink_to_fit();
  }

  inline bool has_polygon_point_vehicle() {
    return (polygon_point_vehicle_size() != 0);
  }

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

  inline void set_drops(std::vector<legionclaw::interface::Point3D> *drops) {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops_.assign(drops->begin(), drops->end());
  }

  inline void set_drops(const std::vector<legionclaw::interface::Point3D> &drops) {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops_ = drops;
  }

  inline void set_drops(const uint32_t index,
                        legionclaw::interface::Point3D &drops) {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops_[index] = drops;
  }

  inline void add_drops(const legionclaw::interface::Point3D &drops) {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops_.emplace_back(drops);
  }

  inline const legionclaw::interface::Point3D &drops(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    return drops_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_drops() {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    return &drops_;
  }

  inline void drops(std::vector<legionclaw::interface::Point3D> &drops) const {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops.assign(drops_.begin(), drops_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &drops() const {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    return drops_;
  }

  inline uint32_t drops_size() const {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    return drops_.size();
  }

  inline void clear_drops() {
    std::lock_guard<std::mutex> lock(*drops_mutex_);
    drops_.clear();
    drops_.shrink_to_fit();
  }

  inline bool has_drops() { return (drops_size() != 0); }

  inline void set_acceleration_vehicle(
      const legionclaw::interface::Point3D &acceleration_vehicle) {
    acceleration_vehicle_ = acceleration_vehicle;
    acceleration_vehicle_ptr_ = &acceleration_vehicle_;
  }

  inline const legionclaw::interface::Point3D &acceleration_vehicle() const {
    return acceleration_vehicle_;
  }

  inline legionclaw::interface::Point3D *mutable_acceleration_vehicle() {
    return &acceleration_vehicle_;
  }

  inline bool has_acceleration_vehicle() {
    return (acceleration_vehicle_ptr_ != nullptr);
  }

  inline void
  set_acceleration_abs(const legionclaw::interface::Point3D &acceleration_abs) {
    acceleration_abs_ = acceleration_abs;
    acceleration_abs_ptr_ = &acceleration_abs_;
  }

  inline const legionclaw::interface::Point3D &acceleration_abs() const {
    return acceleration_abs_;
  }

  inline legionclaw::interface::Point3D *mutable_acceleration_abs() {
    return &acceleration_abs_;
  }

  inline bool has_acceleration_abs() {
    return (acceleration_abs_ptr_ != nullptr);
  }

  inline void
  set_anchor_point_image(const legionclaw::interface::Point2D &anchor_point_image) {
    anchor_point_image_ = anchor_point_image;
    anchor_point_image_ptr_ = &anchor_point_image_;
  }

  inline const legionclaw::interface::Point2D &anchor_point_image() const {
    return anchor_point_image_;
  }

  inline legionclaw::interface::Point2D *mutable_anchor_point_image() {
    return &anchor_point_image_;
  }

  inline bool has_anchor_point_image() {
    return (anchor_point_image_ptr_ != nullptr);
  }

  inline void set_anchor_point_vehicle(
      const legionclaw::interface::Point3D &anchor_point_vehicle) {
    anchor_point_vehicle_ = anchor_point_vehicle;
    anchor_point_vehicle_ptr_ = &anchor_point_vehicle_;
  }

  inline const legionclaw::interface::Point3D &anchor_point_vehicle() const {
    return anchor_point_vehicle_;
  }

  inline legionclaw::interface::Point3D *mutable_anchor_point_vehicle() {
    return &anchor_point_vehicle_;
  }

  inline bool has_anchor_point_vehicle() {
    return (anchor_point_vehicle_ptr_ != nullptr);
  }

  inline void
  set_anchor_point_abs(const legionclaw::interface::Point3D &anchor_point_abs) {
    anchor_point_abs_ = anchor_point_abs;
    anchor_point_abs_ptr_ = &anchor_point_abs_;
  }

  inline const legionclaw::interface::Point3D &anchor_point_abs() const {
    return anchor_point_abs_;
  }

  inline legionclaw::interface::Point3D *mutable_anchor_point_abs() {
    return &anchor_point_abs_;
  }

  inline bool has_anchor_point_abs() {
    return (anchor_point_abs_ptr_ != nullptr);
  }

  inline void set_bbox2d(const legionclaw::interface::BBox2D &bbox2d) {
    bbox2d_ = bbox2d;
    bbox2d_ptr_ = &bbox2d_;
  }

  inline const legionclaw::interface::BBox2D &bbox2d() const { return bbox2d_; }

  inline legionclaw::interface::BBox2D *mutable_bbox2d() { return &bbox2d_; }

  inline bool has_bbox2d() { return (bbox2d_ptr_ != nullptr); }

  inline void set_bbox2d_rear(const legionclaw::interface::BBox2D &bbox2d_rear) {
    bbox2d_rear_ = bbox2d_rear;
    bbox2d_rear_ptr_ = &bbox2d_rear_;
  }

  inline const legionclaw::interface::BBox2D &bbox2d_rear() const {
    return bbox2d_rear_;
  }

  inline legionclaw::interface::BBox2D *mutable_bbox2d_rear() {
    return &bbox2d_rear_;
  }

  inline bool has_bbox2d_rear() { return (bbox2d_rear_ptr_ != nullptr); }

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
  set_position_abs_covariance(std::vector<double> *position_abs_covariance) {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance_.assign(position_abs_covariance->begin(),
                                    position_abs_covariance->end());
  }

  inline void set_position_abs_covariance(
      const std::vector<double> &position_abs_covariance) {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance_ = position_abs_covariance;
  }

  inline void set_position_abs_covariance(const uint32_t index,
                                          double &position_abs_covariance) {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance_[index] = position_abs_covariance;
  }

  inline void
  add_position_abs_covariance(const double &position_abs_covariance) {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance_.emplace_back(position_abs_covariance);
  }

  inline const double &position_abs_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    return position_abs_covariance_[index];
  }

  inline std::vector<double> *mutable_position_abs_covariance() {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    return &position_abs_covariance_;
  }

  inline void
  position_abs_covariance(std::vector<double> &position_abs_covariance) const {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance.assign(position_abs_covariance_.begin(),
                                   position_abs_covariance_.end());
  }

  inline const std::vector<double> &position_abs_covariance() const {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    return position_abs_covariance_;
  }

  inline uint32_t position_abs_covariance_size() const {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    return position_abs_covariance_.size();
  }

  inline void clear_position_abs_covariance() {
    std::lock_guard<std::mutex> lock(*position_abs_covariance_mutex_);
    position_abs_covariance_.clear();
    position_abs_covariance_.shrink_to_fit();
  }

  inline bool has_position_abs_covariance() {
    return (position_abs_covariance_size() != 0);
  }

  inline void
  set_velocity_abs_covariance(std::vector<double> *velocity_abs_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance_.assign(velocity_abs_covariance->begin(),
                                    velocity_abs_covariance->end());
  }

  inline void set_velocity_abs_covariance(
      const std::vector<double> &velocity_abs_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance_ = velocity_abs_covariance;
  }

  inline void set_velocity_abs_covariance(const uint32_t index,
                                          double &velocity_abs_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance_[index] = velocity_abs_covariance;
  }

  inline void
  add_velocity_abs_covariance(const double &velocity_abs_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance_.emplace_back(velocity_abs_covariance);
  }

  inline const double &velocity_abs_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    return velocity_abs_covariance_[index];
  }

  inline std::vector<double> *mutable_velocity_abs_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    return &velocity_abs_covariance_;
  }

  inline void
  velocity_abs_covariance(std::vector<double> &velocity_abs_covariance) const {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance.assign(velocity_abs_covariance_.begin(),
                                   velocity_abs_covariance_.end());
  }

  inline const std::vector<double> &velocity_abs_covariance() const {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    return velocity_abs_covariance_;
  }

  inline uint32_t velocity_abs_covariance_size() const {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    return velocity_abs_covariance_.size();
  }

  inline void clear_velocity_abs_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_abs_covariance_mutex_);
    velocity_abs_covariance_.clear();
    velocity_abs_covariance_.shrink_to_fit();
  }

  inline bool has_velocity_abs_covariance() {
    return (velocity_abs_covariance_size() != 0);
  }

  inline void set_acceleration_abs_covariance(
      std::vector<double> *acceleration_abs_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance_.assign(acceleration_abs_covariance->begin(),
                                        acceleration_abs_covariance->end());
  }

  inline void set_acceleration_abs_covariance(
      const std::vector<double> &acceleration_abs_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance_ = acceleration_abs_covariance;
  }

  inline void
  set_acceleration_abs_covariance(const uint32_t index,
                                  double &acceleration_abs_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance_[index] = acceleration_abs_covariance;
  }

  inline void
  add_acceleration_abs_covariance(const double &acceleration_abs_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance_.emplace_back(acceleration_abs_covariance);
  }

  inline const double &acceleration_abs_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    return acceleration_abs_covariance_[index];
  }

  inline std::vector<double> *mutable_acceleration_abs_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    return &acceleration_abs_covariance_;
  }

  inline void acceleration_abs_covariance(
      std::vector<double> &acceleration_abs_covariance) const {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance.assign(acceleration_abs_covariance_.begin(),
                                       acceleration_abs_covariance_.end());
  }

  inline const std::vector<double> &acceleration_abs_covariance() const {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    return acceleration_abs_covariance_;
  }

  inline uint32_t acceleration_abs_covariance_size() const {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    return acceleration_abs_covariance_.size();
  }

  inline void clear_acceleration_abs_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_abs_covariance_mutex_);
    acceleration_abs_covariance_.clear();
    acceleration_abs_covariance_.shrink_to_fit();
  }

  inline bool has_acceleration_abs_covariance() {
    return (acceleration_abs_covariance_size() != 0);
  }

  inline void set_theta_abs_covariance(const double &theta_abs_covariance) {
    theta_abs_covariance_ = theta_abs_covariance;
    theta_abs_covariance_ptr_ = &theta_abs_covariance_;
  }

  inline const double &theta_abs_covariance() const {
    return theta_abs_covariance_;
  }

  inline double *mutable_theta_abs_covariance() {
    return &theta_abs_covariance_;
  }

  inline bool has_theta_abs_covariance() {
    return (theta_abs_covariance_ptr_ != nullptr);
  }

  inline void set_position_vehicle_covariance(
      std::vector<double> *position_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance_.assign(position_vehicle_covariance->begin(),
                                        position_vehicle_covariance->end());
  }

  inline void set_position_vehicle_covariance(
      const std::vector<double> &position_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance_ = position_vehicle_covariance;
  }

  inline void
  set_position_vehicle_covariance(const uint32_t index,
                                  double &position_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance_[index] = position_vehicle_covariance;
  }

  inline void
  add_position_vehicle_covariance(const double &position_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance_.emplace_back(position_vehicle_covariance);
  }

  inline const double &position_vehicle_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    return position_vehicle_covariance_[index];
  }

  inline std::vector<double> *mutable_position_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    return &position_vehicle_covariance_;
  }

  inline void position_vehicle_covariance(
      std::vector<double> &position_vehicle_covariance) const {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance.assign(position_vehicle_covariance_.begin(),
                                       position_vehicle_covariance_.end());
  }

  inline const std::vector<double> &position_vehicle_covariance() const {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    return position_vehicle_covariance_;
  }

  inline uint32_t position_vehicle_covariance_size() const {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    return position_vehicle_covariance_.size();
  }

  inline void clear_position_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*position_vehicle_covariance_mutex_);
    position_vehicle_covariance_.clear();
    position_vehicle_covariance_.shrink_to_fit();
  }

  inline bool has_position_vehicle_covariance() {
    return (position_vehicle_covariance_size() != 0);
  }

  inline void set_velocity_vehicle_covariance(
      std::vector<double> *velocity_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance_.assign(velocity_vehicle_covariance->begin(),
                                        velocity_vehicle_covariance->end());
  }

  inline void set_velocity_vehicle_covariance(
      const std::vector<double> &velocity_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance_ = velocity_vehicle_covariance;
  }

  inline void
  set_velocity_vehicle_covariance(const uint32_t index,
                                  double &velocity_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance_[index] = velocity_vehicle_covariance;
  }

  inline void
  add_velocity_vehicle_covariance(const double &velocity_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance_.emplace_back(velocity_vehicle_covariance);
  }

  inline const double &velocity_vehicle_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    return velocity_vehicle_covariance_[index];
  }

  inline std::vector<double> *mutable_velocity_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    return &velocity_vehicle_covariance_;
  }

  inline void velocity_vehicle_covariance(
      std::vector<double> &velocity_vehicle_covariance) const {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance.assign(velocity_vehicle_covariance_.begin(),
                                       velocity_vehicle_covariance_.end());
  }

  inline const std::vector<double> &velocity_vehicle_covariance() const {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    return velocity_vehicle_covariance_;
  }

  inline uint32_t velocity_vehicle_covariance_size() const {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    return velocity_vehicle_covariance_.size();
  }

  inline void clear_velocity_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*velocity_vehicle_covariance_mutex_);
    velocity_vehicle_covariance_.clear();
    velocity_vehicle_covariance_.shrink_to_fit();
  }

  inline bool has_velocity_vehicle_covariance() {
    return (velocity_vehicle_covariance_size() != 0);
  }

  inline void set_acceleration_vehicle_covariance(
      std::vector<double> *acceleration_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance_.assign(
        acceleration_vehicle_covariance->begin(),
        acceleration_vehicle_covariance->end());
  }

  inline void set_acceleration_vehicle_covariance(
      const std::vector<double> &acceleration_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance_ = acceleration_vehicle_covariance;
  }

  inline void
  set_acceleration_vehicle_covariance(const uint32_t index,
                                      double &acceleration_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance_[index] = acceleration_vehicle_covariance;
  }

  inline void add_acceleration_vehicle_covariance(
      const double &acceleration_vehicle_covariance) {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance_.emplace_back(
        acceleration_vehicle_covariance);
  }

  inline const double &acceleration_vehicle_covariance(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    return acceleration_vehicle_covariance_[index];
  }

  inline std::vector<double> *mutable_acceleration_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    return &acceleration_vehicle_covariance_;
  }

  inline void acceleration_vehicle_covariance(
      std::vector<double> &acceleration_vehicle_covariance) const {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance.assign(
        acceleration_vehicle_covariance_.begin(),
        acceleration_vehicle_covariance_.end());
  }

  inline const std::vector<double> &acceleration_vehicle_covariance() const {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    return acceleration_vehicle_covariance_;
  }

  inline uint32_t acceleration_vehicle_covariance_size() const {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    return acceleration_vehicle_covariance_.size();
  }

  inline void clear_acceleration_vehicle_covariance() {
    std::lock_guard<std::mutex> lock(*acceleration_vehicle_covariance_mutex_);
    acceleration_vehicle_covariance_.clear();
    acceleration_vehicle_covariance_.shrink_to_fit();
  }

  inline bool has_acceleration_vehicle_covariance() {
    return (acceleration_vehicle_covariance_size() != 0);
  }

  inline void
  set_theta_vehicle_covariance(const double &theta_vehicle_covariance) {
    theta_vehicle_covariance_ = theta_vehicle_covariance;
    theta_vehicle_covariance_ptr_ = &theta_vehicle_covariance_;
  }

  inline const double &theta_vehicle_covariance() const {
    return theta_vehicle_covariance_;
  }

  inline double *mutable_theta_vehicle_covariance() {
    return &theta_vehicle_covariance_;
  }

  inline bool has_theta_vehicle_covariance() {
    return (theta_vehicle_covariance_ptr_ != nullptr);
  }

  inline void set_sensor_calibrator(
      const legionclaw::interface::SensorCalibrator &sensor_calibrator) {
    sensor_calibrator_ = sensor_calibrator;
    sensor_calibrator_ptr_ = &sensor_calibrator_;
  }

  inline const legionclaw::interface::SensorCalibrator &sensor_calibrator() const {
    return sensor_calibrator_;
  }

  inline legionclaw::interface::SensorCalibrator *mutable_sensor_calibrator() {
    return &sensor_calibrator_;
  }

  inline bool has_sensor_calibrator() {
    return (sensor_calibrator_ptr_ != nullptr);
  }

  inline void set_cipv_flag(const uint8_t &cipv_flag) {
    cipv_flag_ = cipv_flag;
    cipv_flag_ptr_ = &cipv_flag_;
  }

  inline const uint8_t &cipv_flag() const { return cipv_flag_; }

  inline uint8_t *mutable_cipv_flag() { return &cipv_flag_; }

  inline bool has_cipv_flag() { return (cipv_flag_ptr_ != nullptr); }

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

  inline void set_pihp_percentage(const double &pihp_percentage) {
    pihp_percentage_ = pihp_percentage;
    pihp_percentage_ptr_ = &pihp_percentage_;
  }

  inline const double &pihp_percentage() const { return pihp_percentage_; }

  inline double *mutable_pihp_percentage() { return &pihp_percentage_; }

  inline bool has_pihp_percentage() {
    return (pihp_percentage_ptr_ != nullptr);
  }

  inline void set_blinker_flag(const legionclaw::common::BlinkerFlag &blinker_flag) {
    blinker_flag_ = blinker_flag;
    blinker_flag_ptr_ = &blinker_flag_;
  }

  inline const legionclaw::common::BlinkerFlag &blinker_flag() const {
    return blinker_flag_;
  }

  inline legionclaw::common::BlinkerFlag *mutable_blinker_flag() {
    return &blinker_flag_;
  }

  inline bool has_blinker_flag() { return (blinker_flag_ptr_ != nullptr); }

  inline void
  set_fusion_type(const legionclaw::interface::Obstacle::FusionType &fusion_type) {
    fusion_type_ = fusion_type;
    fusion_type_ptr_ = &fusion_type_;
  }

  inline const legionclaw::interface::Obstacle::FusionType &fusion_type() const {
    return fusion_type_;
  }

  inline legionclaw::interface::Obstacle::FusionType *mutable_fusion_type() {
    return &fusion_type_;
  }

  inline bool has_fusion_type() { return (fusion_type_ptr_ != nullptr); }

  void operator=(const Obstacle &obstacle) { CopyFrom(obstacle); }

  void CopyFrom(const Obstacle &obstacle) {
    timestamp_ = obstacle.timestamp();
    id_ = obstacle.id();
    existence_prob_ = obstacle.existence_prob();
    create_time_ = obstacle.create_time();
    last_updated_time_ = obstacle.last_updated_time();
    center_pos_vehicle_ = obstacle.center_pos_vehicle();
    center_pos_abs_ = obstacle.center_pos_abs();
    theta_vehicle_ = obstacle.theta_vehicle();
    theta_abs_ = obstacle.theta_abs();
    velocity_vehicle_ = obstacle.velocity_vehicle();
    velocity_abs_ = obstacle.velocity_abs();
    length_ = obstacle.length();
    width_ = obstacle.width();
    height_ = obstacle.height();
    image_key_points_ = obstacle.image_key_points();
    polygon_point_abs_ = obstacle.polygon_point_abs();
    polygon_point_vehicle_ = obstacle.polygon_point_vehicle();
    tracking_time_ = obstacle.tracking_time();
    type_ = obstacle.type();
    confidence_ = obstacle.confidence();
    confidence_type_ = obstacle.confidence_type();
    drops_ = obstacle.drops();
    acceleration_vehicle_ = obstacle.acceleration_vehicle();
    acceleration_abs_ = obstacle.acceleration_abs();
    anchor_point_image_ = obstacle.anchor_point_image();
    anchor_point_vehicle_ = obstacle.anchor_point_vehicle();
    anchor_point_abs_ = obstacle.anchor_point_abs();
    bbox2d_ = obstacle.bbox2d();
    bbox2d_rear_ = obstacle.bbox2d_rear();
    sub_type_ = obstacle.sub_type();
    height_above_ground_ = obstacle.height_above_ground();
    position_abs_covariance_ = obstacle.position_abs_covariance();
    velocity_abs_covariance_ = obstacle.velocity_abs_covariance();
    acceleration_abs_covariance_ = obstacle.acceleration_abs_covariance();
    theta_abs_covariance_ = obstacle.theta_abs_covariance();
    position_vehicle_covariance_ = obstacle.position_vehicle_covariance();
    velocity_vehicle_covariance_ = obstacle.velocity_vehicle_covariance();
    acceleration_vehicle_covariance_ =
        obstacle.acceleration_vehicle_covariance();
    theta_vehicle_covariance_ = obstacle.theta_vehicle_covariance();
    sensor_calibrator_ = obstacle.sensor_calibrator();
    cipv_flag_ = obstacle.cipv_flag();
    lane_position_ = obstacle.lane_position();
    pihp_percentage_ = obstacle.pihp_percentage();
    blinker_flag_ = obstacle.blinker_flag();
    fusion_type_ = obstacle.fusion_type();
  }

protected:
  std::shared_ptr<std::mutex> image_key_points_mutex_;
  std::shared_ptr<std::mutex> polygon_point_abs_mutex_;
  std::shared_ptr<std::mutex> polygon_point_vehicle_mutex_;
  std::shared_ptr<std::mutex> drops_mutex_;
  std::shared_ptr<std::mutex> position_abs_covariance_mutex_;
  std::shared_ptr<std::mutex> velocity_abs_covariance_mutex_;
  std::shared_ptr<std::mutex> acceleration_abs_covariance_mutex_;
  std::shared_ptr<std::mutex> position_vehicle_covariance_mutex_;
  std::shared_ptr<std::mutex> velocity_vehicle_covariance_mutex_;
  std::shared_ptr<std::mutex> acceleration_vehicle_covariance_mutex_;
  //时间戳
  legionclaw::interface::Time timestamp_;
  legionclaw::interface::Time *timestamp_ptr_ = nullptr;
  //障碍物id
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //障碍物存在的概率
  double existence_prob_;
  double *existence_prob_ptr_ = nullptr;
  //障碍物被识别的时间戳
  legionclaw::interface::Time create_time_;
  legionclaw::interface::Time *create_time_ptr_ = nullptr;
  //障碍物上一次更新的时间
  legionclaw::interface::Time last_updated_time_;
  legionclaw::interface::Time *last_updated_time_ptr_ = nullptr;
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
  //车辆坐标系下障碍物的速度
  legionclaw::interface::Point3D velocity_vehicle_;
  legionclaw::interface::Point3D *velocity_vehicle_ptr_ = nullptr;
  //世界坐标系下障碍物的速度
  legionclaw::interface::Point3D velocity_abs_;
  legionclaw::interface::Point3D *velocity_abs_ptr_ = nullptr;
  //障碍物长度
  double length_;
  double *length_ptr_ = nullptr;
  //障碍物宽度
  double width_;
  double *width_ptr_ = nullptr;
  //障碍物高度
  double height_;
  double *height_ptr_ = nullptr;
  //图像坐标系下障碍物多边形
  std::vector<legionclaw::interface::ImageKeyPoint> image_key_points_;
  //世界坐标系下障碍物多边形
  std::vector<legionclaw::interface::Point3D> polygon_point_abs_;
  //车辆坐标系下障碍物多边形
  std::vector<legionclaw::interface::Point3D> polygon_point_vehicle_;
  //障碍物被追踪的时间
  double tracking_time_;
  double *tracking_time_ptr_ = nullptr;
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
  //障碍物轨迹点
  std::vector<legionclaw::interface::Point3D> drops_;
  //车辆坐标系下障碍物的加速度
  legionclaw::interface::Point3D acceleration_vehicle_;
  legionclaw::interface::Point3D *acceleration_vehicle_ptr_ = nullptr;
  //世界坐标系下障碍物的加速度
  legionclaw::interface::Point3D acceleration_abs_;
  legionclaw::interface::Point3D *acceleration_abs_ptr_ = nullptr;
  //障碍物尾框中心点(图像坐标系)
  legionclaw::interface::Point2D anchor_point_image_;
  legionclaw::interface::Point2D *anchor_point_image_ptr_ = nullptr;
  //障碍物尾框中心点(车辆坐标系)
  legionclaw::interface::Point3D anchor_point_vehicle_;
  legionclaw::interface::Point3D *anchor_point_vehicle_ptr_ = nullptr;
  //障碍物尾框中心点(世界坐标系)
  legionclaw::interface::Point3D anchor_point_abs_;
  legionclaw::interface::Point3D *anchor_point_abs_ptr_ = nullptr;
  //障碍物图像框
  legionclaw::interface::BBox2D bbox2d_;
  legionclaw::interface::BBox2D *bbox2d_ptr_ = nullptr;
  //障碍物图像尾框
  legionclaw::interface::BBox2D bbox2d_rear_;
  legionclaw::interface::BBox2D *bbox2d_rear_ptr_ = nullptr;
  //障碍物类别： ST_UNKNOWN = 0; ST_UNKNOWN_MOVABLE = 1; ST_UNKNOWN_UNMOVABLE =
  //2; ST_CAR = 3; ST_VAN = 4; ST_TRUCK = 5; ST_BUS = 6; ST_CYCLIST = 7;
  //ST_MOTORCYCLIST = 8; ST_TRICYCLIST = 9; ST_PEDESTRIAN = 10; ST_TRAFFICCONE =
  //11; ST_PILLAR = 12; ST_SPEED_BUMP = 13;
  legionclaw::common::ObstacleSubType sub_type_;
  legionclaw::common::ObstacleSubType *sub_type_ptr_ = nullptr;
  //障碍物近地点到地面的高度
  double height_above_ground_;
  double *height_above_ground_ptr_ = nullptr;
  //世界坐标系下障碍物中心位置的协方差矩阵
  std::vector<double> position_abs_covariance_;
  //世界坐标系下障碍物速度的协方差矩阵
  std::vector<double> velocity_abs_covariance_;
  //世界坐标系下障碍物加速度的协方差矩阵
  std::vector<double> acceleration_abs_covariance_;
  //世界坐标系下障碍物朝向的协方差矩阵
  double theta_abs_covariance_;
  double *theta_abs_covariance_ptr_ = nullptr;
  //车辆坐标系下障碍物中心位置的协方差矩阵
  std::vector<double> position_vehicle_covariance_;
  //车辆坐标系下障碍物速度的协方差矩阵
  std::vector<double> velocity_vehicle_covariance_;
  //车辆坐标系下障碍物加速度的协方差矩阵
  std::vector<double> acceleration_vehicle_covariance_;
  //车辆坐标系下障碍物朝向的协方差矩阵
  double theta_vehicle_covariance_;
  double *theta_vehicle_covariance_ptr_ = nullptr;
  //传感器标定参数
  legionclaw::interface::SensorCalibrator sensor_calibrator_;
  legionclaw::interface::SensorCalibrator *sensor_calibrator_ptr_ = nullptr;
  //障碍物状态标志（0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE）
  uint8_t cipv_flag_;
  uint8_t *cipv_flag_ptr_ = nullptr;
  //车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE
  //2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN
  legionclaw::common::LanePosition lane_position_;
  legionclaw::common::LanePosition *lane_position_ptr_ = nullptr;
  //临车道车辆压线比例
  double pihp_percentage_;
  double *pihp_percentage_ptr_ = nullptr;
  //障碍物车辆信号灯状态： 0-OFF 1-LEFT_TURN_VISIBLE 2-LEFT_TURN_ON
  //3-RIGHT_TURN_VISIBLE 4-RIGHT_TURN_ON 5-BRAKE_VISIBLE 6-BRAKE_ON 7-UNKNOWN
  legionclaw::common::BlinkerFlag blinker_flag_;
  legionclaw::common::BlinkerFlag *blinker_flag_ptr_ = nullptr;
  //融合障碍物类型 0-CAMERA 1-RADAR 2-LIDAR 3-ULTRASONIC 4-FUSED 5-UNKNOWN
  legionclaw::interface::Obstacle::FusionType fusion_type_;
  legionclaw::interface::Obstacle::FusionType *fusion_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
