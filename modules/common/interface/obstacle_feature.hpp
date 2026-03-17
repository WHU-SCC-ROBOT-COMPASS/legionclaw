/**
 * @file    obstacle_feature.hpp
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
#include "modules/common/interface/obstacle_priority.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/prediction_trajectory_point.hpp"
#include "modules/common/interface/trajectory_in_prediction.hpp"
#include "modules/common/interface/trajectory_point_in_prediction.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ObstacleFeature {
public:
  ObstacleFeature() {
    future_trajectory_points_mutex_ = std::make_shared<std::mutex>();
    short_term_predicted_trajectory_points_mutex_ =
        std::make_shared<std::mutex>();
    predicted_trajectory_mutex_ = std::make_shared<std::mutex>();
    adc_trajectory_point_mutex_ = std::make_shared<std::mutex>();

    obstacle_id_ = 0;
    velocity_heading_ = 0.0;
    speed_ = 0.0;
    acc_ = 0.0;
    theta_ = 0.0;
    length_ = 0.0;
    width_ = 0.0;
    height_ = 0.0;
    tracking_time_ = 0.0;
    timestamp_ = 0.0;
    t_velocity_heading_ = 0.0;
    t_speed_ = 0.0;
    t_acc_ = 0.0;
    is_still_ = false;
    type_ = legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN;
    label_update_time_delta_ = 0.0;
    is_near_junction_ = false;
    clear_future_trajectory_points();
    clear_short_term_predicted_trajectory_points();
    clear_predicted_trajectory();
    clear_adc_trajectory_point();
  }
  ~ObstacleFeature() = default;

  inline void set_obstacle_id(const int32_t &obstacle_id) {
    obstacle_id_ = obstacle_id;
    obstacle_id_ptr_ = &obstacle_id_;
  }

  inline const int32_t &obstacle_id() const { return obstacle_id_; }

  inline int32_t *mutable_obstacle_id() { return &obstacle_id_; }

  inline bool has_obstacle_id() { return (obstacle_id_ptr_ != nullptr); }

  inline void
  set_polygon_point(const legionclaw::interface::Point3D &polygon_point) {
    polygon_point_ = polygon_point;
    polygon_point_ptr_ = &polygon_point_;
  }

  inline const legionclaw::interface::Point3D &polygon_point() const {
    return polygon_point_;
  }

  inline legionclaw::interface::Point3D *mutable_polygon_point() {
    return &polygon_point_;
  }

  inline bool has_polygon_point() { return (polygon_point_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::Point3D &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::Point3D &position() const { return position_; }

  inline legionclaw::interface::Point3D *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void
  set_front_position(const legionclaw::interface::Point3D &front_position) {
    front_position_ = front_position;
    front_position_ptr_ = &front_position_;
  }

  inline const legionclaw::interface::Point3D &front_position() const {
    return front_position_;
  }

  inline legionclaw::interface::Point3D *mutable_front_position() {
    return &front_position_;
  }

  inline bool has_front_position() { return (front_position_ptr_ != nullptr); }

  inline void set_velocity(const legionclaw::interface::Point3D &velocity) {
    velocity_ = velocity;
    velocity_ptr_ = &velocity_;
  }

  inline const legionclaw::interface::Point3D &velocity() const { return velocity_; }

  inline legionclaw::interface::Point3D *mutable_velocity() { return &velocity_; }

  inline bool has_velocity() { return (velocity_ptr_ != nullptr); }

  inline void set_raw_velocity(const legionclaw::interface::Point3D &raw_velocity) {
    raw_velocity_ = raw_velocity;
    raw_velocity_ptr_ = &raw_velocity_;
  }

  inline const legionclaw::interface::Point3D &raw_velocity() const {
    return raw_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_raw_velocity() {
    return &raw_velocity_;
  }

  inline bool has_raw_velocity() { return (raw_velocity_ptr_ != nullptr); }

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

  inline void set_velocity_heading(const double &velocity_heading) {
    velocity_heading_ = velocity_heading;
    velocity_heading_ptr_ = &velocity_heading_;
  }

  inline const double &velocity_heading() const { return velocity_heading_; }

  inline double *mutable_velocity_heading() { return &velocity_heading_; }

  inline bool has_velocity_heading() {
    return (velocity_heading_ptr_ != nullptr);
  }

  inline void set_speed(const double &speed) {
    speed_ = speed;
    speed_ptr_ = &speed_;
  }

  inline const double &speed() const { return speed_; }

  inline double *mutable_speed() { return &speed_; }

  inline bool has_speed() { return (speed_ptr_ != nullptr); }

  inline void set_acc(const double &acc) {
    acc_ = acc;
    acc_ptr_ = &acc_;
  }

  inline const double &acc() const { return acc_; }

  inline double *mutable_acc() { return &acc_; }

  inline bool has_acc() { return (acc_ptr_ != nullptr); }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

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

  inline void set_tracking_time(const double &tracking_time) {
    tracking_time_ = tracking_time;
    tracking_time_ptr_ = &tracking_time_;
  }

  inline const double &tracking_time() const { return tracking_time_; }

  inline double *mutable_tracking_time() { return &tracking_time_; }

  inline bool has_tracking_time() { return (tracking_time_ptr_ != nullptr); }

  inline void set_timestamp(const double &timestamp) {
    timestamp_ = timestamp;
    timestamp_ptr_ = &timestamp_;
  }

  inline const double &timestamp() const { return timestamp_; }

  inline double *mutable_timestamp() { return &timestamp_; }

  inline bool has_timestamp() { return (timestamp_ptr_ != nullptr); }

  inline void set_t_position(const legionclaw::interface::Point3D &t_position) {
    t_position_ = t_position;
    t_position_ptr_ = &t_position_;
  }

  inline const legionclaw::interface::Point3D &t_position() const {
    return t_position_;
  }

  inline legionclaw::interface::Point3D *mutable_t_position() {
    return &t_position_;
  }

  inline bool has_t_position() { return (t_position_ptr_ != nullptr); }

  inline void set_t_velocity(const legionclaw::interface::Point3D &t_velocity) {
    t_velocity_ = t_velocity;
    t_velocity_ptr_ = &t_velocity_;
  }

  inline const legionclaw::interface::Point3D &t_velocity() const {
    return t_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_t_velocity() {
    return &t_velocity_;
  }

  inline bool has_t_velocity() { return (t_velocity_ptr_ != nullptr); }

  inline void set_t_velocity_heading(const double &t_velocity_heading) {
    t_velocity_heading_ = t_velocity_heading;
    t_velocity_heading_ptr_ = &t_velocity_heading_;
  }

  inline const double &t_velocity_heading() const {
    return t_velocity_heading_;
  }

  inline double *mutable_t_velocity_heading() { return &t_velocity_heading_; }

  inline bool has_t_velocity_heading() {
    return (t_velocity_heading_ptr_ != nullptr);
  }

  inline void set_t_speed(const double &t_speed) {
    t_speed_ = t_speed;
    t_speed_ptr_ = &t_speed_;
  }

  inline const double &t_speed() const { return t_speed_; }

  inline double *mutable_t_speed() { return &t_speed_; }

  inline bool has_t_speed() { return (t_speed_ptr_ != nullptr); }

  inline void
  set_t_acceleration(const legionclaw::interface::Point3D &t_acceleration) {
    t_acceleration_ = t_acceleration;
    t_acceleration_ptr_ = &t_acceleration_;
  }

  inline const legionclaw::interface::Point3D &t_acceleration() const {
    return t_acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_t_acceleration() {
    return &t_acceleration_;
  }

  inline bool has_t_acceleration() { return (t_acceleration_ptr_ != nullptr); }

  inline void set_t_acc(const double &t_acc) {
    t_acc_ = t_acc;
    t_acc_ptr_ = &t_acc_;
  }

  inline const double &t_acc() const { return t_acc_; }

  inline double *mutable_t_acc() { return &t_acc_; }

  inline bool has_t_acc() { return (t_acc_ptr_ != nullptr); }

  inline void set_is_still(const bool &is_still) {
    is_still_ = is_still;
    is_still_ptr_ = &is_still_;
  }

  inline const bool &is_still() const { return is_still_; }

  inline bool *mutable_is_still() { return &is_still_; }

  inline bool has_is_still() { return (is_still_ptr_ != nullptr); }

  inline void set_type(const legionclaw::common::ObstacleType &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::common::ObstacleType &type() const { return type_; }

  inline legionclaw::common::ObstacleType *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void
  set_label_update_time_delta(const double &label_update_time_delta) {
    label_update_time_delta_ = label_update_time_delta;
    label_update_time_delta_ptr_ = &label_update_time_delta_;
  }

  inline const double &label_update_time_delta() const {
    return label_update_time_delta_;
  }

  inline double *mutable_label_update_time_delta() {
    return &label_update_time_delta_;
  }

  inline bool has_label_update_time_delta() {
    return (label_update_time_delta_ptr_ != nullptr);
  }

  inline void set_priority(const legionclaw::interface::ObstaclePriority &priority) {
    priority_ = priority;
    priority_ptr_ = &priority_;
  }

  inline const legionclaw::interface::ObstaclePriority &priority() const {
    return priority_;
  }

  inline legionclaw::interface::ObstaclePriority *mutable_priority() {
    return &priority_;
  }

  inline bool has_priority() { return (priority_ptr_ != nullptr); }

  inline void set_is_near_junction(const bool &is_near_junction) {
    is_near_junction_ = is_near_junction;
    is_near_junction_ptr_ = &is_near_junction_;
  }

  inline const bool &is_near_junction() const { return is_near_junction_; }

  inline bool *mutable_is_near_junction() { return &is_near_junction_; }

  inline bool has_is_near_junction() {
    return (is_near_junction_ptr_ != nullptr);
  }

  inline void set_future_trajectory_points(
      std::vector<legionclaw::interface::PredictionTrajectoryPoint>
          *future_trajectory_points) {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points_.assign(future_trajectory_points->begin(),
                                     future_trajectory_points->end());
  }

  inline void set_future_trajectory_points(
      const std::vector<legionclaw::interface::PredictionTrajectoryPoint>
          &future_trajectory_points) {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points_ = future_trajectory_points;
  }

  inline void set_future_trajectory_points(
      const uint32_t index,
      legionclaw::interface::PredictionTrajectoryPoint &future_trajectory_points) {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points_[index] = future_trajectory_points;
  }

  inline void
  add_future_trajectory_points(const legionclaw::interface::PredictionTrajectoryPoint
                                   &future_trajectory_points) {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points_.emplace_back(future_trajectory_points);
  }

  inline const legionclaw::interface::PredictionTrajectoryPoint &
  future_trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    return future_trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::PredictionTrajectoryPoint> *
  mutable_future_trajectory_points() {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    return &future_trajectory_points_;
  }

  inline void future_trajectory_points(
      std::vector<legionclaw::interface::PredictionTrajectoryPoint>
          &future_trajectory_points) const {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points.assign(future_trajectory_points_.begin(),
                                    future_trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::PredictionTrajectoryPoint> &
  future_trajectory_points() const {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    return future_trajectory_points_;
  }

  inline uint32_t future_trajectory_points_size() const {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    return future_trajectory_points_.size();
  }

  inline void clear_future_trajectory_points() {
    std::lock_guard<std::mutex> lock(*future_trajectory_points_mutex_);
    future_trajectory_points_.clear();
    future_trajectory_points_.shrink_to_fit();
  }

  inline bool has_future_trajectory_points() {
    return (future_trajectory_points_size() != 0);
  }

  inline void set_short_term_predicted_trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          *short_term_predicted_trajectory_points) {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points_.assign(
        short_term_predicted_trajectory_points->begin(),
        short_term_predicted_trajectory_points->end());
  }

  inline void set_short_term_predicted_trajectory_points(
      const std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          &short_term_predicted_trajectory_points) {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points_ =
        short_term_predicted_trajectory_points;
  }

  inline void set_short_term_predicted_trajectory_points(
      const uint32_t index, legionclaw::interface::TrajectoryPointInPrediction
                                &short_term_predicted_trajectory_points) {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points_[index] =
        short_term_predicted_trajectory_points;
  }

  inline void add_short_term_predicted_trajectory_points(
      const legionclaw::interface::TrajectoryPointInPrediction
          &short_term_predicted_trajectory_points) {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points_.emplace_back(
        short_term_predicted_trajectory_points);
  }

  inline const legionclaw::interface::TrajectoryPointInPrediction &
  short_term_predicted_trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    return short_term_predicted_trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPointInPrediction> *
  mutable_short_term_predicted_trajectory_points() {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    return &short_term_predicted_trajectory_points_;
  }

  inline void short_term_predicted_trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          &short_term_predicted_trajectory_points) const {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points.assign(
        short_term_predicted_trajectory_points_.begin(),
        short_term_predicted_trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPointInPrediction> &
  short_term_predicted_trajectory_points() const {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    return short_term_predicted_trajectory_points_;
  }

  inline uint32_t short_term_predicted_trajectory_points_size() const {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    return short_term_predicted_trajectory_points_.size();
  }

  inline void clear_short_term_predicted_trajectory_points() {
    std::lock_guard<std::mutex> lock(
        *short_term_predicted_trajectory_points_mutex_);
    short_term_predicted_trajectory_points_.clear();
    short_term_predicted_trajectory_points_.shrink_to_fit();
  }

  inline bool has_short_term_predicted_trajectory_points() {
    return (short_term_predicted_trajectory_points_size() != 0);
  }

  inline void
  set_predicted_trajectory(std::vector<legionclaw::interface::TrajectoryInPrediction>
                               *predicted_trajectory) {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory_.assign(predicted_trajectory->begin(),
                                 predicted_trajectory->end());
  }

  inline void set_predicted_trajectory(
      const std::vector<legionclaw::interface::TrajectoryInPrediction>
          &predicted_trajectory) {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory_ = predicted_trajectory;
  }

  inline void set_predicted_trajectory(
      const uint32_t index,
      legionclaw::interface::TrajectoryInPrediction &predicted_trajectory) {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory_[index] = predicted_trajectory;
  }

  inline void add_predicted_trajectory(
      const legionclaw::interface::TrajectoryInPrediction &predicted_trajectory) {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory_.emplace_back(predicted_trajectory);
  }

  inline const legionclaw::interface::TrajectoryInPrediction &
  predicted_trajectory(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    return predicted_trajectory_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryInPrediction> *
  mutable_predicted_trajectory() {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    return &predicted_trajectory_;
  }

  inline void
  predicted_trajectory(std::vector<legionclaw::interface::TrajectoryInPrediction>
                           &predicted_trajectory) const {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory.assign(predicted_trajectory_.begin(),
                                predicted_trajectory_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryInPrediction> &
  predicted_trajectory() const {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    return predicted_trajectory_;
  }

  inline uint32_t predicted_trajectory_size() const {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    return predicted_trajectory_.size();
  }

  inline void clear_predicted_trajectory() {
    std::lock_guard<std::mutex> lock(*predicted_trajectory_mutex_);
    predicted_trajectory_.clear();
    predicted_trajectory_.shrink_to_fit();
  }

  inline bool has_predicted_trajectory() {
    return (predicted_trajectory_size() != 0);
  }

  inline void set_adc_trajectory_point(
      std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          *adc_trajectory_point) {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point_.assign(adc_trajectory_point->begin(),
                                 adc_trajectory_point->end());
  }

  inline void set_adc_trajectory_point(
      const std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          &adc_trajectory_point) {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point_ = adc_trajectory_point;
  }

  inline void set_adc_trajectory_point(
      const uint32_t index,
      legionclaw::interface::TrajectoryPointInPrediction &adc_trajectory_point) {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point_[index] = adc_trajectory_point;
  }

  inline void
  add_adc_trajectory_point(const legionclaw::interface::TrajectoryPointInPrediction
                               &adc_trajectory_point) {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point_.emplace_back(adc_trajectory_point);
  }

  inline const legionclaw::interface::TrajectoryPointInPrediction &
  adc_trajectory_point(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    return adc_trajectory_point_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPointInPrediction> *
  mutable_adc_trajectory_point() {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    return &adc_trajectory_point_;
  }

  inline void adc_trajectory_point(
      std::vector<legionclaw::interface::TrajectoryPointInPrediction>
          &adc_trajectory_point) const {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point.assign(adc_trajectory_point_.begin(),
                                adc_trajectory_point_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPointInPrediction> &
  adc_trajectory_point() const {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    return adc_trajectory_point_;
  }

  inline uint32_t adc_trajectory_point_size() const {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    return adc_trajectory_point_.size();
  }

  inline void clear_adc_trajectory_point() {
    std::lock_guard<std::mutex> lock(*adc_trajectory_point_mutex_);
    adc_trajectory_point_.clear();
    adc_trajectory_point_.shrink_to_fit();
  }

  inline bool has_adc_trajectory_point() {
    return (adc_trajectory_point_size() != 0);
  }

  void operator=(const ObstacleFeature &obstacle_feature) {
    CopyFrom(obstacle_feature);
  }

  void CopyFrom(const ObstacleFeature &obstacle_feature) {
    obstacle_id_ = obstacle_feature.obstacle_id();
    polygon_point_ = obstacle_feature.polygon_point();
    position_ = obstacle_feature.position();
    front_position_ = obstacle_feature.front_position();
    velocity_ = obstacle_feature.velocity();
    raw_velocity_ = obstacle_feature.raw_velocity();
    acceleration_ = obstacle_feature.acceleration();
    velocity_heading_ = obstacle_feature.velocity_heading();
    speed_ = obstacle_feature.speed();
    acc_ = obstacle_feature.acc();
    theta_ = obstacle_feature.theta();
    length_ = obstacle_feature.length();
    width_ = obstacle_feature.width();
    height_ = obstacle_feature.height();
    tracking_time_ = obstacle_feature.tracking_time();
    timestamp_ = obstacle_feature.timestamp();
    t_position_ = obstacle_feature.t_position();
    t_velocity_ = obstacle_feature.t_velocity();
    t_velocity_heading_ = obstacle_feature.t_velocity_heading();
    t_speed_ = obstacle_feature.t_speed();
    t_acceleration_ = obstacle_feature.t_acceleration();
    t_acc_ = obstacle_feature.t_acc();
    is_still_ = obstacle_feature.is_still();
    type_ = obstacle_feature.type();
    label_update_time_delta_ = obstacle_feature.label_update_time_delta();
    priority_ = obstacle_feature.priority();
    is_near_junction_ = obstacle_feature.is_near_junction();
    future_trajectory_points_ = obstacle_feature.future_trajectory_points();
    short_term_predicted_trajectory_points_ =
        obstacle_feature.short_term_predicted_trajectory_points();
    predicted_trajectory_ = obstacle_feature.predicted_trajectory();
    adc_trajectory_point_ = obstacle_feature.adc_trajectory_point();
  }

protected:
  std::shared_ptr<std::mutex> future_trajectory_points_mutex_;
  std::shared_ptr<std::mutex> short_term_predicted_trajectory_points_mutex_;
  std::shared_ptr<std::mutex> predicted_trajectory_mutex_;
  std::shared_ptr<std::mutex> adc_trajectory_point_mutex_;
  // Obstacle ID
  int32_t obstacle_id_;
  int32_t *obstacle_id_ptr_ = nullptr;
  //障碍物边框点列
  legionclaw::interface::Point3D polygon_point_;
  legionclaw::interface::Point3D *polygon_point_ptr_ = nullptr;
  legionclaw::interface::Point3D position_;
  legionclaw::interface::Point3D *position_ptr_ = nullptr;
  legionclaw::interface::Point3D front_position_;
  legionclaw::interface::Point3D *front_position_ptr_ = nullptr;
  legionclaw::interface::Point3D velocity_;
  legionclaw::interface::Point3D *velocity_ptr_ = nullptr;
  // from perception
  legionclaw::interface::Point3D raw_velocity_;
  legionclaw::interface::Point3D *raw_velocity_ptr_ = nullptr;
  legionclaw::interface::Point3D acceleration_;
  legionclaw::interface::Point3D *acceleration_ptr_ = nullptr;
  double velocity_heading_;
  double *velocity_heading_ptr_ = nullptr;
  double speed_;
  double *speed_ptr_ = nullptr;
  double acc_;
  double *acc_ptr_ = nullptr;
  double theta_;
  double *theta_ptr_ = nullptr;
  double length_;
  double *length_ptr_ = nullptr;
  double width_;
  double *width_ptr_ = nullptr;
  double height_;
  double *height_ptr_ = nullptr;
  double tracking_time_;
  double *tracking_time_ptr_ = nullptr;
  double timestamp_;
  double *timestamp_ptr_ = nullptr;
  legionclaw::interface::Point3D t_position_;
  legionclaw::interface::Point3D *t_position_ptr_ = nullptr;
  //[deprecated = true]
  legionclaw::interface::Point3D t_velocity_;
  legionclaw::interface::Point3D *t_velocity_ptr_ = nullptr;
  //[deprecated = true]
  double t_velocity_heading_;
  double *t_velocity_heading_ptr_ = nullptr;
  //[deprecated = true]
  double t_speed_;
  double *t_speed_ptr_ = nullptr;
  //[deprecated = true]
  legionclaw::interface::Point3D t_acceleration_;
  legionclaw::interface::Point3D *t_acceleration_ptr_ = nullptr;
  //[deprecated = true]
  double t_acc_;
  double *t_acc_ptr_ = nullptr;
  //[deprecated = false]
  bool is_still_;
  bool *is_still_ptr_ = nullptr;
  //感知到的障碍物类型         0-UNKNOWN, 1-UNKNOWN_MOVABLE,
  //2-UNKNOWN_UNMOVABLE, 3-PEDESTRIAN, 4-BICYCLE,     5-VEHICLE
  legionclaw::common::ObstacleType type_;
  legionclaw::common::ObstacleType *type_ptr_ = nullptr;
  double label_update_time_delta_;
  double *label_update_time_delta_ptr_ = nullptr;
  // CAUTION = 1; NORMAL = 2; IGNORE = 3;
  legionclaw::interface::ObstaclePriority priority_;
  legionclaw::interface::ObstaclePriority *priority_ptr_ = nullptr;
  bool is_near_junction_;
  bool *is_near_junction_ptr_ = nullptr;
  // Obstacle ground-truth labels:
  std::vector<legionclaw::interface::PredictionTrajectoryPoint>
      future_trajectory_points_;
  // Obstacle short-term predicted trajectory points
  std::vector<legionclaw::interface::TrajectoryPointInPrediction>
      short_term_predicted_trajectory_points_;
  // Obstacle predicted trajectories
  std::vector<legionclaw::interface::TrajectoryInPrediction> predicted_trajectory_;
  // ADC trajectory at the same frame
  std::vector<legionclaw::interface::TrajectoryPointInPrediction>
      adc_trajectory_point_;
};
} // namespace interface
} // namespace legionclaw
