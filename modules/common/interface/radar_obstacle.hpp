/**
 * @file    radar_obstacle.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/point_2d.hpp"
#include "modules/common/interface/status.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class RadarObstacle {
public:
  RadarObstacle() {
    id_ = 0;
    life_time_ = 0;
    rcs_ = 0.0;
    snr_ = 0.0;
    moving_status_ = legionclaw::interface::RadarObstacle::MovingStatus::STATIONARY;
    width_ = 0.0;
    length_ = 0.0;
    height_ = 0.0;
    theta_ = 0.0;
    orientation_ = 0.0;
    orient_rms_ = 0.0;
    yaw_ = 0.0;
    yaw_rms_ = 0.0;
    count_ = 0;
    moving_frames_count_ = 0;
    underpass_probability_ = 0.0;
    overpass_probability_ = 0.0;
    exist_probability_ = 0;
    mov_property_ = 0;
    track_state_ = 0;
    track_type_ = 0;
  }
  ~RadarObstacle() = default;

  enum MovingStatus {
    STATIONARY = 0,
    NEARING = 1,
    AWAYING = 2,
    NONE = 3,
  };

  inline void set_id(const int32_t &id) {
    id_ = id;
    id_ptr_ = &id_;
  }

  inline const int32_t &id() const { return id_; }

  inline int32_t *mutable_id() { return &id_; }

  inline bool has_id() { return (id_ptr_ != nullptr); }

  inline void set_life_time(const uint8_t &life_time) {
    life_time_ = life_time;
    life_time_ptr_ = &life_time_;
  }

  inline const uint8_t &life_time() const { return life_time_; }

  inline uint8_t *mutable_life_time() { return &life_time_; }

  inline bool has_life_time() { return (life_time_ptr_ != nullptr); }

  inline void
  set_relative_position(const legionclaw::interface::Point2D &relative_position) {
    relative_position_ = relative_position;
    relative_position_ptr_ = &relative_position_;
  }

  inline const legionclaw::interface::Point2D &relative_position() const {
    return relative_position_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_position() {
    return &relative_position_;
  }

  inline bool has_relative_position() {
    return (relative_position_ptr_ != nullptr);
  }

  inline void set_relative_position_rms(
      const legionclaw::interface::Point2D &relative_position_rms) {
    relative_position_rms_ = relative_position_rms;
    relative_position_rms_ptr_ = &relative_position_rms_;
  }

  inline const legionclaw::interface::Point2D &relative_position_rms() const {
    return relative_position_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_position_rms() {
    return &relative_position_rms_;
  }

  inline bool has_relative_position_rms() {
    return (relative_position_rms_ptr_ != nullptr);
  }

  inline void
  set_relative_velocity(const legionclaw::interface::Point2D &relative_velocity) {
    relative_velocity_ = relative_velocity;
    relative_velocity_ptr_ = &relative_velocity_;
  }

  inline const legionclaw::interface::Point2D &relative_velocity() const {
    return relative_velocity_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_velocity() {
    return &relative_velocity_;
  }

  inline bool has_relative_velocity() {
    return (relative_velocity_ptr_ != nullptr);
  }

  inline void set_relative_velocity_rms(
      const legionclaw::interface::Point2D &relative_velocity_rms) {
    relative_velocity_rms_ = relative_velocity_rms;
    relative_velocity_rms_ptr_ = &relative_velocity_rms_;
  }

  inline const legionclaw::interface::Point2D &relative_velocity_rms() const {
    return relative_velocity_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_velocity_rms() {
    return &relative_velocity_rms_;
  }

  inline bool has_relative_velocity_rms() {
    return (relative_velocity_rms_ptr_ != nullptr);
  }

  inline void set_relative_acceleration(
      const legionclaw::interface::Point2D &relative_acceleration) {
    relative_acceleration_ = relative_acceleration;
    relative_acceleration_ptr_ = &relative_acceleration_;
  }

  inline const legionclaw::interface::Point2D &relative_acceleration() const {
    return relative_acceleration_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_acceleration() {
    return &relative_acceleration_;
  }

  inline bool has_relative_acceleration() {
    return (relative_acceleration_ptr_ != nullptr);
  }

  inline void set_relative_acceleration_rms(
      const legionclaw::interface::Point2D &relative_acceleration_rms) {
    relative_acceleration_rms_ = relative_acceleration_rms;
    relative_acceleration_rms_ptr_ = &relative_acceleration_rms_;
  }

  inline const legionclaw::interface::Point2D &relative_acceleration_rms() const {
    return relative_acceleration_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_relative_acceleration_rms() {
    return &relative_acceleration_rms_;
  }

  inline bool has_relative_acceleration_rms() {
    return (relative_acceleration_rms_ptr_ != nullptr);
  }

  inline void set_rcs(const double &rcs) {
    rcs_ = rcs;
    rcs_ptr_ = &rcs_;
  }

  inline const double &rcs() const { return rcs_; }

  inline double *mutable_rcs() { return &rcs_; }

  inline bool has_rcs() { return (rcs_ptr_ != nullptr); }

  inline void set_snr(const double &snr) {
    snr_ = snr;
    snr_ptr_ = &snr_;
  }

  inline const double &snr() const { return snr_; }

  inline double *mutable_snr() { return &snr_; }

  inline bool has_snr() { return (snr_ptr_ != nullptr); }

  inline void set_moving_status(
      const legionclaw::interface::RadarObstacle::MovingStatus &moving_status) {
    moving_status_ = moving_status;
    moving_status_ptr_ = &moving_status_;
  }

  inline const legionclaw::interface::RadarObstacle::MovingStatus &
  moving_status() const {
    return moving_status_;
  }

  inline legionclaw::interface::RadarObstacle::MovingStatus *
  mutable_moving_status() {
    return &moving_status_;
  }

  inline bool has_moving_status() { return (moving_status_ptr_ != nullptr); }

  inline void set_width(const double &width) {
    width_ = width;
    width_ptr_ = &width_;
  }

  inline const double &width() const { return width_; }

  inline double *mutable_width() { return &width_; }

  inline bool has_width() { return (width_ptr_ != nullptr); }

  inline void set_length(const double &length) {
    length_ = length;
    length_ptr_ = &length_;
  }

  inline const double &length() const { return length_; }

  inline double *mutable_length() { return &length_; }

  inline bool has_length() { return (length_ptr_ != nullptr); }

  inline void set_height(const double &height) {
    height_ = height;
    height_ptr_ = &height_;
  }

  inline const double &height() const { return height_; }

  inline double *mutable_height() { return &height_; }

  inline bool has_height() { return (height_ptr_ != nullptr); }

  inline void set_theta(const double &theta) {
    theta_ = theta;
    theta_ptr_ = &theta_;
  }

  inline const double &theta() const { return theta_; }

  inline double *mutable_theta() { return &theta_; }

  inline bool has_theta() { return (theta_ptr_ != nullptr); }

  inline void
  set_absolute_position(const legionclaw::interface::Point2D &absolute_position) {
    absolute_position_ = absolute_position;
    absolute_position_ptr_ = &absolute_position_;
  }

  inline const legionclaw::interface::Point2D &absolute_position() const {
    return absolute_position_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_position() {
    return &absolute_position_;
  }

  inline bool has_absolute_position() {
    return (absolute_position_ptr_ != nullptr);
  }

  inline void set_absolute_position_rms(
      const legionclaw::interface::Point2D &absolute_position_rms) {
    absolute_position_rms_ = absolute_position_rms;
    absolute_position_rms_ptr_ = &absolute_position_rms_;
  }

  inline const legionclaw::interface::Point2D &absolute_position_rms() const {
    return absolute_position_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_position_rms() {
    return &absolute_position_rms_;
  }

  inline bool has_absolute_position_rms() {
    return (absolute_position_rms_ptr_ != nullptr);
  }

  inline void
  set_absolute_velocity(const legionclaw::interface::Point2D &absolute_velocity) {
    absolute_velocity_ = absolute_velocity;
    absolute_velocity_ptr_ = &absolute_velocity_;
  }

  inline const legionclaw::interface::Point2D &absolute_velocity() const {
    return absolute_velocity_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_velocity() {
    return &absolute_velocity_;
  }

  inline bool has_absolute_velocity() {
    return (absolute_velocity_ptr_ != nullptr);
  }

  inline void set_absolute_velocity_rms(
      const legionclaw::interface::Point2D &absolute_velocity_rms) {
    absolute_velocity_rms_ = absolute_velocity_rms;
    absolute_velocity_rms_ptr_ = &absolute_velocity_rms_;
  }

  inline const legionclaw::interface::Point2D &absolute_velocity_rms() const {
    return absolute_velocity_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_velocity_rms() {
    return &absolute_velocity_rms_;
  }

  inline bool has_absolute_velocity_rms() {
    return (absolute_velocity_rms_ptr_ != nullptr);
  }

  inline void set_absolute_acceleration(
      const legionclaw::interface::Point2D &absolute_acceleration) {
    absolute_acceleration_ = absolute_acceleration;
    absolute_acceleration_ptr_ = &absolute_acceleration_;
  }

  inline const legionclaw::interface::Point2D &absolute_acceleration() const {
    return absolute_acceleration_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_acceleration() {
    return &absolute_acceleration_;
  }

  inline bool has_absolute_acceleration() {
    return (absolute_acceleration_ptr_ != nullptr);
  }

  inline void set_absolute_acceleration_rms(
      const legionclaw::interface::Point2D &absolute_acceleration_rms) {
    absolute_acceleration_rms_ = absolute_acceleration_rms;
    absolute_acceleration_rms_ptr_ = &absolute_acceleration_rms_;
  }

  inline const legionclaw::interface::Point2D &absolute_acceleration_rms() const {
    return absolute_acceleration_rms_;
  }

  inline legionclaw::interface::Point2D *mutable_absolute_acceleration_rms() {
    return &absolute_acceleration_rms_;
  }

  inline bool has_absolute_acceleration_rms() {
    return (absolute_acceleration_rms_ptr_ != nullptr);
  }

  inline void set_orientation(const double &orientation) {
    orientation_ = orientation;
    orientation_ptr_ = &orientation_;
  }

  inline const double &orientation() const { return orientation_; }

  inline double *mutable_orientation() { return &orientation_; }

  inline bool has_orientation() { return (orientation_ptr_ != nullptr); }

  inline void set_orient_rms(const double &orient_rms) {
    orient_rms_ = orient_rms;
    orient_rms_ptr_ = &orient_rms_;
  }

  inline const double &orient_rms() const { return orient_rms_; }

  inline double *mutable_orient_rms() { return &orient_rms_; }

  inline bool has_orient_rms() { return (orient_rms_ptr_ != nullptr); }

  inline void set_yaw(const double &yaw) {
    yaw_ = yaw;
    yaw_ptr_ = &yaw_;
  }

  inline const double &yaw() const { return yaw_; }

  inline double *mutable_yaw() { return &yaw_; }

  inline bool has_yaw() { return (yaw_ptr_ != nullptr); }

  inline void set_yaw_rms(const double &yaw_rms) {
    yaw_rms_ = yaw_rms;
    yaw_rms_ptr_ = &yaw_rms_;
  }

  inline const double &yaw_rms() const { return yaw_rms_; }

  inline double *mutable_yaw_rms() { return &yaw_rms_; }

  inline bool has_yaw_rms() { return (yaw_rms_ptr_ != nullptr); }

  inline void set_count(const int32_t &count) {
    count_ = count;
    count_ptr_ = &count_;
  }

  inline const int32_t &count() const { return count_; }

  inline int32_t *mutable_count() { return &count_; }

  inline bool has_count() { return (count_ptr_ != nullptr); }

  inline void set_moving_frames_count(const int32_t &moving_frames_count) {
    moving_frames_count_ = moving_frames_count;
    moving_frames_count_ptr_ = &moving_frames_count_;
  }

  inline const int32_t &moving_frames_count() const {
    return moving_frames_count_;
  }

  inline int32_t *mutable_moving_frames_count() {
    return &moving_frames_count_;
  }

  inline bool has_moving_frames_count() {
    return (moving_frames_count_ptr_ != nullptr);
  }

  inline void set_status(const legionclaw::interface::Status &status) {
    status_ = status;
    status_ptr_ = &status_;
  }

  inline const legionclaw::interface::Status &status() const { return status_; }

  inline legionclaw::interface::Status *mutable_status() { return &status_; }

  inline bool has_status() { return (status_ptr_ != nullptr); }

  inline void set_underpass_probability(const double &underpass_probability) {
    underpass_probability_ = underpass_probability;
    underpass_probability_ptr_ = &underpass_probability_;
  }

  inline const double &underpass_probability() const {
    return underpass_probability_;
  }

  inline double *mutable_underpass_probability() {
    return &underpass_probability_;
  }

  inline bool has_underpass_probability() {
    return (underpass_probability_ptr_ != nullptr);
  }

  inline void set_overpass_probability(const double &overpass_probability) {
    overpass_probability_ = overpass_probability;
    overpass_probability_ptr_ = &overpass_probability_;
  }

  inline const double &overpass_probability() const {
    return overpass_probability_;
  }

  inline double *mutable_overpass_probability() {
    return &overpass_probability_;
  }

  inline bool has_overpass_probability() {
    return (overpass_probability_ptr_ != nullptr);
  }

  inline void set_exist_probability(const uint8_t &exist_probability) {
    exist_probability_ = exist_probability;
    exist_probability_ptr_ = &exist_probability_;
  }

  inline const uint8_t &exist_probability() const { return exist_probability_; }

  inline uint8_t *mutable_exist_probability() { return &exist_probability_; }

  inline bool has_exist_probability() {
    return (exist_probability_ptr_ != nullptr);
  }

  inline void set_mov_property(const uint8_t &mov_property) {
    mov_property_ = mov_property;
    mov_property_ptr_ = &mov_property_;
  }

  inline const uint8_t &mov_property() const { return mov_property_; }

  inline uint8_t *mutable_mov_property() { return &mov_property_; }

  inline bool has_mov_property() { return (mov_property_ptr_ != nullptr); }

  inline void set_track_state(const uint8_t &track_state) {
    track_state_ = track_state;
    track_state_ptr_ = &track_state_;
  }

  inline const uint8_t &track_state() const { return track_state_; }

  inline uint8_t *mutable_track_state() { return &track_state_; }

  inline bool has_track_state() { return (track_state_ptr_ != nullptr); }

  inline void set_track_type(const uint8_t &track_type) {
    track_type_ = track_type;
    track_type_ptr_ = &track_type_;
  }

  inline const uint8_t &track_type() const { return track_type_; }

  inline uint8_t *mutable_track_type() { return &track_type_; }

  inline bool has_track_type() { return (track_type_ptr_ != nullptr); }

  void operator=(const RadarObstacle &radar_obstacle) {
    CopyFrom(radar_obstacle);
  }

  void CopyFrom(const RadarObstacle &radar_obstacle) {
    id_ = radar_obstacle.id();
    life_time_ = radar_obstacle.life_time();
    relative_position_ = radar_obstacle.relative_position();
    relative_position_rms_ = radar_obstacle.relative_position_rms();
    relative_velocity_ = radar_obstacle.relative_velocity();
    relative_velocity_rms_ = radar_obstacle.relative_velocity_rms();
    relative_acceleration_ = radar_obstacle.relative_acceleration();
    relative_acceleration_rms_ = radar_obstacle.relative_acceleration_rms();
    rcs_ = radar_obstacle.rcs();
    snr_ = radar_obstacle.snr();
    moving_status_ = radar_obstacle.moving_status();
    width_ = radar_obstacle.width();
    length_ = radar_obstacle.length();
    height_ = radar_obstacle.height();
    theta_ = radar_obstacle.theta();
    absolute_position_ = radar_obstacle.absolute_position();
    absolute_position_rms_ = radar_obstacle.absolute_position_rms();
    absolute_velocity_ = radar_obstacle.absolute_velocity();
    absolute_velocity_rms_ = radar_obstacle.absolute_velocity_rms();
    absolute_acceleration_ = radar_obstacle.absolute_acceleration();
    absolute_acceleration_rms_ = radar_obstacle.absolute_acceleration_rms();
    orientation_ = radar_obstacle.orientation();
    orient_rms_ = radar_obstacle.orient_rms();
    yaw_ = radar_obstacle.yaw();
    yaw_rms_ = radar_obstacle.yaw_rms();
    count_ = radar_obstacle.count();
    moving_frames_count_ = radar_obstacle.moving_frames_count();
    status_ = radar_obstacle.status();
    underpass_probability_ = radar_obstacle.underpass_probability();
    overpass_probability_ = radar_obstacle.overpass_probability();
    exist_probability_ = radar_obstacle.exist_probability();
    mov_property_ = radar_obstacle.mov_property();
    track_state_ = radar_obstacle.track_state();
    track_type_ = radar_obstacle.track_type();
  }

protected:
  // obstacle ID.
  int32_t id_;
  int32_t *id_ptr_ = nullptr;
  //障碍物存在时间
  uint8_t life_time_;
  uint8_t *life_time_ptr_ = nullptr;
  //雷达坐标系障碍物位置
  legionclaw::interface::Point2D relative_position_;
  legionclaw::interface::Point2D *relative_position_ptr_ = nullptr;
  //雷达坐标系障碍物位置标准差
  legionclaw::interface::Point2D relative_position_rms_;
  legionclaw::interface::Point2D *relative_position_rms_ptr_ = nullptr;
  //雷达坐标系障碍物速度
  legionclaw::interface::Point2D relative_velocity_;
  legionclaw::interface::Point2D *relative_velocity_ptr_ = nullptr;
  //雷达坐标系障碍物速度标准差
  legionclaw::interface::Point2D relative_velocity_rms_;
  legionclaw::interface::Point2D *relative_velocity_rms_ptr_ = nullptr;
  //雷达坐标系障碍物加速度
  legionclaw::interface::Point2D relative_acceleration_;
  legionclaw::interface::Point2D *relative_acceleration_ptr_ = nullptr;
  //雷达坐标系障碍物加速度标准差
  legionclaw::interface::Point2D relative_acceleration_rms_;
  legionclaw::interface::Point2D *relative_acceleration_rms_ptr_ = nullptr;
  //雷达散射面积
  double rcs_;
  double *rcs_ptr_ = nullptr;
  double snr_;
  double *snr_ptr_ = nullptr;
  // STATIONARY = 0; NEARING = 1; AWAYING = 2; NONE = 3;
  legionclaw::interface::RadarObstacle::MovingStatus moving_status_;
  legionclaw::interface::RadarObstacle::MovingStatus *moving_status_ptr_ = nullptr;
  //雷达障碍物宽度
  double width_;
  double *width_ptr_ = nullptr;
  //雷达障碍物长度
  double length_;
  double *length_ptr_ = nullptr;
  //雷达障碍物高度
  double height_;
  double *height_ptr_ = nullptr;
  double theta_;
  double *theta_ptr_ = nullptr;
  // obstacle position in map coordinate system
  legionclaw::interface::Point2D absolute_position_;
  legionclaw::interface::Point2D *absolute_position_ptr_ = nullptr;
  legionclaw::interface::Point2D absolute_position_rms_;
  legionclaw::interface::Point2D *absolute_position_rms_ptr_ = nullptr;
  // obstacle position in map coordinate system
  legionclaw::interface::Point2D absolute_velocity_;
  legionclaw::interface::Point2D *absolute_velocity_ptr_ = nullptr;
  legionclaw::interface::Point2D absolute_velocity_rms_;
  legionclaw::interface::Point2D *absolute_velocity_rms_ptr_ = nullptr;
  legionclaw::interface::Point2D absolute_acceleration_;
  legionclaw::interface::Point2D *absolute_acceleration_ptr_ = nullptr;
  legionclaw::interface::Point2D absolute_acceleration_rms_;
  legionclaw::interface::Point2D *absolute_acceleration_rms_ptr_ = nullptr;
  //雷达障碍物方位角
  double orientation_;
  double *orientation_ptr_ = nullptr;
  //雷达障碍物方位角标准差
  double orient_rms_;
  double *orient_rms_ptr_ = nullptr;
  double yaw_;
  double *yaw_ptr_ = nullptr;
  double yaw_rms_;
  double *yaw_rms_ptr_ = nullptr;
  int32_t count_;
  int32_t *count_ptr_ = nullptr;
  int32_t moving_frames_count_;
  int32_t *moving_frames_count_ptr_ = nullptr;
  //状态码
  legionclaw::interface::Status status_;
  legionclaw::interface::Status *status_ptr_ = nullptr;
  double underpass_probability_;
  double *underpass_probability_ptr_ = nullptr;
  double overpass_probability_;
  double *overpass_probability_ptr_ = nullptr;
  //障碍物存在的概率
  uint8_t exist_probability_;
  uint8_t *exist_probability_ptr_ = nullptr;
  //障碍物移动属性：MOVING_PROPERTY_MOVING=0，MOVING_PROPERTY_STATIONARY=1，MOVING_PROPERTY_ONCOMING=2，MOVING_PROPERTY_STATIONARY_CANDIDATE=3，MOVING_PROPERTY_UNKOWN=4，MOVING_PROPERTY_CROSSING_STATIONARY=5，MOVING_PROPERTY_CROSSING_MOVING=6，MOVING_PROPERTY_STOPPED=7
  uint8_t mov_property_;
  uint8_t *mov_property_ptr_ = nullptr;
  //跟踪状态：TRACK_STATE_DELETED=0；TRACK_STATE_INITED=1；TRACK_STATE_MEASURED=2；TRACK_STATE_PREDICTED=3；TRACK_STATE_DELETED_FOR_MERGE=4；TRACK_STATE_NEW_FOR_MERGE=5；
  uint8_t track_state_;
  uint8_t *track_state_ptr_ = nullptr;
  //追踪的障碍物类型：TRACK_TYPE_POINT=0；TRACK_TYPE_CAR=1；TRACK_TYPE_TRUCK=2；TRACK_TYPE_PEDESTRIAN=3；TRACK_TYPE_MOTORCYCLE=4；TRACK_TYPE_BICYCLE=5；TRACK_TYPE_WIDE=6；TRACK_TYPE_RESERVED=7；TRACK_TYPE_BRIDGE=8；
  uint8_t track_type_;
  uint8_t *track_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
