/**
 * @file    location.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/enum/enum.h"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/point_llh.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Location {
public:
  Location() {
    pitch_ = 0.0;
    roll_ = 0.0;
    heading_ = 0.0;
    rtk_flag_ = legionclaw::interface::Location::RTKFlag::RTK_NULL;
    odom_type_ = legionclaw::interface::Location::OdomType::GNSS;
    auxiliary_type_ =
        legionclaw::interface::Location::AuxiliaryType::AUXILIARY_TYPE_NULL;
    location_valid_flag_ = legionclaw::common::IsValid::INVALID;
    origin_lat_ = 0.0;
    origin_lon_ = 0.0;
    change_origin_flag_ =
        legionclaw::interface::Location::ChangeOriginFlag::CHANGE_NULL;
  }
  ~Location() = default;

  enum RTKFlag {
    RTK_NULL = 0,
    RTK_FIXED = 1,
    RTK_FLOAT = 2,
    GNSS_ERROR = 3,
  };

  enum OdomType {
    GNSS = 0,
    Wheel = 1,
    WG = 2,
    VG = 3,
    VWG = 4,
  };

  enum AuxiliaryType {
    AUXILIARY_TYPE_NULL = 0,
    LANELINE_MAP = 1,
    LIDAR_FEATURE_MAP = 2,
    LIDAR_CAMERA_FEATURE_MAP = 3,
  };

  enum ChangeOriginFlag {
    CHANGE_NULL = 0,
    CHANGE_ORIGIN_SUCCEED = 1,
    CHANGE_ORIGIN_SWITCHING = 2,
    CHANGE_ORIGIN_FAILED = 3,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_position(const legionclaw::interface::PointLLH &position) {
    position_ = position;
    position_ptr_ = &position_;
  }

  inline const legionclaw::interface::PointLLH &position() const {
    return position_;
  }

  inline legionclaw::interface::PointLLH *mutable_position() { return &position_; }

  inline bool has_position() { return (position_ptr_ != nullptr); }

  inline void set_pitch(const double &pitch) {
    pitch_ = pitch;
    pitch_ptr_ = &pitch_;
  }

  inline const double &pitch() const { return pitch_; }

  inline double *mutable_pitch() { return &pitch_; }

  inline bool has_pitch() { return (pitch_ptr_ != nullptr); }

  inline void set_roll(const double &roll) {
    roll_ = roll;
    roll_ptr_ = &roll_;
  }

  inline const double &roll() const { return roll_; }

  inline double *mutable_roll() { return &roll_; }

  inline bool has_roll() { return (roll_ptr_ != nullptr); }

  inline void set_heading(const double &heading) {
    heading_ = heading;
    heading_ptr_ = &heading_;
  }

  inline const double &heading() const { return heading_; }

  inline double *mutable_heading() { return &heading_; }

  inline bool has_heading() { return (heading_ptr_ != nullptr); }

  inline void
  set_linear_velocity(const legionclaw::interface::Point3D &linear_velocity) {
    linear_velocity_ = linear_velocity;
    linear_velocity_ptr_ = &linear_velocity_;
  }

  inline const legionclaw::interface::Point3D &linear_velocity() const {
    return linear_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_velocity() {
    return &linear_velocity_;
  }

  inline bool has_linear_velocity() {
    return (linear_velocity_ptr_ != nullptr);
  }

  inline void set_linear_acceleration(
      const legionclaw::interface::Point3D &linear_acceleration) {
    linear_acceleration_ = linear_acceleration;
    linear_acceleration_ptr_ = &linear_acceleration_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration() const {
    return linear_acceleration_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration() {
    return &linear_acceleration_;
  }

  inline bool has_linear_acceleration() {
    return (linear_acceleration_ptr_ != nullptr);
  }

  inline void
  set_angular_velocity(const legionclaw::interface::Point3D &angular_velocity) {
    angular_velocity_ = angular_velocity;
    angular_velocity_ptr_ = &angular_velocity_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity() const {
    return angular_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity() {
    return &angular_velocity_;
  }

  inline bool has_angular_velocity() {
    return (angular_velocity_ptr_ != nullptr);
  }

  inline void
  set_rtk_flag(const legionclaw::interface::Location::RTKFlag &rtk_flag) {
    rtk_flag_ = rtk_flag;
    rtk_flag_ptr_ = &rtk_flag_;
  }

  inline const legionclaw::interface::Location::RTKFlag &rtk_flag() const {
    return rtk_flag_;
  }

  inline legionclaw::interface::Location::RTKFlag *mutable_rtk_flag() {
    return &rtk_flag_;
  }

  inline bool has_rtk_flag() { return (rtk_flag_ptr_ != nullptr); }

  inline void
  set_odom_type(const legionclaw::interface::Location::OdomType &odom_type) {
    odom_type_ = odom_type;
    odom_type_ptr_ = &odom_type_;
  }

  inline const legionclaw::interface::Location::OdomType &odom_type() const {
    return odom_type_;
  }

  inline legionclaw::interface::Location::OdomType *mutable_odom_type() {
    return &odom_type_;
  }

  inline bool has_odom_type() { return (odom_type_ptr_ != nullptr); }

  inline void set_auxiliary_type(
      const legionclaw::interface::Location::AuxiliaryType &auxiliary_type) {
    auxiliary_type_ = auxiliary_type;
    auxiliary_type_ptr_ = &auxiliary_type_;
  }

  inline const legionclaw::interface::Location::AuxiliaryType &
  auxiliary_type() const {
    return auxiliary_type_;
  }

  inline legionclaw::interface::Location::AuxiliaryType *mutable_auxiliary_type() {
    return &auxiliary_type_;
  }

  inline bool has_auxiliary_type() { return (auxiliary_type_ptr_ != nullptr); }

  inline void
  set_location_valid_flag(const legionclaw::common::IsValid &location_valid_flag) {
    location_valid_flag_ = location_valid_flag;
    location_valid_flag_ptr_ = &location_valid_flag_;
  }

  inline const legionclaw::common::IsValid &location_valid_flag() const {
    return location_valid_flag_;
  }

  inline legionclaw::common::IsValid *mutable_location_valid_flag() {
    return &location_valid_flag_;
  }

  inline bool has_location_valid_flag() {
    return (location_valid_flag_ptr_ != nullptr);
  }

  inline void set_origin_lat(const double &origin_lat) {
    origin_lat_ = origin_lat;
    origin_lat_ptr_ = &origin_lat_;
  }

  inline const double &origin_lat() const { return origin_lat_; }

  inline double *mutable_origin_lat() { return &origin_lat_; }

  inline bool has_origin_lat() { return (origin_lat_ptr_ != nullptr); }

  inline void set_origin_lon(const double &origin_lon) {
    origin_lon_ = origin_lon;
    origin_lon_ptr_ = &origin_lon_;
  }

  inline const double &origin_lon() const { return origin_lon_; }

  inline double *mutable_origin_lon() { return &origin_lon_; }

  inline bool has_origin_lon() { return (origin_lon_ptr_ != nullptr); }

  inline void set_utm_position(const legionclaw::interface::PointENU &utm_position) {
    utm_position_ = utm_position;
    utm_position_ptr_ = &utm_position_;
  }

  inline const legionclaw::interface::PointENU &utm_position() const {
    return utm_position_;
  }

  inline legionclaw::interface::PointENU *mutable_utm_position() {
    return &utm_position_;
  }

  inline bool has_utm_position() { return (utm_position_ptr_ != nullptr); }

  inline void set_change_origin_flag(
      const legionclaw::interface::Location::ChangeOriginFlag &change_origin_flag) {
    change_origin_flag_ = change_origin_flag;
    change_origin_flag_ptr_ = &change_origin_flag_;
  }

  inline const legionclaw::interface::Location::ChangeOriginFlag &
  change_origin_flag() const {
    return change_origin_flag_;
  }

  inline legionclaw::interface::Location::ChangeOriginFlag *
  mutable_change_origin_flag() {
    return &change_origin_flag_;
  }

  inline bool has_change_origin_flag() {
    return (change_origin_flag_ptr_ != nullptr);
  }

  inline void
  set_utm_position_next(const legionclaw::interface::PointENU &utm_position_next) {
    utm_position_next_ = utm_position_next;
    utm_position_next_ptr_ = &utm_position_next_;
  }

  inline const legionclaw::interface::PointENU &utm_position_next() const {
    return utm_position_next_;
  }

  inline legionclaw::interface::PointENU *mutable_utm_position_next() {
    return &utm_position_next_;
  }

  inline bool has_utm_position_next() {
    return (utm_position_next_ptr_ != nullptr);
  }

  inline void
  set_position_std_dev(const legionclaw::interface::Point3D &position_std_dev) {
    position_std_dev_ = position_std_dev;
    position_std_dev_ptr_ = &position_std_dev_;
  }

  inline const legionclaw::interface::Point3D &position_std_dev() const {
    return position_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_position_std_dev() {
    return &position_std_dev_;
  }

  inline bool has_position_std_dev() {
    return (position_std_dev_ptr_ != nullptr);
  }

  inline void set_orientation_std_dev(
      const legionclaw::interface::Point3D &orientation_std_dev) {
    orientation_std_dev_ = orientation_std_dev;
    orientation_std_dev_ptr_ = &orientation_std_dev_;
  }

  inline const legionclaw::interface::Point3D &orientation_std_dev() const {
    return orientation_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_orientation_std_dev() {
    return &orientation_std_dev_;
  }

  inline bool has_orientation_std_dev() {
    return (orientation_std_dev_ptr_ != nullptr);
  }

  inline void set_linear_velocity_std_dev(
      const legionclaw::interface::Point3D &linear_velocity_std_dev) {
    linear_velocity_std_dev_ = linear_velocity_std_dev;
    linear_velocity_std_dev_ptr_ = &linear_velocity_std_dev_;
  }

  inline const legionclaw::interface::Point3D &linear_velocity_std_dev() const {
    return linear_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_velocity_std_dev() {
    return &linear_velocity_std_dev_;
  }

  inline bool has_linear_velocity_std_dev() {
    return (linear_velocity_std_dev_ptr_ != nullptr);
  }

  inline void set_linear_acceleration_std_dev(
      const legionclaw::interface::Point3D &linear_acceleration_std_dev) {
    linear_acceleration_std_dev_ = linear_acceleration_std_dev;
    linear_acceleration_std_dev_ptr_ = &linear_acceleration_std_dev_;
  }

  inline const legionclaw::interface::Point3D &linear_acceleration_std_dev() const {
    return linear_acceleration_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_linear_acceleration_std_dev() {
    return &linear_acceleration_std_dev_;
  }

  inline bool has_linear_acceleration_std_dev() {
    return (linear_acceleration_std_dev_ptr_ != nullptr);
  }

  inline void set_angular_velocity_std_dev(
      const legionclaw::interface::Point3D &angular_velocity_std_dev) {
    angular_velocity_std_dev_ = angular_velocity_std_dev;
    angular_velocity_std_dev_ptr_ = &angular_velocity_std_dev_;
  }

  inline const legionclaw::interface::Point3D &angular_velocity_std_dev() const {
    return angular_velocity_std_dev_;
  }

  inline legionclaw::interface::Point3D *mutable_angular_velocity_std_dev() {
    return &angular_velocity_std_dev_;
  }

  inline bool has_angular_velocity_std_dev() {
    return (angular_velocity_std_dev_ptr_ != nullptr);
  }

  void operator=(const Location &location) { CopyFrom(location); }

  void CopyFrom(const Location &location) {
    header_ = location.header();
    position_ = location.position();
    pitch_ = location.pitch();
    roll_ = location.roll();
    heading_ = location.heading();
    linear_velocity_ = location.linear_velocity();
    linear_acceleration_ = location.linear_acceleration();
    angular_velocity_ = location.angular_velocity();
    rtk_flag_ = location.rtk_flag();
    odom_type_ = location.odom_type();
    auxiliary_type_ = location.auxiliary_type();
    location_valid_flag_ = location.location_valid_flag();
    origin_lat_ = location.origin_lat();
    origin_lon_ = location.origin_lon();
    utm_position_ = location.utm_position();
    change_origin_flag_ = location.change_origin_flag();
    utm_position_next_ = location.utm_position_next();
    position_std_dev_ = location.position_std_dev();
    orientation_std_dev_ = location.orientation_std_dev();
    linear_velocity_std_dev_ = location.linear_velocity_std_dev();
    linear_acceleration_std_dev_ = location.linear_acceleration_std_dev();
    angular_velocity_std_dev_ = location.angular_velocity_std_dev();
  }

protected:
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //经纬度高程
  legionclaw::interface::PointLLH position_;
  legionclaw::interface::PointLLH *position_ptr_ = nullptr;
  //方向(单位弧度，-π～π,正东:0, 逆时针。都满足右手定则)
  double pitch_;
  double *pitch_ptr_ = nullptr;
  double roll_;
  double *roll_ptr_ = nullptr;
  double heading_;
  double *heading_ptr_ = nullptr;
  //线速度（m/s，北东地坐标系）
  legionclaw::interface::Point3D linear_velocity_;
  legionclaw::interface::Point3D *linear_velocity_ptr_ = nullptr;
  //线加速度（m/s2，前右下坐标系）
  legionclaw::interface::Point3D linear_acceleration_;
  legionclaw::interface::Point3D *linear_acceleration_ptr_ = nullptr;
  //角速度（度/s，前右下，满足右手定则）
  legionclaw::interface::Point3D angular_velocity_;
  legionclaw::interface::Point3D *angular_velocity_ptr_ = nullptr;
  // RTK标志值
  legionclaw::interface::Location::RTKFlag rtk_flag_;
  legionclaw::interface::Location::RTKFlag *rtk_flag_ptr_ = nullptr;
  //融合标志:0-(纯ins) 1-(纯轮速) 2-(泊车递推) 3-(无) 4-(orb匹配定位)
  legionclaw::interface::Location::OdomType odom_type_;
  legionclaw::interface::Location::OdomType *odom_type_ptr_ = nullptr;
  //辅助方式:0-NULL(无辅助方式)
  //1-laneliine_map(车道线/标识牌)2-feature_map(lidar)3-feature_map(lidar_camera)
  legionclaw::interface::Location::AuxiliaryType auxiliary_type_;
  legionclaw::interface::Location::AuxiliaryType *auxiliary_type_ptr_ = nullptr;
  // Location有效性标志位0: invalid； 1: valid
  legionclaw::common::IsValid location_valid_flag_;
  legionclaw::common::IsValid *location_valid_flag_ptr_ = nullptr;
  //参考点纬度 GCJ-02格式
  double origin_lat_;
  double *origin_lat_ptr_ = nullptr;
  //参考点经度 GCJ-02格式
  double origin_lon_;
  double *origin_lon_ptr_ = nullptr;
  //相对参考点xyz坐标
  legionclaw::interface::PointENU utm_position_;
  legionclaw::interface::PointENU *utm_position_ptr_ = nullptr;
  // 0::无坐标系切换 1:坐标系切换成功2:坐标系切换中3:坐标系切换故障
  legionclaw::interface::Location::ChangeOriginFlag change_origin_flag_;
  legionclaw::interface::Location::ChangeOriginFlag *change_origin_flag_ptr_ =
      nullptr;
  //相对参考点坐标(切换状态下，为新的原点坐标系下)
  legionclaw::interface::PointENU utm_position_next_;
  legionclaw::interface::PointENU *utm_position_next_ptr_ = nullptr;
  //位置标准差
  legionclaw::interface::Point3D position_std_dev_;
  legionclaw::interface::Point3D *position_std_dev_ptr_ = nullptr;
  //方向标准差
  legionclaw::interface::Point3D orientation_std_dev_;
  legionclaw::interface::Point3D *orientation_std_dev_ptr_ = nullptr;
  //速度标准差
  legionclaw::interface::Point3D linear_velocity_std_dev_;
  legionclaw::interface::Point3D *linear_velocity_std_dev_ptr_ = nullptr;
  //加速度标准差
  legionclaw::interface::Point3D linear_acceleration_std_dev_;
  legionclaw::interface::Point3D *linear_acceleration_std_dev_ptr_ = nullptr;
  //角速度标准差
  legionclaw::interface::Point3D angular_velocity_std_dev_;
  legionclaw::interface::Point3D *angular_velocity_std_dev_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
