/**
 * @file    fins.hpp
 * @author  zdhy
 * @date    2022-04-11
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/header.hpp"
#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Fins {
public:
  Fins() {
    is_valid_ = false;
    latitude_ = 0.0;
    longitude_ = 0.0;
    elevation_ = 0.0;
    utm_zone_mumber_ = 0;
    utm_zone_char_ = 0;
    cep68_ = 0.0;
    cep95_ = 0.0;
    second_ = 0.0;
    sat_use_num_ = 0;
    sat_in_view_num_ = 0;
    solution_status_ = 0;
    position_type_ = 0;
    p_dop_ = 0.0;
    h_dop_ = 0.0;
    v_dop_ = 0.0;
    base_line_length_dual_ = 0.0;
    solution_status_dual_ = 0;
    position_type_dual_ = 0;
    solution_source_dual_ = 0;
    aoc_ = 0;
    rtk_baseline_ = 0;
  }
  ~Fins() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_is_valid(const bool &is_valid) {
    is_valid_ = is_valid;
    is_valid_ptr_ = &is_valid_;
  }

  inline const bool &is_valid() const { return is_valid_; }

  inline bool *mutable_is_valid() { return &is_valid_; }

  inline bool has_is_valid() { return (is_valid_ptr_ != nullptr); }

  inline void set_latitude(const double &latitude) {
    latitude_ = latitude;
    latitude_ptr_ = &latitude_;
  }

  inline const double &latitude() const { return latitude_; }

  inline double *mutable_latitude() { return &latitude_; }

  inline bool has_latitude() { return (latitude_ptr_ != nullptr); }

  inline void set_longitude(const double &longitude) {
    longitude_ = longitude;
    longitude_ptr_ = &longitude_;
  }

  inline const double &longitude() const { return longitude_; }

  inline double *mutable_longitude() { return &longitude_; }

  inline bool has_longitude() { return (longitude_ptr_ != nullptr); }

  inline void set_elevation(const double &elevation) {
    elevation_ = elevation;
    elevation_ptr_ = &elevation_;
  }

  inline const double &elevation() const { return elevation_; }

  inline double *mutable_elevation() { return &elevation_; }

  inline bool has_elevation() { return (elevation_ptr_ != nullptr); }

  inline void set_utm_position(const legionclaw::interface::Point3D &utm_position) {
    utm_position_ = utm_position;
    utm_position_ptr_ = &utm_position_;
  }

  inline const legionclaw::interface::Point3D &utm_position() const {
    return utm_position_;
  }

  inline legionclaw::interface::Point3D *mutable_utm_position() {
    return &utm_position_;
  }

  inline bool has_utm_position() { return (utm_position_ptr_ != nullptr); }

  inline void set_utm_zone_mumber(const int32_t &utm_zone_mumber) {
    utm_zone_mumber_ = utm_zone_mumber;
    utm_zone_mumber_ptr_ = &utm_zone_mumber_;
  }

  inline const int32_t &utm_zone_mumber() const { return utm_zone_mumber_; }

  inline int32_t *mutable_utm_zone_mumber() { return &utm_zone_mumber_; }

  inline bool has_utm_zone_mumber() {
    return (utm_zone_mumber_ptr_ != nullptr);
  }

  inline void set_utm_zone_char(const uint8_t &utm_zone_char) {
    utm_zone_char_ = utm_zone_char;
    utm_zone_char_ptr_ = &utm_zone_char_;
  }

  inline const uint8_t &utm_zone_char() const { return utm_zone_char_; }

  inline uint8_t *mutable_utm_zone_char() { return &utm_zone_char_; }

  inline bool has_utm_zone_char() { return (utm_zone_char_ptr_ != nullptr); }

  inline void set_attitude(const legionclaw::interface::Point3D &attitude) {
    attitude_ = attitude;
    attitude_ptr_ = &attitude_;
  }

  inline const legionclaw::interface::Point3D &attitude() const { return attitude_; }

  inline legionclaw::interface::Point3D *mutable_attitude() { return &attitude_; }

  inline bool has_attitude() { return (attitude_ptr_ != nullptr); }

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

  inline void set_sd_position(const legionclaw::interface::Point3D &sd_position) {
    sd_position_ = sd_position;
    sd_position_ptr_ = &sd_position_;
  }

  inline const legionclaw::interface::Point3D &sd_position() const {
    return sd_position_;
  }

  inline legionclaw::interface::Point3D *mutable_sd_position() {
    return &sd_position_;
  }

  inline bool has_sd_position() { return (sd_position_ptr_ != nullptr); }

  inline void set_sd_attitude(const legionclaw::interface::Point3D &sd_attitude) {
    sd_attitude_ = sd_attitude;
    sd_attitude_ptr_ = &sd_attitude_;
  }

  inline const legionclaw::interface::Point3D &sd_attitude() const {
    return sd_attitude_;
  }

  inline legionclaw::interface::Point3D *mutable_sd_attitude() {
    return &sd_attitude_;
  }

  inline bool has_sd_attitude() { return (sd_attitude_ptr_ != nullptr); }

  inline void set_sd_velocity(const legionclaw::interface::Point3D &sd_velocity) {
    sd_velocity_ = sd_velocity;
    sd_velocity_ptr_ = &sd_velocity_;
  }

  inline const legionclaw::interface::Point3D &sd_velocity() const {
    return sd_velocity_;
  }

  inline legionclaw::interface::Point3D *mutable_sd_velocity() {
    return &sd_velocity_;
  }

  inline bool has_sd_velocity() { return (sd_velocity_ptr_ != nullptr); }

  inline void set_cep68(const double &cep68) {
    cep68_ = cep68;
    cep68_ptr_ = &cep68_;
  }

  inline const double &cep68() const { return cep68_; }

  inline double *mutable_cep68() { return &cep68_; }

  inline bool has_cep68() { return (cep68_ptr_ != nullptr); }

  inline void set_cep95(const double &cep95) {
    cep95_ = cep95;
    cep95_ptr_ = &cep95_;
  }

  inline const double &cep95() const { return cep95_; }

  inline double *mutable_cep95() { return &cep95_; }

  inline bool has_cep95() { return (cep95_ptr_ != nullptr); }

  inline void set_second(const double &second) {
    second_ = second;
    second_ptr_ = &second_;
  }

  inline const double &second() const { return second_; }

  inline double *mutable_second() { return &second_; }

  inline bool has_second() { return (second_ptr_ != nullptr); }

  inline void set_sat_use_num(const int32_t &sat_use_num) {
    sat_use_num_ = sat_use_num;
    sat_use_num_ptr_ = &sat_use_num_;
  }

  inline const int32_t &sat_use_num() const { return sat_use_num_; }

  inline int32_t *mutable_sat_use_num() { return &sat_use_num_; }

  inline bool has_sat_use_num() { return (sat_use_num_ptr_ != nullptr); }

  inline void set_sat_in_view_num(const int32_t &sat_in_view_num) {
    sat_in_view_num_ = sat_in_view_num;
    sat_in_view_num_ptr_ = &sat_in_view_num_;
  }

  inline const int32_t &sat_in_view_num() const { return sat_in_view_num_; }

  inline int32_t *mutable_sat_in_view_num() { return &sat_in_view_num_; }

  inline bool has_sat_in_view_num() {
    return (sat_in_view_num_ptr_ != nullptr);
  }

  inline void set_solution_status(const uint16_t &solution_status) {
    solution_status_ = solution_status;
    solution_status_ptr_ = &solution_status_;
  }

  inline const uint16_t &solution_status() const { return solution_status_; }

  inline uint16_t *mutable_solution_status() { return &solution_status_; }

  inline bool has_solution_status() {
    return (solution_status_ptr_ != nullptr);
  }

  inline void set_position_type(const uint16_t &position_type) {
    position_type_ = position_type;
    position_type_ptr_ = &position_type_;
  }

  inline const uint16_t &position_type() const { return position_type_; }

  inline uint16_t *mutable_position_type() { return &position_type_; }

  inline bool has_position_type() { return (position_type_ptr_ != nullptr); }

  inline void set_p_dop(const double &p_dop) {
    p_dop_ = p_dop;
    p_dop_ptr_ = &p_dop_;
  }

  inline const double &p_dop() const { return p_dop_; }

  inline double *mutable_p_dop() { return &p_dop_; }

  inline bool has_p_dop() { return (p_dop_ptr_ != nullptr); }

  inline void set_h_dop(const double &h_dop) {
    h_dop_ = h_dop;
    h_dop_ptr_ = &h_dop_;
  }

  inline const double &h_dop() const { return h_dop_; }

  inline double *mutable_h_dop() { return &h_dop_; }

  inline bool has_h_dop() { return (h_dop_ptr_ != nullptr); }

  inline void set_v_dop(const double &v_dop) {
    v_dop_ = v_dop;
    v_dop_ptr_ = &v_dop_;
  }

  inline const double &v_dop() const { return v_dop_; }

  inline double *mutable_v_dop() { return &v_dop_; }

  inline bool has_v_dop() { return (v_dop_ptr_ != nullptr); }

  inline void
  set_attitude_dual(const legionclaw::interface::Point3D &attitude_dual) {
    attitude_dual_ = attitude_dual;
    attitude_dual_ptr_ = &attitude_dual_;
  }

  inline const legionclaw::interface::Point3D &attitude_dual() const {
    return attitude_dual_;
  }

  inline legionclaw::interface::Point3D *mutable_attitude_dual() {
    return &attitude_dual_;
  }

  inline bool has_attitude_dual() { return (attitude_dual_ptr_ != nullptr); }

  inline void
  set_sd_angle_dual(const legionclaw::interface::Point3D &sd_angle_dual) {
    sd_angle_dual_ = sd_angle_dual;
    sd_angle_dual_ptr_ = &sd_angle_dual_;
  }

  inline const legionclaw::interface::Point3D &sd_angle_dual() const {
    return sd_angle_dual_;
  }

  inline legionclaw::interface::Point3D *mutable_sd_angle_dual() {
    return &sd_angle_dual_;
  }

  inline bool has_sd_angle_dual() { return (sd_angle_dual_ptr_ != nullptr); }

  inline void set_base_line_length_dual(const double &base_line_length_dual) {
    base_line_length_dual_ = base_line_length_dual;
    base_line_length_dual_ptr_ = &base_line_length_dual_;
  }

  inline const double &base_line_length_dual() const {
    return base_line_length_dual_;
  }

  inline double *mutable_base_line_length_dual() {
    return &base_line_length_dual_;
  }

  inline bool has_base_line_length_dual() {
    return (base_line_length_dual_ptr_ != nullptr);
  }

  inline void set_solution_status_dual(const int32_t &solution_status_dual) {
    solution_status_dual_ = solution_status_dual;
    solution_status_dual_ptr_ = &solution_status_dual_;
  }

  inline const int32_t &solution_status_dual() const {
    return solution_status_dual_;
  }

  inline int32_t *mutable_solution_status_dual() {
    return &solution_status_dual_;
  }

  inline bool has_solution_status_dual() {
    return (solution_status_dual_ptr_ != nullptr);
  }

  inline void set_position_type_dual(const int32_t &position_type_dual) {
    position_type_dual_ = position_type_dual;
    position_type_dual_ptr_ = &position_type_dual_;
  }

  inline const int32_t &position_type_dual() const {
    return position_type_dual_;
  }

  inline int32_t *mutable_position_type_dual() { return &position_type_dual_; }

  inline bool has_position_type_dual() {
    return (position_type_dual_ptr_ != nullptr);
  }

  inline void set_solution_source_dual(const int32_t &solution_source_dual) {
    solution_source_dual_ = solution_source_dual;
    solution_source_dual_ptr_ = &solution_source_dual_;
  }

  inline const int32_t &solution_source_dual() const {
    return solution_source_dual_;
  }

  inline int32_t *mutable_solution_source_dual() {
    return &solution_source_dual_;
  }

  inline bool has_solution_source_dual() {
    return (solution_source_dual_ptr_ != nullptr);
  }

  inline void set_aoc(const uint32_t &aoc) {
    aoc_ = aoc;
    aoc_ptr_ = &aoc_;
  }

  inline const uint32_t &aoc() const { return aoc_; }

  inline uint32_t *mutable_aoc() { return &aoc_; }

  inline bool has_aoc() { return (aoc_ptr_ != nullptr); }

  inline void set_rtk_baseline(const uint32_t &rtk_baseline) {
    rtk_baseline_ = rtk_baseline;
    rtk_baseline_ptr_ = &rtk_baseline_;
  }

  inline const uint32_t &rtk_baseline() const { return rtk_baseline_; }

  inline uint32_t *mutable_rtk_baseline() { return &rtk_baseline_; }

  inline bool has_rtk_baseline() { return (rtk_baseline_ptr_ != nullptr); }

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

  void operator=(const Fins &fins) { CopyFrom(fins); }

  void CopyFrom(const Fins &fins) {
    header_ = fins.header();
    is_valid_ = fins.is_valid();
    latitude_ = fins.latitude();
    longitude_ = fins.longitude();
    elevation_ = fins.elevation();
    utm_position_ = fins.utm_position();
    utm_zone_mumber_ = fins.utm_zone_mumber();
    utm_zone_char_ = fins.utm_zone_char();
    attitude_ = fins.attitude();
    linear_velocity_ = fins.linear_velocity();
    sd_position_ = fins.sd_position();
    sd_attitude_ = fins.sd_attitude();
    sd_velocity_ = fins.sd_velocity();
    cep68_ = fins.cep68();
    cep95_ = fins.cep95();
    second_ = fins.second();
    sat_use_num_ = fins.sat_use_num();
    sat_in_view_num_ = fins.sat_in_view_num();
    solution_status_ = fins.solution_status();
    position_type_ = fins.position_type();
    p_dop_ = fins.p_dop();
    h_dop_ = fins.h_dop();
    v_dop_ = fins.v_dop();
    attitude_dual_ = fins.attitude_dual();
    sd_angle_dual_ = fins.sd_angle_dual();
    base_line_length_dual_ = fins.base_line_length_dual();
    solution_status_dual_ = fins.solution_status_dual();
    position_type_dual_ = fins.position_type_dual();
    solution_source_dual_ = fins.solution_source_dual();
    aoc_ = fins.aoc();
    rtk_baseline_ = fins.rtk_baseline();
    angular_velocity_ = fins.angular_velocity();
    acceleration_ = fins.acceleration();
  }

protected:
  //头部信息
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  //数据有效性标志位
  bool is_valid_;
  bool *is_valid_ptr_ = nullptr;
  //纬度信息
  double latitude_;
  double *latitude_ptr_ = nullptr;
  //经度信息
  double longitude_;
  double *longitude_ptr_ = nullptr;
  //海拔信息
  double elevation_;
  double *elevation_ptr_ = nullptr;
  // UTM坐标位置
  legionclaw::interface::Point3D utm_position_;
  legionclaw::interface::Point3D *utm_position_ptr_ = nullptr;
  // UTM区号
  int32_t utm_zone_mumber_;
  int32_t *utm_zone_mumber_ptr_ = nullptr;
  // UTM区号
  uint8_t utm_zone_char_;
  uint8_t *utm_zone_char_ptr_ = nullptr;
  //三轴姿态
  legionclaw::interface::Point3D attitude_;
  legionclaw::interface::Point3D *attitude_ptr_ = nullptr;
  //线速度信息
  legionclaw::interface::Point3D linear_velocity_;
  legionclaw::interface::Point3D *linear_velocity_ptr_ = nullptr;
  //位置标准差
  legionclaw::interface::Point3D sd_position_;
  legionclaw::interface::Point3D *sd_position_ptr_ = nullptr;
  //姿态标准差
  legionclaw::interface::Point3D sd_attitude_;
  legionclaw::interface::Point3D *sd_attitude_ptr_ = nullptr;
  //速度标准差
  legionclaw::interface::Point3D sd_velocity_;
  legionclaw::interface::Point3D *sd_velocity_ptr_ = nullptr;
  // cep68数据位
  double cep68_;
  double *cep68_ptr_ = nullptr;
  // cep95数据位
  double cep95_;
  double *cep95_ptr_ = nullptr;
  //时间
  double second_;
  double *second_ptr_ = nullptr;
  //使用卫星数
  int32_t sat_use_num_;
  int32_t *sat_use_num_ptr_ = nullptr;
  //可见卫星数
  int32_t sat_in_view_num_;
  int32_t *sat_in_view_num_ptr_ = nullptr;
  //求解状态
  uint16_t solution_status_;
  uint16_t *solution_status_ptr_ = nullptr;
  //定位状态
  uint16_t position_type_;
  uint16_t *position_type_ptr_ = nullptr;
  // pDop数据位
  double p_dop_;
  double *p_dop_ptr_ = nullptr;
  // hDop数据位
  double h_dop_;
  double *h_dop_ptr_ = nullptr;
  // vDop数据位
  double v_dop_;
  double *v_dop_ptr_ = nullptr;
  //双天线姿态
  legionclaw::interface::Point3D attitude_dual_;
  legionclaw::interface::Point3D *attitude_dual_ptr_ = nullptr;
  //双天线角度标准差
  legionclaw::interface::Point3D sd_angle_dual_;
  legionclaw::interface::Point3D *sd_angle_dual_ptr_ = nullptr;
  //双天线距离
  double base_line_length_dual_;
  double *base_line_length_dual_ptr_ = nullptr;
  //双天线求解状态
  int32_t solution_status_dual_;
  int32_t *solution_status_dual_ptr_ = nullptr;
  //双天线定位状态
  int32_t position_type_dual_;
  int32_t *position_type_dual_ptr_ = nullptr;
  //双天线求解源
  int32_t solution_source_dual_;
  int32_t *solution_source_dual_ptr_ = nullptr;
  // Aoc数据位
  uint32_t aoc_;
  uint32_t *aoc_ptr_ = nullptr;
  // RTK基线信息
  uint32_t rtk_baseline_;
  uint32_t *rtk_baseline_ptr_ = nullptr;
  //角速度信息
  legionclaw::interface::Point3D angular_velocity_;
  legionclaw::interface::Point3D *angular_velocity_ptr_ = nullptr;
  //加速度信息
  legionclaw::interface::Point3D acceleration_;
  legionclaw::interface::Point3D *acceleration_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
