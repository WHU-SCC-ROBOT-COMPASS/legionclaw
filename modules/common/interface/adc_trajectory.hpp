/**
 * @file    adc_trajectory.hpp
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
#include "modules/common/interface/estop.hpp"
#include "modules/common/interface/header.hpp"
#include "modules/common/interface/rss_info.hpp"
#include "modules/common/interface/trajectory_point.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ADCTrajectory {
public:
  ADCTrajectory() {
    trajectory_points_mutex_ = std::make_shared<std::mutex>();

    total_path_length_ = 0.0;
    total_path_time_ = 0.0;
    clear_trajectory_points();
    car_action_ = legionclaw::interface::ADCTrajectory::CarAction::CAR_ACTION_NONE;
    behaviour_lat_state_ = legionclaw::interface::ADCTrajectory::BehaviourLatState::
        LAT_NOT_ACTIVE_STATE;
    behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::
        LON_NOT_ACTIVE_STATE;
    scenario_ = legionclaw::interface::ADCTrajectory::Scenario::SCENARIO_UNKNOWN;
    driving_mode_ = legionclaw::common::DrivingMode::COMPLETE_MANUAL;
    adc_trajectory_type_ = legionclaw::interface::ADCTrajectory::ADCTrajectoryType::
        TRAJECTORY_TYPE_ABSOLUTE;
    is_replan_ = false;
    replan_reason_.clear();
    right_of_way_status_ =
        legionclaw::interface::ADCTrajectory::RightOfWayStatus::UNPROTECTED;
  }
  ~ADCTrajectory() = default;

  enum CarAction {
    CAR_ACTION_NONE = 0,
    CAR_ACTION_SINGLE = 1,
    CAR_ACTION_PLATOON_HEAD = 2,
    CAR_ACTION_PLATOON_OTHER = 3,
  };

  enum BehaviourLatState {
    LAT_NOT_ACTIVE_STATE = 0,
    WAITING_STATE = 1,
    FORWARD_STATE = 2,
    STOPPING_STATE = 3,
    EMERGENCY_STATE = 4,
    FOLLOW_STATE = 5,
    LANE_CHANGE_STATE = 6,
    OBSTACLE_AVOIDANCE_STATE = 7,
    STATION_STATE = 8,
    STATION_WAIT_STATE = 9,
    STATION_ARRIVED_STATE = 10,
    YIELDING_STATE = 11,
    BRANCH_LEFT_STATE = 12,
    BRANCH_RIGHT_STATE = 13,
    SLOW_DOWN_STATE = 14,
    AVOIDANCE_PRE_STATE = 15,
    AVOIDANCING_STATE = 16,
    AVOIDANCE_KEEP_STATE = 17,
    AVOIDANCE_BACK_PRE_STATE = 18,
    AVOIDANCE_BACK_STATE = 19,
    LANE_CHANGE_LEFT_PRE_STATE = 20,
    LANE_CHANGE_RIGHT_PRE_STATE = 21,
    LANE_CHANGE_LEFT_STATE = 22,
    LANE_CHANGE_RIGHT_STATE = 23,
    PARKING_INITIAL_STATE = 24,
    PARKING_FINISH_STATE = 25,
    PARALLEL_PARKING_BEGIN_STATE = 26,
    PARALLEL_PARKING_FORWARD_BEGIN_STATE = 27,
    PARALLEL_PARKING_FORWARD_STATE = 28,
    PARALLEL_PARKING_FORWARD_FINISH_STATE = 29,
    PARALLEL_PARKING_BACKWARD_BEGIN_STATE = 30,
    PARALLEL_PARKING_BACKWARD_STATE = 31,
    PARALLEL_PARKING_BACKWARD_FINISH_STATE = 32,
    PARALLEL_PARKING_FINISH_STATE = 33,
    VERTICAL_PARKING_BEGIN_STATE = 34,
    VERTICAL_PARKING_FORWARD_BEGIN_STATE = 35,
    VERTICAL_PARKING_FORWARD_STATE = 36,
    VERTICAL_PARKING_FORWARD_FINISH_STATE = 37,
    VERTICAL_PARKING_BACKWARD_BEGIN_STATE = 38,
    VERTICAL_PARKING_BACKWARD_STATE = 39,
    VERTICAL_PARKING_BACKWARD_FINISH_STATE = 40,
    VERTICAL_PARKING_FINISH_STATE = 41,
    OBLIQUE_PARKING_BEGIN_STATE = 42,
    OBLIQUE_PARKING_FORWARD_BEGIN_STATE = 43,
    OBLIQUE_PARKING_FORWARD_STATE = 44,
    OBLIQUE_PARKING_FORWARD_FINISH_STATE = 45,
    OBLIQUE_PARKING_BACKWARD_BEGIN_STATE = 46,
    OBLIQUE_PARKING_BACKWARD_STATE = 47,
    OBLIQUE_PARKING_BACKWARD_FINISH_STATE = 48,
    OBLIQUE_PARKING_FINISH_STATE = 49,
    FALLBACK = 50,
    PULLOVER = 51,
    LANECHANGE = 52,
    LEFT = 53,
    RIGHT = 54,
    SELF = 55,
  };

  enum BehaviourLonState {
    LON_NOT_ACTIVE_STATE = 0,
    NORMAL = 1,
    CONTINUE = 2,
    FOLLOW = 3,
    ACC = 4,
    TJP = 5,
    CUT_IN = 6,
    STOP = 7,
    ONCOMING = 8,
    CROSSING = 9,
    WAITE_AND_SEE = 10,
    CREEP = 11,
    PASS_THROUGH = 12,
    PRECISE_STOP = 13,
    TRAFFIC_LIGHT_STOP_STATE = 14,
    TRAFFIC_LIGHT_WAIT_STATE = 15,
    TRAFFIC_SIGN_STOP_STATE = 16,
    TRAFFIC_SIGN_WAIT_STATE = 17,
    KEEP_CLEAR = 18,
    EMIGNORE = 19,
    EMSTOP = 20,
    EMFOLLOW = 21,
    EMYIELD = 22,
    EMOVERTAKE = 23,
    EMNUDGE = 24,
    EMAVOID = 25,
    EMSIDEPASS = 26,
    OBJECT_TAG_NOT_SET = 27,
    INWARD = 28,
    OUTWARD = 29,
  };

  enum Scenario {
    SCENARIO_UNKNOWN = 0,
    CRUISE = 1000,
    CRUISE_URBAN = 1001,
    CRUISE_HIGHWAY = 1002,
    JUNCTION = 2000,
    JUNCTION_TRAFFIC_LIGHT = 2001,
    JUNCTION_STOP_SIGN = 2002,
  };

  enum ADCTrajectoryType {
    TRAJECTORY_TYPE_ABSOLUTE = 0,
    TRAJECTORY_TYPE_RELATIVE = 1,
    TRAJECTORY_TYPE_INVALID = 2,
  };

  enum RightOfWayStatus {
    UNPROTECTED = 0,
    PROTECTED = 1,
  };

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_total_path_length(const double &total_path_length) {
    total_path_length_ = total_path_length;
    total_path_length_ptr_ = &total_path_length_;
  }

  inline const double &total_path_length() const { return total_path_length_; }

  inline double *mutable_total_path_length() { return &total_path_length_; }

  inline bool has_total_path_length() {
    return (total_path_length_ptr_ != nullptr);
  }

  inline void set_total_path_time(const double &total_path_time) {
    total_path_time_ = total_path_time;
    total_path_time_ptr_ = &total_path_time_;
  }

  inline const double &total_path_time() const { return total_path_time_; }

  inline double *mutable_total_path_time() { return &total_path_time_; }

  inline bool has_total_path_time() {
    return (total_path_time_ptr_ != nullptr);
  }

  inline void set_trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> *trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.assign(trajectory_points->begin(),
                              trajectory_points->end());
  }

  inline void set_trajectory_points(
      const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_ = trajectory_points;
  }

  inline void
  set_trajectory_points(const uint32_t index,
                        legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_[index] = trajectory_points;
  }

  inline void add_trajectory_points(
      const legionclaw::interface::TrajectoryPoint &trajectory_points) {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.emplace_back(trajectory_points);
  }

  inline const legionclaw::interface::TrajectoryPoint &
  trajectory_points(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_[index];
  }

  inline std::vector<legionclaw::interface::TrajectoryPoint> *
  mutable_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return &trajectory_points_;
  }

  inline void trajectory_points(
      std::vector<legionclaw::interface::TrajectoryPoint> &trajectory_points) const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points.assign(trajectory_points_.begin(),
                             trajectory_points_.end());
  }

  inline const std::vector<legionclaw::interface::TrajectoryPoint> &
  trajectory_points() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_;
  }

  inline uint32_t trajectory_points_size() const {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    return trajectory_points_.size();
  }

  inline void clear_trajectory_points() {
    std::lock_guard<std::mutex> lock(*trajectory_points_mutex_);
    trajectory_points_.clear();
    trajectory_points_.shrink_to_fit();
  }

  inline bool has_trajectory_points() {
    return (trajectory_points_size() != 0);
  }

  inline void
  set_car_action(const legionclaw::interface::ADCTrajectory::CarAction &car_action) {
    car_action_ = car_action;
    car_action_ptr_ = &car_action_;
  }

  inline const legionclaw::interface::ADCTrajectory::CarAction &car_action() const {
    return car_action_;
  }

  inline legionclaw::interface::ADCTrajectory::CarAction *mutable_car_action() {
    return &car_action_;
  }

  inline bool has_car_action() { return (car_action_ptr_ != nullptr); }

  inline void set_behaviour_lat_state(
      const legionclaw::interface::ADCTrajectory::BehaviourLatState
          &behaviour_lat_state) {
    behaviour_lat_state_ = behaviour_lat_state;
    behaviour_lat_state_ptr_ = &behaviour_lat_state_;
  }

  inline const legionclaw::interface::ADCTrajectory::BehaviourLatState &
  behaviour_lat_state() const {
    return behaviour_lat_state_;
  }

  inline legionclaw::interface::ADCTrajectory::BehaviourLatState *
  mutable_behaviour_lat_state() {
    return &behaviour_lat_state_;
  }

  inline bool has_behaviour_lat_state() {
    return (behaviour_lat_state_ptr_ != nullptr);
  }

  inline void set_behaviour_lon_state(
      const legionclaw::interface::ADCTrajectory::BehaviourLonState
          &behaviour_lon_state) {
    behaviour_lon_state_ = behaviour_lon_state;
    behaviour_lon_state_ptr_ = &behaviour_lon_state_;
  }

  inline const legionclaw::interface::ADCTrajectory::BehaviourLonState &
  behaviour_lon_state() const {
    return behaviour_lon_state_;
  }

  inline legionclaw::interface::ADCTrajectory::BehaviourLonState *
  mutable_behaviour_lon_state() {
    return &behaviour_lon_state_;
  }

  inline bool has_behaviour_lon_state() {
    return (behaviour_lon_state_ptr_ != nullptr);
  }

  inline void
  set_scenario(const legionclaw::interface::ADCTrajectory::Scenario &scenario) {
    scenario_ = scenario;
    scenario_ptr_ = &scenario_;
  }

  inline const legionclaw::interface::ADCTrajectory::Scenario &scenario() const {
    return scenario_;
  }

  inline legionclaw::interface::ADCTrajectory::Scenario *mutable_scenario() {
    return &scenario_;
  }

  inline bool has_scenario() { return (scenario_ptr_ != nullptr); }

  inline void set_driving_mode(const legionclaw::common::DrivingMode &driving_mode) {
    driving_mode_ = driving_mode;
    driving_mode_ptr_ = &driving_mode_;
  }

  inline const legionclaw::common::DrivingMode &driving_mode() const {
    return driving_mode_;
  }

  inline legionclaw::common::DrivingMode *mutable_driving_mode() {
    return &driving_mode_;
  }

  inline bool has_driving_mode() { return (driving_mode_ptr_ != nullptr); }

  inline void set_adc_trajectory_type(
      const legionclaw::interface::ADCTrajectory::ADCTrajectoryType
          &adc_trajectory_type) {
    adc_trajectory_type_ = adc_trajectory_type;
    adc_trajectory_type_ptr_ = &adc_trajectory_type_;
  }

  inline const legionclaw::interface::ADCTrajectory::ADCTrajectoryType &
  adc_trajectory_type() const {
    return adc_trajectory_type_;
  }

  inline legionclaw::interface::ADCTrajectory::ADCTrajectoryType *
  mutable_adc_trajectory_type() {
    return &adc_trajectory_type_;
  }

  inline bool has_adc_trajectory_type() {
    return (adc_trajectory_type_ptr_ != nullptr);
  }

  inline void set_estop(const legionclaw::interface::EStop &estop) {
    estop_ = estop;
    estop_ptr_ = &estop_;
  }

  inline const legionclaw::interface::EStop &estop() const { return estop_; }

  inline legionclaw::interface::EStop *mutable_estop() { return &estop_; }

  inline bool has_estop() { return (estop_ptr_ != nullptr); }

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

  inline void set_right_of_way_status(
      const legionclaw::interface::ADCTrajectory::RightOfWayStatus
          &right_of_way_status) {
    right_of_way_status_ = right_of_way_status;
    right_of_way_status_ptr_ = &right_of_way_status_;
  }

  inline const legionclaw::interface::ADCTrajectory::RightOfWayStatus &
  right_of_way_status() const {
    return right_of_way_status_;
  }

  inline legionclaw::interface::ADCTrajectory::RightOfWayStatus *
  mutable_right_of_way_status() {
    return &right_of_way_status_;
  }

  inline bool has_right_of_way_status() {
    return (right_of_way_status_ptr_ != nullptr);
  }

  inline void set_rss_info(const legionclaw::interface::RSSInfo &rss_info) {
    rss_info_ = rss_info;
    rss_info_ptr_ = &rss_info_;
  }

  inline const legionclaw::interface::RSSInfo &rss_info() const { return rss_info_; }

  inline legionclaw::interface::RSSInfo *mutable_rss_info() { return &rss_info_; }

  inline bool has_rss_info() { return (rss_info_ptr_ != nullptr); }

  void operator=(const ADCTrajectory &adc_trajectory) {
    CopyFrom(adc_trajectory);
  }

  void CopyFrom(const ADCTrajectory &adc_trajectory) {
    header_ = adc_trajectory.header();
    total_path_length_ = adc_trajectory.total_path_length();
    total_path_time_ = adc_trajectory.total_path_time();
    trajectory_points_ = adc_trajectory.trajectory_points();
    car_action_ = adc_trajectory.car_action();
    behaviour_lat_state_ = adc_trajectory.behaviour_lat_state();
    behaviour_lon_state_ = adc_trajectory.behaviour_lon_state();
    scenario_ = adc_trajectory.scenario();
    driving_mode_ = adc_trajectory.driving_mode();
    adc_trajectory_type_ = adc_trajectory.adc_trajectory_type();
    estop_ = adc_trajectory.estop();
    is_replan_ = adc_trajectory.is_replan();
    replan_reason_ = adc_trajectory.replan_reason();
    right_of_way_status_ = adc_trajectory.right_of_way_status();
    rss_info_ = adc_trajectory.rss_info();
  }

protected:
  std::shared_ptr<std::mutex> trajectory_points_mutex_;
  // timestamp is included in header
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // unit meters(m)
  double total_path_length_;
  double *total_path_length_ptr_ = nullptr;
  // unit seconds(s)
  double total_path_time_;
  double *total_path_time_ptr_ = nullptr;
  //规划轨迹点列
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points_;
  //车辆行为： CAR_ACTION_NONE=0  //无效值 CAR_ACTION_SINGLE=1 //单车
  //CAR_ACTION_PLATOON_HEAD=2 //车队头车 CAR_ACTION_PLATOON_OTHER=3
  ////车队其他车辆
  legionclaw::interface::ADCTrajectory::CarAction car_action_;
  legionclaw::interface::ADCTrajectory::CarAction *car_action_ptr_ = nullptr;
  //自动驾驶状态（主要指横向状态）
  legionclaw::interface::ADCTrajectory::BehaviourLatState behaviour_lat_state_;
  legionclaw::interface::ADCTrajectory::BehaviourLatState *behaviour_lat_state_ptr_ =
      nullptr;
  //自动驾驶状态（主要指纵向状态）
  legionclaw::interface::ADCTrajectory::BehaviourLonState behaviour_lon_state_;
  legionclaw::interface::ADCTrajectory::BehaviourLonState *behaviour_lon_state_ptr_ =
      nullptr;
  //驾驶场景 0-UNKNOWN, 1-CRUISE, 2-CRUISE_URBAN 3-CRUISE_HIGHWAY, 2000-JUNCTION
  //2001-JUNCTION_TRAFFIC_LIGHT 2002-JUNCTION_STOP_SIGN
  legionclaw::interface::ADCTrajectory::Scenario scenario_;
  legionclaw::interface::ADCTrajectory::Scenario *scenario_ptr_ = nullptr;
  //驾驶模式： 1：人工驾驶， 2：辅助驾驶， 3：自动驾驶
  legionclaw::common::DrivingMode driving_mode_;
  legionclaw::common::DrivingMode *driving_mode_ptr_ = nullptr;
  //轨迹类型： TRAJECTORY_TYPE_ABSOLUTE=0 //绝对规划轨迹
  //TRAJECTORY_TYPE_RELATIVE=1 //相对规划轨迹 TRAJECTORY_TYPE_INVALID=2
  ////无效规划轨迹
  legionclaw::interface::ADCTrajectory::ADCTrajectoryType adc_trajectory_type_;
  legionclaw::interface::ADCTrajectory::ADCTrajectoryType *adc_trajectory_type_ptr_ =
      nullptr;
  // emergency stop
  legionclaw::interface::EStop estop_;
  legionclaw::interface::EStop *estop_ptr_ = nullptr;
  // is_replan == truemean replan triggered(default = false)
  bool is_replan_;
  bool *is_replan_ptr_ = nullptr;
  std::string replan_reason_;
  std::string *replan_reason_ptr_ = nullptr;
  // UNPROTECTED = 0;PROTECTED = 1;
  legionclaw::interface::ADCTrajectory::RightOfWayStatus right_of_way_status_;
  legionclaw::interface::ADCTrajectory::RightOfWayStatus *right_of_way_status_ptr_ =
      nullptr;
  // output related to RSS
  legionclaw::interface::RSSInfo rss_info_;
  legionclaw::interface::RSSInfo *rss_info_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
