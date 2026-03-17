#pragma once

#include <float.h>

#include <string>
#include <vector>

#include "enum.h"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/adc_trajectory.hpp"

using namespace std;
using  legionclaw::interface::ADCTrajectory;

namespace legionclaw {
namespace planning {

class CalculatedConditions {
 public:
  CalculatedConditions() = default;
  ~CalculatedConditions() = default;

  double MinDistanceToStop() {
    if (stopping_distances_.size() == 0) return DBL_MAX;
    double minS = stopping_distances_.at(0);
    for (unsigned int i = 0; i < stopping_distances_.size(); i++) {
      if (stopping_distances_.at(i) < minS) minS = stopping_distances_.at(i);
    }
    return minS;
  }

  std::string ToStringHeader() { return "Time:General>>"; }

  std::string ToString(ADCTrajectory::BehaviourLatState beh) {
    std::string str = "Unknown";
    switch (beh) {
      case ADCTrajectory::LAT_NOT_ACTIVE_STATE:
        str = "Init";
        break;
      case ADCTrajectory::WAITING_STATE:
        str = "Waiting";
        break;
      case ADCTrajectory::FORWARD_STATE:
        str = "Forward";
        break;
      case ADCTrajectory::STOPPING_STATE:
        str = "Stop";
        break;
      case ADCTrajectory::STATION_ARRIVED_STATE:
        str = "End";
        break;
      case ADCTrajectory::OBSTACLE_AVOIDANCE_STATE:
        str = "Swerving";
        break;
      case ADCTrajectory::TRAFFIC_LIGHT_WAIT_STATE:
        str = "Light Wait";
        break;
      case ADCTrajectory::TRAFFIC_SIGN_STOP_STATE:
        str = "Sign Stop";
        break;
      default:
        str = "Unknown";
        break;
    }

    return str;
  }

  void Init() {
    current_goal_id_ = 0;
    pre_goal_id_ = -1;
    is_static_ = false;
    min_stopping_distance_ = -1;
    // distance to stop
    closest_obj_collision_dis_ = -1;
    closest_obj_velocity_ = 0;
    closest_obj_theta_abs_ = 0.0;
    closest_obj_theta_veh_ = 0.0;
    target_lane_closest_obj_distance_ = 0.0;
    current_lane_closest_obj_distance_ = 0.0;
    avoidance_distance_ = -1;
    current_stop_sign_id_ = -1;
    prev_stop_sign_id_ = -1;
    cur_traffic_light_id_ = -1;
    pre_traffic_light_id_ = -1;
    is_trafficIs_red_ = false;
    distance_to_stop_line_ = DBL_MAX;

      //  Inward and Outward
    is_outward_ = false;
    is_inward_ = false;
    outwardstatus_ = OutwardStatus::Outward_Invalid;
    inwardstatus_ = InwardStatus::Inward_Invalid;
    distance_to_junction_ = DBL_MAX;

    is_limit_speed_kappa_ = false;
    limit_speed_kappa_deceleration_ = DBL_MAX;

    central_trajectory_index_ = {-1, -1};

    distance_to_change_lane_ = 0;
    time_to_change_lane_ = 0;
    target_lane_index_ = -1;
    current_lane_index_ = -1;
    stop_to_wait_change_lane_ = false;

    start_lat_offset_ = 0.0;
    end_lat_offset_ = 0.0;
    swerve_lat_offset_ = 0.0;
  }
  
 public:
  // Global Goals
  int current_goal_id_;
  int pre_goal_id_;
  //-------------------------------------------//
  // Following
  double closest_obj_collision_dis_;
  double closest_obj_velocity_;
  double closest_obj_theta_abs_;
  double closest_obj_theta_veh_;
  double current_lane_closest_obj_distance_;
  double target_lane_closest_obj_distance_;

  double avoidance_distance_;
  //-------------------------------------------//
  // For Lane Change
  double distance_to_change_lane_;  // unused
  double time_to_change_lane_;      // unused
  int current_lane_index_;
  int target_lane_index_;
  bool stop_to_wait_change_lane_;   //停车继续等待换道时机
  //-------------------------------------------//
  // Traffic Lights & Stop Sign
  int current_stop_sign_id_;
  int prev_stop_sign_id_;
  int cur_traffic_light_id_;
  int pre_traffic_light_id_;
  bool is_trafficIs_red_;  // On , off status
  double distance_to_stop_line_;
  //-------------------------------------------//
  //  Inward and Outward
  bool is_outward_;
  bool is_inward_;
  OutwardStatus  outwardstatus_;
  InwardStatus  inwardstatus_;
  double distance_to_junction_;
  //-------------------------------------------//

  // Limit Speed Sign
  bool is_limit_speed_kappa_;
  double limit_speed_kappa_deceleration_;
  //-------------------------------------------//
  
  // Swerving
  std::pair<int, int> central_trajectory_index_;
  double start_lat_offset_;   // Current distance to lane center.
  double end_lat_offset_;     // Current max distance to lane center.
  double swerve_lat_offset_;  // Max distance to lane center.

  //-------------------------------------------//
  // General
  bool is_static_;
  double min_stopping_distance_;  // comfortably
  std::vector<double> stopping_distances_;
  double goal_distance_ = DBL_MAX;  //距离站点的距离（参考线上）
  double goal_distance_trajectory_ = DBL_MAX;  //距离站点的距离（轨迹上）
  //-------------------------------------------//
  // Parking
  bool is_in_parking_area_ = false;
  bool is_recalculated_ = false;
  bool is_vehicle_static_ = false;
  bool is_near_terminal_ = false;
  bool is_at_terminal_ = false;

  



  

  
};

}  // namespace planning
}  // namespace legionclaw
