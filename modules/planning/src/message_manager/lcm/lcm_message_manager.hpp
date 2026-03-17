/**
 * @file    lcm_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "lcm_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include "modules/common/base_message/message.h"
#include "modules/common/math/euler_angles_zxy.h"

#if LCM_ENABLE
/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
template <typename T> void LcmMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  std::map<std::string, legionclaw::common::Message> messages =
      instance_->GetConf()->messages();
  lcm_ = std::make_shared<lcm::LCM>(messages["LCM"].url);

  if (!lcm_->good()) {
    AERROR << "lcm init error!";
    return;
  }

  lcm_->subscribe("/vui_client/ObuCmdMsg",
                  &LcmMessageManager::HandleObuCmdMsgMessage, this);
  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool LcmMessageManager<T>::Activate() {
  if (is_active_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  routing_response_sub_ =
      lcm_->subscribe("/routing/RoutingResponse",
                      &LcmMessageManager::HandleRoutingResponseMessage, this);

  local_routing_response_sub_ = lcm_->subscribe(
      "/local_map/LocalRoutingResponse",
      &LcmMessageManager::HandleLocalRoutingResponseMessage, this);

  parking_info_sub_ =
      lcm_->subscribe("/routing/ParkingInfo",
                      &LcmMessageManager::HandleParkingInfoMessage, this);

  stop_info_sub_ =
      lcm_->subscribe("/routing/StopInfo",
                      &LcmMessageManager::HandleStopInfoMessage, this);

  traffic_light_msg_sub_ =
      lcm_->subscribe("/perception/fusion/traffic_sign_fusion/TrafficLightMsg",
                      &LcmMessageManager::HandleTrafficLightMsgMessage, this);

  location_sub_ =
      lcm_->subscribe("/localization/global_fusion/Location",
                      &LcmMessageManager::HandleLocationMessage, this);

  prediction_obstacles_sub_ = lcm_->subscribe(
      "/prediction/PredictionObstacles",
      &LcmMessageManager::HandlePredictionObstaclesMessage, this);

  lane_list_sub_ = lcm_->subscribe(
      "LaneList", &LcmMessageManager::HandleLaneListMessage, this);

  chassis_sub_ =
      lcm_->subscribe("/drivers/canbus/Chassis",
                      &LcmMessageManager::HandleChassisMessage, this);

  sotif_monitor_result_sub_ = lcm_->subscribe(
      "/safety/sotif_monitor/SotifMonitorResult",
      &LcmMessageManager::HandleSotifMonitorResultMessage, this);

  drivable_region_sub_ =
      lcm_->subscribe("/perception/fusion/traffic_sign_fusion/DrivableRegion",
                      &LcmMessageManager::HandleDrivableRegionMessage, this);

  parking_out_info_sub_ =
      lcm_->subscribe("/avp/parking_out/ParkingOutInfo",
                      &LcmMessageManager::HandleParkingOutInfoMessage, this);

  guide_info_sub_ =
      lcm_->subscribe("/routing/GuideInfo",
                      &LcmMessageManager::HandleGuideInfoMessage, this);

  traffic_events_sub_ =
      lcm_->subscribe("/routing/TrafficEvents",
                      &LcmMessageManager::HandleTrafficEventsMessage, this);

  std::cout << "lcm activate" << std::endl;
  is_active_ = true;
  return true;
}

template <typename T> bool LcmMessageManager<T>::DeActivate() {
  if (is_active_ == false) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  lcm_->unsubscribe(routing_response_sub_);

  lcm_->unsubscribe(local_routing_response_sub_);

  lcm_->unsubscribe(parking_info_sub_);

  lcm_->unsubscribe(stop_info_sub_);

  lcm_->unsubscribe(traffic_light_msg_sub_);

  lcm_->unsubscribe(location_sub_);

  lcm_->unsubscribe(prediction_obstacles_sub_);

  lcm_->unsubscribe(lane_list_sub_);

  lcm_->unsubscribe(chassis_sub_);

  lcm_->unsubscribe(sotif_monitor_result_sub_);

  lcm_->unsubscribe(drivable_region_sub_);

  lcm_->unsubscribe(parking_out_info_sub_);

  lcm_->unsubscribe(guide_info_sub_);

  lcm_->unsubscribe(traffic_events_sub_);

  is_active_ = false;
  std::cout << "lcm deactivate" << std::endl;
  return true;
}

template <typename T>
void LcmMessageManager<T>::PublishADCTrajectory(
    legionclaw::interface::ADCTrajectory msg) {
  if (is_init_ == false)
    return;
  lcm_interface::ADCTrajectory adc_trajectory;
  MESSAGE_HEADER_ASSIGN(lcm_interface, adc_trajectory)
  adc_trajectory.total_path_length = msg.total_path_length();
  adc_trajectory.total_path_time = msg.total_path_time();
  std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points;
  msg.trajectory_points(legion_trajectory_points);
  for (auto it_trajectory_points : legion_trajectory_points) {
    lcm_interface::TrajectoryPoint adc_trajectory_trajectory_point;
    lcm_interface::PathPoint adc_trajectory_trajectory_point_path_point;
    adc_trajectory_trajectory_point_path_point.x =
        it_trajectory_points.path_point().x();
    adc_trajectory_trajectory_point_path_point.y =
        it_trajectory_points.path_point().y();
    adc_trajectory_trajectory_point_path_point.z =
        it_trajectory_points.path_point().z();
    adc_trajectory_trajectory_point_path_point.theta =
        it_trajectory_points.path_point().theta();
    adc_trajectory_trajectory_point_path_point.kappa =
        it_trajectory_points.path_point().kappa();
    adc_trajectory_trajectory_point_path_point.s =
        it_trajectory_points.path_point().s();
    adc_trajectory_trajectory_point_path_point.dkappa =
        it_trajectory_points.path_point().dkappa();
    adc_trajectory_trajectory_point_path_point.ddkappa =
        it_trajectory_points.path_point().ddkappa();
    adc_trajectory_trajectory_point_path_point.lane_id =
        it_trajectory_points.path_point().lane_id();
    adc_trajectory_trajectory_point_path_point.x_derivative =
        it_trajectory_points.path_point().x_derivative();
    adc_trajectory_trajectory_point_path_point.y_derivative =
        it_trajectory_points.path_point().y_derivative();
    adc_trajectory_trajectory_point.path_point =
        adc_trajectory_trajectory_point_path_point;
    adc_trajectory_trajectory_point.v = it_trajectory_points.v();
    adc_trajectory_trajectory_point.a = it_trajectory_points.a();
    adc_trajectory_trajectory_point.relative_time =
        it_trajectory_points.relative_time();
    adc_trajectory_trajectory_point.da = it_trajectory_points.da();
    adc_trajectory_trajectory_point.is_steer_valid =
        it_trajectory_points.is_steer_valid();
    adc_trajectory_trajectory_point.front_steer =
        it_trajectory_points.front_steer();
    adc_trajectory_trajectory_point.rear_steer =
        it_trajectory_points.rear_steer();
    adc_trajectory_trajectory_point.gear = it_trajectory_points.gear();
    lcm_trajectory_points.emplace_back(adc_trajectory_trajectory_point);
  }
  adc_trajectory.trajectory_points_size = lcm_trajectory_points.size();
  adc_trajectory.trajectory_points = lcm_trajectory_points;
  adc_trajectory.car_action = msg.car_action();
  adc_trajectory.behaviour_lat_state = msg.behaviour_lat_state();
  adc_trajectory.behaviour_lon_state = msg.behaviour_lon_state();
  adc_trajectory.scenario = msg.scenario();
  adc_trajectory.driving_mode = msg.driving_mode();
  adc_trajectory.adc_trajectory_type = msg.adc_trajectory_type();
  lcm_interface::EStop adc_trajectory_estop;
  adc_trajectory_estop.is_estop = msg.estop().is_estop();
  adc_trajectory_estop.reason = msg.estop().reason();
  adc_trajectory.estop = adc_trajectory_estop;
  adc_trajectory.is_replan = msg.is_replan();
  adc_trajectory.replan_reason = msg.replan_reason();
  adc_trajectory.right_of_way_status = msg.right_of_way_status();
  lcm_interface::RSSInfo adc_trajectory_rss_info;
  adc_trajectory_rss_info.is_rss_safe = msg.rss_info().is_rss_safe();
  adc_trajectory_rss_info.cur_dist_lon = msg.rss_info().cur_dist_lon();
  adc_trajectory_rss_info.rss_safe_dist_lon =
      msg.rss_info().rss_safe_dist_lon();
  adc_trajectory_rss_info.acc_lon_range_minimum =
      msg.rss_info().acc_lon_range_minimum();
  adc_trajectory_rss_info.acc_lon_range_maximum =
      msg.rss_info().acc_lon_range_maximum();
  adc_trajectory_rss_info.acc_lat_left_range_minimum =
      msg.rss_info().acc_lat_left_range_minimum();
  adc_trajectory_rss_info.acc_lat_left_range_maximum =
      msg.rss_info().acc_lat_left_range_maximum();
  adc_trajectory_rss_info.acc_lat_right_range_minimum =
      msg.rss_info().acc_lat_right_range_minimum();
  adc_trajectory_rss_info.acc_lat_right_range_maximum =
      msg.rss_info().acc_lat_right_range_maximum();
  adc_trajectory.rss_info = adc_trajectory_rss_info;

  lcm_->publish("/planning/ADCTrajectory", &adc_trajectory);
}

template <typename T>
void LcmMessageManager<T>::PublishPlanningCmd(
    legionclaw::interface::PlanningCmd msg) {
  if (is_init_ == false)
    return;
  lcm_interface::PlanningCmd planning_cmd;
  MESSAGE_HEADER_ASSIGN(lcm_interface, planning_cmd)
  planning_cmd.turn_lamp_ctrl = msg.turn_lamp_ctrl();
  planning_cmd.high_beam_ctrl = msg.high_beam_ctrl();
  planning_cmd.low_beam_ctrl = msg.low_beam_ctrl();
  planning_cmd.horn_ctrl = msg.horn_ctrl();
  planning_cmd.front_wiper_ctrl = msg.front_wiper_ctrl();
  planning_cmd.rear_wiper_ctrl = msg.rear_wiper_ctrl();
  planning_cmd.position_lamp_ctrl = msg.position_lamp_ctrl();
  planning_cmd.front_fog_lamp_ctrl = msg.front_fog_lamp_ctrl();
  planning_cmd.rear_fog_lamp_ctrl = msg.rear_fog_lamp_ctrl();
  planning_cmd.brake_lamp_ctrl = msg.brake_lamp_ctrl();
  planning_cmd.alarm_lamp_ctrl = msg.alarm_lamp_ctrl();
  planning_cmd.lf_door_ctrl = msg.lf_door_ctrl();
  planning_cmd.rf_door_ctrl = msg.rf_door_ctrl();
  planning_cmd.lr_door_ctrl = msg.lr_door_ctrl();
  planning_cmd.rr_door_ctrl = msg.rr_door_ctrl();

  lcm_->publish("/planning/PlanningCmd", &planning_cmd);
}

template <typename T>
void LcmMessageManager<T>::PublishPlanningAnalysis(
    legionclaw::interface::PlanningAnalysis msg) {
  if (is_init_ == false)
    return;
  lcm_interface::PlanningAnalysis planning_analysis;
  MESSAGE_HEADER_ASSIGN(lcm_interface, planning_analysis)
  planning_analysis.frame_update_time = msg.frame_update_time();
  planning_analysis.generator_time = msg.generator_time();
  std::vector<lcm_interface::TimeConsume> lcm_evaluator_time;
  std::vector<legionclaw::interface::TimeConsume> legion_evaluator_time;
  msg.evaluator_time(legion_evaluator_time);
  for (auto it_evaluator_time : legion_evaluator_time) {
    lcm_interface::TimeConsume planning_analysis_time_consume;
    planning_analysis_time_consume.name = it_evaluator_time.name();
    planning_analysis_time_consume.time_consume =
        it_evaluator_time.time_consume();
    lcm_evaluator_time.emplace_back(planning_analysis_time_consume);
  }
  planning_analysis.evaluator_time_size = lcm_evaluator_time.size();
  planning_analysis.evaluator_time = lcm_evaluator_time;
  planning_analysis.selector_time = msg.selector_time();
  lcm_interface::PlanningParkingDebug planning_analysis_planning_parking_debug;
  std::vector<lcm_interface::Polygon2D> lcm_vehicle_preiew_polygon;
  std::vector<legionclaw::interface::Polygon2D> legion_vehicle_preiew_polygon;
  msg.planning_parking_debug().vehicle_preiew_polygon(
      legion_vehicle_preiew_polygon);
  for (auto it_vehicle_preiew_polygon : legion_vehicle_preiew_polygon) {
    lcm_interface::Polygon2D
        planning_analysis_planning_parking_debug_polygon_2d;
    planning_analysis_planning_parking_debug_polygon_2d.coordinate_system =
        it_vehicle_preiew_polygon.coordinate_system();
    std::vector<lcm_interface::Point2D> lcm_points;
    std::vector<legionclaw::interface::Point2D> legion_points;
    it_vehicle_preiew_polygon.points(legion_points);
    for (auto it_points : legion_points) {
      lcm_interface::Point2D
          planning_analysis_planning_parking_debug_polygon_2d_point_2d;
      planning_analysis_planning_parking_debug_polygon_2d_point_2d.x =
          it_points.x();
      planning_analysis_planning_parking_debug_polygon_2d_point_2d.y =
          it_points.y();
      lcm_points.emplace_back(
          planning_analysis_planning_parking_debug_polygon_2d_point_2d);
    }
    planning_analysis_planning_parking_debug_polygon_2d.points_size =
        lcm_points.size();
    planning_analysis_planning_parking_debug_polygon_2d.points = lcm_points;
    lcm_vehicle_preiew_polygon.emplace_back(
        planning_analysis_planning_parking_debug_polygon_2d);
  }
  planning_analysis_planning_parking_debug.vehicle_preiew_polygon_size =
      lcm_vehicle_preiew_polygon.size();
  planning_analysis_planning_parking_debug.vehicle_preiew_polygon =
      lcm_vehicle_preiew_polygon;
  std::vector<lcm_interface::Polygon2D> lcm_obstacles_polygon;
  std::vector<legionclaw::interface::Polygon2D> legion_obstacles_polygon;
  msg.planning_parking_debug().obstacles_polygon(legion_obstacles_polygon);
  for (auto it_obstacles_polygon : legion_obstacles_polygon) {
    lcm_interface::Polygon2D
        planning_analysis_planning_parking_debug_polygon_2d;
    planning_analysis_planning_parking_debug_polygon_2d.coordinate_system =
        it_obstacles_polygon.coordinate_system();
    std::vector<lcm_interface::Point2D> lcm_points_1;
    std::vector<legionclaw::interface::Point2D> legion_points_1;
    it_obstacles_polygon.points(legion_points_1);
    for (auto it_points_1 : legion_points_1) {
      lcm_interface::Point2D
          planning_analysis_planning_parking_debug_polygon_2d_point_2d;
      planning_analysis_planning_parking_debug_polygon_2d_point_2d.x =
          it_points_1.x();
      planning_analysis_planning_parking_debug_polygon_2d_point_2d.y =
          it_points_1.y();
      lcm_points_1.emplace_back(
          planning_analysis_planning_parking_debug_polygon_2d_point_2d);
    }
    planning_analysis_planning_parking_debug_polygon_2d.points_size =
        lcm_points_1.size();
    planning_analysis_planning_parking_debug_polygon_2d.points = lcm_points_1;
    lcm_obstacles_polygon.emplace_back(
        planning_analysis_planning_parking_debug_polygon_2d);
  }
  planning_analysis_planning_parking_debug.obstacles_polygon_size =
      lcm_obstacles_polygon.size();
  planning_analysis_planning_parking_debug.obstacles_polygon =
      lcm_obstacles_polygon;
  std::vector<lcm_interface::PathPoint> lcm_path_points;
  std::vector<legionclaw::interface::PathPoint> legion_path_points;
  msg.planning_parking_debug().path_points(legion_path_points);
  for (auto it_path_points : legion_path_points) {
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_path_point;
    planning_analysis_planning_parking_debug_path_point.x = it_path_points.x();
    planning_analysis_planning_parking_debug_path_point.y = it_path_points.y();
    planning_analysis_planning_parking_debug_path_point.z = it_path_points.z();
    planning_analysis_planning_parking_debug_path_point.theta =
        it_path_points.theta();
    planning_analysis_planning_parking_debug_path_point.kappa =
        it_path_points.kappa();
    planning_analysis_planning_parking_debug_path_point.s = it_path_points.s();
    planning_analysis_planning_parking_debug_path_point.dkappa =
        it_path_points.dkappa();
    planning_analysis_planning_parking_debug_path_point.ddkappa =
        it_path_points.ddkappa();
    planning_analysis_planning_parking_debug_path_point.lane_id =
        it_path_points.lane_id();
    planning_analysis_planning_parking_debug_path_point.x_derivative =
        it_path_points.x_derivative();
    planning_analysis_planning_parking_debug_path_point.y_derivative =
        it_path_points.y_derivative();
    lcm_path_points.emplace_back(
        planning_analysis_planning_parking_debug_path_point);
  }
  planning_analysis_planning_parking_debug.path_points_size =
      lcm_path_points.size();
  planning_analysis_planning_parking_debug.path_points = lcm_path_points;
  planning_analysis_planning_parking_debug.lat_error =
      msg.planning_parking_debug().lat_error();
  planning_analysis_planning_parking_debug.lon_error =
      msg.planning_parking_debug().lon_error();
  planning_analysis_planning_parking_debug.yaw_error =
      msg.planning_parking_debug().yaw_error();
  std::vector<lcm_interface::Point2dList> lcm_obstacles_vec;
  std::vector<legionclaw::interface::Point2dList> legion_obstacles_vec;
  msg.planning_parking_debug().obstacles_vec(legion_obstacles_vec);
  for (auto it_obstacles_vec : legion_obstacles_vec) {
    lcm_interface::Point2dList
        planning_analysis_planning_parking_debug_point_2d_list;
    std::vector<lcm_interface::Point2D> lcm_point2d_list;
    std::vector<legionclaw::interface::Point2D> legion_point2d_list;
    it_obstacles_vec.point2d_list(legion_point2d_list);
    for (auto it_point2d_list : legion_point2d_list) {
      lcm_interface::Point2D
          planning_analysis_planning_parking_debug_point_2d_list_point_2d;
      planning_analysis_planning_parking_debug_point_2d_list_point_2d.x =
          it_point2d_list.x();
      planning_analysis_planning_parking_debug_point_2d_list_point_2d.y =
          it_point2d_list.y();
      lcm_point2d_list.emplace_back(
          planning_analysis_planning_parking_debug_point_2d_list_point_2d);
    }
    planning_analysis_planning_parking_debug_point_2d_list.point2d_list_size =
        lcm_point2d_list.size();
    planning_analysis_planning_parking_debug_point_2d_list.point2d_list =
        lcm_point2d_list;
    lcm_obstacles_vec.emplace_back(
        planning_analysis_planning_parking_debug_point_2d_list);
  }
  planning_analysis_planning_parking_debug.obstacles_vec_size =
      lcm_obstacles_vec.size();
  planning_analysis_planning_parking_debug.obstacles_vec = lcm_obstacles_vec;
  std::vector<lcm_interface::TrajectoryPoint> lcm_warm_start_traj;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_warm_start_traj;
  msg.planning_parking_debug().warm_start_traj(legion_warm_start_traj);
  for (auto it_warm_start_traj : legion_warm_start_traj) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point_path_point.x =
        it_warm_start_traj.path_point().x();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.y =
        it_warm_start_traj.path_point().y();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.z =
        it_warm_start_traj.path_point().z();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.theta =
        it_warm_start_traj.path_point().theta();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.kappa =
        it_warm_start_traj.path_point().kappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.s =
        it_warm_start_traj.path_point().s();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .dkappa = it_warm_start_traj.path_point().dkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .ddkappa = it_warm_start_traj.path_point().ddkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .lane_id = it_warm_start_traj.path_point().lane_id();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .x_derivative = it_warm_start_traj.path_point().x_derivative();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .y_derivative = it_warm_start_traj.path_point().y_derivative();
    planning_analysis_planning_parking_debug_trajectory_point.path_point =
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point.v =
        it_warm_start_traj.v();
    planning_analysis_planning_parking_debug_trajectory_point.a =
        it_warm_start_traj.a();
    planning_analysis_planning_parking_debug_trajectory_point.relative_time =
        it_warm_start_traj.relative_time();
    planning_analysis_planning_parking_debug_trajectory_point.da =
        it_warm_start_traj.da();
    planning_analysis_planning_parking_debug_trajectory_point.is_steer_valid =
        it_warm_start_traj.is_steer_valid();
    planning_analysis_planning_parking_debug_trajectory_point.front_steer =
        it_warm_start_traj.front_steer();
    planning_analysis_planning_parking_debug_trajectory_point.rear_steer =
        it_warm_start_traj.rear_steer();
    planning_analysis_planning_parking_debug_trajectory_point.gear =
        it_warm_start_traj.gear();
    lcm_warm_start_traj.emplace_back(
        planning_analysis_planning_parking_debug_trajectory_point);
  }
  planning_analysis_planning_parking_debug.warm_start_traj_size =
      lcm_warm_start_traj.size();
  planning_analysis_planning_parking_debug.warm_start_traj =
      lcm_warm_start_traj;
  std::vector<lcm_interface::TrajectoryPoint> lcm_smoothed_traj_stage1;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_smoothed_traj_stage1;
  msg.planning_parking_debug().smoothed_traj_stage1(
      legion_smoothed_traj_stage1);
  for (auto it_smoothed_traj_stage1 : legion_smoothed_traj_stage1) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point_path_point.x =
        it_smoothed_traj_stage1.path_point().x();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.y =
        it_smoothed_traj_stage1.path_point().y();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.z =
        it_smoothed_traj_stage1.path_point().z();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.theta =
        it_smoothed_traj_stage1.path_point().theta();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.kappa =
        it_smoothed_traj_stage1.path_point().kappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.s =
        it_smoothed_traj_stage1.path_point().s();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .dkappa = it_smoothed_traj_stage1.path_point().dkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .ddkappa = it_smoothed_traj_stage1.path_point().ddkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .lane_id = it_smoothed_traj_stage1.path_point().lane_id();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .x_derivative = it_smoothed_traj_stage1.path_point().x_derivative();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .y_derivative = it_smoothed_traj_stage1.path_point().y_derivative();
    planning_analysis_planning_parking_debug_trajectory_point.path_point =
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point.v =
        it_smoothed_traj_stage1.v();
    planning_analysis_planning_parking_debug_trajectory_point.a =
        it_smoothed_traj_stage1.a();
    planning_analysis_planning_parking_debug_trajectory_point.relative_time =
        it_smoothed_traj_stage1.relative_time();
    planning_analysis_planning_parking_debug_trajectory_point.da =
        it_smoothed_traj_stage1.da();
    planning_analysis_planning_parking_debug_trajectory_point.is_steer_valid =
        it_smoothed_traj_stage1.is_steer_valid();
    planning_analysis_planning_parking_debug_trajectory_point.front_steer =
        it_smoothed_traj_stage1.front_steer();
    planning_analysis_planning_parking_debug_trajectory_point.rear_steer =
        it_smoothed_traj_stage1.rear_steer();
    planning_analysis_planning_parking_debug_trajectory_point.gear =
        it_smoothed_traj_stage1.gear();
    lcm_smoothed_traj_stage1.emplace_back(
        planning_analysis_planning_parking_debug_trajectory_point);
  }
  planning_analysis_planning_parking_debug.smoothed_traj_stage1_size =
      lcm_smoothed_traj_stage1.size();
  planning_analysis_planning_parking_debug.smoothed_traj_stage1 =
      lcm_smoothed_traj_stage1;
  std::vector<lcm_interface::TrajectoryPoint> lcm_smoothed_traj_stage2;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_smoothed_traj_stage2;
  msg.planning_parking_debug().smoothed_traj_stage2(
      legion_smoothed_traj_stage2);
  for (auto it_smoothed_traj_stage2 : legion_smoothed_traj_stage2) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point_path_point.x =
        it_smoothed_traj_stage2.path_point().x();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.y =
        it_smoothed_traj_stage2.path_point().y();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.z =
        it_smoothed_traj_stage2.path_point().z();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.theta =
        it_smoothed_traj_stage2.path_point().theta();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.kappa =
        it_smoothed_traj_stage2.path_point().kappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point.s =
        it_smoothed_traj_stage2.path_point().s();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .dkappa = it_smoothed_traj_stage2.path_point().dkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .ddkappa = it_smoothed_traj_stage2.path_point().ddkappa();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .lane_id = it_smoothed_traj_stage2.path_point().lane_id();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .x_derivative = it_smoothed_traj_stage2.path_point().x_derivative();
    planning_analysis_planning_parking_debug_trajectory_point_path_point
        .y_derivative = it_smoothed_traj_stage2.path_point().y_derivative();
    planning_analysis_planning_parking_debug_trajectory_point.path_point =
        planning_analysis_planning_parking_debug_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_trajectory_point.v =
        it_smoothed_traj_stage2.v();
    planning_analysis_planning_parking_debug_trajectory_point.a =
        it_smoothed_traj_stage2.a();
    planning_analysis_planning_parking_debug_trajectory_point.relative_time =
        it_smoothed_traj_stage2.relative_time();
    planning_analysis_planning_parking_debug_trajectory_point.da =
        it_smoothed_traj_stage2.da();
    planning_analysis_planning_parking_debug_trajectory_point.is_steer_valid =
        it_smoothed_traj_stage2.is_steer_valid();
    planning_analysis_planning_parking_debug_trajectory_point.front_steer =
        it_smoothed_traj_stage2.front_steer();
    planning_analysis_planning_parking_debug_trajectory_point.rear_steer =
        it_smoothed_traj_stage2.rear_steer();
    planning_analysis_planning_parking_debug_trajectory_point.gear =
        it_smoothed_traj_stage2.gear();
    lcm_smoothed_traj_stage2.emplace_back(
        planning_analysis_planning_parking_debug_trajectory_point);
  }
  planning_analysis_planning_parking_debug.smoothed_traj_stage2_size =
      lcm_smoothed_traj_stage2.size();
  planning_analysis_planning_parking_debug.smoothed_traj_stage2 =
      lcm_smoothed_traj_stage2;
  lcm_interface::Trajectory
      planning_analysis_planning_parking_debug_reference_line;
  planning_analysis_planning_parking_debug_reference_line.name =
      msg.planning_parking_debug().reference_line().name();
  std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points;
  msg.planning_parking_debug().reference_line().trajectory_points(
      legion_trajectory_points);
  for (auto it_trajectory_points : legion_trajectory_points) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_reference_line_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .x = it_trajectory_points.path_point().x();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .y = it_trajectory_points.path_point().y();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .z = it_trajectory_points.path_point().z();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .theta = it_trajectory_points.path_point().theta();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .kappa = it_trajectory_points.path_point().kappa();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .s = it_trajectory_points.path_point().s();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .dkappa = it_trajectory_points.path_point().dkappa();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .ddkappa = it_trajectory_points.path_point().ddkappa();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .lane_id = it_trajectory_points.path_point().lane_id();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .x_derivative = it_trajectory_points.path_point().x_derivative();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point
        .y_derivative = it_trajectory_points.path_point().y_derivative();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .path_point =
        planning_analysis_planning_parking_debug_reference_line_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_reference_line_trajectory_point.v =
        it_trajectory_points.v();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point.a =
        it_trajectory_points.a();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .relative_time = it_trajectory_points.relative_time();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .da = it_trajectory_points.da();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .is_steer_valid = it_trajectory_points.is_steer_valid();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .front_steer = it_trajectory_points.front_steer();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .rear_steer = it_trajectory_points.rear_steer();
    planning_analysis_planning_parking_debug_reference_line_trajectory_point
        .gear = it_trajectory_points.gear();
    lcm_trajectory_points.emplace_back(
        planning_analysis_planning_parking_debug_reference_line_trajectory_point);
  }
  planning_analysis_planning_parking_debug_reference_line
      .trajectory_points_size = lcm_trajectory_points.size();
  planning_analysis_planning_parking_debug_reference_line.trajectory_points =
      lcm_trajectory_points;
  planning_analysis_planning_parking_debug.reference_line =
      planning_analysis_planning_parking_debug_reference_line;
  std::vector<lcm_interface::Trajectory> lcm_trajectory_array;
  std::vector<legionclaw::interface::Trajectory> legion_trajectory_array;
  msg.planning_parking_debug().trajectory_array(legion_trajectory_array);
  for (auto it_trajectory_array : legion_trajectory_array) {
    lcm_interface::Trajectory
        planning_analysis_planning_parking_debug_trajectory;
    planning_analysis_planning_parking_debug_trajectory.name =
        it_trajectory_array.name();
    std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points_1;
    std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points_1;
    it_trajectory_array.trajectory_points(legion_trajectory_points_1);
    for (auto it_trajectory_points_1 : legion_trajectory_points_1) {
      lcm_interface::TrajectoryPoint
          planning_analysis_planning_parking_debug_trajectory_trajectory_point;
      lcm_interface::PathPoint
          planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point;
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .x = it_trajectory_points_1.path_point().x();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .y = it_trajectory_points_1.path_point().y();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .z = it_trajectory_points_1.path_point().z();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .theta = it_trajectory_points_1.path_point().theta();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .kappa = it_trajectory_points_1.path_point().kappa();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .s = it_trajectory_points_1.path_point().s();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .dkappa = it_trajectory_points_1.path_point().dkappa();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .ddkappa = it_trajectory_points_1.path_point().ddkappa();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .lane_id = it_trajectory_points_1.path_point().lane_id();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .x_derivative = it_trajectory_points_1.path_point().x_derivative();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point
          .y_derivative = it_trajectory_points_1.path_point().y_derivative();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .path_point =
          planning_analysis_planning_parking_debug_trajectory_trajectory_point_path_point;
      planning_analysis_planning_parking_debug_trajectory_trajectory_point.v =
          it_trajectory_points_1.v();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point.a =
          it_trajectory_points_1.a();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .relative_time = it_trajectory_points_1.relative_time();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point.da =
          it_trajectory_points_1.da();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .is_steer_valid = it_trajectory_points_1.is_steer_valid();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .front_steer = it_trajectory_points_1.front_steer();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .rear_steer = it_trajectory_points_1.rear_steer();
      planning_analysis_planning_parking_debug_trajectory_trajectory_point
          .gear = it_trajectory_points_1.gear();
      lcm_trajectory_points_1.emplace_back(
          planning_analysis_planning_parking_debug_trajectory_trajectory_point);
    }
    planning_analysis_planning_parking_debug_trajectory.trajectory_points_size =
        lcm_trajectory_points_1.size();
    planning_analysis_planning_parking_debug_trajectory.trajectory_points =
        lcm_trajectory_points_1;
    lcm_trajectory_array.emplace_back(
        planning_analysis_planning_parking_debug_trajectory);
  }
  planning_analysis_planning_parking_debug.trajectory_array_size =
      lcm_trajectory_array.size();
  planning_analysis_planning_parking_debug.trajectory_array =
      lcm_trajectory_array;
  lcm_interface::Trajectory
      planning_analysis_planning_parking_debug_optimal_coarse_trajectory;
  planning_analysis_planning_parking_debug_optimal_coarse_trajectory.name =
      msg.planning_parking_debug().optimal_coarse_trajectory().name();
  std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points_2;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points_2;
  msg.planning_parking_debug().optimal_coarse_trajectory().trajectory_points(
      legion_trajectory_points_2);
  for (auto it_trajectory_points_2 : legion_trajectory_points_2) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .x = it_trajectory_points_2.path_point().x();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .y = it_trajectory_points_2.path_point().y();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .z = it_trajectory_points_2.path_point().z();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .theta = it_trajectory_points_2.path_point().theta();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .kappa = it_trajectory_points_2.path_point().kappa();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .s = it_trajectory_points_2.path_point().s();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .dkappa = it_trajectory_points_2.path_point().dkappa();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .ddkappa = it_trajectory_points_2.path_point().ddkappa();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .lane_id = it_trajectory_points_2.path_point().lane_id();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .x_derivative = it_trajectory_points_2.path_point().x_derivative();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point
        .y_derivative = it_trajectory_points_2.path_point().y_derivative();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .path_point =
        planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .v = it_trajectory_points_2.v();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .a = it_trajectory_points_2.a();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .relative_time = it_trajectory_points_2.relative_time();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .da = it_trajectory_points_2.da();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .is_steer_valid = it_trajectory_points_2.is_steer_valid();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .front_steer = it_trajectory_points_2.front_steer();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .rear_steer = it_trajectory_points_2.rear_steer();
    planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point
        .gear = it_trajectory_points_2.gear();
    lcm_trajectory_points_2.emplace_back(
        planning_analysis_planning_parking_debug_optimal_coarse_trajectory_trajectory_point);
  }
  planning_analysis_planning_parking_debug_optimal_coarse_trajectory
      .trajectory_points_size = lcm_trajectory_points_2.size();
  planning_analysis_planning_parking_debug_optimal_coarse_trajectory
      .trajectory_points = lcm_trajectory_points_2;
  planning_analysis_planning_parking_debug.optimal_coarse_trajectory =
      planning_analysis_planning_parking_debug_optimal_coarse_trajectory;
  lcm_interface::Trajectory
      planning_analysis_planning_parking_debug_optimal_smooth_trajectory;
  planning_analysis_planning_parking_debug_optimal_smooth_trajectory.name =
      msg.planning_parking_debug().optimal_smooth_trajectory().name();
  std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points_3;
  std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points_3;
  msg.planning_parking_debug().optimal_smooth_trajectory().trajectory_points(
      legion_trajectory_points_3);
  for (auto it_trajectory_points_3 : legion_trajectory_points_3) {
    lcm_interface::TrajectoryPoint
        planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point;
    lcm_interface::PathPoint
        planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .x = it_trajectory_points_3.path_point().x();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .y = it_trajectory_points_3.path_point().y();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .z = it_trajectory_points_3.path_point().z();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .theta = it_trajectory_points_3.path_point().theta();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .kappa = it_trajectory_points_3.path_point().kappa();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .s = it_trajectory_points_3.path_point().s();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .dkappa = it_trajectory_points_3.path_point().dkappa();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .ddkappa = it_trajectory_points_3.path_point().ddkappa();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .lane_id = it_trajectory_points_3.path_point().lane_id();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .x_derivative = it_trajectory_points_3.path_point().x_derivative();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point
        .y_derivative = it_trajectory_points_3.path_point().y_derivative();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .path_point =
        planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point_path_point;
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .v = it_trajectory_points_3.v();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .a = it_trajectory_points_3.a();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .relative_time = it_trajectory_points_3.relative_time();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .da = it_trajectory_points_3.da();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .is_steer_valid = it_trajectory_points_3.is_steer_valid();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .front_steer = it_trajectory_points_3.front_steer();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .rear_steer = it_trajectory_points_3.rear_steer();
    planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point
        .gear = it_trajectory_points_3.gear();
    lcm_trajectory_points_3.emplace_back(
        planning_analysis_planning_parking_debug_optimal_smooth_trajectory_trajectory_point);
  }
  planning_analysis_planning_parking_debug_optimal_smooth_trajectory
      .trajectory_points_size = lcm_trajectory_points_3.size();
  planning_analysis_planning_parking_debug_optimal_smooth_trajectory
      .trajectory_points = lcm_trajectory_points_3;
  planning_analysis_planning_parking_debug.optimal_smooth_trajectory =
      planning_analysis_planning_parking_debug_optimal_smooth_trajectory;
  planning_analysis_planning_parking_debug.hybrid_a_star_map_time =
      msg.planning_parking_debug().hybrid_a_star_map_time();
  planning_analysis_planning_parking_debug.hybrid_a_star_heuristic_time =
      msg.planning_parking_debug().hybrid_a_star_heuristic_time();
  planning_analysis_planning_parking_debug.hybrid_a_star_rs_time =
      msg.planning_parking_debug().hybrid_a_star_rs_time();
  planning_analysis_planning_parking_debug.hybrid_a_star_total_time =
      msg.planning_parking_debug().hybrid_a_star_total_time();
  planning_analysis_planning_parking_debug.ias_collision_avoidance_time =
      msg.planning_parking_debug().ias_collision_avoidance_time();
  planning_analysis_planning_parking_debug.ias_path_smooth_time =
      msg.planning_parking_debug().ias_path_smooth_time();
  planning_analysis_planning_parking_debug.ias_speed_smooth_time =
      msg.planning_parking_debug().ias_speed_smooth_time();
  planning_analysis_planning_parking_debug.ias_total_time =
      msg.planning_parking_debug().ias_total_time();
  planning_analysis_planning_parking_debug.samping_trajectory_time =
      msg.planning_parking_debug().samping_trajectory_time();
  planning_analysis_planning_parking_debug.is_replan =
      msg.planning_parking_debug().is_replan();
  planning_analysis_planning_parking_debug.replan_reason =
      msg.planning_parking_debug().replan_reason();
  planning_analysis_planning_parking_debug.replan_time =
      msg.planning_parking_debug().replan_time();
  planning_analysis_planning_parking_debug.replan_num =
      msg.planning_parking_debug().replan_num();
  planning_analysis_planning_parking_debug.optimizer_thread_counter =
      msg.planning_parking_debug().optimizer_thread_counter();
  planning_analysis_planning_parking_debug.replan_by_context_update_counter =
      msg.planning_parking_debug().replan_by_context_update_counter();
  planning_analysis_planning_parking_debug.replan_by_large_error_counter =
      msg.planning_parking_debug().replan_by_large_error_counter();
  planning_analysis_planning_parking_debug.parking_type =
      msg.planning_parking_debug().parking_type();
  planning_analysis_planning_parking_debug.moves_counter =
      msg.planning_parking_debug().moves_counter();
  planning_analysis_planning_parking_debug.remain_distance =
      msg.planning_parking_debug().remain_distance();
  planning_analysis_planning_parking_debug.distance_to_leader_obj =
      msg.planning_parking_debug().distance_to_leader_obj();
  planning_analysis_planning_parking_debug.state =
      msg.planning_parking_debug().state();
  planning_analysis.planning_parking_debug =
      planning_analysis_planning_parking_debug;
  planning_analysis.dis_to_center_line = msg.dis_to_center_line();
  planning_analysis.diff_to_velocity = msg.diff_to_velocity();
  planning_analysis.referencline_kappa = msg.referencline_kappa();

  lcm_->publish("/planning/PlanningAnalysis", &planning_analysis);
}

template <typename T>
void LcmMessageManager<T>::PublishParkingStateDisplay(
    legionclaw::interface::ParkingStateDisplay msg) {
  if (is_init_ == false)
    return;
  lcm_interface::ParkingStateDisplay parking_state_display;
  MESSAGE_HEADER_ASSIGN(lcm_interface, parking_state_display)
  parking_state_display.parking_type = msg.parking_type();
  parking_state_display.moves_counter = msg.moves_counter();
  parking_state_display.remian_distance = msg.remian_distance();
  parking_state_display.display_info = msg.display_info();
  parking_state_display.distance_to_leader_obj = msg.distance_to_leader_obj();
  parking_state_display.state = msg.state();

  lcm_->publish("/planning/ParkingStateDisplay",
                &parking_state_display);
}

template <typename T>
void LcmMessageManager<T>::PublishTrajectoryArray(
    legionclaw::interface::TrajectoryArray msg) {
  if (is_init_ == false)
    return;
  lcm_interface::TrajectoryArray trajectory_array;
  MESSAGE_HEADER_ASSIGN(lcm_interface, trajectory_array)
  std::vector<lcm_interface::Path> lcm_spline_s;
  std::vector<legionclaw::interface::Path> legion_spline_s;
  msg.spline_s(legion_spline_s);
  for (auto it_spline_s : legion_spline_s) {
    lcm_interface::Path trajectory_array_path;
    trajectory_array_path.name = it_spline_s.name();
    std::vector<lcm_interface::PathPoint> lcm_path_points;
    std::vector<legionclaw::interface::PathPoint> legion_path_points;
    it_spline_s.path_points(legion_path_points);
    for (auto it_path_points : legion_path_points) {
      lcm_interface::PathPoint trajectory_array_path_path_point;
      trajectory_array_path_path_point.x = it_path_points.x();
      trajectory_array_path_path_point.y = it_path_points.y();
      trajectory_array_path_path_point.z = it_path_points.z();
      trajectory_array_path_path_point.theta = it_path_points.theta();
      trajectory_array_path_path_point.kappa = it_path_points.kappa();
      trajectory_array_path_path_point.s = it_path_points.s();
      trajectory_array_path_path_point.dkappa = it_path_points.dkappa();
      trajectory_array_path_path_point.ddkappa = it_path_points.ddkappa();
      trajectory_array_path_path_point.lane_id = it_path_points.lane_id();
      trajectory_array_path_path_point.x_derivative =
          it_path_points.x_derivative();
      trajectory_array_path_path_point.y_derivative =
          it_path_points.y_derivative();
      lcm_path_points.emplace_back(trajectory_array_path_path_point);
    }
    trajectory_array_path.path_points_size = lcm_path_points.size();
    trajectory_array_path.path_points = lcm_path_points;
    lcm_spline_s.emplace_back(trajectory_array_path);
  }
  trajectory_array.spline_s_size = lcm_spline_s.size();
  trajectory_array.spline_s = lcm_spline_s;
  std::vector<lcm_interface::Path> lcm_qp_smooth;
  std::vector<legionclaw::interface::Path> legion_qp_smooth;
  msg.qp_smooth(legion_qp_smooth);
  for (auto it_qp_smooth : legion_qp_smooth) {
    lcm_interface::Path trajectory_array_path;
    trajectory_array_path.name = it_qp_smooth.name();
    std::vector<lcm_interface::PathPoint> lcm_path_points_1;
    std::vector<legionclaw::interface::PathPoint> legion_path_points_1;
    it_qp_smooth.path_points(legion_path_points_1);
    for (auto it_path_points_1 : legion_path_points_1) {
      lcm_interface::PathPoint trajectory_array_path_path_point;
      trajectory_array_path_path_point.x = it_path_points_1.x();
      trajectory_array_path_path_point.y = it_path_points_1.y();
      trajectory_array_path_path_point.z = it_path_points_1.z();
      trajectory_array_path_path_point.theta = it_path_points_1.theta();
      trajectory_array_path_path_point.kappa = it_path_points_1.kappa();
      trajectory_array_path_path_point.s = it_path_points_1.s();
      trajectory_array_path_path_point.dkappa = it_path_points_1.dkappa();
      trajectory_array_path_path_point.ddkappa = it_path_points_1.ddkappa();
      trajectory_array_path_path_point.lane_id = it_path_points_1.lane_id();
      trajectory_array_path_path_point.x_derivative =
          it_path_points_1.x_derivative();
      trajectory_array_path_path_point.y_derivative =
          it_path_points_1.y_derivative();
      lcm_path_points_1.emplace_back(trajectory_array_path_path_point);
    }
    trajectory_array_path.path_points_size = lcm_path_points_1.size();
    trajectory_array_path.path_points = lcm_path_points_1;
    lcm_qp_smooth.emplace_back(trajectory_array_path);
  }
  trajectory_array.qp_smooth_size = lcm_qp_smooth.size();
  trajectory_array.qp_smooth = lcm_qp_smooth;
  std::vector<lcm_interface::Trajectory> lcm_trajectory_list;
  std::vector<legionclaw::interface::Trajectory> legion_trajectory_list;
  msg.trajectory_list(legion_trajectory_list);
  for (auto it_trajectory_list : legion_trajectory_list) {
    lcm_interface::Trajectory trajectory_array_trajectory;
    trajectory_array_trajectory.name = it_trajectory_list.name();
    std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points;
    std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points;
    it_trajectory_list.trajectory_points(legion_trajectory_points);
    for (auto it_trajectory_points : legion_trajectory_points) {
      lcm_interface::TrajectoryPoint
          trajectory_array_trajectory_trajectory_point;
      lcm_interface::PathPoint
          trajectory_array_trajectory_trajectory_point_path_point;
      trajectory_array_trajectory_trajectory_point_path_point.x =
          it_trajectory_points.path_point().x();
      trajectory_array_trajectory_trajectory_point_path_point.y =
          it_trajectory_points.path_point().y();
      trajectory_array_trajectory_trajectory_point_path_point.z =
          it_trajectory_points.path_point().z();
      trajectory_array_trajectory_trajectory_point_path_point.theta =
          it_trajectory_points.path_point().theta();
      trajectory_array_trajectory_trajectory_point_path_point.kappa =
          it_trajectory_points.path_point().kappa();
      trajectory_array_trajectory_trajectory_point_path_point.s =
          it_trajectory_points.path_point().s();
      trajectory_array_trajectory_trajectory_point_path_point.dkappa =
          it_trajectory_points.path_point().dkappa();
      trajectory_array_trajectory_trajectory_point_path_point.ddkappa =
          it_trajectory_points.path_point().ddkappa();
      trajectory_array_trajectory_trajectory_point_path_point.lane_id =
          it_trajectory_points.path_point().lane_id();
      trajectory_array_trajectory_trajectory_point_path_point.x_derivative =
          it_trajectory_points.path_point().x_derivative();
      trajectory_array_trajectory_trajectory_point_path_point.y_derivative =
          it_trajectory_points.path_point().y_derivative();
      trajectory_array_trajectory_trajectory_point.path_point =
          trajectory_array_trajectory_trajectory_point_path_point;
      trajectory_array_trajectory_trajectory_point.v = it_trajectory_points.v();
      trajectory_array_trajectory_trajectory_point.a = it_trajectory_points.a();
      trajectory_array_trajectory_trajectory_point.relative_time =
          it_trajectory_points.relative_time();
      trajectory_array_trajectory_trajectory_point.da =
          it_trajectory_points.da();
      trajectory_array_trajectory_trajectory_point.is_steer_valid =
          it_trajectory_points.is_steer_valid();
      trajectory_array_trajectory_trajectory_point.front_steer =
          it_trajectory_points.front_steer();
      trajectory_array_trajectory_trajectory_point.rear_steer =
          it_trajectory_points.rear_steer();
      trajectory_array_trajectory_trajectory_point.gear =
          it_trajectory_points.gear();
      lcm_trajectory_points.emplace_back(
          trajectory_array_trajectory_trajectory_point);
    }
    trajectory_array_trajectory.trajectory_points_size =
        lcm_trajectory_points.size();
    trajectory_array_trajectory.trajectory_points = lcm_trajectory_points;
    lcm_trajectory_list.emplace_back(trajectory_array_trajectory);
  }
  trajectory_array.trajectory_list_size = lcm_trajectory_list.size();
  trajectory_array.trajectory_list = lcm_trajectory_list;

  lcm_->publish("/planning/TrajectoryArray", &trajectory_array);
}

template <typename T>
void LcmMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  lcm_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(lcm_interface, faults)
  FAULTS_PARSER(lcm, faults)
  faults.faults_size = faults.faults.size();

  lcm_->publish("/planning/Faults", &faults);
}

template <typename T>
void LcmMessageManager<T>::PublishEvents(legionclaw::interface::Events msg) {
  if (is_init_ == false)
    return;
  lcm_interface::Events events;
  MESSAGE_HEADER_ASSIGN(lcm_interface, events)
  events.version = msg.version();
  std::vector<lcm_interface::Event> lcm_events_vector;
  std::vector<legionclaw::interface::Event> legion_events_vector;
  msg.events(legion_events_vector);
  for (auto it_events_vector : legion_events_vector) {
    lcm_interface::Event events_event;
    lcm_interface::Time events_event_timestamp;
    events_event_timestamp.sec = it_events_vector.timestamp().sec();
    events_event_timestamp.nsec = it_events_vector.timestamp().nsec();
    events_event.timestamp = events_event_timestamp;
    events_event.code = it_events_vector.code();
    events_event.reason = it_events_vector.reason();
    lcm_events_vector.emplace_back(events_event);
  }
  events.events_size = lcm_events_vector.size();
  events.events = lcm_events_vector;

  lcm_->publish("/planning/Events", &events);
}

template <typename T>
void LcmMessageManager<T>::HandleRoutingResponseMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::RoutingResponse* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::RoutingResponse routing_response;
  MESSAGE_HEADER_PARSER(routing_response)
  routing_response.set_plan_status(
      (legionclaw::interface::RoutingResponse::PlanStatus)msg->plan_status);
  routing_response.set_replan_flag(
      (legionclaw::interface::RoutingResponse::ReplanFlag)msg->replan_flag);
  routing_response.set_route_reason(
      (legionclaw::interface::RoutingResponse::RouteReason)msg->route_reason);
  std::vector<legionclaw::interface::LaneInfo> lane_list;
  for (auto it_lane_list : msg->lane_list) {
    legionclaw::interface::LaneInfo routing_response_lane_info;
    routing_response_lane_info.set_priority(it_lane_list.priority);
    routing_response_lane_info.set_global_id(it_lane_list.global_id);
    routing_response_lane_info.set_type(it_lane_list.type);
    routing_response_lane_info.set_predecessor_id(it_lane_list.predecessor_id);
    routing_response_lane_info.set_successor_id(it_lane_list.successor_id);
    routing_response_lane_info.set_left_neighbor_id(it_lane_list.left_neighbor_id);
    routing_response_lane_info.set_right_neighbor_id(it_lane_list.right_neighbor_id);
    std::vector<legionclaw::interface::LanePoint> lane_points;
    for (auto it_lane_points : it_lane_list.lane_points) {
      legionclaw::interface::LanePoint routing_response_lane_info_lane_point;
      legionclaw::interface::Point3D routing_response_lane_info_lane_point_point;
      routing_response_lane_info_lane_point_point.set_x(it_lane_points.point.x);
      routing_response_lane_info_lane_point_point.set_y(it_lane_points.point.y);
      routing_response_lane_info_lane_point_point.set_z(it_lane_points.point.z);
      routing_response_lane_info_lane_point.set_point(
          routing_response_lane_info_lane_point_point);
      routing_response_lane_info_lane_point.set_theta(it_lane_points.theta);
      routing_response_lane_info_lane_point.set_mileage(it_lane_points.mileage);
      routing_response_lane_info_lane_point.set_limit_speed(
          it_lane_points.limit_speed);
      routing_response_lane_info_lane_point.set_left_road_width(
          it_lane_points.left_road_width);
      routing_response_lane_info_lane_point.set_right_road_width(
          it_lane_points.right_road_width);
      routing_response_lane_info_lane_point.set_left_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.left_line_type);
      routing_response_lane_info_lane_point.set_right_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.right_line_type);
      lane_points.emplace_back(routing_response_lane_info_lane_point);
    }
    routing_response_lane_info.set_lane_points(&lane_points);
    lane_list.emplace_back(routing_response_lane_info);
  }
  routing_response.set_lane_list(&lane_list);

  instance_->HandleRoutingResponse(routing_response);
}

template <typename T>
void LcmMessageManager<T>::HandleLocalRoutingResponseMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::RoutingResponse* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::RoutingResponse routing_response;
  MESSAGE_HEADER_PARSER(routing_response)
  routing_response.set_plan_status(
      (legionclaw::interface::RoutingResponse::PlanStatus)msg->plan_status);
  routing_response.set_replan_flag(
      (legionclaw::interface::RoutingResponse::ReplanFlag)msg->replan_flag);
  routing_response.set_route_reason(
      (legionclaw::interface::RoutingResponse::RouteReason)msg->route_reason);
  std::vector<legionclaw::interface::LaneInfo> lane_list;
  for (auto it_lane_list : msg->lane_list) {
    legionclaw::interface::LaneInfo routing_response_lane_info;
    routing_response_lane_info.set_priority(it_lane_list.priority);
    routing_response_lane_info.set_global_id(it_lane_list.global_id);
    routing_response_lane_info.set_type(it_lane_list.type);
    routing_response_lane_info.set_predecessor_id(it_lane_list.predecessor_id);
    routing_response_lane_info.set_successor_id(it_lane_list.successor_id);
    routing_response_lane_info.set_left_neighbor_id(it_lane_list.left_neighbor_id);
    routing_response_lane_info.set_right_neighbor_id(it_lane_list.right_neighbor_id);
    std::vector<legionclaw::interface::LanePoint> lane_points;
    for (auto it_lane_points : it_lane_list.lane_points) {
      legionclaw::interface::LanePoint routing_response_lane_info_lane_point;
      legionclaw::interface::Point3D routing_response_lane_info_lane_point_point;
      routing_response_lane_info_lane_point_point.set_x(it_lane_points.point.x);
      routing_response_lane_info_lane_point_point.set_y(it_lane_points.point.y);
      routing_response_lane_info_lane_point_point.set_z(it_lane_points.point.z);
      routing_response_lane_info_lane_point.set_point(
          routing_response_lane_info_lane_point_point);
      routing_response_lane_info_lane_point.set_theta(it_lane_points.theta);
      routing_response_lane_info_lane_point.set_mileage(it_lane_points.mileage);
      routing_response_lane_info_lane_point.set_limit_speed(
          it_lane_points.limit_speed);
      routing_response_lane_info_lane_point.set_left_road_width(
          it_lane_points.left_road_width);
      routing_response_lane_info_lane_point.set_right_road_width(
          it_lane_points.right_road_width);
      routing_response_lane_info_lane_point.set_left_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.left_line_type);
      routing_response_lane_info_lane_point.set_right_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.right_line_type);
      lane_points.emplace_back(routing_response_lane_info_lane_point);
    }
    routing_response_lane_info.set_lane_points(&lane_points);
    lane_list.emplace_back(routing_response_lane_info);
  }
  routing_response.set_lane_list(&lane_list);

  instance_->HandleLocalRoutingResponse(routing_response);
}

template <typename T>
void LcmMessageManager<T>::HandleParkingInfoMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ParkingInfo* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::ParkingInfo parking_info;
  MESSAGE_HEADER_PARSER(parking_info)
  parking_info.set_parking_space_id(msg->parking_space_id);
  parking_info.set_parking_type((legionclaw::common::ParkingType)msg->parking_type);
  parking_info.set_parking_status(
      (legionclaw::common::ParkingStatus)msg->parking_status);
  parking_info.set_confidence(msg->confidence);
  legionclaw::interface::Point3D parking_info_center_point_of_parking;
  parking_info_center_point_of_parking.set_x(msg->center_point_of_parking.x);
  parking_info_center_point_of_parking.set_y(msg->center_point_of_parking.y);
  parking_info_center_point_of_parking.set_z(msg->center_point_of_parking.z);
  parking_info.set_center_point_of_parking(
      parking_info_center_point_of_parking);
  parking_info.set_theta(msg->theta);
  parking_info.set_width(msg->width);
  parking_info.set_length(msg->length);
  parking_info.set_yaw_offset(msg->yaw_offset);
  legionclaw::interface::Polygon3D parking_info_polygon;
  parking_info_polygon.set_coordinate_system(
      (legionclaw::common::CoordinateSystem)msg->polygon.coordinate_system);
  std::vector<legionclaw::interface::Point3D> points;
  for (auto it_points : msg->polygon.points) {
    legionclaw::interface::Point3D parking_info_polygon_point_3d;
    parking_info_polygon_point_3d.set_x(it_points.x);
    parking_info_polygon_point_3d.set_y(it_points.y);
    parking_info_polygon_point_3d.set_z(it_points.z);
    points.emplace_back(parking_info_polygon_point_3d);
  }
  parking_info_polygon.set_points(&points);
  parking_info.set_polygon(parking_info_polygon);
  parking_info.set_sensor_id((legionclaw::common::SensorID)msg->sensor_id);
  parking_info.set_is_lane_width_valid(msg->is_lane_width_valid);
  parking_info.set_lane_width(msg->lane_width);
  std::vector<legionclaw::interface::ParkingStopper> parking_stoppers;
  for (auto it_parking_stoppers : msg->parking_stoppers) {
    legionclaw::interface::ParkingStopper parking_info_parking_stopper;
    MESSAGE_HEADER_PARSER(parking_info_parking_stopper)
    legionclaw::interface::Point3D
        parking_info_parking_stopper_center_point_vehicle;
    parking_info_parking_stopper_center_point_vehicle.set_x(
        it_parking_stoppers.center_point_vehicle.x);
    parking_info_parking_stopper_center_point_vehicle.set_y(
        it_parking_stoppers.center_point_vehicle.y);
    parking_info_parking_stopper_center_point_vehicle.set_z(
        it_parking_stoppers.center_point_vehicle.z);
    parking_info_parking_stopper.set_center_point_vehicle(
        parking_info_parking_stopper_center_point_vehicle);
    legionclaw::interface::Point3D parking_info_parking_stopper_center_point_abs;
    parking_info_parking_stopper_center_point_abs.set_x(
        it_parking_stoppers.center_point_abs.x);
    parking_info_parking_stopper_center_point_abs.set_y(
        it_parking_stoppers.center_point_abs.y);
    parking_info_parking_stopper_center_point_abs.set_z(
        it_parking_stoppers.center_point_abs.z);
    parking_info_parking_stopper.set_center_point_abs(
        parking_info_parking_stopper_center_point_abs);
    std::vector<legionclaw::interface::Point3D> stopper_points_vehicle;
    for (auto it_stopper_points_vehicle :
         it_parking_stoppers.stopper_points_vehicle) {
      legionclaw::interface::Point3D parking_info_parking_stopper_point_3d;
      parking_info_parking_stopper_point_3d.set_x(it_stopper_points_vehicle.x);
      parking_info_parking_stopper_point_3d.set_y(it_stopper_points_vehicle.y);
      parking_info_parking_stopper_point_3d.set_z(it_stopper_points_vehicle.z);
      stopper_points_vehicle.emplace_back(
          parking_info_parking_stopper_point_3d);
    }
    parking_info_parking_stopper.set_stopper_points_vehicle(
        &stopper_points_vehicle);
    std::vector<legionclaw::interface::Point3D> stopper_points_abs;
    for (auto it_stopper_points_abs : it_parking_stoppers.stopper_points_abs) {
      legionclaw::interface::Point3D parking_info_parking_stopper_point_3d;
      parking_info_parking_stopper_point_3d.set_x(it_stopper_points_abs.x);
      parking_info_parking_stopper_point_3d.set_y(it_stopper_points_abs.y);
      parking_info_parking_stopper_point_3d.set_z(it_stopper_points_abs.z);
      stopper_points_abs.emplace_back(parking_info_parking_stopper_point_3d);
    }
    parking_info_parking_stopper.set_stopper_points_abs(&stopper_points_abs);
    parking_stoppers.emplace_back(parking_info_parking_stopper);
  }
  parking_info.set_parking_stoppers(&parking_stoppers);
  parking_info.set_parking_direction_type(
      (legionclaw::common::Direction)msg->parking_direction_type);
  parking_info.set_left_occupied_status(
      (legionclaw::common::OccupiedStatus)msg->left_occupied_status);
  parking_info.set_right_occupied_status(
      (legionclaw::common::OccupiedStatus)msg->right_occupied_status);
  parking_info.set_parking_source_type(
      (legionclaw::common::ParkingSourceType)msg->parking_source_type);

  instance_->HandleParkingInfo(parking_info);
}

template <typename T>
void LcmMessageManager<T>::HandleStopInfoMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::StopInfo* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::StopInfo stop_info;
  MESSAGE_HEADER_PARSER(stop_info)
  std::vector<legionclaw::interface::StopPoint> stop_points;
  for (auto it_stop_points : msg->stop_points) {
    legionclaw::interface::StopPoint stop_info_stop_point;
    legionclaw::interface::Point3D stop_info_stop_point_point;
    stop_info_stop_point_point.set_x(it_stop_points.point.x);
    stop_info_stop_point_point.set_y(it_stop_points.point.y);
    stop_info_stop_point_point.set_z(it_stop_points.point.z);
    stop_info_stop_point.set_point(stop_info_stop_point_point);
    stop_info_stop_point.set_theta(it_stop_points.theta);
    stop_info_stop_point.set_type(it_stop_points.type);
    stop_info_stop_point.set_stop_distance(it_stop_points.stop_distance);
    stop_points.emplace_back(stop_info_stop_point);
  }
  stop_info.set_stop_points(&stop_points);

  instance_->HandleStopInfo(stop_info);
}

template <typename T>
void LcmMessageManager<T>::HandleTrafficLightMsgMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::TrafficLightMsg* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::TrafficLightMsg traffic_light_msg;
  MESSAGE_HEADER_PARSER(traffic_light_msg)
  std::vector<legionclaw::interface::TrafficLight> traffic_light;
  for (auto it_traffic_light : msg->traffic_light) {
    legionclaw::interface::TrafficLight traffic_light_msg_traffic_light;
    traffic_light_msg_traffic_light.set_color(
        (legionclaw::common::TrafficLightColor)it_traffic_light.color);
    traffic_light_msg_traffic_light.set_id(it_traffic_light.id);
    traffic_light_msg_traffic_light.set_type(
        (legionclaw::common::TrafficLightType)it_traffic_light.type);
    traffic_light_msg_traffic_light.set_confidence(it_traffic_light.confidence);
    legionclaw::interface::ImageRect traffic_light_msg_traffic_light_light_rect;
    traffic_light_msg_traffic_light_light_rect.set_x(
        it_traffic_light.light_rect.x);
    traffic_light_msg_traffic_light_light_rect.set_y(
        it_traffic_light.light_rect.y);
    traffic_light_msg_traffic_light_light_rect.set_width(
        it_traffic_light.light_rect.width);
    traffic_light_msg_traffic_light_light_rect.set_height(
        it_traffic_light.light_rect.height);
    traffic_light_msg_traffic_light.set_light_rect(
        traffic_light_msg_traffic_light_light_rect);
    legionclaw::interface::Point3D traffic_light_msg_traffic_light_position;
    traffic_light_msg_traffic_light_position.set_x(it_traffic_light.position.x);
    traffic_light_msg_traffic_light_position.set_y(it_traffic_light.position.y);
    traffic_light_msg_traffic_light_position.set_z(it_traffic_light.position.z);
    traffic_light_msg_traffic_light.set_position(
        traffic_light_msg_traffic_light_position);
    traffic_light_msg_traffic_light.set_distance(it_traffic_light.distance);
    std::vector<int32_t> light_lanes;
    for (auto it_light_lanes : it_traffic_light.light_lanes) {
      int32_t light_lanes_item;
      light_lanes_item = it_light_lanes;
      light_lanes.emplace_back(light_lanes_item);
    }
    traffic_light_msg_traffic_light.set_light_lanes(&light_lanes);
    traffic_light_msg_traffic_light.set_tracking_time(
        it_traffic_light.tracking_time);
    traffic_light_msg_traffic_light.set_blink(it_traffic_light.blink);
    traffic_light_msg_traffic_light.set_blinking_time(
        it_traffic_light.blinking_time);
    traffic_light_msg_traffic_light.set_remaining_time(
        it_traffic_light.remaining_time);
    legionclaw::interface::Time traffic_light_msg_traffic_light_create_time;
    traffic_light_msg_traffic_light_create_time.set_sec(
        it_traffic_light.create_time.sec);
    traffic_light_msg_traffic_light_create_time.set_nsec(
        it_traffic_light.create_time.nsec);
    traffic_light_msg_traffic_light.set_create_time(
        traffic_light_msg_traffic_light_create_time);
    traffic_light.emplace_back(traffic_light_msg_traffic_light);
  }
  traffic_light_msg.set_traffic_light(&traffic_light);
  legionclaw::interface::TrafficLightDebug traffic_light_msg_traffic_light_debug;
  legionclaw::interface::TrafficLightBox
      traffic_light_msg_traffic_light_debug_cropbox;
  traffic_light_msg_traffic_light_debug_cropbox.set_x(
      msg->traffic_light_debug.cropbox.x);
  traffic_light_msg_traffic_light_debug_cropbox.set_y(
      msg->traffic_light_debug.cropbox.y);
  traffic_light_msg_traffic_light_debug_cropbox.set_width(
      msg->traffic_light_debug.cropbox.width);
  traffic_light_msg_traffic_light_debug_cropbox.set_height(
      msg->traffic_light_debug.cropbox.height);
  traffic_light_msg_traffic_light_debug_cropbox.set_color(
      (legionclaw::common::TrafficLightColor)
          msg->traffic_light_debug.cropbox.color);
  traffic_light_msg_traffic_light_debug_cropbox.set_selected(
      msg->traffic_light_debug.cropbox.selected);
  traffic_light_msg_traffic_light_debug_cropbox.set_camera_name(
      msg->traffic_light_debug.cropbox.camera_name);
  traffic_light_msg_traffic_light_debug.set_cropbox(
      traffic_light_msg_traffic_light_debug_cropbox);
  std::vector<legionclaw::interface::TrafficLightBox> box;
  for (auto it_box : msg->traffic_light_debug.box) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(it_box.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(it_box.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_box.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_box.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_box.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_box.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_box.camera_name);
    box.emplace_back(traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_box(&box);
  traffic_light_msg_traffic_light_debug.set_signal_num(
      msg->traffic_light_debug.signal_num);
  traffic_light_msg_traffic_light_debug.set_valid_pos(
      msg->traffic_light_debug.valid_pos);
  traffic_light_msg_traffic_light_debug.set_ts_diff_pos(
      msg->traffic_light_debug.ts_diff_pos);
  traffic_light_msg_traffic_light_debug.set_ts_diff_sys(
      msg->traffic_light_debug.ts_diff_sys);
  traffic_light_msg_traffic_light_debug.set_project_error(
      msg->traffic_light_debug.project_error);
  traffic_light_msg_traffic_light_debug.set_distance_to_stop_line(
      msg->traffic_light_debug.distance_to_stop_line);
  traffic_light_msg_traffic_light_debug.set_camera_id(
      msg->traffic_light_debug.camera_id);
  std::vector<legionclaw::interface::TrafficLightBox> crop_roi;
  for (auto it_crop_roi : msg->traffic_light_debug.crop_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_crop_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_crop_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_crop_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_crop_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_crop_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_crop_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_crop_roi.camera_name);
    crop_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_crop_roi(&crop_roi);
  std::vector<legionclaw::interface::TrafficLightBox> projected_roi;
  for (auto it_projected_roi : msg->traffic_light_debug.projected_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_projected_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_projected_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_projected_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_projected_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_projected_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_projected_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_projected_roi.camera_name);
    projected_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_projected_roi(&projected_roi);
  std::vector<legionclaw::interface::TrafficLightBox> rectified_roi;
  for (auto it_rectified_roi : msg->traffic_light_debug.rectified_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_rectified_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_rectified_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_rectified_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_rectified_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_rectified_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_rectified_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_rectified_roi.camera_name);
    rectified_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_rectified_roi(&rectified_roi);
  std::vector<legionclaw::interface::TrafficLightBox> debug_roi;
  for (auto it_debug_roi : msg->traffic_light_debug.debug_roi) {
    legionclaw::interface::TrafficLightBox
        traffic_light_msg_traffic_light_debug_traffic_light_box;
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_x(
        it_debug_roi.x);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_y(
        it_debug_roi.y);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_width(
        it_debug_roi.width);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_height(
        it_debug_roi.height);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_color(
        (legionclaw::common::TrafficLightColor)it_debug_roi.color);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_selected(
        it_debug_roi.selected);
    traffic_light_msg_traffic_light_debug_traffic_light_box.set_camera_name(
        it_debug_roi.camera_name);
    debug_roi.emplace_back(
        traffic_light_msg_traffic_light_debug_traffic_light_box);
  }
  traffic_light_msg_traffic_light_debug.set_debug_roi(&debug_roi);
  traffic_light_msg.set_traffic_light_debug(
      traffic_light_msg_traffic_light_debug);
  traffic_light_msg.set_contain_lights(msg->contain_lights);
  traffic_light_msg.set_camera_id(
      (legionclaw::interface::TrafficLightMsg::CameraID)msg->camera_id);
  traffic_light_msg.set_is_valid(msg->is_valid);

  instance_->HandleTrafficLightMsg(traffic_light_msg);
}

template <typename T>
void LcmMessageManager<T>::HandleLocationMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::Location* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::Location location;
  MESSAGE_HEADER_PARSER(location)
  legionclaw::interface::PointLLH location_position;
  location_position.set_lon(msg->position.lon);
  location_position.set_lat(msg->position.lat);
  location_position.set_height(msg->position.height);
  location.set_position(location_position);
  location.set_pitch(msg->pitch);
  location.set_roll(msg->roll);
  location.set_heading(msg->heading);
  legionclaw::interface::Point3D location_linear_velocity;
  location_linear_velocity.set_x(msg->linear_velocity.x);
  location_linear_velocity.set_y(msg->linear_velocity.y);
  location_linear_velocity.set_z(msg->linear_velocity.z);
  location.set_linear_velocity(location_linear_velocity);
  legionclaw::interface::Point3D location_linear_acceleration;
  location_linear_acceleration.set_x(msg->linear_acceleration.x);
  location_linear_acceleration.set_y(msg->linear_acceleration.y);
  location_linear_acceleration.set_z(msg->linear_acceleration.z);
  location.set_linear_acceleration(location_linear_acceleration);
  legionclaw::interface::Point3D location_angular_velocity;
  location_angular_velocity.set_x(msg->angular_velocity.x);
  location_angular_velocity.set_y(msg->angular_velocity.y);
  location_angular_velocity.set_z(msg->angular_velocity.z);
  location.set_angular_velocity(location_angular_velocity);
  location.set_rtk_flag((legionclaw::interface::Location::RTKFlag)msg->rtk_flag);
  location.set_odom_type((legionclaw::interface::Location::OdomType)msg->odom_type);
  location.set_auxiliary_type(
      (legionclaw::interface::Location::AuxiliaryType)msg->auxiliary_type);
  location.set_location_valid_flag(
      (legionclaw::common::IsValid)msg->location_valid_flag);
  location.set_origin_lat(msg->origin_lat);
  location.set_origin_lon(msg->origin_lon);
  legionclaw::interface::PointENU location_utm_position;
  location_utm_position.set_x(msg->utm_position.x);
  location_utm_position.set_y(msg->utm_position.y);
  location_utm_position.set_z(msg->utm_position.z);
  location.set_utm_position(location_utm_position);
  location.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  legionclaw::interface::PointENU location_utm_position_next;
  location_utm_position_next.set_x(msg->utm_position_next.x);
  location_utm_position_next.set_y(msg->utm_position_next.y);
  location_utm_position_next.set_z(msg->utm_position_next.z);
  location.set_utm_position_next(location_utm_position_next);
  legionclaw::interface::Point3D location_position_std_dev;
  location_position_std_dev.set_x(msg->position_std_dev.x);
  location_position_std_dev.set_y(msg->position_std_dev.y);
  location_position_std_dev.set_z(msg->position_std_dev.z);
  location.set_position_std_dev(location_position_std_dev);
  legionclaw::interface::Point3D location_orientation_std_dev;
  location_orientation_std_dev.set_x(msg->orientation_std_dev.x);
  location_orientation_std_dev.set_y(msg->orientation_std_dev.y);
  location_orientation_std_dev.set_z(msg->orientation_std_dev.z);
  location.set_orientation_std_dev(location_orientation_std_dev);
  legionclaw::interface::Point3D location_linear_velocity_std_dev;
  location_linear_velocity_std_dev.set_x(msg->linear_velocity_std_dev.x);
  location_linear_velocity_std_dev.set_y(msg->linear_velocity_std_dev.y);
  location_linear_velocity_std_dev.set_z(msg->linear_velocity_std_dev.z);
  location.set_linear_velocity_std_dev(location_linear_velocity_std_dev);
  legionclaw::interface::Point3D location_linear_acceleration_std_dev;
  location_linear_acceleration_std_dev.set_x(
      msg->linear_acceleration_std_dev.x);
  location_linear_acceleration_std_dev.set_y(
      msg->linear_acceleration_std_dev.y);
  location_linear_acceleration_std_dev.set_z(
      msg->linear_acceleration_std_dev.z);
  location.set_linear_acceleration_std_dev(
      location_linear_acceleration_std_dev);
  legionclaw::interface::Point3D location_angular_velocity_std_dev;
  location_angular_velocity_std_dev.set_x(msg->angular_velocity_std_dev.x);
  location_angular_velocity_std_dev.set_y(msg->angular_velocity_std_dev.y);
  location_angular_velocity_std_dev.set_z(msg->angular_velocity_std_dev.z);
  location.set_angular_velocity_std_dev(location_angular_velocity_std_dev);
  instance_->HandleLocation(location);
}

template <typename T>
void LcmMessageManager<T>::HandlePredictionObstaclesMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::PredictionObstacles* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::PredictionObstacles prediction_obstacles;
  MESSAGE_HEADER_PARSER(prediction_obstacles)
  std::vector<legionclaw::interface::PredictionObstacle>
      prediction_obstacles_vector;
  for (auto it_prediction_obstacles_vector : msg->prediction_obstacles) {
    legionclaw::interface::PredictionObstacle
        prediction_obstacles_prediction_obstacle;
    legionclaw::interface::PerceptionObstacle
        prediction_obstacles_prediction_obstacle_perception_obstacle;
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_id(
        it_prediction_obstacles_vector.perception_obstacle.id);
    legionclaw::interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_position;
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.set_x(
        it_prediction_obstacles_vector.perception_obstacle.position.x);
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.set_y(
        it_prediction_obstacles_vector.perception_obstacle.position.y);
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.set_z(
        it_prediction_obstacles_vector.perception_obstacle.position.z);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_position(
        prediction_obstacles_prediction_obstacle_perception_obstacle_position);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_theta(
        it_prediction_obstacles_vector.perception_obstacle.theta);
    legionclaw::interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_velocity;
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.set_x(
        it_prediction_obstacles_vector.perception_obstacle.velocity.x);
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.set_y(
        it_prediction_obstacles_vector.perception_obstacle.velocity.y);
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.set_z(
        it_prediction_obstacles_vector.perception_obstacle.velocity.z);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_velocity(
        prediction_obstacles_prediction_obstacle_perception_obstacle_velocity);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_length(
        it_prediction_obstacles_vector.perception_obstacle.length);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_width(
        it_prediction_obstacles_vector.perception_obstacle.width);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_height(
        it_prediction_obstacles_vector.perception_obstacle.height);
    std::vector<legionclaw::interface::Point3D> polygon_point;
    for (auto it_polygon_point :
         it_prediction_obstacles_vector.perception_obstacle.polygon_point) {
      legionclaw::interface::Point3D
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d;
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_x(it_polygon_point.x);
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_y(it_polygon_point.y);
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_z(it_polygon_point.z);
      polygon_point.emplace_back(
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_polygon_point(&polygon_point);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_tracking_time(
            it_prediction_obstacles_vector.perception_obstacle.tracking_time);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_type(
        (legionclaw::common::ObstacleType)
            it_prediction_obstacles_vector.perception_obstacle.type);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_lane_position(
            (legionclaw::common::LanePosition)it_prediction_obstacles_vector
                .perception_obstacle.lane_position);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_confidence(
        it_prediction_obstacles_vector.perception_obstacle.confidence);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_timestamp(
        it_prediction_obstacles_vector.perception_obstacle.timestamp);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_confidence_type(
            it_prediction_obstacles_vector.perception_obstacle.confidence_type);
    legionclaw::interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_drops;
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.set_x(
        it_prediction_obstacles_vector.perception_obstacle.drops.x);
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.set_y(
        it_prediction_obstacles_vector.perception_obstacle.drops.y);
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.set_z(
        it_prediction_obstacles_vector.perception_obstacle.drops.z);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_drops(
        prediction_obstacles_prediction_obstacle_perception_obstacle_drops);
    legionclaw::interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration;
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .set_x(
            it_prediction_obstacles_vector.perception_obstacle.acceleration.x);
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .set_y(
            it_prediction_obstacles_vector.perception_obstacle.acceleration.y);
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .set_z(
            it_prediction_obstacles_vector.perception_obstacle.acceleration.z);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_acceleration(
        prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration);
    legionclaw::interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point;
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .set_x(
            it_prediction_obstacles_vector.perception_obstacle.anchor_point.x);
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .set_y(
            it_prediction_obstacles_vector.perception_obstacle.anchor_point.y);
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .set_z(
            it_prediction_obstacles_vector.perception_obstacle.anchor_point.z);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_anchor_point(
        prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point);
    std::vector<legionclaw::interface::Point3D> bounding_box;
    for (auto it_bounding_box :
         it_prediction_obstacles_vector.perception_obstacle.bounding_box) {
      legionclaw::interface::Point3D
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d;
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_x(it_bounding_box.x);
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_y(it_bounding_box.y);
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d
          .set_z(it_bounding_box.z);
      bounding_box.emplace_back(
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_bounding_box(&bounding_box);
    prediction_obstacles_prediction_obstacle_perception_obstacle.set_sub_type(
        (legionclaw::common::ObstacleSubType)
            it_prediction_obstacles_vector.perception_obstacle.sub_type);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_height_above_ground(it_prediction_obstacles_vector
                                     .perception_obstacle.height_above_ground);
    std::vector<double> position_covariance;
    for (auto it_position_covariance :
         it_prediction_obstacles_vector.perception_obstacle
             .position_covariance) {
      double position_covariance_item;
      position_covariance_item = it_position_covariance;
      position_covariance.emplace_back(position_covariance_item);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_position_covariance(&position_covariance);
    std::vector<double> velocity_covariance;
    for (auto it_velocity_covariance :
         it_prediction_obstacles_vector.perception_obstacle
             .velocity_covariance) {
      double velocity_covariance_item;
      velocity_covariance_item = it_velocity_covariance;
      velocity_covariance.emplace_back(velocity_covariance_item);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_velocity_covariance(&velocity_covariance);
    std::vector<double> acceleration_covariance;
    for (auto it_acceleration_covariance :
         it_prediction_obstacles_vector.perception_obstacle
             .acceleration_covariance) {
      double acceleration_covariance_item;
      acceleration_covariance_item = it_acceleration_covariance;
      acceleration_covariance.emplace_back(acceleration_covariance_item);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_acceleration_covariance(&acceleration_covariance);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .set_light_status(
            it_prediction_obstacles_vector.perception_obstacle.light_status);
    prediction_obstacles_prediction_obstacle.set_perception_obstacle(
        prediction_obstacles_prediction_obstacle_perception_obstacle);
    prediction_obstacles_prediction_obstacle.set_timestamp(
        it_prediction_obstacles_vector.timestamp);
    prediction_obstacles_prediction_obstacle.set_predicted_period(
        it_prediction_obstacles_vector.predicted_period);
    std::vector<legionclaw::interface::TrajectoryInPrediction> trajectory;
    for (auto it_trajectory : it_prediction_obstacles_vector.trajectory) {
      legionclaw::interface::TrajectoryInPrediction
          prediction_obstacles_prediction_obstacle_trajectory_in_prediction;
      prediction_obstacles_prediction_obstacle_trajectory_in_prediction
          .set_probability(it_trajectory.probability);
      std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
      for (auto it_trajectory_points : it_trajectory.trajectory_points) {
        legionclaw::interface::TrajectoryPoint
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point;
        legionclaw::interface::PathPoint
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point;
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_x(it_trajectory_points.path_point.x);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_y(it_trajectory_points.path_point.y);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_z(it_trajectory_points.path_point.z);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_theta(it_trajectory_points.path_point.theta);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_kappa(it_trajectory_points.path_point.kappa);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_s(it_trajectory_points.path_point.s);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_dkappa(it_trajectory_points.path_point.dkappa);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_ddkappa(it_trajectory_points.path_point.ddkappa);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_lane_id(it_trajectory_points.path_point.lane_id);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_x_derivative(it_trajectory_points.path_point.x_derivative);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .set_y_derivative(it_trajectory_points.path_point.y_derivative);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_path_point(
                prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_v(it_trajectory_points.v);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_a(it_trajectory_points.a);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_relative_time(it_trajectory_points.relative_time);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_da(it_trajectory_points.da);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_is_steer_valid(it_trajectory_points.is_steer_valid);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_front_steer(it_trajectory_points.front_steer);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_rear_steer(it_trajectory_points.rear_steer);
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .set_gear((legionclaw::common::GearPosition)it_trajectory_points.gear);
        trajectory_points.emplace_back(
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point);
      }
      prediction_obstacles_prediction_obstacle_trajectory_in_prediction
          .set_trajectory_points(&trajectory_points);
      trajectory.emplace_back(
          prediction_obstacles_prediction_obstacle_trajectory_in_prediction);
    }
    prediction_obstacles_prediction_obstacle.set_trajectory(&trajectory);
    legionclaw::interface::ObstacleIntent
        prediction_obstacles_prediction_obstacle_intent;
    prediction_obstacles_prediction_obstacle_intent.set_type(
        (legionclaw::interface::ObstacleIntent::Type)
            it_prediction_obstacles_vector.intent.type);
    prediction_obstacles_prediction_obstacle.set_intent(
        prediction_obstacles_prediction_obstacle_intent);
    legionclaw::interface::ObstaclePriority
        prediction_obstacles_prediction_obstacle_priority;
    prediction_obstacles_prediction_obstacle_priority.set_priority(
        (legionclaw::interface::ObstaclePriority::Priority)
            it_prediction_obstacles_vector.priority.priority);
    prediction_obstacles_prediction_obstacle.set_priority(
        prediction_obstacles_prediction_obstacle_priority);
    legionclaw::interface::ObstacleInteractiveTag
        prediction_obstacles_prediction_obstacle_interactive_tag;
    prediction_obstacles_prediction_obstacle_interactive_tag
        .set_interactive_tag(
            (legionclaw::interface::ObstacleInteractiveTag::InteractiveTag)
                it_prediction_obstacles_vector.interactive_tag.interactive_tag);
    prediction_obstacles_prediction_obstacle.set_interactive_tag(
        prediction_obstacles_prediction_obstacle_interactive_tag);
    prediction_obstacles_prediction_obstacle.set_is_static(
        it_prediction_obstacles_vector.is_static);
    prediction_obstacles_vector.emplace_back(
        prediction_obstacles_prediction_obstacle);
  }
  prediction_obstacles.set_prediction_obstacles(&prediction_obstacles_vector);
  prediction_obstacles.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  prediction_obstacles.set_start_timestamp(msg->start_timestamp);
  prediction_obstacles.set_end_timestamp(msg->end_timestamp);
  prediction_obstacles.set_self_intent(
      (legionclaw::interface::PredictionObstacles::SelfIntent)msg->self_intent);
  prediction_obstacles.set_scenario(
      (legionclaw::interface::ADCTrajectory::Scenario)msg->scenario);

  instance_->HandlePredictionObstacles(prediction_obstacles);
}

template <typename T>
void LcmMessageManager<T>::HandleLaneListMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::LaneList* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::LaneList lane_list;
  MESSAGE_HEADER_PARSER(lane_list)
  lane_list.set_sensor_id((legionclaw::common::SensorID)msg->sensor_id);
  lane_list.set_error_code(msg->error_code);
  lane_list.set_sensor_status(msg->sensor_status);
  lane_list.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  lane_list.set_is_valid(msg->is_valid);
  legionclaw::interface::SensorCalibrator lane_list_sensor_calibrator;
  legionclaw::interface::Point3D lane_list_sensor_calibrator_pose;
  lane_list_sensor_calibrator_pose.set_x(msg->sensor_calibrator.pose.x);
  lane_list_sensor_calibrator_pose.set_y(msg->sensor_calibrator.pose.y);
  lane_list_sensor_calibrator_pose.set_z(msg->sensor_calibrator.pose.z);
  lane_list_sensor_calibrator.set_pose(lane_list_sensor_calibrator_pose);
  legionclaw::interface::Point3D lane_list_sensor_calibrator_angle;
  lane_list_sensor_calibrator_angle.set_x(msg->sensor_calibrator.angle.x);
  lane_list_sensor_calibrator_angle.set_y(msg->sensor_calibrator.angle.y);
  lane_list_sensor_calibrator_angle.set_z(msg->sensor_calibrator.angle.z);
  lane_list_sensor_calibrator.set_angle(lane_list_sensor_calibrator_angle);
  lane_list.set_sensor_calibrator(lane_list_sensor_calibrator);
  std::vector<legionclaw::interface::LaneLine> camera_laneline;
  for (auto it_camera_laneline : msg->camera_laneline) {
    legionclaw::interface::LaneLine lane_list_lane_line;
    lane_list_lane_line.set_lane_type(
        (legionclaw::interface::LaneLine::LaneType)it_camera_laneline.lane_type);
    lane_list_lane_line.set_lane_color(
        (legionclaw::interface::LaneLine::LaneColor)it_camera_laneline.lane_color);
    lane_list_lane_line.set_pos_type(
        (legionclaw::interface::LaneLine::PosType)it_camera_laneline.pos_type);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_vehicle;
    lane_list_lane_line_curve_vehicle.set_start_x(
        it_camera_laneline.curve_vehicle.start_x);
    lane_list_lane_line_curve_vehicle.set_end_x(
        it_camera_laneline.curve_vehicle.end_x);
    lane_list_lane_line_curve_vehicle.set_a(it_camera_laneline.curve_vehicle.a);
    lane_list_lane_line_curve_vehicle.set_b(it_camera_laneline.curve_vehicle.b);
    lane_list_lane_line_curve_vehicle.set_c(it_camera_laneline.curve_vehicle.c);
    lane_list_lane_line_curve_vehicle.set_d(it_camera_laneline.curve_vehicle.d);
    lane_list_lane_line.set_curve_vehicle(lane_list_lane_line_curve_vehicle);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_image;
    lane_list_lane_line_curve_image.set_start_x(
        it_camera_laneline.curve_image.start_x);
    lane_list_lane_line_curve_image.set_end_x(
        it_camera_laneline.curve_image.end_x);
    lane_list_lane_line_curve_image.set_a(it_camera_laneline.curve_image.a);
    lane_list_lane_line_curve_image.set_b(it_camera_laneline.curve_image.b);
    lane_list_lane_line_curve_image.set_c(it_camera_laneline.curve_image.c);
    lane_list_lane_line_curve_image.set_d(it_camera_laneline.curve_image.d);
    lane_list_lane_line.set_curve_image(lane_list_lane_line_curve_image);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_abs;
    lane_list_lane_line_curve_abs.set_start_x(
        it_camera_laneline.curve_abs.start_x);
    lane_list_lane_line_curve_abs.set_end_x(it_camera_laneline.curve_abs.end_x);
    lane_list_lane_line_curve_abs.set_a(it_camera_laneline.curve_abs.a);
    lane_list_lane_line_curve_abs.set_b(it_camera_laneline.curve_abs.b);
    lane_list_lane_line_curve_abs.set_c(it_camera_laneline.curve_abs.c);
    lane_list_lane_line_curve_abs.set_d(it_camera_laneline.curve_abs.d);
    lane_list_lane_line.set_curve_abs(lane_list_lane_line_curve_abs);
    std::vector<legionclaw::interface::Point3D> pts_vehicle;
    for (auto it_pts_vehicle : it_camera_laneline.pts_vehicle) {
      legionclaw::interface::Point3D lane_list_lane_line_point_3d;
      lane_list_lane_line_point_3d.set_x(it_pts_vehicle.x);
      lane_list_lane_line_point_3d.set_y(it_pts_vehicle.y);
      lane_list_lane_line_point_3d.set_z(it_pts_vehicle.z);
      pts_vehicle.emplace_back(lane_list_lane_line_point_3d);
    }
    lane_list_lane_line.set_pts_vehicle(&pts_vehicle);
    std::vector<legionclaw::interface::Point2D> pts_image;
    for (auto it_pts_image : it_camera_laneline.pts_image) {
      legionclaw::interface::Point2D lane_list_lane_line_point_2d;
      lane_list_lane_line_point_2d.set_x(it_pts_image.x);
      lane_list_lane_line_point_2d.set_y(it_pts_image.y);
      pts_image.emplace_back(lane_list_lane_line_point_2d);
    }
    lane_list_lane_line.set_pts_image(&pts_image);
    std::vector<legionclaw::interface::Point3D> pts_abs;
    for (auto it_pts_abs : it_camera_laneline.pts_abs) {
      legionclaw::interface::Point3D lane_list_lane_line_point_3d;
      lane_list_lane_line_point_3d.set_x(it_pts_abs.x);
      lane_list_lane_line_point_3d.set_y(it_pts_abs.y);
      lane_list_lane_line_point_3d.set_z(it_pts_abs.z);
      pts_abs.emplace_back(lane_list_lane_line_point_3d);
    }
    lane_list_lane_line.set_pts_abs(&pts_abs);
    legionclaw::interface::EndPoints lane_list_lane_line_image_end_point;
    legionclaw::interface::Point2D lane_list_lane_line_image_end_point_start;
    lane_list_lane_line_image_end_point_start.set_x(
        it_camera_laneline.image_end_point.start.x);
    lane_list_lane_line_image_end_point_start.set_y(
        it_camera_laneline.image_end_point.start.y);
    lane_list_lane_line_image_end_point.set_start(
        lane_list_lane_line_image_end_point_start);
    legionclaw::interface::Point2D lane_list_lane_line_image_end_point_end;
    lane_list_lane_line_image_end_point_end.set_x(
        it_camera_laneline.image_end_point.end.x);
    lane_list_lane_line_image_end_point_end.set_y(
        it_camera_laneline.image_end_point.end.y);
    lane_list_lane_line_image_end_point.set_end(
        lane_list_lane_line_image_end_point_end);
    lane_list_lane_line.set_image_end_point(
        lane_list_lane_line_image_end_point);
    std::vector<legionclaw::interface::Point2D> pts_key;
    for (auto it_pts_key : it_camera_laneline.pts_key) {
      legionclaw::interface::Point2D lane_list_lane_line_point_2d;
      lane_list_lane_line_point_2d.set_x(it_pts_key.x);
      lane_list_lane_line_point_2d.set_y(it_pts_key.y);
      pts_key.emplace_back(lane_list_lane_line_point_2d);
    }
    lane_list_lane_line.set_pts_key(&pts_key);
    lane_list_lane_line.set_hd_lane_id(it_camera_laneline.hd_lane_id);
    lane_list_lane_line.set_confidence(it_camera_laneline.confidence);
    lane_list_lane_line.set_lane_quality(
        (legionclaw::interface::LaneLine::LaneQuality)
            it_camera_laneline.lane_quality);
    lane_list_lane_line.set_fused_lane_type(
        (legionclaw::interface::LaneLine::FusedLaneType)
            it_camera_laneline.fused_lane_type);
    std::vector<double> homography_mat;
    for (auto it_homography_mat : it_camera_laneline.homography_mat) {
      double homography_mat_item;
      homography_mat_item = it_homography_mat;
      homography_mat.emplace_back(homography_mat_item);
    }
    lane_list_lane_line.set_homography_mat(&homography_mat);
    std::vector<double> homography_mat_inv;
    for (auto it_homography_mat_inv : it_camera_laneline.homography_mat_inv) {
      double homography_mat_inv_item;
      homography_mat_inv_item = it_homography_mat_inv;
      homography_mat_inv.emplace_back(homography_mat_inv_item);
    }
    lane_list_lane_line.set_homography_mat_inv(&homography_mat_inv);
    lane_list_lane_line.set_lane_coordinate_type(
        (legionclaw::interface::LaneLine::LaneCoordinateType)
            it_camera_laneline.lane_coordinate_type);
    lane_list_lane_line.set_use_type(
        (legionclaw::interface::LaneLine::UseType)it_camera_laneline.use_type);
    legionclaw::interface::Time lane_list_lane_line_create_time;
    lane_list_lane_line_create_time.set_sec(it_camera_laneline.create_time.sec);
    lane_list_lane_line_create_time.set_nsec(
        it_camera_laneline.create_time.nsec);
    lane_list_lane_line.set_create_time(lane_list_lane_line_create_time);
    camera_laneline.emplace_back(lane_list_lane_line);
  }
  lane_list.set_camera_laneline(&camera_laneline);
  legionclaw::interface::HolisticPathPrediction lane_list_hpp;
  legionclaw::interface::LaneLineCubicCurve lane_list_hpp_hpp;
  lane_list_hpp_hpp.set_start_x(msg->hpp.hpp.start_x);
  lane_list_hpp_hpp.set_end_x(msg->hpp.hpp.end_x);
  lane_list_hpp_hpp.set_a(msg->hpp.hpp.a);
  lane_list_hpp_hpp.set_b(msg->hpp.hpp.b);
  lane_list_hpp_hpp.set_c(msg->hpp.hpp.c);
  lane_list_hpp_hpp.set_d(msg->hpp.hpp.d);
  lane_list_hpp.set_hpp(lane_list_hpp_hpp);
  lane_list_hpp.set_planning_source(
      (legionclaw::interface::HolisticPathPrediction::PlanningSource)
          msg->hpp.planning_source);
  lane_list_hpp.set_ego_lane_width(msg->hpp.ego_lane_width);
  lane_list_hpp.set_confidence(msg->hpp.confidence);
  lane_list.set_hpp(lane_list_hpp);
  std::vector<legionclaw::interface::RoadMark> road_marks;
  for (auto it_road_marks : msg->road_marks) {
    legionclaw::interface::RoadMark lane_list_road_mark;
    lane_list_road_mark.set_longitude_dist(it_road_marks.longitude_dist);
    lane_list_road_mark.set_lateral_dist(it_road_marks.lateral_dist);
    lane_list_road_mark.set_x(it_road_marks.x);
    lane_list_road_mark.set_y(it_road_marks.y);
    lane_list_road_mark.set_z(it_road_marks.z);
    lane_list_road_mark.set_confidence(it_road_marks.confidence);
    lane_list_road_mark.set_type(
        (legionclaw::common::RoadMarkType)it_road_marks.type);
    road_marks.emplace_back(lane_list_road_mark);
  }
  lane_list.set_road_marks(&road_marks);

  instance_->HandleLaneList(lane_list);
}

template <typename T>
void LcmMessageManager<T>::HandleChassisMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::Chassis* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::Chassis chassis;
  MESSAGE_HEADER_PARSER(chassis)
  chassis.set_moving_status((legionclaw::common::MovingStatus)msg->moving_status);
  chassis.set_driving_mode((legionclaw::common::DrivingMode)msg->driving_mode);
  chassis.set_steer_driving_mode(
      (legionclaw::common::DrivingMode)msg->steer_driving_mode);
  chassis.set_steering_status(
      (legionclaw::common::ControlStatus)msg->steering_status);
  chassis.set_front_steering_value(msg->front_steering_value);
  chassis.set_rear_steering_value(msg->rear_steering_value);
  chassis.set_steering_torque_nm(msg->steering_torque_nm);
  chassis.set_front_steering_rate_dps(msg->front_steering_rate_dps);
  chassis.set_rear_steering_rate_dps(msg->rear_steering_rate_dps);
  chassis.set_accel_driving_mode(
      (legionclaw::common::DrivingMode)msg->accel_driving_mode);
  chassis.set_accel_status((legionclaw::common::ControlStatus)msg->accel_status);
  chassis.set_accel_value(msg->accel_value);
  chassis.set_brake_driving_mode(
      (legionclaw::common::DrivingMode)msg->brake_driving_mode);
  chassis.set_brake_status((legionclaw::common::ControlStatus)msg->brake_status);
  chassis.set_brake_value(msg->brake_value);
  chassis.set_backup_brake_driving_mode(
      (legionclaw::common::DrivingMode)msg->backup_brake_driving_mode);
  chassis.set_backup_brake_status(
      (legionclaw::common::ControlStatus)msg->backup_brake_status);
  chassis.set_backup_brake_value(msg->backup_brake_value);
  chassis.set_epb_driving_mode(
      (legionclaw::common::DrivingMode)msg->epb_driving_mode);
  chassis.set_epb_status((legionclaw::common::ControlStatus)msg->epb_status);
  chassis.set_epb_level((legionclaw::common::EPBLevel)msg->epb_level);
  chassis.set_engine_status((legionclaw::common::EngineStauts)msg->engine_status);
  chassis.set_engine_rpm(msg->engine_rpm);
  chassis.set_engine_torque(msg->engine_torque);
  chassis.set_speed_mps(msg->speed_mps);
  chassis.set_odometer_m(msg->odometer_m);
  chassis.set_fuel_range_m(msg->fuel_range_m);
  chassis.set_gear_driving_mode(
      (legionclaw::common::DrivingMode)msg->gear_driving_mode);
  chassis.set_gear_status((legionclaw::common::ControlStatus)msg->gear_status);
  chassis.set_gear_location((legionclaw::common::GearPosition)msg->gear_location);
  chassis.set_driver_seat_belt(
      (legionclaw::common::SwitchStatus)msg->driver_seat_belt);
  chassis.set_high_beam_status(
      (legionclaw::common::SwitchStatus)msg->high_beam_status);
  chassis.set_low_beam_status(
      (legionclaw::common::SwitchStatus)msg->low_beam_status);
  chassis.set_horn_status((legionclaw::common::SwitchStatus)msg->horn_status);
  chassis.set_turn_lamp_status(
      (legionclaw::common::TurnSignal)msg->turn_lamp_status);
  chassis.set_front_wiper_status(
      (legionclaw::common::SwitchStatus)msg->front_wiper_status);
  chassis.set_rear_wiper_status(
      (legionclaw::common::SwitchStatus)msg->rear_wiper_status);
  chassis.set_position_lamp_status(
      (legionclaw::common::SwitchStatus)msg->position_lamp_status);
  chassis.set_front_fog_lamp_status(
      (legionclaw::common::SwitchStatus)msg->front_fog_lamp_status);
  chassis.set_rear_fog_lamp_status(
      (legionclaw::common::SwitchStatus)msg->rear_fog_lamp_status);
  chassis.set_brake_lamp_status(
      (legionclaw::common::SwitchStatus)msg->brake_lamp_status);
  chassis.set_alarm_lamp_status(
      (legionclaw::common::SwitchStatus)msg->alarm_lamp_status);
  chassis.set_lf_door_status((legionclaw::common::DoorStatus)msg->lf_door_status);
  chassis.set_rf_door_status((legionclaw::common::DoorStatus)msg->rf_door_status);
  chassis.set_lr_door_status((legionclaw::common::DoorStatus)msg->lr_door_status);
  chassis.set_rr_door_status((legionclaw::common::DoorStatus)msg->rr_door_status);
  chassis.set_rearview_mirror_status(
      (legionclaw::common::FoldUnfoldStatus)msg->rearview_mirror_status);
  chassis.set_trunk_status((legionclaw::common::DoorStatus)msg->trunk_status);
  chassis.set_engine_bay_door_status(
      (legionclaw::common::DoorStatus)msg->engine_bay_door_status);
  chassis.set_wheel_direction_rr(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_rr);
  chassis.set_wheel_spd_rr(msg->wheel_spd_rr);
  chassis.set_wheel_direction_rl(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_rl);
  chassis.set_wheel_spd_rl(msg->wheel_spd_rl);
  chassis.set_wheel_direction_fr(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_fr);
  chassis.set_wheel_spd_fr(msg->wheel_spd_fr);
  chassis.set_wheel_direction_fl(
      (legionclaw::common::WheelSpeedType)msg->wheel_direction_fl);
  chassis.set_wheel_spd_fl(msg->wheel_spd_fl);
  chassis.set_is_tire_pressure_ok(
      (legionclaw::common::FailureStatus)msg->is_tire_pressure_ok);
  chassis.set_is_tire_pressure_lf_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_lf_valid);
  chassis.set_tire_pressure_lf(msg->tire_pressure_lf);
  chassis.set_is_tire_pressure_rf_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_rf_valid);
  chassis.set_tire_pressure_rf(msg->tire_pressure_rf);
  chassis.set_is_tire_pressure_lr_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_lr_valid);
  chassis.set_tire_pressure_lr(msg->tire_pressure_lr);
  chassis.set_is_tire_pressure_rr_valid(
      (legionclaw::common::IsValid)msg->is_tire_pressure_rr_valid);
  chassis.set_tire_pressure_rr(msg->tire_pressure_rr);
  chassis.set_battery_power_percentage(msg->battery_power_percentage);
  chassis.set_air_bag_status(
      (legionclaw::common::FailureStatus)msg->air_bag_status);
  chassis.set_charging_gun_status(
      (legionclaw::common::PlugStatus)msg->charging_gun_status);
  chassis.set_vehicle_power_status(
      (legionclaw::common::FailureStatus)msg->vehicle_power_status);
  std::vector<legionclaw::interface::Chassis::ErrorCode> chassis_error_code;
  for (auto it_chassis_error_code : msg->chassis_error_code) {
    legionclaw::interface::Chassis::ErrorCode error_code;
    error_code = (legionclaw::interface::Chassis::ErrorCode)it_chassis_error_code;
    chassis_error_code.emplace_back(error_code);
  }
  chassis.set_chassis_error_code(&chassis_error_code);

  instance_->HandleChassis(chassis);
}

template <typename T>
void LcmMessageManager<T>::HandleSotifMonitorResultMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::SotifMonitorResult* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::SotifMonitorResult sotif_monitor_result;
  MESSAGE_HEADER_PARSER(sotif_monitor_result)
  std::vector<legionclaw::interface::Region> region_value;
  for (auto it_region_value : msg->region_value) {
    legionclaw::interface::Region sotif_monitor_result_region;
    sotif_monitor_result_region.set_name_region(
        (legionclaw::interface::Region::RegionID)it_region_value.name_region);
    sotif_monitor_result_region.set_score(it_region_value.score);
    sotif_monitor_result_region.set_rank_risk(
        (legionclaw::interface::Region::RankRisk)it_region_value.rank_risk);
    std::vector<legionclaw::interface::Point3D> region_polygon;
    for (auto it_region_polygon : it_region_value.region_polygon) {
      legionclaw::interface::Point3D sotif_monitor_result_region_point_3d;
      sotif_monitor_result_region_point_3d.set_x(it_region_polygon.x);
      sotif_monitor_result_region_point_3d.set_y(it_region_polygon.y);
      sotif_monitor_result_region_point_3d.set_z(it_region_polygon.z);
      region_polygon.emplace_back(sotif_monitor_result_region_point_3d);
    }
    sotif_monitor_result_region.set_region_polygon(&region_polygon);
    region_value.emplace_back(sotif_monitor_result_region);
  }
  sotif_monitor_result.set_region_value(&region_value);
  std::vector<legionclaw::interface::Grid> grid_map;
  for (auto it_grid_map : msg->grid_map) {
    legionclaw::interface::Grid sotif_monitor_result_grid;
    sotif_monitor_result_grid.set_x(it_grid_map.x);
    sotif_monitor_result_grid.set_y(it_grid_map.y);
    legionclaw::interface::SLPoint sotif_monitor_result_grid_sl_point;
    sotif_monitor_result_grid_sl_point.set_s(it_grid_map.sl_point.s);
    sotif_monitor_result_grid_sl_point.set_l(it_grid_map.sl_point.l);
    sotif_monitor_result_grid.set_sl_point(sotif_monitor_result_grid_sl_point);
    sotif_monitor_result_grid.set_yaw(it_grid_map.yaw);
    sotif_monitor_result_grid.set_potential(it_grid_map.potential);
    sotif_monitor_result_grid.set_region_id(
        (legionclaw::interface::Region::RegionID)it_grid_map.region_id);
    grid_map.emplace_back(sotif_monitor_result_grid);
  }
  sotif_monitor_result.set_grid_map(&grid_map);

  instance_->HandleSotifMonitorResult(sotif_monitor_result);
}

template <typename T>
void LcmMessageManager<T>::HandleObuCmdMsgMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ObuCmdMsg* msg) {
  if (is_init_ == false)
    return;

  legionclaw::interface::ObuCmdMsg obu_cmd_msg;
  MESSAGE_HEADER_PARSER(obu_cmd_msg)
  obu_cmd_msg.set_id(msg->id);
  obu_cmd_msg.set_name(msg->name);
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  for (auto it_obu_cmd_list : msg->obu_cmd_list) {
    legionclaw::interface::ObuCmd obu_cmd_msg_obu_cmd;
    obu_cmd_msg_obu_cmd.set_code(it_obu_cmd_list.code);
    obu_cmd_msg_obu_cmd.set_val(it_obu_cmd_list.val);
    obu_cmd_list.emplace_back(obu_cmd_msg_obu_cmd);
  }
  obu_cmd_msg.set_obu_cmd_list(&obu_cmd_list);

  instance_->HandleObuCmdMsg(obu_cmd_msg);
}

template <typename T>
void LcmMessageManager<T>::HandleDrivableRegionMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::DrivableRegion* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::DrivableRegion drivable_region;
  MESSAGE_HEADER_PARSER(drivable_region)
  std::vector<legionclaw::interface::Polygon3D> drivable_region_vector;
  for (auto it_drivable_region_vector : msg->drivable_region) {
    legionclaw::interface::Polygon3D drivable_region_polygon_3d;
    drivable_region_polygon_3d.set_coordinate_system(
        (legionclaw::common::CoordinateSystem)
            it_drivable_region_vector.coordinate_system);
    std::vector<legionclaw::interface::Point3D> points;
    for (auto it_points : it_drivable_region_vector.points) {
      legionclaw::interface::Point3D drivable_region_polygon_3d_point_3d;
      drivable_region_polygon_3d_point_3d.set_x(it_points.x);
      drivable_region_polygon_3d_point_3d.set_y(it_points.y);
      drivable_region_polygon_3d_point_3d.set_z(it_points.z);
      points.emplace_back(drivable_region_polygon_3d_point_3d);
    }
    drivable_region_polygon_3d.set_points(&points);
    drivable_region_vector.emplace_back(drivable_region_polygon_3d);
  }
  drivable_region.set_drivable_region(&drivable_region_vector);

  instance_->HandleDrivableRegion(drivable_region);
}

template <typename T>
void LcmMessageManager<T>::HandleParkingOutInfoMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ParkingOutInfo* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::ParkingOutInfo parking_out_info;
  MESSAGE_HEADER_PARSER(parking_out_info)
  parking_out_info.set_parking_out_id(msg->parking_out_id);
  parking_out_info.set_parking_direction_type(
      (legionclaw::common::Direction)msg->parking_direction_type);
  parking_out_info.set_is_parking_out_enable(msg->is_parking_out_enable);
  legionclaw::interface::Point3D parking_out_info_parking_out_point;
  parking_out_info_parking_out_point.set_x(msg->parking_out_point.x);
  parking_out_info_parking_out_point.set_y(msg->parking_out_point.y);
  parking_out_info_parking_out_point.set_z(msg->parking_out_point.z);
  parking_out_info.set_parking_out_point(parking_out_info_parking_out_point);
  parking_out_info.set_theta(msg->theta);

  instance_->HandleParkingOutInfo(parking_out_info);
}

template <typename T>
void LcmMessageManager<T>::HandleGuideInfoMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::GuideInfo* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::GuideInfo guide_info;
  MESSAGE_HEADER_PARSER(guide_info)
  guide_info.set_next_dis(msg->next_dis);
  legionclaw::interface::GuideRoad guide_info_current_road;
  guide_info_current_road.set_road_id(msg->current_road.road_id);
  guide_info_current_road.set_road_type(msg->current_road.road_type);
  guide_info_current_road.set_turn_type(msg->current_road.turn_type);
  guide_info_current_road.set_avg_curvature(msg->current_road.avg_curvature);
//   guide_info_current_road.set_curvature_size(msg->current_road.curvature_size);
  std::vector<legionclaw::interface::CurvatureInfo> curvature;
  for (auto it_curvature : msg->current_road.curvature) {
    legionclaw::interface::CurvatureInfo guide_info_current_road_curvature_info;
    guide_info_current_road_curvature_info.set_offset(it_curvature.offset);
    guide_info_current_road_curvature_info.set_value(it_curvature.value);
    curvature.emplace_back(guide_info_current_road_curvature_info);
  }
  guide_info_current_road.set_curvature(&curvature);
  guide_info.set_current_road(guide_info_current_road);
  legionclaw::interface::GuideRoad guide_info_next_road;
  guide_info_next_road.set_road_id(msg->next_road.road_id);
  guide_info_next_road.set_road_type(msg->next_road.road_type);
  guide_info_next_road.set_turn_type(msg->next_road.turn_type);
  guide_info_next_road.set_avg_curvature(msg->next_road.avg_curvature);
//   guide_info_next_road.set_curvature_size(msg->next_road.curvature_size);
  std::vector<legionclaw::interface::CurvatureInfo> curvature_1;
  for (auto it_curvature_1 : msg->next_road.curvature) {
    legionclaw::interface::CurvatureInfo guide_info_next_road_curvature_info;
    guide_info_next_road_curvature_info.set_offset(it_curvature_1.offset);
    guide_info_next_road_curvature_info.set_value(it_curvature_1.value);
    curvature_1.emplace_back(guide_info_next_road_curvature_info);
  }
  guide_info_next_road.set_curvature(&curvature_1);
  guide_info.set_next_road(guide_info_next_road);
  guide_info.set_round_status(msg->round_status);
  guide_info.set_intersection_status(msg->intersection_status);
  guide_info.set_roads_status(msg->roads_status);

  instance_->HandleGuideInfo(guide_info);
}

template <typename T>
void LcmMessageManager<T>::HandleTrafficEventsMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::TrafficEvents* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::TrafficEvents traffic_events;
  MESSAGE_HEADER_PARSER(traffic_events)
  legionclaw::interface::RouteFusionInfo traffic_events_route_fusion_info;
  traffic_events_route_fusion_info.set_fusion_flag(
      (legionclaw::common::IsValid)msg->route_fusion_info.fusion_flag);
  traffic_events_route_fusion_info.set_fusion_reason(
      msg->route_fusion_info.fusion_reason);
  traffic_events.set_route_fusion_info(traffic_events_route_fusion_info);
  legionclaw::interface::JunctionInfo traffic_events_junction_info;
  traffic_events_junction_info.set_id(msg->junction_info.id);
  traffic_events_junction_info.set_light_flag(
      (legionclaw::common::IsValid)msg->junction_info.light_flag);
  traffic_events_junction_info.set_light_color(
      (legionclaw::common::TrafficLightColor)msg->junction_info.light_color);
  traffic_events_junction_info.set_light_remain_time(
      msg->junction_info.light_remain_time);
  traffic_events_junction_info.set_distance_to_stop(
      msg->junction_info.distance_to_stop);
  traffic_events_junction_info.set_direction_flag(
      (legionclaw::common::IsValid)msg->junction_info.direction_flag);
  traffic_events_junction_info.set_direction(
      (legionclaw::common::Direction)msg->junction_info.direction);
  traffic_events_junction_info.set_distance_to_junction(
      msg->junction_info.distance_to_junction);
  std::vector<legionclaw::interface::Point3D> stop_line;
  for (auto it_stop_line : msg->junction_info.stop_line) {
    legionclaw::interface::Point3D traffic_events_junction_info_point_3d;
    traffic_events_junction_info_point_3d.set_x(it_stop_line.x);
    traffic_events_junction_info_point_3d.set_y(it_stop_line.y);
    traffic_events_junction_info_point_3d.set_z(it_stop_line.z);
    stop_line.emplace_back(traffic_events_junction_info_point_3d);
  }
  traffic_events_junction_info.set_stop_line(&stop_line);
  traffic_events.set_junction_info(traffic_events_junction_info);
  legionclaw::interface::LimitSpeedInfo traffic_events_limit_speed_info;
  traffic_events_limit_speed_info.set_limitspeed_valid_flag(
      (legionclaw::common::IsValid)msg->limit_speed_info.limitspeed_valid_flag);
  traffic_events_limit_speed_info.set_limit_speed(
      msg->limit_speed_info.limit_speed);
  traffic_events_limit_speed_info.set_limit_distance(
      msg->limit_speed_info.limit_distance);
  traffic_events.set_limit_speed_info(traffic_events_limit_speed_info);
  instance_->HandleTrafficEvents(traffic_events);
}

template <typename T> void LcmMessageManager<T>::Run() {
  while (0 == lcm_->handle())
    ;
}

template <typename T> void LcmMessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace planning
} // namespace legionclaw
#endif
