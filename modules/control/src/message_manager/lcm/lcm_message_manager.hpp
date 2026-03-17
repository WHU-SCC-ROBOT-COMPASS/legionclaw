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
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
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

template <typename T>
bool LcmMessageManager<T>::Activate()
{
    if(is_active_)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    ADCTrajectory_sub_ = lcm_->subscribe("/planning/ADCTrajectory",
                  &LcmMessageManager::HandleADCTrajectoryMessage, this);

    chassis_sub_ = lcm_->subscribe("/drivers/canbus/Chassis",
                  &LcmMessageManager::HandleChassisMessage, this);

    location_sub_ = lcm_->subscribe("/localization/global_fusion/Location",
                  &LcmMessageManager::HandleLocationMessage, this);

    PlanningCmd_sub_ = lcm_->subscribe("/planning/PlanningCmd",
                  &LcmMessageManager::HandlePlanningCmdMessage, this);

    is_active_ = true;
    return true;
}

template <typename T>
bool LcmMessageManager<T>::DeActivate()
{
    if(is_active_==false)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    lcm_->unsubscribe(ADCTrajectory_sub_);
    lcm_->unsubscribe(location_sub_);
    lcm_->unsubscribe(PlanningCmd_sub_);
    lcm_->unsubscribe(chassis_sub_);
    is_active_ = false;
    std::cout<<"lcm deactivate"<<std::endl;
    return true;
}

template <typename T>
void LcmMessageManager<T>::PublishControlCommand(
    legionclaw::interface::ControlCommand msg) {
  if (is_init_ == false)
    return;
  lcm_interface::ControlCommand control_command;
  MESSAGE_HEADER_ASSIGN(lcm_interface, control_command)
  control_command.steer_driving_mode = msg.steer_driving_mode();
  control_command.front_steering_target = msg.front_steering_target();
  control_command.rear_steering_target = msg.rear_steering_target();
  control_command.front_steering_rate = msg.front_steering_rate();
  control_command.rear_steering_rate = msg.rear_steering_rate();
  control_command.accel_driving_mode = msg.accel_driving_mode();
  control_command.accel_value = msg.accel_value();
  control_command.brake_driving_mode = msg.brake_driving_mode();
  control_command.brake_value = msg.brake_value();
  control_command.backup_brake_driving_mode = msg.backup_brake_driving_mode();
  control_command.backup_brake_value = msg.backup_brake_value();
  control_command.epb_driving_mode = msg.epb_driving_mode();
  control_command.epb_level = msg.epb_level();
  control_command.gear_driving_mode = msg.gear_driving_mode();
  control_command.emergency_brake_enable = msg.emergency_brake_enable();
  control_command.gear_location = msg.gear_location();
  control_command.speed = msg.speed();
  control_command.acceleration = msg.acceleration();
  control_command.turn_lamp_ctrl = msg.turn_lamp_ctrl();
  control_command.high_beam_ctrl = msg.high_beam_ctrl();
  control_command.low_beam_ctrl = msg.low_beam_ctrl();
  control_command.horn_ctrl = msg.horn_ctrl();
  control_command.front_wiper_ctrl = msg.front_wiper_ctrl();
  control_command.rear_wiper_ctrl = msg.rear_wiper_ctrl();
  control_command.position_lamp_ctrl = msg.position_lamp_ctrl();
  control_command.front_fog_lamp_ctrl = msg.front_fog_lamp_ctrl();
  control_command.rear_fog_lamp_ctrl = msg.rear_fog_lamp_ctrl();
  control_command.brake_lamp_ctrl = msg.brake_lamp_ctrl();
  control_command.alarm_lamp_ctrl = msg.alarm_lamp_ctrl();
  control_command.lf_door_ctrl = msg.lf_door_ctrl();
  control_command.rf_door_ctrl = msg.rf_door_ctrl();
  control_command.lr_door_ctrl = msg.lr_door_ctrl();
  control_command.rr_door_ctrl = msg.rr_door_ctrl();

  lcm_->publish("/control/ControlCommand", &control_command);
}

template <typename T>
void LcmMessageManager<T>::PublishControlAnalysis(
    legionclaw::interface::ControlAnalysis msg) {
  if (is_init_ == false)
    return;
  lcm_interface::ControlAnalysis control_analysis;
  MESSAGE_HEADER_ASSIGN(lcm_interface, control_analysis)
  control_analysis.driving_mode = msg.driving_mode();
  control_analysis.driving_mode_fd = msg.driving_mode_fd();
  control_analysis.gear_location_fd = msg.gear_location_fd();
  control_analysis.gear_location_cmd = msg.gear_location_cmd();
  control_analysis.epb_level_fd = msg.epb_level_fd();
  control_analysis.epb_level_cmd = msg.epb_level_cmd();
  control_analysis.speed_mps = msg.speed_mps();
  control_analysis.speed_reference = msg.speed_reference();
  control_analysis.accel_value_fd = msg.accel_value_fd();
  control_analysis.accel_value_cmd = msg.accel_value_cmd();
  control_analysis.brake_value_fd = msg.brake_value_fd();
  control_analysis.brake_value_cmd = msg.brake_value_cmd();
  control_analysis.path_remain = msg.path_remain();
  control_analysis.has_stop_point = msg.has_stop_point();
  control_analysis.is_full_stop = msg.is_full_stop();
  control_analysis.is_stopped = msg.is_stopped();
  control_analysis.lon_acc_jerk = msg.lon_acc_jerk();
  control_analysis.acceleration_cmd = msg.acceleration_cmd();
  control_analysis.acceleration_cmd_closeloop =
      msg.acceleration_cmd_closeloop();
  control_analysis.preview_acceleration_reference =
      msg.preview_acceleration_reference();
  control_analysis.slope_offset_compensation = msg.slope_offset_compensation();
  control_analysis.turning_offset_compensation =
      msg.turning_offset_compensation();
  control_analysis.speed_error_limited = msg.speed_error_limited();
  control_analysis.speed_error = msg.speed_error();
  control_analysis.speed_offset = msg.speed_offset();
  control_analysis.station_error_limited = msg.station_error_limited();
  control_analysis.station_error = msg.station_error();
  control_analysis.lon_target_point_s = msg.lon_target_point_s();
  control_analysis.lon_calculate_time = msg.lon_calculate_time();
  control_analysis.lon_calculate_time_max = msg.lon_calculate_time_max();
  control_analysis.ref_curvature = msg.ref_curvature();
  control_analysis.ref_heading = msg.ref_heading();
  control_analysis.current_heading = msg.current_heading();
  control_analysis.heading_error = msg.heading_error();
  control_analysis.heading_error_rate = msg.heading_error_rate();
  control_analysis.lateral_error = msg.lateral_error();
  control_analysis.lateral_error_rate = msg.lateral_error_rate();
  control_analysis.lon_error = msg.lon_error();
  control_analysis.front_steering_value_fd = msg.front_steering_value_fd();
  control_analysis.front_steering_target = msg.front_steering_target();
  control_analysis.front_steering_rate = msg.front_steering_rate();
  control_analysis.front_steer_angle_feedforward =
      msg.front_steer_angle_feedforward();
  control_analysis.front_steer_angle_feedback =
      msg.front_steer_angle_feedback();
  control_analysis.front_steer_angle_lateral_contribution =
      msg.front_steer_angle_lateral_contribution();
  control_analysis.front_steer_angle_lateral_rate_contribution =
      msg.front_steer_angle_lateral_rate_contribution();
  control_analysis.front_steer_angle_heading_contribution =
      msg.front_steer_angle_heading_contribution();
  control_analysis.front_steer_angle_heading_rate_contribution =
      msg.front_steer_angle_heading_rate_contribution();
  control_analysis.rear_steering_value_fd = msg.rear_steering_value_fd();
  control_analysis.rear_steering_target = msg.rear_steering_target();
  control_analysis.rear_steering_rate = msg.rear_steering_rate();
  control_analysis.rear_steer_angle_feedforward =
      msg.rear_steer_angle_feedforward();
  control_analysis.rear_steer_angle_feedback = msg.rear_steer_angle_feedback();
  control_analysis.rear_steer_angle_lateral_contribution =
      msg.rear_steer_angle_lateral_contribution();
  control_analysis.rear_steer_angle_lateral_rate_contribution =
      msg.rear_steer_angle_lateral_rate_contribution();
  control_analysis.rear_steer_angle_heading_contribution =
      msg.rear_steer_angle_heading_contribution();
  control_analysis.rear_steer_angle_heading_rate_contribution =
      msg.rear_steer_angle_heading_rate_contribution();
  control_analysis.matrix_k_00 = msg.matrix_k_00();
  control_analysis.matrix_k_01 = msg.matrix_k_01();
  control_analysis.matrix_k_02 = msg.matrix_k_02();
  control_analysis.matrix_k_03 = msg.matrix_k_03();
  control_analysis.matrix_k_10 = msg.matrix_k_10();
  control_analysis.matrix_k_11 = msg.matrix_k_11();
  control_analysis.matrix_k_12 = msg.matrix_k_12();
  control_analysis.matrix_k_13 = msg.matrix_k_13();
  control_analysis.matrix_state_0 = msg.matrix_state_0();
  control_analysis.matrix_state_1 = msg.matrix_state_1();
  control_analysis.matrix_state_2 = msg.matrix_state_2();
  control_analysis.matrix_state_3 = msg.matrix_state_3();
  control_analysis.matrix_q_updated_0 = msg.matrix_q_updated_0();
  control_analysis.matrix_q_updated_1 = msg.matrix_q_updated_1();
  control_analysis.matrix_q_updated_2 = msg.matrix_q_updated_2();
  control_analysis.matrix_q_updated_3 = msg.matrix_q_updated_3();
  control_analysis.current_x = msg.current_x();
  control_analysis.current_y = msg.current_y();
  control_analysis.target_point_x = msg.target_point_x();
  control_analysis.target_point_y = msg.target_point_y();
  control_analysis.lat_target_point_s = msg.lat_target_point_s();
  control_analysis.lqr_calculate_time = msg.lqr_calculate_time();
  control_analysis.lqr_calculate_time_max = msg.lqr_calculate_time_max();

  lcm_->publish("/control/ControlAnalysis", &control_analysis);
}

template <typename T>
void LcmMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  lcm_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(lcm_interface, faults)
  FAULTS_PARSER(lcm, faults)
  faults.faults_size = faults.faults.size();

  lcm_->publish("/control/Faults", &faults);
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

  lcm_->publish("/control/Events", &events);
}

template <typename T>
void LcmMessageManager<T>::HandleADCTrajectoryMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ADCTrajectory* msg) {
    if (is_active_ == false)
    return;

  legionclaw::interface::ADCTrajectory adc_trajectory;
  MESSAGE_HEADER_PARSER(adc_trajectory)
  adc_trajectory.set_total_path_length(msg->total_path_length);
  adc_trajectory.set_total_path_time(msg->total_path_time);
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  for (auto it_trajectory_points : msg->trajectory_points) {
    legionclaw::interface::TrajectoryPoint adc_trajectory_trajectory_point;
    legionclaw::interface::PathPoint adc_trajectory_trajectory_point_path_point;
    adc_trajectory_trajectory_point_path_point.set_x(
        it_trajectory_points.path_point.x);
    adc_trajectory_trajectory_point_path_point.set_y(
        it_trajectory_points.path_point.y);
    adc_trajectory_trajectory_point_path_point.set_z(
        it_trajectory_points.path_point.z);
    adc_trajectory_trajectory_point_path_point.set_theta(
        it_trajectory_points.path_point.theta);
    adc_trajectory_trajectory_point_path_point.set_kappa(
        it_trajectory_points.path_point.kappa);
    adc_trajectory_trajectory_point_path_point.set_s(
        it_trajectory_points.path_point.s);
    adc_trajectory_trajectory_point_path_point.set_dkappa(
        it_trajectory_points.path_point.dkappa);
    adc_trajectory_trajectory_point_path_point.set_ddkappa(
        it_trajectory_points.path_point.ddkappa);
    adc_trajectory_trajectory_point_path_point.set_lane_id(
        it_trajectory_points.path_point.lane_id);
    adc_trajectory_trajectory_point_path_point.set_x_derivative(
        it_trajectory_points.path_point.x_derivative);
    adc_trajectory_trajectory_point_path_point.set_y_derivative(
        it_trajectory_points.path_point.y_derivative);
    adc_trajectory_trajectory_point.set_path_point(
        adc_trajectory_trajectory_point_path_point);
    adc_trajectory_trajectory_point.set_v(it_trajectory_points.v);
    adc_trajectory_trajectory_point.set_a(it_trajectory_points.a);
    adc_trajectory_trajectory_point.set_relative_time(
        it_trajectory_points.relative_time);
    adc_trajectory_trajectory_point.set_da(it_trajectory_points.da);
    adc_trajectory_trajectory_point.set_is_steer_valid(
        it_trajectory_points.is_steer_valid);
    adc_trajectory_trajectory_point.set_front_steer(
        it_trajectory_points.front_steer);
    adc_trajectory_trajectory_point.set_rear_steer(
        it_trajectory_points.rear_steer);
    adc_trajectory_trajectory_point.set_gear(
        (legionclaw::common::GearPosition)it_trajectory_points.gear);
    trajectory_points.emplace_back(adc_trajectory_trajectory_point);
  }
  adc_trajectory.set_trajectory_points(&trajectory_points);
  adc_trajectory.set_car_action(
      (legionclaw::interface::ADCTrajectory::CarAction)msg->car_action);
  adc_trajectory.set_behaviour_lat_state(
      (legionclaw::interface::ADCTrajectory::BehaviourLatState)
          msg->behaviour_lat_state);
  adc_trajectory.set_behaviour_lon_state(
      (legionclaw::interface::ADCTrajectory::BehaviourLonState)
          msg->behaviour_lon_state);
  adc_trajectory.set_scenario(
      (legionclaw::interface::ADCTrajectory::Scenario)msg->scenario);
  adc_trajectory.set_driving_mode(
      (legionclaw::common::DrivingMode)msg->driving_mode);
  adc_trajectory.set_adc_trajectory_type(
      (legionclaw::interface::ADCTrajectory::ADCTrajectoryType)
          msg->adc_trajectory_type);
  legionclaw::interface::EStop adc_trajectory_estop;
  adc_trajectory_estop.set_is_estop(msg->estop.is_estop);
  adc_trajectory_estop.set_reason(msg->estop.reason);
  adc_trajectory.set_estop(adc_trajectory_estop);
  adc_trajectory.set_is_replan(msg->is_replan);
  adc_trajectory.set_replan_reason(msg->replan_reason);
  adc_trajectory.set_right_of_way_status(
      (legionclaw::interface::ADCTrajectory::RightOfWayStatus)
          msg->right_of_way_status);
  legionclaw::interface::RSSInfo adc_trajectory_rss_info;
  adc_trajectory_rss_info.set_is_rss_safe(msg->rss_info.is_rss_safe);
  adc_trajectory_rss_info.set_cur_dist_lon(msg->rss_info.cur_dist_lon);
  adc_trajectory_rss_info.set_rss_safe_dist_lon(
      msg->rss_info.rss_safe_dist_lon);
  adc_trajectory_rss_info.set_acc_lon_range_minimum(
      msg->rss_info.acc_lon_range_minimum);
  adc_trajectory_rss_info.set_acc_lon_range_maximum(
      msg->rss_info.acc_lon_range_maximum);
  adc_trajectory_rss_info.set_acc_lat_left_range_minimum(
      msg->rss_info.acc_lat_left_range_minimum);
  adc_trajectory_rss_info.set_acc_lat_left_range_maximum(
      msg->rss_info.acc_lat_left_range_maximum);
  adc_trajectory_rss_info.set_acc_lat_right_range_minimum(
      msg->rss_info.acc_lat_right_range_minimum);
  adc_trajectory_rss_info.set_acc_lat_right_range_maximum(
      msg->rss_info.acc_lat_right_range_maximum);
  adc_trajectory.set_rss_info(adc_trajectory_rss_info);
  instance_->HandleADCTrajectory(adc_trajectory);
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
void LcmMessageManager<T>::HandlePlanningCmdMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::PlanningCmd* msg) {
    if (is_active_ == false)
    return;

  legionclaw::interface::PlanningCmd planning_cmd;
  MESSAGE_HEADER_PARSER(planning_cmd)
  planning_cmd.set_turn_lamp_ctrl(
      (legionclaw::common::TurnSignal)msg->turn_lamp_ctrl);
  planning_cmd.set_high_beam_ctrl(
      (legionclaw::common::SwitchStatus)msg->high_beam_ctrl);
  planning_cmd.set_low_beam_ctrl(
      (legionclaw::common::SwitchStatus)msg->low_beam_ctrl);
  planning_cmd.set_horn_ctrl((legionclaw::common::SwitchStatus)msg->horn_ctrl);
  planning_cmd.set_front_wiper_ctrl(
      (legionclaw::common::SwitchStatus)msg->front_wiper_ctrl);
  planning_cmd.set_rear_wiper_ctrl(
      (legionclaw::common::SwitchStatus)msg->rear_wiper_ctrl);
  planning_cmd.set_position_lamp_ctrl(
      (legionclaw::common::SwitchStatus)msg->position_lamp_ctrl);
  planning_cmd.set_front_fog_lamp_ctrl(
      (legionclaw::common::SwitchStatus)msg->front_fog_lamp_ctrl);
  planning_cmd.set_rear_fog_lamp_ctrl(
      (legionclaw::common::SwitchStatus)msg->rear_fog_lamp_ctrl);
  planning_cmd.set_brake_lamp_ctrl(
      (legionclaw::common::SwitchStatus)msg->brake_lamp_ctrl);
  planning_cmd.set_alarm_lamp_ctrl(
      (legionclaw::common::SwitchStatus)msg->alarm_lamp_ctrl);
  planning_cmd.set_lf_door_ctrl((legionclaw::common::DoorStatus)msg->lf_door_ctrl);
  planning_cmd.set_rf_door_ctrl((legionclaw::common::DoorStatus)msg->rf_door_ctrl);
  planning_cmd.set_lr_door_ctrl((legionclaw::common::DoorStatus)msg->lr_door_ctrl);
  planning_cmd.set_rr_door_ctrl((legionclaw::common::DoorStatus)msg->rr_door_ctrl);

  instance_->HandlePlanningCmd(planning_cmd);
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
} // namespace control
} // namespace legionclaw
#endif
