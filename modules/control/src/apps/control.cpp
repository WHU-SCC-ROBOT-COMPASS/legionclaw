/**
 * @file              control.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2020-03-27 11:57:49
 * @copyright Copyright (c) 2020
 * @license      GNU General Public License (GPL)
 */
#include "control.h"

#include <sys/time.h>
#include <time.h>

#include <fstream>

#include "modules/common/fault/fault_client.hpp"
#include "modules/common/file/file.h"
#include "modules/common/interface/chassis.hpp"
// #include "modules/common/interface/vehicle.hpp"
#include "modules/common/interface/events.hpp"
#include "modules/common/time/time_tool.h"
#include "modules/control/src/proto/scheduler.pb.h"

namespace legionclaw {
namespace control {
using namespace legionclaw::common;
using legionclaw::common::Logging;
void Control::Init() {
  // step1 ????????false
  {
    is_init_ = false;
  }

  // step2 ?????
  {
    VariableInit();
  }

  // step6 ??????
  {
  }

  // step3 ???????
  {
    if (GetProtoFromJsonFile(FLAGS_control_config_file, control_conf_.get()) ==
        false) {
      AERROR << "GetProtoFrom control_config_file failed.";
      return;
    }

    LonCalibrationTable lon_calibration_table;
    if (GetProtoFromJsonFile(FLAGS_lon_calibration_table_file,
                             &lon_calibration_table) == false) {
      AERROR << "GetProtoFrom lon_calibration_table_file failed.";
      return;
    }
    control_conf_->mutable_lon_controller_conf()
        ->mutable_calibration_table()
        ->CopyFrom(lon_calibration_table);

    StopDistanceCalibrationTable stop_distance_calibration_table;
    if (GetProtoFromJsonFile(FLAGS_stop_distance_calibration_table_file,
                             &stop_distance_calibration_table) == false) {
      AERROR << "GetProtoFrom stop_distance_calibration_table_file failed.";
      return;
    }
    control_conf_->mutable_stop_distance_controller_conf()
        ->mutable_calibration_table()
        ->CopyFrom(stop_distance_calibration_table);

    LqrCalibrationTable lqr_calibration_table;
    if (GetProtoFromJsonFile(FLAGS_lqr_calibration_table_file,
                             &lqr_calibration_table) == false) {
      AERROR << "GetProtoFrom lqr_calibration_table_file failed.";
      return;
    }
    control_conf_->mutable_lqr_controller_conf()
        ->mutable_lqr_calibration_table()
        ->CopyFrom(lqr_calibration_table);

    legionclaw::interface::VehicleParam vehicle_param;
    if (GetProtoFromJsonFile(FLAGS_vehicle_config_path, &vehicle_param) ==
        false) {
      AERROR << "GetProtoFrom vehicle_param_file failed.";
      return;
    }
    control_conf_->mutable_vehicle_param()->CopyFrom(vehicle_param);
  }
  // step4 ?????
  {
    LOGGING_INIT(control_conf_)
  }

  // step4 IPC???
  {
    MESSAGE_INIT(control_conf_)
  }

  // step5 ??????
  {
    ts_ = control_conf_->produce_control_command_duration();
    produce_control_command_duration_ =
        (uint32_t)1000 *
        (double)(control_conf_->produce_control_command_duration());
    publish_control_command_duration_ =
        (uint32_t)1000 *
        (double)(control_conf_->publish_control_command_duration());
    control_conf_->set_use_system_timestamp(
        control_conf_->use_system_timestamp());

    // ????????
    {
      enable_shared_autonomy_ =
          control_conf_->shared_autonomy_conf().enable_shared_autonomy();
      steer_angle_rate_ =
          control_conf_->shared_autonomy_conf().steer_angle_rate();
      steer_angle_tolerance_ =
          control_conf_->shared_autonomy_conf().steer_angle_tolerance();
      std::vector<Scheduler> steer_rate_speed_scheduler;
      uint32_t steer_rate_speed_scheduler_size =
          control_conf_->shared_autonomy_conf()
              .steer_rate_speed_scheduler_size();
      for (uint32_t i = 0; i < steer_rate_speed_scheduler_size; ++i) {
        Scheduler scheduler;
        scheduler.set_speed(control_conf_->shared_autonomy_conf()
                                .steer_rate_speed_scheduler()
                                .at(i)
                                .speed());
        scheduler.set_ratio(control_conf_->shared_autonomy_conf()
                                .steer_rate_speed_scheduler()
                                .at(i)
                                .ratio());
        steer_rate_speed_scheduler.push_back(scheduler);
      }

      Interpolation1D::DataType xy;
      for (const auto& scheduler : steer_rate_speed_scheduler) {
        xy.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }
      steer_rate_speed_interpolation_.reset(new Interpolation1D);
      if (steer_rate_speed_interpolation_->Init(xy) == false) {
        AERROR << "Fail to load steer rate speed gain scheduler";
      }
    }
  }

  // step7 ????? controller???
  {
    controller_agent_.Init(injector_, &*control_conf_);
  }

  // step8 ?????????
  {
    // ???????
    TimerManager<Control>::AddTimer(produce_control_command_duration_,
                                    &Control::ComputeControlCommandOnTimer,
                                    this);
    // ????
    ad_timer_manager_ = std::make_shared<ADTimerManager<Control, void>>();
    // task_10ms_ = std::make_shared<WheelTimer<Control,
    // void>>(ad_timer_manager_);
    // task_10ms_->AddTimer(publish_control_command_duration_,
    // &Control::Task10ms,this);
    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create can task_thread_ thread.";
      return;
    }
  }
  // step9 ??????true
  {
    is_init_ = true;
  }
}

void Control::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void Control::VariableInit() {
  trajectory_switch_index_ = 0;
  trajectory_initialized_ = false;
  shared_autonomy_ = false;
  pre_steer_cmd_ = 0.0;
  injector_ = std::make_shared<DependencyInjector>();
  control_conf_ = std::make_shared<ControlConf>();
  stop_obs_false_timestamp_ = TimeTool::Now2Seconds();
}

void Control::Print() {
  // return;
  printf("\033[2J");
  // std::cout.imbue(locale("chs"));
  std::cout.precision(8);
  std::cout << "=============device info end=============" << endl;
  std::cout << endl;

  std::cout << "lat                : " << localization_debug_.position().lat()
            << endl
            << "lon                : " << localization_debug_.position().lon()
            << endl
            << "lon_acceleration   : "
            << localization_debug_.linear_acceleration().x() << endl
            << "longitudinal_speed : "
            << localization_debug_.linear_velocity().x() << endl
            << "roll_              : " << localization_debug_.roll() << endl
            << "pitch_             : " << localization_debug_.pitch() << endl
            << "heading            : " << localization_debug_.heading() << endl
            << "origin_lat         : " << localization_debug_.origin_lat()
            << endl
            << "origin_lon         : " << localization_debug_.origin_lon()
            << endl
            << "x                  : " << localization_debug_.utm_position().x()
            << endl
            << "y                  : " << localization_debug_.utm_position().y()
            << endl
            << "speed_mps_         : " << chassis_.speed_mps() << endl
            << "rtk_flag           : " << localization_debug_.rtk_flag()
            << endl;
}

void Control::Log() {
  // string log_filename = "./log/control" + start_time_ + ".log";
  // ofstream logfile(log_filename.c_str(), std::ofstream::app); // 45  speed=7
  // logfile.precision(13);
  // logfile << dec
  //         << software_version_ << " "
  //         << control_cmd_.steering_tar << " "
  //         << control_cmd_.steering_rate_ << " "
  //         << control_cmd_.accel_driving_mode_ << " "
  //         << control_cmd_.brake_value_ << " "
  //         << control_cmd_.brake_driving_mode_ << " "
  //         << control_cmd_.epb_level_ << " "
  //         << control_cmd_.epb_driving_mode_ << " "
  //         << control_cmd_.gear_location_ << " "
  //         << endl;
  // logfile.close();
}

void Control::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<Control>> message_manager) {
  message_manager_.insert(
      std::pair<std::string, std::shared_ptr<MessageManager<Control>>>(
          name, message_manager));
}

// Task10ms?????????????????????
void Control::Task10ms(void* param) {
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(control_cmd_)
  SetBcmControlCmd(&control_cmd_, planning_cmd_);
  PublishControlCommand(control_cmd_);

  INTERFACE_HEADER_ASSIGN(control_analysis_)
  control_analysis_.set_accel_value_cmd(control_cmd_.accel_value());
  control_analysis_.set_brake_value_cmd(control_cmd_.brake_value());
  control_analysis_.set_driving_mode(control_cmd_.accel_driving_mode());
  control_analysis_.set_driving_mode_fd(chassis_.driving_mode());
  PublishControlAnalysis(control_analysis_);
}

void Control::PublishControlCommand(
    legionclaw::interface::ControlCommand control_command) {
#if LCM_ENABLE
  message_manager_["LCM"]->PublishControlCommand(control_command);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishControlCommand(control_command);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishControlCommand(control_command);
#endif

#if ROS2_ENABLE
  message_manager_["ROS2"]->PublishControlCommand(control_command);
#endif
}

void Control::PublishControlAnalysis(
    legionclaw::interface::ControlAnalysis control_analysis) {
#if LCM_ENABLE
  message_manager_["LCM"]->PublishControlAnalysis(control_analysis);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishControlAnalysis(control_analysis);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishControlAnalysis(control_analysis);
#endif

#if ROS2_ENABLE
  message_manager_["ROS2"]->PublishControlAnalysis(control_analysis);
#endif
}

void Control::PublishEvents(legionclaw::interface::Events events) {
#if LCM_ENABLE
  message_manager_["LCM"]->PublishEvents(events);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishEvents(events);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishEvents(events);
#endif

#if ROS2_ENABLE
  message_manager_["ROS2"]->PublishEvents(events);
#endif
}

void Control::PublishFaults() {
  if (is_init_ == false) {
    return;
  }

#if LCM_ENABLE
  message_manager_["LCM"]->PublishFaults(faults_);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishFaults(faults_);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishFaults(faults_);
#endif

#if ROS2_ENABLE
  message_manager_["ROS2"]->PublishFaults(faults_);
#endif
}

std::shared_ptr<ControlConf> Control::GetConf() const { return control_conf_; }

void Control::HandleADCTrajectory(
    legionclaw::interface::ADCTrajectory adc_trajectory) {
  if (is_init_ == false) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = adc_trajectory.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      adc_trajectory.set_header(header);
    }
    adc_trajectory_ = adc_trajectory;

    if (control_conf_->use_adctrajectory_system_timestamp_enable() == true) {
      adc_trajectory_ = adc_trajectory;
      legionclaw::interface::Header header = adc_trajectory_.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      adc_trajectory_.set_header(header);
    }
    // ?heading??????????rad,????
  }
  if (!trajectory_initialized_) {
    AINFO << "trajectory initialized ";
    if (adc_trajectory_.driving_mode() ==
        legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE) {
      trajectory_initialized_ = true;
    }
  }
}

void Control::HandleChassis(legionclaw::interface::Chassis chassis) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = chassis.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      chassis.set_header(header);
    }
    chassis_ = chassis;
    chassis_.set_front_steering_value(D2R(chassis.front_steering_value()));
  }
}

// void Control::HandleVehicle(legionclaw::interface::Vehicle vehicle) {
//   if (is_init_ == false) {
//     return;
//   }
//   {
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (control_conf_->use_system_timestamp() == true) {
//       legionclaw::interface::Header header = vehicle.header();
//       header.set_stamp(TimeTool::Now2TmeStruct());
//       vehicle.set_header(header);
//     }
//     vehicle_ = vehicle;
//   }
//}

// void Control::HandleObstacleList(
//     legionclaw::interface::ObstacleList obstacle_list) {
//   if (is_init_ == false) {
//     return;
//   }
//   {
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (control_conf_->use_system_timestamp() == true) {
//       legionclaw::interface::Header header = obstacle_list.header();
//       header.set_stamp(TimeTool::Now2TmeStruct());
//       obstacle_list.set_header(header);
//     }
//     obstacle_list_ = obstacle_list;
//   }
// }

void Control::HandlePlanningCmd(legionclaw::interface::PlanningCmd planning_cmd) {
  if (is_init_ == false) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = planning_cmd.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      planning_cmd.set_header(header);
    }
    planning_cmd_ = planning_cmd;
  }
}

void Control::HandleLocation(legionclaw::interface::Location location) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = location.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      location.set_header(header);
    }
    localization_debug_ = location;
    // Print();

    legionclaw::interface::Pose pose;

    legionclaw::interface::PointENU position;
    position.set_x(location.utm_position().x());
    position.set_y(location.utm_position().y());
    position.set_z(location.utm_position().z());
    pose.set_position(position);

    legionclaw::interface::Quaternion orientation;

    math::EulerAnglesZXYd a(location.roll(), location.pitch(),
                            location.heading());

    auto q = a.ToQuaternion();
    orientation.set_qw(q.w());
    orientation.set_qx(q.x());
    orientation.set_qy(q.y());
    orientation.set_qz(q.z());
    pose.set_orientation(orientation);
    pose.set_linear_velocity(location.linear_velocity());
    pose.set_linear_acceleration(location.linear_acceleration());
    pose.set_angular_velocity(location.angular_velocity());
    double heading = location.heading();
    pose.set_heading(heading);

    localization_.set_pose(pose);
    localization_.set_header(location.header());
  }
}

void Control::ComputeControlCommandOnTimer() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.chassis_ = chassis_;
    local_view_.adc_trajectory_ = adc_trajectory_;
    local_view_.localization_ = localization_;
    local_view_.obstacle_list_ = obstacle_list_;
    if (local_view_.adc_trajectory_.trajectory_points_size()) {
      for (auto iter =
               local_view_.adc_trajectory_.mutable_trajectory_points()->begin();
           iter !=
           local_view_.adc_trajectory_.mutable_trajectory_points()->end();
           iter++)  // ??mutable_trajectory_points()????????&trajectory_points
      {
        // double theta = iter->mutable_path_point()->theta();
        // theta = math::WrapAngle(0.5 * M_PI - theta);
        // iter->mutable_path_point()->set_theta(theta);
        iter->mutable_path_point()->set_kappa(
            -1.0 * iter->mutable_path_point()->kappa());  // ????
      }
    }
  }

  if (trajectory_initialized_) {
    //??trajectory??,size,v,a
    Status status = CheckInput(&local_view_);
    if (!status.ok()) {
      AERROR << "Input data failed: " << status.error_message();
      Estop();
      legionclaw::interface::Header header;
      INTERFACE_HEADER_ASSIGN(control_cmd_)
      PublishControlCommand(control_cmd_);
      return;
    }
    // ??????????
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      return;
    }
  }

  // check estop(??false)
  estop_ = control_conf_->enable_persistent_estop()
               ? estop_ || local_view_.adc_trajectory_.estop().is_estop()
               : local_view_.adc_trajectory_.estop().is_estop();

  //规划轨迹处于自动驾驶状态并且存在紧急制动命令
  if (local_view_.adc_trajectory_.driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE &&
      local_view_.adc_trajectory_.estop().is_estop()) {
    estop_ = true;
    estop_reason_ = "estop from planning : ";
    estop_reason_ += local_view_.adc_trajectory_.estop().reason();
  }

  if (control_conf_->enable_gear_dirve_negative_speed_protection() == true) {
    const double kEpsilon = 0.001;
    auto first_trajectory_point =
        local_view_.adc_trajectory_.trajectory_points(0);
    if (local_view_.chassis_.gear_location() ==
            legionclaw::common::GearPosition::GEAR_DRIVE &&
        first_trajectory_point.v() < -1 * kEpsilon) {
      estop_ = true;
      estop_reason_ = "estop for negative speed when gear_drive";
    }
  }

  // bool stop_obs = false;
  // bool stop_touch = false;
  // if (control_conf_->use_lidar_obstacle_avoidance() == true ||
  //     control_conf_->use_uss_obstacle_avoidance() == true) {
  //   stop_obs = StopIfObstacleClose(local_view_.obstacle_list_);
  // }

  if (!estop_) {
    if (local_view_.chassis_.gear_location() ==
        legionclaw::common::GearPosition::GEAR_NEUTRAL) {
      // estop???????????N??
      controller_agent_.Reset();
      // AINFO << "Reset Controllers in Manual Mode";
    }
  }

  // ????????
  if (trajectory_initialized_) {
    Status status_compute = controller_agent_.ComputeControlCommand(
        &local_view_.localization_, &local_view_.chassis_,
        &local_view_.adc_trajectory_, &control_cmd_, &control_analysis_);
    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " status:" << status_compute.error_message();
      estop_ = true;
      estop_reason_ = status_compute.error_message();
      AWARN << estop_reason_;
    }

    // 检查横向偏差和航向角偏差是否超过最大限值，如果超过则触发EStop
    if (!estop_ && control_conf_->enable_lateral_error_estop()) {
      double lateral_error_abs = std::abs(control_analysis_.lateral_error());
      double heading_error_abs = std::abs(control_analysis_.heading_error() *
                                          M_PI / 180.0);  // 度转弧度

      if (lateral_error_abs > control_conf_->max_lateral_error() ||
          heading_error_abs > control_conf_->max_heading_error()) {
        estop_ = true;
        estop_reason_ =
            "EStop: lateral or heading error exceeds limit. "
            "lateral_error: " +
            std::to_string(lateral_error_abs) +
            " (limit: " + std::to_string(control_conf_->max_lateral_error()) +
            "), "
            "heading_error: " +
            std::to_string(heading_error_abs) +
            " (limit: " + std::to_string(control_conf_->max_heading_error()) +
            ")";
        AWARN << estop_reason_;
      }
    }
  }
  // if planning set estop, then no control process triggered
  if (estop_) {
    ADEBUG << "EStop triggered! No control core method executed!";
    Estop();
  }

  // if (stop_obs == false) {
  //   stop_obs_false_timestamp_ = TimeTool::Now2Seconds();
  // }
  // ????
  // if (stop_obs) {
  //   double stop_obs_true_timestamp = TimeTool::Now2Seconds();
  //   double dt = stop_obs_true_timestamp - stop_obs_false_timestamp_;
  //   legionclaw::interface::Header header;
  //   INTERFACE_HEADER_ASSIGN(events_)
  //   legionclaw::interface::Event event;
  //   event.set_code(5);
  //   event.set_reason("IsStopping");
  //   events_.add_events(event);
  //   event.set_code(dt);
  //   event.set_reason("IsStoppingTime");
  //   events_.add_events(event);
  // }
  // PublishEvents(events_);
  // events_.clear_events();

  // publish
  {
    legionclaw::interface::Header header;
    INTERFACE_HEADER_ASSIGN(control_cmd_)
    SetBcmControlCmd(&control_cmd_, planning_cmd_);
    PublishControlCommand(control_cmd_);

    INTERFACE_HEADER_ASSIGN(control_analysis_)
    control_analysis_.set_accel_value_cmd(control_cmd_.accel_value());
    control_analysis_.set_brake_value_cmd(control_cmd_.brake_value());
    control_analysis_.set_driving_mode(control_cmd_.accel_driving_mode());
    control_analysis_.set_driving_mode_fd(chassis_.driving_mode());
    PublishControlAnalysis(control_analysis_);
  }

  if (control_cmd_.accel_driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_MANUAL &&
      control_cmd_.steer_driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_MANUAL &&
      (local_view_.chassis_.driving_mode() ==
       legionclaw::common::DrivingMode::COMPLETE_MANUAL)) {
    adc_trajectory_ = legionclaw::interface::ADCTrajectory();
    control_cmd_ = legionclaw::interface::ControlCommand();
    control_analysis_ = legionclaw::interface::ControlAnalysis();
    trajectory_initialized_ = false;
  }
}

void Control::Estop() {
  control_cmd_.set_speed(0.0);
  control_cmd_.set_accel_value(0.0);
  control_cmd_.set_acceleration(0.0);
  control_cmd_.set_brake_value(control_conf_->soft_estop_brake());
  control_cmd_.set_gear_location(legionclaw::common::GEAR_NEUTRAL);
  AWARN << estop_reason_;
}

Status Control::CheckInput(LocalView* local_view) {
  if (local_view->adc_trajectory_.trajectory_points_size() == 0) {
    AWARN << "planning has no trajectory point. ";
    std::string msg("planning has no trajectory point. planning_seq_num:");
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR, msg);
  }

  std::vector<interface::TrajectoryPoint> trajectory_points;
  local_view->adc_trajectory_.trajectory_points(trajectory_points);
  for (auto& trajectory_point : trajectory_points) {
    // CheckInput?????????v?a??????0
    if (std::abs(trajectory_point.v()) <
            control_conf_->minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_->max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  // ????,???vehicle_state_
  injector_->vehicle_state()->Update(local_view->localization_,
                                     local_view->chassis_);

  return Status::Ok();
}

Status Control::CheckTimestamp(const LocalView& local_view) {
  if (!control_conf_->enable_input_timestamp_check() ||
      control_conf_->is_control_test_mode()) {
    // ADEBUG << "Skip input timestamp check by gflags.";
    return Status::Ok();
  }
  double localization_diff =
      TimeTool::GetTimeDiffNow(local_view_.localization_.header().stamp());
  if (localization_diff > (control_conf_->max_localization_miss_num() *
                           control_conf_->localization_period())) {
    AERROR << "Localization msg lost for :" << std::fixed
           << std::setprecision(2) << localization_diff << "s";
    // monitor_logger_buffer_.ERROR("legionclaw::interface::LocalizationEstimate msg
    // lost");
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Localization msg timeout");
  }

  double chassis_diff =
      TimeTool::GetTimeDiffNow(local_view_.chassis_.header().stamp());
  if (chassis_diff > (control_conf_->max_chassis_miss_num() *
                      control_conf_->chassis_period())) {
    AERROR << "Chassis msg lost for " << std::fixed << std::setprecision(2)
           << chassis_diff << "s";
    // monitor_logger_buffer_.ERROR("legionclaw::common msg lost");
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Chassis msg timeout");
  }

  double trajectory_diff =
      TimeTool::GetTimeDiffNow(local_view_.adc_trajectory_.header().stamp());
  if (trajectory_diff > (control_conf_->max_planning_miss_num() *
                         control_conf_->trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::fixed << std::setprecision(2)
           << trajectory_diff << "s";
    // monitor_logger_buffer_.ERROR("Trajectory msg lost");
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Trajectory msg timeout");
  }

  // for (size_t i = 1; i < local_view.adc_trajectory_.trajectory_points_size();
  //      i++)
  // {
  //   double time_diff =
  //   local_view.adc_trajectory_.trajectory_points(i).relative_time() -
  //                      local_view.adc_trajectory_.trajectory_points(i -
  //                      1).relative_time();
  //   if (time_diff == 0.0)
  //   {
  //     return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,"Trajectory msg
  //     relative_time is not update");
  //   }
  // }
  return Status::Ok();
}

void Control::MessagesInit() {
  if (control_conf_ == nullptr) return;

  for (auto it : control_conf_->messages().active_message()) {
    auto message = control_conf_->messages().message_info().at(it);
    switch (message.type()) {
#if LCM_ENABLE
      case legionclaw::common::MessageType::LCM: {
        AINFO << "message type:LCM";

        lcm_message_manager_ = std::make_shared<LcmMessageManager<Planning>>();
        ResigerMessageManager(message.name(), lcm_message_manager_);

        lcm_message_manager_->Init(this);
      } break;
#endif
#if DDS_ENABLE
      case legionclaw::common::MessageType::DDS: {
        AINFO << "message type:DDS";

        dds_message_manager_ = std::make_shared<DdsMessageManager<Planning>>();
        ResigerMessageManager(message.name(), dds_message_manager_);

        dds_message_manager_->Init(this);
      } break;
#endif
#if ROS_ENABLE
      case legionclaw::common::MessageType::ROS: {
        AINFO << "message type:ROS";

        ros_message_manager_ = std::make_shared<RosMessageManager<Planning>>();
        ResigerMessageManager(message.name(), ros_message_manager_);
        ros_message_manager_->Init(this);
      } break;
#endif

#if ADSFI_ENABLE
      case legionclaw::common::MessageType::ADSFI: {
        AINFO << "message type:ADSFI";

        adsfi_message_manager_ =
            std::make_shared<AdsfiMessageManager<Planning>>();
        ResigerMessageManager(message.name(), adsfi_message_manager_);

        adsfi_message_manager_->Init(this);
      } break;
#endif

#if ROS2_ENABLE
      case legionclaw::common::MessageType::ROS2: {
        AINFO << "message type:ROS2";

        ros2_message_manager_ = std::make_shared<Ros2MessageManager<Control>>();
        ResigerMessageManager(message.name(), ros2_message_manager_);

        ros2_message_manager_->Init(this);
      } break;
#endif
      default: {
        AERROR << "unknown message type";
      } break;
    }
  }
}

void Control::SetBcmControlCmd(legionclaw::interface::ControlCommand* control_cmd,
                               legionclaw::interface::PlanningCmd planning_cmd) {
  control_cmd->set_turn_lamp_ctrl(
      (legionclaw::common::TurnSignal)planning_cmd.turn_lamp_ctrl());
  control_cmd->set_high_beam_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.high_beam_ctrl());
  control_cmd->set_low_beam_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.low_beam_ctrl());
  control_cmd->set_front_wiper_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.front_wiper_ctrl());
  control_cmd->set_rear_wiper_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.rear_wiper_ctrl());
  control_cmd->set_position_lamp_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.position_lamp_ctrl());
  control_cmd->set_front_fog_lamp_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.front_fog_lamp_ctrl());
  control_cmd->set_rear_fog_lamp_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.rear_fog_lamp_ctrl());
  // control_cmd->set_brake_lamp_ctrl(
  //     (legionclaw::common::SwitchStatus)planning_cmd.brake_lamp_ctrl());
  control_cmd->set_alarm_lamp_ctrl(
      (legionclaw::common::SwitchStatus)planning_cmd.alarm_lamp_ctrl());
  control_cmd->set_lf_door_ctrl(
      (legionclaw::common::DoorStatus)planning_cmd.lf_door_ctrl());
  control_cmd->set_rf_door_ctrl(
      (legionclaw::common::DoorStatus)planning_cmd.rf_door_ctrl());
  control_cmd->set_lr_door_ctrl(
      (legionclaw::common::DoorStatus)planning_cmd.lr_door_ctrl());
  control_cmd->set_rr_door_ctrl(
      (legionclaw::common::DoorStatus)planning_cmd.rr_door_ctrl());
}

// TODO ??????????
bool Control::SharedAutonomyTransition(const double& steer_target,
                                       double& steer_cmd) {
  // ????????????????
  if (control_cmd_.steer_driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE &&
      local_view_.chassis_.steer_driving_mode() !=
          legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE) {
    shared_autonomy_ = true;
    pre_steer_cmd_ = R2D(local_view_.chassis_.front_steering_value());
  }

  if (!enable_shared_autonomy_ || !shared_autonomy_) return false;

  double steer_rate =
      steer_angle_rate_ * steer_rate_speed_interpolation_->Interpolate(
                              std::fabs(chassis_.speed_mps()));

  if (steer_target - pre_steer_cmd_ < 0) {
    steer_rate *= -1.0;
  }

  steer_cmd = pre_steer_cmd_ + steer_rate * ts_;
  pre_steer_cmd_ = steer_cmd;
  if (fabs(steer_target - steer_cmd) < steer_angle_tolerance_) {
    steer_cmd = steer_target;
    shared_autonomy_ = false;
  }
  return true;
}

// bool Control::StopIfObstacleClose(
//     legionclaw::interface::ObstacleList obstacle_list) {
//   bool is_too_close = false;
//   // ???????
//   for (uint32_t i = 0; i < obstacle_list.obstacle_size(); i++) {
//     if (is_too_close) {
//       return is_too_close;
//     }
//     std::vector<legionclaw::interface::Obstacle> obstacle;
//     obstacle = obstacle_list.obstacle();
//     Vec3d base_point;
//     Vec3d center_point;
//     //????????????????  x?  y?  z? +:
//     base_point.set_x(obstacle[i].center_pos_vehicle().x());
//     base_point.set_y(obstacle[i].center_pos_vehicle().y());
//     //????????????????
//     double buffer = 0.5 * control_conf_->vehicle_param().length() -
//                     control_conf_->vehicle_param().back_edge_to_center();
//     center_point.set_x(base_point.x() - buffer);
//     center_point.set_y(base_point.y());
//     if (control_conf_->use_lidar_obstacle_avoidance() == true) {
//       if (obstacle[i].fusion_type() ==
//           legionclaw::interface::Obstacle::FusionType::CAMERA) {
//         // ?????????
//         if (center_point.x() <=
//             -control_conf_->vehicle_param().back_edge_to_center())
//           continue;

//         // ???????????????
//         for (size_t j = 0; j < obstacle[i].polygon_point_vehicle().size();
//              j++) {
//           Vec3d polygon_point;
//           //??????
//           double avoidance_distance =
//               control_conf_->obstacle_avoidance().lidar().b() +
//               control_conf_->obstacle_avoidance().lidar().k() *
//                   chassis_.speed_mps();

//           double side_avoidance_distance;
//           if (center_point.x() >
//               control_conf_->vehicle_param().front_edge_to_center()) {
//             //???????????????
//             side_avoidance_distance =
//                 control_conf_->obstacle_avoidance().lidar().side_front_b() +
//                 control_conf_->obstacle_avoidance().lidar().side_front_k() *
//                     chassis_.speed_mps();
//           } else {
//             //?????????????
//             side_avoidance_distance =
//                 control_conf_->obstacle_avoidance().lidar().side_back_b() +
//                 control_conf_->obstacle_avoidance().lidar().side_back_k() *
//                     chassis_.speed_mps();
//           }
//           polygon_point.set_x(obstacle[i].polygon_point_vehicle()[j].x());
//           polygon_point.set_y(obstacle[i].polygon_point_vehicle()[j].y());
//           polygon_point.set_z(obstacle[i].polygon_point_vehicle()[j].z());
//           // ????????????????????????????????????
//           if ((sqrt(pow(polygon_point.x(), 2) + pow(polygon_point.y(), 2)) <
//                avoidance_distance) &&
//               abs(polygon_point.y()) < side_avoidance_distance) {
//             is_too_close = true;
//             AWARN << "The obstacle type is: LIDAR ";
//             AWARN << "The obstacle is too close , the station is : "
//                   << polygon_point.x() << "   " << polygon_point.y();
//             AWARN << "The obstacle distance is: "
//                   << sqrt(pow(polygon_point.x(), 2) +
//                           pow(polygon_point.y(), 2));
//             break;
//           }
//         }
//       }
//     }

//     if (control_conf_->use_uss_obstacle_avoidance() == true) {
//       if (obstacle[i].fusion_type() ==
//           legionclaw::interface::Obstacle::FusionType::ULTRASONIC) {
//         //??0???1???2???3???4???5???6???7???8???9???10
//         //??0???1.front
//         if ((obstacle[i].sensor_pos_id() == 0 ||
//              obstacle[i].sensor_pos_id() == 1) &&
//             obstacle[i].range_vehicle() <
//                 control_conf_->obstacle_avoidance().uss().front_distance()) {
//           is_too_close = true;
//           AWARN << "The obstacle type is: Uss ,the id is :"
//                 << double(obstacle[i].sensor_pos_id());
//           AWARN << "The obstacle is too close , the distance is : "
//                 << obstacle[i].range_vehicle();
//           break;
//         }
//         //??5???6???7.back
//         if ((obstacle[i].sensor_pos_id() == 5 ||
//              obstacle[i].sensor_pos_id() == 6 ||
//              obstacle[i].sensor_pos_id() == 7) &&
//             obstacle[i].range_vehicle() <
//                 control_conf_->obstacle_avoidance().uss().back_distance()) {
//           is_too_close = true;
//           AWARN << "The obstacle type is Uss ,the id is :"
//                 << double(obstacle[i].sensor_pos_id());
//           AWARN << "The obstacle is too close , the distance is : "
//                 << obstacle[i].range_vehicle();
//           break;
//         }
//         //??9,??3.mid
//         if ((obstacle[i].sensor_pos_id() == 3 ||
//              obstacle[i].sensor_pos_id() == 9) &&
//             obstacle[i].range_vehicle() <
//                 control_conf_->obstacle_avoidance().uss().mid_distance()) {
//           is_too_close = true;
//           AWARN << "The obstacle type is Uss ,the id is :"
//                 << double(obstacle[i].sensor_pos_id());
//           AWARN << "The obstacle is too close , the distance is : "
//                 << obstacle[i].range_vehicle();
//           break;
//         }
//         //??10,??8,??2???4.side
//         if ((obstacle[i].sensor_pos_id() == 2 ||
//              obstacle[i].sensor_pos_id() == 4 ||
//              obstacle[i].sensor_pos_id() == 8 ||
//              obstacle[i].sensor_pos_id() == 10) &&
//             obstacle[i].range_vehicle() <
//                 control_conf_->obstacle_avoidance().uss().side_distance()) {
//           is_too_close = true;
//           AWARN << "The obstacle type is Uss ,the id is :"
//                 << double(obstacle[i].sensor_pos_id());
//           AWARN << "The obstacle is too close , the distance is : "
//                 << obstacle[i].range_vehicle();
//           break;
//         }
//       }
//     }
//   }
//   return is_too_close;
// }

void Control::Spin() {
  while (1) {
    ad_timer_manager_->DetectTimers(NULL);
    usleep(10);
  }
}

}  // namespace control
}  // namespace legionclaw
