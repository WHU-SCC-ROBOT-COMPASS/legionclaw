
/**
 * @file lon_lat_controller.cpp
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2018-07-07
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */
#include "lon_controller.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <string>
#include <utility>

#include "common/logging/logging.h"
#include "common/math/math_utils.h"
#include "common/time/time_tool.h"
#include "gear_state_machine.hpp"

namespace legionclaw {
namespace control {

using namespace legionclaw::common;
using namespace legionclaw::common::math;

constexpr double GRA_ACC = 9.8;

std::string LonController::GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "./log/speed_log__%F_%H%M%S.csv", &time_tm);
  return std::string(name_buffer);
}

void LonController::WriteHeaders(std::ofstream &file_stream) {
  file_stream << "time,"
              << "accel_driving_mode,"
              << "accel_value,"
              << "brake_driving_mode,"
              << "brake_value,"
              << "epb_driving_mode,"
              << "epb_level,"
              << "gear_driving_mode,"
              << "gear_location,"
              << "station_reference,"
              << "station_error,"
              << "station_error_limited,"
              << "speed_offset,"
              << "preview_station_error,"
              << "speed_reference,"
              << "current_speed,"
              << "speed_error,"
              << "speed_error_limited,"
              << "acceleration_cmd_closeloop,"
              << "preview_speed_reference,"
              << "preview_speed_error,"
              << "preview_acceleration_reference,"
              << "turning_offset_compensation,"
              << "slope_offset_compensation,"
              << "acceleration_cmd,"
              << "acceleration_lookup,"
              << "speed_lookup,"
              << "calibration_value,"
              << "throttle_cmd,"
              << "brake_cmd,"
              << "is_full_stop,"
              << "lon_calculate_time,"
              << "has_stop_point," << "\n";
}

LonController::LonController() : name_("longitudinal controller") {
  // if (enable_log_debug_) {
  //   speed_log_file_.open(GetLogFileName());
  //   std::cout << "speed_log_file_.is_open() ::::::" <<
  //   speed_log_file_.is_open() <<"\n"; speed_log_file_ << std::fixed;
  //   speed_log_file_ << std::setprecision(6);
  //   WriteHeaders(speed_log_file_);
  // }
  AINFO << "Using " << name_;
}

void LonController::CloseLogFile() {
  if (enable_log_debug_ && speed_log_file_.is_open()) {
    speed_log_file_.close();
  }
}
void LonController::Stop() { CloseLogFile(); }

LonController::~LonController() { CloseLogFile(); }

Status LonController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  injector_ = injector;
  control_conf_ = control_conf;
  lon_dynamics_.reset(new LonDynamics());
  lon_dynamics_->Init(control_conf);
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    AERROR << "get_longitudinal_param() nullptr";
    return Status(Status::ErrorCode::CONTROL_INIT_ERROR,
                  "Failed to load LonController conf");
  }

  use_new_planning_mode_ = control_conf->use_new_planning_mode();
  use_acceleration_mode_ = control_conf->use_acceleration_mode();
  use_deceleration_mode_ = control_conf->use_deceleration_mode();
  trajectory_update_duration_ = control_conf_->trajectory_period();
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();
  double ts_ = lon_controller_conf.ts();
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  station_pid_controller_.SetPID(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
  enable_slope_offset_ =
      control_conf_->lon_controller_conf().enable_slope_offset();
  enable_aero_offset_ =
      control_conf_->lon_controller_conf().enable_aero_offset();
  enable_turning_offset_ =
      control_conf_->lon_controller_conf().enable_turning_offset();
  enable_rolling_offset_ =
      control_conf_->lon_controller_conf().enable_rolling_offset();
  mass_ = control_conf_->vehicle_param().mass_fl() +
          control_conf_->vehicle_param().mass_fr() +
          control_conf_->vehicle_param().mass_rl() +
          control_conf_->vehicle_param().mass_rr();
  steer_ratio_ = control_conf_->vehicle_param().steer_ratio();
  use_calibration_table_ =
      control_conf_->lon_controller_conf().use_calibration_table();
  accel_value_kp_ = control_conf_->lon_controller_conf().accel_value_kp();
  max_accel_value_ = control_conf_->lon_controller_conf().max_accel_value();

  if (enable_leadlag) {
    station_leadlag_controller_.Init(
        lon_controller_conf.reverse_station_leadlag_conf(), ts_);
    speed_leadlag_controller_.Init(
        lon_controller_conf.reverse_speed_leadlag_conf(), ts_);
  }
  vehicle_param_ = control_conf->vehicle_param();

  SetDigitalFilterPitchAngle(lon_controller_conf);
  // 这里true
  if (use_calibration_table_) {
    LoadControlCalibrationTable(lon_controller_conf);
  }

  controller_initialized_ = true;

  brake_delay_time_ = control_conf->lon_controller_conf().brake_delay_time_d();
  vehicle_speed_in_idle_ =
      control_conf->lon_controller_conf().vehicle_speed_in_idle();
  deceleration_in_idle_speed_ =
      control_conf->lon_controller_conf().deceleration_in_idle_speed();
  starting_acceleration_in_d_ =
      control_conf->lon_controller_conf().starting_acceleration_in_d();
  starting_acceleration_in_r_ =
      control_conf->lon_controller_conf().starting_acceleration_in_r();
  enable_log_debug_ = control_conf->lon_controller_conf().enable_log_debug();
  lon_print_enable_ = control_conf->lon_controller_conf().lon_print_enable();
  brake_sign_ = control_conf->lon_controller_conf().brake_sign();
  kp_brake_ = control_conf->lon_controller_conf().kp_brake();
  starting_speed_ = control_conf->lon_controller_conf().starting_speed();
  pitch_offset_ = control_conf->lon_controller_conf().pitch_offset();
  speed_limit_ = vehicle_param_.speed_limit();
  max_deceleration_ = vehicle_param_.max_deceleration();
  max_deceleration_ = vehicle_param_.max_deceleration();
  max_acceleration_jerk_ = vehicle_param_.max_acceleration_jerk() * ts_;
  standstill_acceleration_ = control_conf_->standstill_acceleration();

  if (enable_log_debug_) {
    speed_log_file_.open(GetLogFileName());
    speed_log_file_ << std::fixed;
    speed_log_file_ << std::setprecision(6);
    WriteHeaders(speed_log_file_);
  }

  GearTransitionInit();
  is_stopped_first_hit_ = true;
  is_in_brake_ = false;

  return Status::Ok();
}

void LonController::SetDigitalFilterPitchAngle(
    const LonControllerConf &lon_controller_conf) {
  double cutoff_freq =
      lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
  SetDigitalFilter(ts_, cutoff_freq, &digital_filter_pitch_angle_);
}

void LonController::LoadControlCalibrationTable(
    const LonControllerConf &lon_controller_conf) {
  const auto &control_table = lon_controller_conf.calibration_table();
  AINFO << "Control calibration table loaded";
  AINFO << "Control calibration table size is "
        << control_table.calibration_table_size();
  // 插值
  Interpolation2D::DataType xyz;

  // 总轴距m
  wheelbase_ = vehicle_param_.wheelbase();
  // 左前质量（随着载重变化）kg
  const double mass_fl = vehicle_param_.mass_fl();
  const double mass_fr = vehicle_param_.mass_fr();
  const double mass_rl = vehicle_param_.mass_rl();
  const double mass_rr = vehicle_param_.mass_rr();
  // 前质量
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  // 总质量
  mass_ = mass_front + mass_rear;

  // 前向转向系数。根据车辆的轴距（wheelbase_）和前轮质量（mass_front）与整车质量（mass_）之比计算得出的，转向系数用于确定车辆在转弯时前后轮胎所受到的力的分配情况，进而影响车辆的操控性能
  // 前轮质量相对较轻时，转向系数较大，意味着车辆在转弯时前轮受到的力更多，有利于提高操控的灵活性和响应性；而当前轮质量相对较重时，转向系数较小，意味着车辆在转弯时前轮受到的力相对减小，有利于提高车辆的稳定性。
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  for (const auto &calibration : control_table.calibration_table()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  if (control_interpolation_->Init(xyz) == false)
    AERROR << "Fail to load control calibration table";
}

legionclaw::common::Status LonController::ComputeControlCommand(
    const legionclaw::interface::LocalizationEstimate *localization,
    const legionclaw::interface::Chassis *chassis,
    const legionclaw::interface::ADCTrajectory *trajectory,
    legionclaw::interface::ControlCommand *cmd,
    legionclaw::interface::ControlAnalysis *analysis) {
  int64_t current_time = legionclaw::common::TimeTool::Now2Us();
  calculate_period_ = (current_time - previous_timestamp_) * 1e-6;
  previous_timestamp_ = current_time;
  auto vehicle_state = injector_->vehicle_state();
  localization_ = localization;
  chassis_ = chassis;
  TrajectoryPoint target_point;

  // 判断车辆是否静止
  // 车辆静止表现为：车速度小于速度阈值
  // ，且持续时间大于max_abs_speed_when_stopped_duration（500ms）
  if (fabs(injector_->vehicle_state()->linear_velocity()) <
      vehicle_param_.max_abs_speed_when_stopped()) {
    // 第一次记录当前时间
    if (is_stopped_first_hit_ == true) {
      last_time_vehicle_is_stopped_or_not_ = TimeTool::Now2Ms();
      is_stopped_first_hit_ = false;
    }
    // 当前时间
    int64_t cur_time = TimeTool::Now2Ms();
    int64_t diff = cur_time - last_time_vehicle_is_stopped_or_not_;
    if (diff >= vehicle_param_.max_abs_speed_when_stopped_duration()) {
      is_stopped_ = true;
    } else {
      is_stopped_ = false;
    }
  } else {
    is_stopped_ = false;
    is_stopped_first_hit_ = true;
  }

  trajectory_message_ = trajectory;
  //  指针control_interpolation_非空 && 这里true
  if (!control_interpolation_ && use_calibration_table_) {
    AERROR << "Fail to initialize calibration table.";
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                  "Fail to initialize calibration table.");
  }

  if (trajectory_analyzer_ == nullptr ||
      trajectory_analyzer_->seq_num() != trajectory_message_->header().seq()) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
    trajectory_update_flag_ = true;
  } else {
    trajectory_update_flag_ = false;
  }

  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  simple_longitudinal_debug_ = std::make_shared<SimpleLongitudinalDebug>();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double preview_time = lon_controller_conf.preview_window();  // 100 * 0.02
  bool enable_leadlag =
      lon_controller_conf.enable_reverse_leadlag_compensation();  // 这里为false

  if (preview_time < 0.0) {
    stringstream sstream;
    sstream << "Preview time set as: " << preview_time << " less than 0";
    AERROR << sstream.str();
    return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR, sstream.str());
  }
  ComputeLongitudinalErrors(trajectory_analyzer_.get(), localization, chassis,
                            preview_time, ts_,
                            simple_longitudinal_debug_.get());

  // double pitch = 0;
  // double slope_offset_compenstaion = 0;
  // //坡度补偿
  // double pitch = digital_filter_pitch_angle_.Filter(
  //     injector_->vehicle_state()->pitch() + pitch_offset_);
  // pitch = (vehicle_state->gear() ==
  // legionclaw::common::GearPosition::GEAR_REVERSE)
  //             ? (-pitch)
  //             : pitch;

  // double slope_offset_compenstaion = lon_dynamics_->acc_grade(pitch);

  // if (std::isnan(slope_offset_compenstaion)) {
  //   slope_offset_compenstaion = 0;
  // }

  // /*针对throttle加速度控制的车，上坡取消坡度补偿*/
  // if (use_acceleration_mode_ && slope_offset_compenstaion > 0) {
  //   slope_offset_compenstaion = 0;
  // }

  // /*针对brake减速度控制的车，下坡取消坡度补偿*/
  // if (use_deceleration_mode_ && slope_offset_compenstaion < 0) {
  //   slope_offset_compenstaion = 0;
  // }

  // PID参数设置
  double station_error_limited = 0.0;
  if (enable_speed_station_preview_)  // false
  {
    station_error_limited =
        common::math::Clamp(simple_longitudinal_debug_->preview_station_error(),
                            -lon_controller_conf.station_error_limit(),  //-5
                            lon_controller_conf.station_error_limit());  // 5
  } else {
    station_error_limited =
        common::math::Clamp(simple_longitudinal_debug_->station_error(),
                            -lon_controller_conf.station_error_limit(),  //-5
                            lon_controller_conf.station_error_limit());  // 5
  }
  // TODO 大阻力标志 减速度和人为踩刹车条件 ？

  // 起步过慢PID参数调整
  // TODO 只适用起步不给油的怠速起步特性的车辆,具有局限性
  if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE) {
    station_pid_controller_.SetPID(
        lon_controller_conf.reverse_station_pid_conf());
    if (chassis_->speed_mps() < vehicle_param_.max_abs_speed_when_stopped() &&
        cmd->accel_value() != 0.0 && vehicle_speed_in_idle_ > 0) {
      speed_pid_controller_.SetPID(
          lon_controller_conf.large_resistance_reverse_speed_pid_conf());
      max_acceleration_jerk_ =
          lon_controller_conf.max_acceleration_jerk_in_large_resistance() * ts_;
    } else {
      speed_pid_controller_.SetPID(
          lon_controller_conf.reverse_speed_pid_conf());
      max_acceleration_jerk_ = vehicle_param_.max_acceleration_jerk() * ts_;
    }
    if (enable_leadlag)  // false
    {
      station_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_station_leadlag_conf());
      speed_leadlag_controller_.SetLeadlag(
          lon_controller_conf.reverse_speed_leadlag_conf());
    }
  } else if (injector_->vehicle_state()->linear_velocity() <=
             lon_controller_conf.switch_speed())  // switch_speed这里为5m/s
  {
    // vehicle_speed_in_idle_怠速时车速0.833,max_abs_speed_when_stopped 0.05
    if (chassis_->speed_mps() < vehicle_param_.max_abs_speed_when_stopped() &&
        cmd->accel_value() != 0.0 && vehicle_speed_in_idle_ > 0) {
      speed_pid_controller_.SetPID(
          lon_controller_conf.large_resistance_speed_pid_conf());
      max_acceleration_jerk_ =
          lon_controller_conf.max_acceleration_jerk_in_large_resistance() * ts_;
    } else  // 一般进入这个
    {
      speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
      max_acceleration_jerk_ = vehicle_param_.max_acceleration_jerk() * ts_;
    }
  } else {
    speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
  }

  // if (!(chassis_->gear_location() == legionclaw::common::GearPosition::GEAR_DRIVE
  // ||
  //       chassis_->gear_location() ==
  //           legionclaw::common::GearPosition::GEAR_REVERSE) ||
  //     chassis_->brake_driving_mode() !=
  //         legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE) {
  //   Reset();
  // }

  double speed_offset =
      station_pid_controller_.Control(station_error_limited, ts_);

  if (enable_leadlag)  // false
  {
    speed_offset = station_leadlag_controller_.Control(speed_offset, ts_);
  }
  double speed_controller_input = 0.0;

  if (enable_speed_station_preview_)  // false
  {
    speed_controller_input =
        speed_offset + simple_longitudinal_debug_->preview_speed_error();
  } else  // 一般进入这个
  {
    speed_controller_input =
        speed_offset + simple_longitudinal_debug_->speed_error();
  }
  AINFO << "*******************************************************************"
           "***";
  AINFO << "实际速度差         :" << simple_longitudinal_debug_->speed_error();
  AINFO << "位置pid输出(速度差）  :" << speed_offset;

  double speed_controller_input_limited = 0.0;
  speed_controller_input_limited = common::math::Clamp(
      speed_controller_input,
      -lon_controller_conf.speed_controller_input_limit(),  //-3
      lon_controller_conf.speed_controller_input_limit());  // 3
  AINFO << "位置pid输入(限制后) :" << station_error_limited;
  AINFO << "速度pid输入(限制后) :" << speed_controller_input_limited;

  // acc PID计算，输入速度差值，采样时间间隔
  double acceleration_cmd_closeloop = 0.0;
  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input_limited, ts_);

  simple_longitudinal_debug_->set_pid_saturation_status(
      speed_pid_controller_.IntegratorSaturationStatus());

  // 超前滞后控制器
  if (enable_leadlag)  // false
  {
    acceleration_cmd_closeloop =
        speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts_);
    simple_longitudinal_debug_->set_leadlag_saturation_status(
        speed_leadlag_controller_.InnerstateSaturationStatus());
  }

  // 根据给定的空气密度、物体横截面积、速度和质量计算出物体在空气阻力作用下的加速度
  double aero_offset_compensation =
      lon_dynamics_->acc_aero(chassis->speed_mps());
  simple_longitudinal_debug_->set_aero_offset_compensation(
      aero_offset_compensation);
  // 根据车辆当前的速度、转向角度以及车辆的尺寸特性（如前后轴距）来计算车辆在转弯时的加速度
  double turning_offset_compensation = lon_dynamics_->acc_turning(
      chassis->speed_mps(), chassis->front_steering_value() / steer_ratio_);
  simple_longitudinal_debug_->set_turning_offset_compensation(
      turning_offset_compensation);
  // double rolling_offset_compensation = lon_dynamics_->acc_rolling(pitch);
  // simple_longitudinal_debug_->set_rolling_offset_compensation(rolling_offset_compensation);

  // 计算acc
  double acceleration_cmd =
      acceleration_cmd_closeloop +
      simple_longitudinal_debug_->preview_acceleration_reference() +
      enable_aero_offset_ *
          simple_longitudinal_debug_->aero_offset_compensation() +
      enable_turning_offset_ *
          simple_longitudinal_debug_->turning_offset_compensation();
  AINFO << "acceleration_cmd                     (总加速度指令):"
        << acceleration_cmd;
  AINFO << "acceleration_cmd_closeloop         (闭环加速度指令):"
        << acceleration_cmd_closeloop;
  AINFO << "aero_offset_compensation       (空气阻力加速度指令):"
        << simple_longitudinal_debug_->aero_offset_compensation();
  AINFO << "turning_offset_compensation        (转向加速度指令):"
        << simple_longitudinal_debug_->turning_offset_compensation();
  AINFO << "preview_acceleration_reference (预览参考加速度指令):"
        << simple_longitudinal_debug_->preview_acceleration_reference();

  if (acceleration_cmd > 0 && previous_acceleration_cmd_ < 0) {
    previous_acceleration_cmd_ = 0.0;
  }
  // std::cout<<"acceleration_cmd01:"<<acceleration_cmd<<"\n";
  /*车辆处于提前停车的状态下，此时acc_jerk无法发挥作用*/
  // if (!(chassis_->gear_location() == legionclaw::common::GearPosition::GEAR_DRIVE
  // ||
  //       chassis_->gear_location() ==
  //           legionclaw::common::GearPosition::GEAR_REVERSE) ||
  //     chassis_->brake_driving_mode() ==
  //         legionclaw::common::DrivingMode::MANUAL_INTERVENTION ||
  //     chassis_->epb_level() != legionclaw::common::EPBLevel::RELEASED) {
  //   acceleration_cmd = 0;
  //   Reset();
  // }
  double lon_cmd_jerk = (acceleration_cmd - previous_acceleration_cmd_) / ts_;
  // if (lon_cmd_jerk > max_acceleration_jerk_ && acceleration_cmd >= 0) {
  //   acceleration_cmd = previous_acceleration_cmd_ + max_acceleration_jerk_;
  // }
  analysis->set_lon_acc_jerk(lon_cmd_jerk);

  previous_acceleration_cmd_ = acceleration_cmd;
  // 是否考虑斜坡（是）,这里暂时没有用到
  acceleration_cmd += enable_slope_offset_ *
                      simple_longitudinal_debug_->slope_offset_compensation();

  simple_longitudinal_debug_->set_is_full_stop(false);

  GetPathRemain(simple_longitudinal_debug_.get());
  // 不考虑坡度 给定需求减速度停车距离计算，包括刹车时延
  if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE) {
    brake_delay_time_ = lon_controller_conf.brake_delay_time_r();
  } else if (vehicle_state->gear() ==
             legionclaw::common::GearPosition::GEAR_DRIVE) {
    brake_delay_time_ = lon_controller_conf.brake_delay_time_d();
  }

  // 计算车辆停下时还需行驶的路径长度
  double path_remain_when_stopped =
      chassis_->speed_mps() * brake_delay_time_ - 0.5 * chassis_->speed_mps() *
                                                      chassis_->speed_mps() /
                                                      standstill_acceleration_;
  path_remain_when_stopped = common::math::Clamp(
      path_remain_when_stopped, control_conf_->min_path_remain_when_stopped(),
      control_conf_->max_path_remain_when_stopped());

  double speed_reference;
  // 正常状态下
  if (!(trajectory_message_->behaviour_lat_state() ==
            legionclaw::interface::ADCTrajectory::BehaviourLatState::
                VERTICAL_PARKING_FORWARD_STATE ||
        trajectory_message_->behaviour_lat_state() ==
            legionclaw::interface::ADCTrajectory::BehaviourLatState::
                VERTICAL_PARKING_BACKWARD_STATE)) {
    path_remain_when_stopped = control_conf_->max_path_remain_when_stopped();
    speed_reference = simple_longitudinal_debug_->preview_speed_reference();
  } else  // 处于垂直停车状态
  {
    speed_reference = simple_longitudinal_debug_->speed_reference();
  }

  // At near-stop stage, replace the brake control command with the standstill
  // acceleration if the former is even softer than the latter
  if ((std::fabs(
           simple_longitudinal_debug_->preview_acceleration_reference()) <=
           control_conf_->max_acceleration_when_stopped() &&
       std::fabs(simple_longitudinal_debug_->preview_speed_reference()) <=
           vehicle_param_.max_abs_speed_when_stopped()) ||
      std::abs(simple_longitudinal_debug_->path_remain()) <
          path_remain_when_stopped) {
    acceleration_cmd = std::min(acceleration_cmd, standstill_acceleration_);
    ADEBUG << "Stop location reached";
    simple_longitudinal_debug_->set_is_full_stop(true);
  }

  // double throttle_lowerbound =
  // std::max(vehicle_param_.accel_deadzone(),lon_controller_conf.throttle_minimum_action());
  // double brake_lowerbound =
  // std::min(vehicle_param_.brake_deadzone(),lon_controller_conf.brake_minimum_action());

  // double calibration_value = 0.0;
  // double compute_value = 0.0;
  double output_value = 0.0;

  double acceleration_lookup = acceleration_cmd;
  // std::cout<<"chassis_->speed_mps(): "<<chassis_->speed_mps()<<"\n";
  // std::cout<<"acceleration_lookup: "<<acceleration_lookup<<"\n";
  // if (use_calibration_table_) {
  //   if (use_preview_speed_for_table_) {
  //     calibration_value = control_interpolation_->Interpolate(
  //         std::make_pair(simple_longitudinal_debug_->preview_speed_reference(),
  //                        acceleration_lookup));
  //   } else {
  //     calibration_value = control_interpolation_->Interpolate(
  //         std::make_pair(chassis_->speed_mps(), acceleration_lookup));
  //   }

  //   // std::cout<<"chassis_->speed_mps():
  //   "<<chassis_->speed_mps()<<"\n";
  //   // std::cout<<"acceleration_lookup: "<<acceleration_lookup<<"\n";
  //   // std::cout<<"calibration_value: "<<calibration_value<<"\n";
  //   output_value = calibration_value;
  // } else {
  //   // TODO(jiang):计算值根据车辆情况来定
  //   // output_value = compute_value;
  //   output_value = simple_longitudinal_debug_->speed_reference();
  // }

  // 怠速减速度，油车低速不存在怠速减速度，怠速
  //  double deceleration_in_idle = vehicle_param_.deceleration_in_idle();
  //  // TODO怠速控制，需进一步标定怠速起步加速曲线
  //  IdleSpeedControlOfOilCar(speed_reference,
  //                           deceleration_in_idle, acceleration_lookup,
  //                           output_value);
  //
  AINFO << "闭环加速度指令（限制前）：" << acceleration_lookup;
  acceleration_lookup = common::math::Clamp(acceleration_lookup,
                                            vehicle_param_.max_deceleration(),
                                            vehicle_param_.max_acceleration());
  AINFO << "闭环加速度指令（限制后）：" << acceleration_lookup;

  // 油门限制
  if (acceleration_lookup >= 0) {
    throttle_cmd = (accel_value_kp_ * acceleration_lookup) > max_accel_value_
                       ? max_accel_value_
                       : (accel_value_kp_ * acceleration_lookup);
  }
  AINFO << "油门开度    :" << throttle_cmd;
  AINFO << "车辆速度m/s :" << injector_->vehicle_state()->linear_velocity();
  // 起步油门限制
  //  if(chassis_->speed_mps() < 0.05)
  //  {
  //    throttle_cmd = (throttle_cmd > 4)?4 :throttle_cmd;
  //  }
  //  if (acceleration_lookup >= 0) {
  //    is_in_brake_ = false;
  //    if (output_value >= 0) {
  //      throttle_cmd = std::max(output_value, throttle_lowerbound);
  //    } else {
  //      throttle_cmd = throttle_lowerbound;
  //    }
  //    brake_cmd = 0.0;
  //  } else {
  //    throttle_cmd = 0.0;
  //    if (acceleration_lookup < deceleration_in_idle)
  //      is_in_brake_ = true;
  //    if (output_value >= 0) {
  //      brake_cmd = 0.0;
  //    } else {
  //      // if (trajectory_message_->adc_trajectory_type() !=
  //      //
  //      legionclaw::interface::ADCTrajectory::ADCTrajectoryType::ADCTrajectory_TYPE_OBSTACLE)
  //      //     //前方无障碍物
  //      if (trajectory_message_->behaviour_state() ==
  //              legionclaw::interface::ADCTrajectory::BehaviourState::GOAL_STATE
  //              ||
  //          trajectory_message_->adc_trajectory_type() ==
  //              legionclaw::interface::ADCTrajectory::BehaviourState::
  //                  FINISH_STATE ||
  //          IsEventFlagTrue("has_stop_point") == false)  //前方无障碍物
  //        brake_cmd = output_value;
  //      else {
  //        is_in_brake_ = true;  //前方障碍物标志
  //        brake_cmd = std::min(output_value, brake_lowerbound);
  //      }
  //    }
  //    if (is_in_brake_)
  //      brake_cmd = kp_brake_ * brake_cmd;
  //    else
  //      brake_cmd = 0.0;
  //  }
  //  throttle_cmd = throttle_cmd + 4;
  // 缓刹车场景：下坡减速度控制策略，低幅高频脉冲函数 Impulse function
  //  Impulse(lon_controller_conf.downhill_scene_conf(),pitch,brake_cmd,throttle_cmd);

  simple_longitudinal_debug_->set_station_error_limited(station_error_limited);
  simple_longitudinal_debug_->set_speed_offset(speed_offset);
  simple_longitudinal_debug_->set_speed_controller_input_limited(
      speed_controller_input_limited);
  simple_longitudinal_debug_->set_acceleration_cmd(acceleration_cmd);
  // simple_longitudinal_debug_->set_throttle_cmd(acceleration_cmd * 10);
  simple_longitudinal_debug_->set_throttle_cmd(throttle_cmd);
  simple_longitudinal_debug_->set_brake_cmd(brake_cmd);
  simple_longitudinal_debug_->set_acceleration_lookup(acceleration_lookup);
  simple_longitudinal_debug_->set_speed_lookup(chassis_->speed_mps());
  simple_longitudinal_debug_->set_calibration_value(output_value);
  simple_longitudinal_debug_->set_acceleration_cmd_closeloop(
      acceleration_cmd_closeloop);

  // if the car is driven by acceleration, disgard the cmd->throttle and brake
  cmd->set_accel_value(throttle_cmd);
  cmd->set_brake_value(brake_cmd);
  cmd->set_acceleration(acceleration_cmd);
  target_point = simple_longitudinal_debug_->current_matched_point();

  // 提前到站逻辑,避免临界区车辆档位来回切换的问题
  if (simple_longitudinal_debug_->is_full_stop() &&
      simple_longitudinal_debug_->current_speed() <
          vehicle_param_.max_abs_speed_when_stopped()) {
    gear_sm_debug_.set_tar_gear(legionclaw::common::GearPosition::GEAR_NEUTRAL);
  } else {
    gear_sm_debug_.set_tar_gear(target_point.gear());
  }

  gear_sm_debug_.set_cur_gear(chassis_->gear_location());
  gear_sm_debug_.set_cur_epb_state(chassis_->epb_level());
  gear_sm_debug_.set_is_stopped(is_stopped_);

  // 是否使能拉epb逻辑 enable_epb_applied case A: 车辆静止时间过长
  // B:finish状态&方向盘处于零位状态
  if (is_stopped_ &&
      gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_DRIVE &&
      gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_REVERSE) {
    if (is_enable_epb_applied_first_hit_ == true) {
      last_time_enable_epb_applied_ = TimeTool::Now2Ms();
      is_enable_epb_applied_first_hit_ = false;
    }
    int64_t cur_time = TimeTool::Now2Ms();
    int64_t diff = cur_time - last_time_enable_epb_applied_;
    if (diff >= control_conf_->max_abs_speed_when_epb_applied_duration() ||
        ((trajectory_message_->behaviour_lat_state() ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::
                  PARKING_FINISH_STATE) &&
         fabs(chassis_->front_steering_value()) <
             control_conf_->is_in_zero_steer_angle())) {
      enable_epb_applied_ = true;
    } else {
      enable_epb_applied_ = false;
    }
  } else {
    is_enable_epb_applied_first_hit_ = true;
    enable_epb_applied_ = false;
  }
  gear_sm_debug_.set_enable_epb_applied(enable_epb_applied_);

  gear_sm_debug_.set_brake_value_when_gear_transitioning(std::min(
      brake_cmd, vehicle_param_.brake_value_when_gear_transitioning()));

  // 档位和EPB切换逻辑
  GearTransition(cmd);
  // std::cout<<"throttle_cmd: "<<cmd->accel_value()<<"\n";
  // std::cout<<"brake_value: "<<cmd->brake_value()<<"\n";
  if (gear_sm_) {
    gear_sm_->OnUpdate();
  }

  if (epb_sm_) {
    epb_sm_->OnUpdate();
  }

  switch (gear_trans_state_) {
    case GearTransState::SUCCESS:
      ADEBUG << "gear transition success";
      break;
    case GearTransState::TRANSITIONING:
      ADEBUG << "gear transitioning";
      break;
    case GearTransState::FAILED:
      ADEBUG << "gear transition failed";
      break;
    default:
      break;
  }

  // if (cmd->brake_value() != 0.0 ||
  //     chassis_->brake_driving_mode() ==
  //         legionclaw::common::DrivingMode::MANUAL_INTERVENTION) {
  //   cmd->set_accel_value(0.0);
  // }

  // 刹车最大值限制，由于目前障碍物会误入导致产生很大刹车，暂时采用该方法，避免高速急刹车
  if (simple_longitudinal_debug_->current_speed() > 2.0 ||
      simple_longitudinal_debug_->preview_acceleration_reference() <
          vehicle_param_.max_deceleration()) {
    brake_cmd = std::max(cmd->brake_value(), vehicle_param_.max_brake_value());
  }

  brake_cmd = std::min(cmd->brake_value(), brake_cmd);
  // 刹车符号处理
  if (brake_sign_ == 1) {
    brake_cmd = -1.0 * brake_cmd;
  }
  cmd->set_brake_value(brake_cmd);

  // 电车在起步前，先给减速度再给0，车有可能不会走，怠速起步逻辑失效，必须给油车才能走
  if (chassis->epb_level() != legionclaw::common::EPBLevel::RELEASED) {
    cmd->set_accel_value(0.0);
    cmd->set_brake_value(0.0);
  }
  cmd->set_accel_driving_mode(trajectory->driving_mode());
  cmd->set_brake_driving_mode(trajectory->driving_mode());
  cmd->set_epb_driving_mode(trajectory->driving_mode());
  cmd->set_gear_driving_mode(trajectory->driving_mode());
  // 上一次驱动命令大于当前值，刹车灯亮，否则不亮
  if (last_accel_value_ > cmd->accel_value()) {
    cmd->set_brake_lamp_ctrl(legionclaw::common::SwitchStatus::SWITCH_STATUS_ON);
  } else {
    cmd->set_brake_lamp_ctrl(legionclaw::common::SwitchStatus::SWITCH_STATUS_OFF);
  }
  last_accel_value_ = cmd->accel_value();

  if (lon_print_enable_) {
    static int index_display = 0;
    if (index_display == 1)  //
    {
      index_display = 0;

      printf("\033[2J");
      std::cout.precision(5);
      std::cout << "--------------longitidute info---------" << endl;
      std::cout << "enable: " << cmd->accel_driving_mode()
                << "   enable_log_debug_: " << enable_log_debug_
                << "   calculate_period_: " << calculate_period_
                << "   lon_calculate_time: " << lon_calculate_time_ << endl;
      std::cout << endl;
      std::cout << "simple_longitudinal_debug_->path_remain: "
                << simple_longitudinal_debug_->path_remain() << endl;
      std::cout << "station_error: "
                << simple_longitudinal_debug_->station_error()
                << "    station_error_limited: "
                << simple_longitudinal_debug_->station_error_limited()
                << "  pid_speed_offset:"
                << simple_longitudinal_debug_->speed_offset() << endl;
      std::cout << endl;
      std::cout << "speed_reference: "
                << simple_longitudinal_debug_->speed_reference()
                << "  current_speed: "
                << simple_longitudinal_debug_->current_speed()
                << "   speed_error(): "
                << simple_longitudinal_debug_->speed_error() << endl;
      std::cout << "speed_controller_input_limited ="
                << simple_longitudinal_debug_->speed_controller_input_limited()
                << "  acceleration_cmd_speed_pid: "
                << simple_longitudinal_debug_->acceleration_cmd_closeloop()
                << endl;
      std::cout << endl;
      std::cout << "preview_acceleration_reference(): "
                << simple_longitudinal_debug_->preview_acceleration_reference()
                << "  slope_offset_compensation():"
                << simple_longitudinal_debug_->slope_offset_compensation()
                << endl;
      std::cout << "turning_offset_compensation():"
                << simple_longitudinal_debug_->turning_offset_compensation()
                << endl;
      std::cout << "acceleration_cmd: "
                << simple_longitudinal_debug_->acceleration_cmd()
                << "  acceleration_lookup: " << acceleration_lookup << endl;
      std::cout << endl;
      std::cout << "output_value : " << output_value << "  jerk_reference : "
                << simple_longitudinal_debug_->jerk_reference()
                << "  lon_cmd_jerk : " << lon_cmd_jerk << endl;
      std::cout << "accel_value: " << cmd->accel_value()
                << "  brake_value : " << cmd->brake_value()
                << "  has_stop_point :" << IsEventFlagTrue("has_stop_point")
                << "  is_stopped :" << gear_sm_debug_.is_stopped()
                << "  is_full_stop :"
                << simple_longitudinal_debug_->is_full_stop() << endl;

      std::cout << endl;
      std::cout << "gear_location: " << cmd->gear_location()
                << "  epb_level: " << cmd->epb_level() << endl;
      std::cout << "gear_location_fb: " << chassis_->gear_location()
                << "  epb_level_fb: " << chassis_->epb_level() << endl;
      std::cout << endl;
    }
    index_display++;
  }

  if (enable_log_debug_ && speed_log_file_.is_open()) {
    stringstream sstream;
    sstream << TimeTool::Now2Ms() << "," << cmd->accel_driving_mode() << ","
            << cmd->accel_value() << "," << cmd->brake_driving_mode() << ","
            << cmd->brake_value() << "," << cmd->epb_driving_mode() << ","
            << cmd->epb_level() << "," << cmd->gear_driving_mode() << ","
            << cmd->gear_location() << ","
            << simple_longitudinal_debug_->station_reference() << ","
            << simple_longitudinal_debug_->station_error() << ","
            << simple_longitudinal_debug_->station_error_limited() << ","
            << simple_longitudinal_debug_->speed_offset() << ","
            << simple_longitudinal_debug_->preview_station_error() << ","
            << simple_longitudinal_debug_->speed_reference() << ","
            << simple_longitudinal_debug_->current_speed() << ","
            << simple_longitudinal_debug_->speed_error() << ","
            << simple_longitudinal_debug_->speed_controller_input_limited()
            << "," << simple_longitudinal_debug_->acceleration_cmd_closeloop()
            << "," << simple_longitudinal_debug_->preview_speed_reference()
            << "," << simple_longitudinal_debug_->preview_speed_error() << ","
            << simple_longitudinal_debug_->preview_acceleration_reference()
            << "," << simple_longitudinal_debug_->turning_offset_compensation()
            << "," << simple_longitudinal_debug_->slope_offset_compensation()
            << "," << simple_longitudinal_debug_->acceleration_cmd() << ","
            << simple_longitudinal_debug_->acceleration_lookup() << ","
            << simple_longitudinal_debug_->speed_lookup() << "," << output_value
            << "," << throttle_cmd << "," << brake_cmd << ","
            << simple_longitudinal_debug_->is_full_stop() << ","
            << lon_calculate_time_ << "," << IsEventFlagTrue("has_stop_point")
            << ",";
    speed_log_file_ << sstream.str() << "\n";
    // AWARN << sstream.str();
  }

  SetAnalysis(cmd, simple_longitudinal_debug_.get(), chassis, analysis);

  lon_calculate_time_ = legionclaw::common::TimeTool::Now2Us() - current_time;
  lon_calculate_time_max_ = max(lon_calculate_time_, lon_calculate_time_max_);
  analysis->set_lon_calculate_time(lon_calculate_time_);
  analysis->set_lon_calculate_time_max(lon_calculate_time_max_);
  return Status::Ok();
}

void LonController::SetAnalysis(const legionclaw::interface::ControlCommand *cmd,
                                const SimpleLongitudinalDebug *debug,
                                const legionclaw::interface::Chassis *chassis,
                                legionclaw::interface::ControlAnalysis *analysis) {
  analysis->set_gear_location_fd(chassis_->gear_location());
  analysis->set_gear_location_cmd(cmd->gear_location());
  analysis->set_epb_level_fd(chassis_->epb_level());
  analysis->set_epb_level_cmd(cmd->epb_level());
  analysis->set_speed_mps(debug->current_speed());
  analysis->set_speed_reference(debug->speed_reference());
  analysis->set_accel_value_fd(chassis_->accel_value());
  analysis->set_accel_value_cmd(cmd->accel_value());
  analysis->set_brake_value_fd(chassis_->brake_value());
  analysis->set_brake_value_cmd(cmd->brake_value());
  analysis->set_path_remain(debug->path_remain());
  analysis->set_has_stop_point(IsEventFlagTrue("has_stop_point"));
  analysis->set_is_full_stop(debug->is_full_stop());
  analysis->set_is_stopped(gear_sm_debug_.is_stopped());
  analysis->set_acceleration_cmd(debug->acceleration_cmd());
  analysis->set_acceleration_cmd_closeloop(debug->acceleration_cmd_closeloop());
  analysis->set_preview_acceleration_reference(
      debug->preview_acceleration_reference());
  analysis->set_slope_offset_compensation(debug->slope_offset_compensation());
  analysis->set_turning_offset_compensation(
      debug->turning_offset_compensation());
  analysis->set_speed_error_limited(debug->speed_controller_input_limited());
  analysis->set_speed_error(debug->speed_error());
  analysis->set_speed_offset(debug->speed_offset());
  analysis->set_station_error_limited(debug->station_error_limited());
  analysis->set_station_error(debug->station_error());
  analysis->set_lon_target_point_s(debug->station_reference());
}

Status LonController::Reset() {
  speed_pid_controller_.Reset();
  station_pid_controller_.Reset();
  return Status::Ok();
}

std::string LonController::Name() const { return name_; }

void LonController::ComputeLongitudinalErrors(
    const TrajectoryAnalyzer *trajectory_analyzer,
    const legionclaw::interface::LocalizationEstimate *localization,
    const legionclaw::interface::Chassis *chassis, const double preview_time,
    const double ts, SimpleLongitudinalDebug *debug) {
  // the decomposed vehicle motion onto Frenet frame
  // s: longitudinal accumulated distance along reference trajectory
  // s_dot: longitudinal velocity along reference trajectory
  // d: lateral distance w.r.t. reference trajectory
  // d_dot: lateral distance change rate, i.e. dd/dt

  // s: 纵向累积走过的距离沿着参考轨迹
  // s_dot: 纵向沿着参考轨迹的速度
  // d: 相对参考轨迹的横向距离
  // d_dot: 横向距离的变化率
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  auto vehicle_state = injector_->vehicle_state();

  // 这里true
  if (use_new_planning_mode_) {
    add_lr_ = 0.0;
  } else {
    switch (state_machine) {
      case 0:
        add_lr_ = lr_;
        if (vehicle_state->gear() == legionclaw::common::GEAR_REVERSE) {
          state_machine = 1;
        } else {
          break;
        }
      case 1:
        add_lr_ = 0.0;
        if (vehicle_state->gear() == legionclaw::common::GEAR_DRIVE) {
          state_machine = 0;
        } else {
          break;
        }
      default:
        break;
    }
  }
  // 实质输出与定位坐标一致
  const auto &com = vehicle_state->ComputeCOMPosition(add_lr_);

  // 找到距离当前位置最近的参考轨迹点(PathPoint)
  auto matched_point =
      trajectory_analyzer->QueryMatchedPathPoint(com.x(), com.y());

  // 找到距离当前位置最近的参考轨迹点(TrajectoryPoint)
  auto current_point =
      trajectory_analyzer->QueryNearestPointByPosition(com.x(), com.y());

  // 计算纵向误差相关参数
  trajectory_analyzer->ToTrajectoryFrame(
      com.x(), com.y(), vehicle_state->heading(),
      fabs(vehicle_state->linear_velocity()), matched_point, &s_matched,
      &s_dot_matched, &d_matched, &d_dot_matched);

  // if (current_point.gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
  // {
  //   s_matched = s_matched * -1.0;
  // }

  double current_control_time = TimeTool::NowToSeconds();
  double preview_control_time = current_control_time + preview_time;

  current_point.set_path_point(matched_point);
  debug->set_current_matched_point(current_point);

  TrajectoryPoint reference_point;
  TrajectoryPoint preview_point;

  // if (current_control_time - trajectory_analyzer->header_time() >
  //     2 * trajectory_update_duration_)  // trajectory_update_duration_ (0.1 *
  //                                       // 1000)
  //   trajectory_update_flag_ = false;
  // else
  //   trajectory_update_flag_ = true;

  if (IsEventFlagTrue("has_stop_point") == false) {
    reference_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(
        current_control_time, query_forward_time_point_only);
    preview_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(
        preview_control_time, query_forward_time_point_only);
    // AINFO <<"Update V = : " << reference_point.v() <<endl;
  } else {
    reference_point = debug->current_matched_point();
    preview_point = debug->current_matched_point();
  }

  debug->set_current_reference_point(reference_point);
  debug->set_preview_reference_point(preview_point);

  double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                      matched_point.theta());
  // 修复人工驾驶状态下车速设置的bug
  if (std::isnan(heading_error) ||
      chassis->accel_driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_MANUAL) {
    heading_error = 0;
  }
  if (std::isnan(s_dot_matched)) {
    s_dot_matched = 0;
  }

  double lon_speed =
      fabs(vehicle_state->linear_velocity()) * std::cos(heading_error);
  double lon_acceleration =
      vehicle_state->linear_acceleration() * std::cos(heading_error);

  double one_minus_kappa_lat_error =
      1 - reference_point.path_point().kappa() *
              fabs(vehicle_state->linear_velocity()) * std::sin(heading_error);

  debug->set_station_reference(reference_point.path_point().s());
  debug->set_current_station(s_matched);
  double diff_s = reference_point.path_point().s() - s_matched;
  // AWARN << "00-" << reference_point.path_point().s() << "-" << s_matched;
  if (std::isnan(diff_s)) {
    diff_s = 0;
  }

  debug->set_station_error(diff_s);
  // std::cout << "station_error  and heading error " << diff_s  <<" "<<
  // heading_error << endl;

  /// 方案二,起步最小速度保护
  if (reference_point.v() < starting_speed_ && reference_point.a() > 0.0)
    debug->set_speed_reference(starting_speed_);
  else
    debug->set_speed_reference(min(reference_point.v(), speed_limit_));

  debug->set_current_speed(lon_speed);

  // 速度误差speed_error=Vdes −V∗cosΔθ/k
  // Vdes 为期望车辆线速度，V 为当前车辆线速度，Δθ 为航向误差，k
  // 为系数，即代码中的one_minus_kappa_r_d
  debug->set_speed_error(debug->speed_reference() - s_dot_matched);
  debug->set_acceleration_reference(reference_point.a());

  debug->set_current_acceleration(lon_acceleration);
  debug->set_acceleration_error(reference_point.a() -
                                lon_acceleration / one_minus_kappa_lat_error);
  double jerk_reference =
      (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
  double lon_jerk =
      (debug->current_acceleration() - previous_acceleration_) / ts;

  debug->set_jerk_reference(jerk_reference);
  debug->set_current_jerk(lon_jerk);
  debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
  previous_acceleration_reference_ = debug->acceleration_reference();
  previous_acceleration_ = debug->current_acceleration();

  debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
  debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
  debug->set_preview_speed_reference(preview_point.v());
  debug->set_preview_acceleration_reference(preview_point.a());
}

void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                     common::DigitalFilter *digital_filter) {
  std::vector<double> denominators;
  std::vector<double> numerators;
  common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
  digital_filter->set_coefficients(denominators, numerators);
}

// 查找给定轨迹中是否包含停车点（GEAR_PARKING），并计算当前点到最近停车点的距离或到路径末尾的距离
void LonController::GetPathRemain(SimpleLongitudinalDebug *debug) {
  // 定义停车速度
  static constexpr double kParkingSpeed = 0.1;
  // 定义起始和结束索引
  uint32_t stop_index = 0;
  uint32_t stop_index_low = 0;
  uint32_t stop_index_high = trajectory_message_->trajectory_points_size() - 1;
  // 定义目标点
  TrajectoryPoint target_point = debug->current_matched_point();
  // 初始化没有停车点的标志
  SetEventFlag("has_stop_point", false);
  // 二分查找法在轨迹中查找停车点
  while (stop_index_low <= stop_index_high) {
    stop_index = (stop_index_low + stop_index_high) / 2;
    if (stop_index == 0 ||
        stop_index == trajectory_message_->trajectory_points_size() - 1) {
      break;
    }
    if (trajectory_message_->trajectory_points(stop_index).gear() ==
            legionclaw::common::GearPosition::GEAR_PARKING &&
        trajectory_message_->trajectory_points(stop_index - 1).gear() !=
            legionclaw::common::GearPosition::GEAR_PARKING) {
      SetEventFlag("has_stop_point", true);
      break;
    } else {
      if (trajectory_message_->trajectory_points(stop_index).gear() ==
          legionclaw::common::GearPosition::GEAR_PARKING) {
        stop_index_high = stop_index - 1;
      } else {
        stop_index_low = stop_index + 1;
      }
    }
  }

  if (stop_index == 0) {
    debug->set_path_remain(-1);
    SetEventFlag("has_stop_point", true);
  } else {
    debug->set_path_remain(
        trajectory_message_->trajectory_points(stop_index).path_point().s() -
        debug->current_station());
  }

  // 若轨迹最后一个点的速度小于停车速度，则将该点作为停车点
  if (stop_index == trajectory_message_->trajectory_points_size() - 1) {
    if (fabs(trajectory_message_->trajectory_points(stop_index).v()) <
        kParkingSpeed) {
      SetEventFlag("has_stop_point", true);
      ADEBUG << "the last point is selected as parking point";
    } else {
      ADEBUG << "the last point found in path and speed > speed_deadzone";
    }
    AINFO << "*****************************************************************"
             "***";
  }

  // if (debug->path_remain() < 0) {
  //   std::cout <<
  //   "************************************异常*********************"
  //                "**************************************************:::"
  //             << endl;
  //   std::cout << "trajectory_points_size():::"
  //             << trajectory_message_->trajectory_points_size() << endl;
  //   std::cout << "stop_index:::" << stop_index << endl;
  //   std::cout
  //       << "path_point().s():::"
  //       << trajectory_message_->trajectory_point(stop_index).path_point().s()
  //       << endl;
  //   std::cout << "debug->current_station():::" << debug->current_station()
  //             << endl;
  //   std::cout << "debug->path_remain():::" << debug->path_remain() << endl;
  // }
}

void LonController::StateMachineSpin(void *param) {
  if (gear_sm_) gear_sm_->OnUpdate();
  if (epb_sm_) epb_sm_->OnUpdate();
}

void LonController::Impulse(const DownhillSceneConf &downhill_scene_conf,
                            const double &pitch, double &brake_cmd,
                            double &throttle_cmd) {
  if (!downhill_scene_conf.enable_brake_impulse()) return;
  static int brake_duration_count = 0;
  static int throttle_duration_count = 0;
  bool is_downhill_scene = false;
  if (pitch < D2R(downhill_scene_conf.downhill_scene_pitch()) &&
      IsEventFlagTrue("has_stop_point") == false) {
    is_downhill_scene = true;
  }
  if (is_downhill_scene) {
    if (brake_duration_count <= downhill_scene_conf.braking_duration() / ts_) {
      throttle_cmd = 0.0;
      brake_cmd = downhill_scene_conf.brake_amplitude();
      brake_duration_count++;
      throttle_duration_count = 0;
    } else if (throttle_duration_count <=
               downhill_scene_conf.driving_duration() / ts_) {
      brake_cmd = downhill_scene_conf.min_brake_amplitude();
      throttle_cmd = 0.0;
      throttle_duration_count++;
    } else {
      brake_duration_count = 0;
      throttle_cmd = 0.0;
      brake_cmd = downhill_scene_conf.brake_amplitude();
    }
  } else {
    brake_duration_count = 0;
    throttle_duration_count = 0;
  }
}

void LonController::IdleSpeedControlOfOilCar(const double &speed_reference,
                                             double &deceleration_in_idle,
                                             double &acceleration_lookup,
                                             double &output_value) {
  // 配置 vehicle_speed_in_idle 为 0,则取消怠速逻辑
  if (speed_reference < vehicle_speed_in_idle_) {
    deceleration_in_idle = deceleration_in_idle_speed_;
    double starting_acceleration = starting_acceleration_in_d_;
    if (chassis_->gear_location() == legionclaw::common::GearPosition::GEAR_REVERSE)
      starting_acceleration = starting_acceleration_in_r_;
    // 限制怠速给油门的条件,当需求的加速度可以被自车的怠速加速度抵消时，不需要额外下发油门
    if (acceleration_lookup >= 0 &&
        acceleration_lookup < starting_acceleration) {
      acceleration_lookup = deceleration_in_idle;
      // 由于刹车存在时延，对车速进行一段预测
      double pre_vehicle_speed =
          chassis_->speed_mps() + starting_acceleration * brake_delay_time_;
      // 如果预测车速会大于目标速度，则提前让刹车进行介入
      if (pre_vehicle_speed > simple_longitudinal_debug_->speed_reference()) {
        output_value = vehicle_param_.brake_deadzone();
      }
    }
  }
  return;
}

}  // namespace control
}  // namespace legionclaw
