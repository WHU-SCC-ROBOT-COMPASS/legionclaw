#include "lon_speed_controller.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <string>
#include <utility>

#include "common/logging/logging.h"
#include "common/math/math_utils.h"
#include "common/time/time_tool.h"

#include "modules/control/src/controller/lon_speed_controller/lon_speed_gear_state_machine.hpp"
#include "modules/control/src/controller/lon_speed_controller/lon_speed_brake_state_machine.hpp"


namespace legionclaw
{
  namespace control
  {
    using namespace legionclaw::common;
    using namespace legionclaw::common::math;
    constexpr double GRA_ACC = 9.8;

    LonSpeedController::LonSpeedController() : name_("LonSpeed Longitudinal Controller")
    {
      AINFO << "Using " << name_;
    }

    LonSpeedController::~LonSpeedController() 
    {  }

    std::string LonSpeedController::Name() const { return name_; }

    Status LonSpeedController::Init(std::shared_ptr<DependencyInjector> injector, const ControlConf *control_conf)
    {
      injector_ = injector;
      control_conf_ = control_conf;
      if (control_conf_ == nullptr)
      {
        controller_initialized_ = false;
        AERROR << "get_longitudinal_param() nullptr";
        return Status(Status::ErrorCode::CONTROL_INIT_ERROR,"Failed to load LonSpeedController conf");
      }
      vehicle_param_ = control_conf->vehicle_param();
      const LonSpeedControllerConf &lon_speed_controller_conf = control_conf_->lon_speed_controller_conf();
      use_station_pid_ = lon_speed_controller_conf.use_station_pid();
      ts_ = lon_speed_controller_conf.ts();
      preview_time_ = lon_speed_controller_conf.preview_window();
      starting_speed_ = lon_speed_controller_conf.starting_speed();
      speed_limit_ = vehicle_param_.speed_limit();
      acceleration_threshold_when_brake_ = lon_speed_controller_conf.acceleration_threshold_when_brake();
      AINFO << "ts_                                 :"<< ts_;
      AINFO << "preview_time_                       :"<< preview_time_;
      AINFO << "use_station_pid_                    :"<< use_station_pid_;
      AINFO << "starting_speed_                     :"<< starting_speed_;
      AINFO << "acceleration_threshold_when_brake_  :"<< acceleration_threshold_when_brake_;

      GearTransitionInit();
      BrakeStatusInit();
      is_stopped_first_hit_ = true;

      return Status::Ok();
    }

    legionclaw::common::Status LonSpeedController::ComputeControlCommand(const legionclaw::interface::LocalizationEstimate *localization,
                                                                     const legionclaw::interface::Chassis *chassis,
                                                                     const legionclaw::interface::ADCTrajectory *trajectory,
                                                                     legionclaw::interface::ControlCommand *cmd,
                                                                     legionclaw::interface::ControlAnalysis *analysis)
    {
      AINFO << "*************************************************";
      brake_sm_debug_.set_is_obs_stop(false);
      brake_sm_debug_.set_is_geartransition_stop(false);
      auto vehicle_state = injector_->vehicle_state();
      localization_ = localization;
      chassis_ = chassis;
      trajectory_message_ = trajectory;
      TrajectoryPoint target_point;

      // 判断车辆是否静止
      // 车辆静止表现为：车速度小于速度阈值
      // ，且持续时间大于max_abs_speed_when_stopped_duration（500ms）
      if (fabs(injector_->vehicle_state()->linear_velocity()) <
          vehicle_param_.max_abs_speed_when_stopped())
      {
        // 第一次记录当前时间
        if (is_stopped_first_hit_ == true)
        {
          last_time_vehicle_is_stopped_or_not_ = TimeTool::Now2Ms();
          is_stopped_first_hit_ = false;
        }
        // 当前时间
        int64_t cur_time = TimeTool::Now2Ms();
        int64_t diff = cur_time - last_time_vehicle_is_stopped_or_not_;
        if (diff >= vehicle_param_.max_abs_speed_when_stopped_duration())
        {
          is_stopped_ = true;
        }
        else
        {
          is_stopped_ = false;
        }
      }
      else
      {
        is_stopped_ = false;
        is_stopped_first_hit_ = true;
      }

      double brake_cmd = 0.0;
      double speed_cmd = 0.0;
      simple_longitudinal_debug_ = std::make_shared<SimpleLongitudinalDebug>();

      if (trajectory_analyzer_ == nullptr || trajectory_analyzer_->seq_num() != trajectory_message_->header().seq())
      {
        trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
      }

      ComputeLongitudinalErrors(trajectory_analyzer_.get(), 
                                localization, 
                                chassis,
                                preview_time_, 
                                ts_,
                                simple_longitudinal_debug_.get());
      speed_cmd = simple_longitudinal_debug_ -> speed_reference();
      static double last_speed;
      double current_a = (speed_cmd - fabs(injector_->vehicle_state()->linear_velocity()))/ts_;
      // std::cout << "当前加速度      :"<< current_a<< "\n";
      AINFO << "当前加速度:"<< current_a;
      if ((speed_cmd < vehicle_param_.max_abs_speed_when_stopped()) && 
          (fabs(injector_->vehicle_state()->linear_velocity()) > vehicle_param_.max_abs_speed_when_stopped()) &&
          current_a < acceleration_threshold_when_brake_)
      {
        AWARN<<"Excessive velocity variation to brake";
        brake_sm_debug_.set_is_obs_stop(true);
      }
      target_point = simple_longitudinal_debug_->current_matched_point();
      AINFO << "速度命令 :"<< speed_cmd;
      cmd->set_speed(speed_cmd);

      if(use_station_pid_)
      {
        double station_error_limited = 0.0;
        // station_error_limited = common::math::Clamp(simple_longitudinal_debug_->station_error(),
        //                                             -lon_speed_controller_conf.station_error_limit(), //-5
        //                                             lon_speed_controller_conf.station_error_limit()); // 5
        // double speed_offset = station_pid_controller_.Control(station_error_limited, ts_);
      }
        // 提前到站逻辑,避免临界区车辆档位来回切换的问题
        if (simple_longitudinal_debug_->is_full_stop() &&
            simple_longitudinal_debug_->current_speed() < vehicle_param_.max_abs_speed_when_stopped())
        {
          gear_sm_debug_.set_tar_gear(legionclaw::common::GearPosition::GEAR_NEUTRAL);
        }
        else
        {
          gear_sm_debug_.set_tar_gear(target_point.gear());
        }
        gear_sm_debug_.set_cur_gear(chassis_->gear_location());
        gear_sm_debug_.set_cur_epb_state(chassis_->epb_level());
        gear_sm_debug_.set_is_stopped(is_stopped_);
        gear_sm_debug_.set_brake_value_when_gear_transitioning(vehicle_param_.brake_value_when_gear_transitioning());

      //档位和EPB切换逻辑
      GearTransition(cmd);
      if (gear_sm_)
      {
        gear_sm_->OnUpdate();
      }
      BrakeTransition(cmd);
      if (brake_sm_)
      {
        brake_sm_->OnUpdate();
      }
    // if (epb_sm_)
      // {
      //   epb_sm_->OnUpdate();
      // }
      AINFO << " 档位命令    :"<< cmd->gear_location();
      AINFO << " 刹车命令    :"<< cmd ->brake_value();
      // std::cout << "刹车命令     :"<< cmd ->brake_value()<< "\n";
      cmd->set_accel_driving_mode(trajectory->driving_mode());
      cmd->set_brake_driving_mode(trajectory->driving_mode());
      cmd->set_epb_driving_mode(trajectory->driving_mode());
      cmd->set_gear_driving_mode(trajectory->driving_mode());
      last_speed = fabs(injector_->vehicle_state()->linear_velocity());

      // SetAnalysis(cmd, simple_longitudinal_debug_.get(), chassis, analysis);
      // lon_calculate_time_ = legionclaw::common::TimeTool::Now2Us() - current_time;
      // lon_calculate_time_max_ = max(lon_calculate_time_, lon_calculate_time_max_);
      // analysis->set_lon_calculate_time(lon_calculate_time_);
      // analysis->set_lon_calculate_time_max(lon_calculate_time_max_);
      AINFO << "*************************************************";
      return Status::Ok();
    }

    void LonSpeedController::Stop()
    {  }

    Status LonSpeedController::Reset()
    {
      return Status::Ok();
    }

    void LonSpeedController::GetPathRemain(SimpleLongitudinalDebug *debug)
    {

    }

    void LonSpeedController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer,
                                                       const legionclaw::interface::LocalizationEstimate *localization,
                                                       const legionclaw::interface::Chassis *chassis, 
                                                       const double preview_time,
                                                       const double ts, 
                                                       SimpleLongitudinalDebug *debug)
    {

      // s: 纵向累积走过的距离沿着参考轨迹
      // s_dot: 纵向沿着参考轨迹的速度
      // d: 相对参考轨迹的横向距离
      // d_dot: 横向距离的变化率
      double s_matched = 0.0;
      double s_dot_matched = 0.0;
      double d_matched = 0.0;
      double d_dot_matched = 0.0;
      auto vehicle_state = injector_->vehicle_state();
      double add_lr = 0.0;

      // 实质输出与定位坐标一致
      const auto &com = vehicle_state->ComputeCOMPosition(add_lr);

      // 找到距离当前位置最近的参考轨迹点(PathPoint)
      auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(com.x(), com.y());

      // 找到距离当前位置最近的参考轨迹点(TrajectoryPoint)
      auto current_point = trajectory_analyzer->QueryNearestPointByPosition(com.x(), com.y());

      // 计算纵向误差相关参数
      trajectory_analyzer->ToTrajectoryFrame(com.x(), 
                                             com.y(), 
                                             vehicle_state->heading(),
                                             fabs(vehicle_state->linear_velocity()), 
                                             matched_point, 
                                             &s_matched,
                                             &s_dot_matched, 
                                             &d_matched, 
                                             &d_dot_matched);

      double current_control_time = TimeTool::NowToSeconds();
      double preview_control_time = current_control_time + preview_time;

      current_point.set_path_point(matched_point);
      debug->set_current_matched_point(current_point);

      TrajectoryPoint reference_point;
      TrajectoryPoint preview_point;

      if (IsEventFlagTrue("has_stop_point") == false)
      {
        reference_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time, false);
        preview_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(preview_control_time, false);
      }
      else
      {
        reference_point = debug->current_matched_point();
        preview_point = debug->current_matched_point();
      }
      debug->set_current_reference_point(reference_point);
      debug->set_preview_reference_point(preview_point);
      
      double heading_error = common::math::NormalizeAngle(vehicle_state->heading() - matched_point.theta());
      // 修复人工驾驶状态下车速设置的bug
      if (std::isnan(heading_error) || chassis-> accel_driving_mode() == legionclaw::common::DrivingMode::COMPLETE_MANUAL)
      {
        heading_error = 0;
      }
      if (std::isnan(s_dot_matched))
      {
        s_dot_matched = 0;
      }

      double lon_speed = fabs(vehicle_state->linear_velocity()) * std::cos(heading_error);
      double lon_acceleration = vehicle_state->linear_acceleration() * std::cos(heading_error);

      double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() * fabs(vehicle_state->linear_velocity()) * std::sin(heading_error);

      debug->set_station_reference(reference_point.path_point().s());
      debug->set_current_station(s_matched);
      double diff_s = reference_point.path_point().s() - s_matched;
      if (std::isnan(diff_s))
      {
        diff_s = 0;
      }
      debug->set_station_error(diff_s);

      /// 方案二,起步最小速度保护
      if (reference_point.v() < starting_speed_ && reference_point.a() > 0.0)
      {
        debug->set_speed_reference(starting_speed_);
      }
      else
      {
        debug->set_speed_reference(min(reference_point.v(), speed_limit_));
      }
      debug->set_current_speed(lon_speed);
      // 速度误差speed_error=Vdes −V∗cosΔθ/k
      // Vdes 为期望车辆线速度，V 为当前车辆线速度，Δθ 为航向误差，k
      // 为系数，即代码中的one_minus_kappa_r_d
      debug->set_speed_error(debug->speed_reference() - s_dot_matched);
      debug->set_acceleration_reference(reference_point.a());
      debug->set_current_acceleration(lon_acceleration);
      debug->set_acceleration_error(reference_point.a() - lon_acceleration / one_minus_kappa_lat_error);
      // double jerk_reference = (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
      // double lon_jerk = (debug->current_acceleration() - previous_acceleration_) / ts;
      // debug->set_jerk_reference(jerk_reference);
      // debug->set_current_jerk(lon_jerk);
      // debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
      // previous_acceleration_reference_ = debug->acceleration_reference();
      // previous_acceleration_ = debug->current_acceleration();

      debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
      debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
      debug->set_preview_speed_reference(preview_point.v());
      debug->set_preview_acceleration_reference(preview_point.a());
    }

    void LonSpeedController::SetAnalysis(const legionclaw::interface::ControlCommand *cmd,
                                         const SimpleLongitudinalDebug *debug,
                                         const legionclaw::interface::Chassis *chassis,
                                         legionclaw::interface::ControlAnalysis *analysis)
    {
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
      analysis->set_preview_acceleration_reference(debug->preview_acceleration_reference());
      analysis->set_slope_offset_compensation(debug->slope_offset_compensation());
      analysis->set_turning_offset_compensation(debug->turning_offset_compensation());
      analysis->set_speed_error_limited(debug->speed_controller_input_limited());
      analysis->set_speed_error(debug->speed_error());
      analysis->set_speed_offset(debug->speed_offset());
      analysis->set_station_error_limited(debug->station_error_limited());
      analysis->set_station_error(debug->station_error());
      analysis->set_lon_target_point_s(debug->station_reference());
    }

    void LonSpeedController::SetEventFlag(std::string key, bool value) 
    { 
      eventflags_[key] = value; 
    }

    bool LonSpeedController::IsEventFlagTrue(std::string key) 
    {
      if (eventflags_.count(key) == 0) 
      {
        eventflags_[key] = false;
      }
      return eventflags_[key];
    }
  }
}
