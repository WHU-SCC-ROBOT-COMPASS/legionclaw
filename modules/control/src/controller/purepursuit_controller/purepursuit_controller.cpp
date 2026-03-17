/**
 * @file              purepursuit_controller.cpp
 * @author            yanglanjiang (yanglanjiang@indrv.cn)
 * @brief
 * @version           1.0.0
 * @date              2023-04-15 04:28:16
 * @copyright         Copyright (c) 2023
 * @license           GNU General Public License (GPL)
 */

#include "purepursuit_controller.h"

#include <time.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "common/logging/logging.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/status/status.h"
#include "common/time/time_tool.h"
#include "common/trajectory_analyzer/trajectory_analyzer.h"

namespace legionclaw
{
  namespace control
  {

    using namespace legionclaw::common;
    using namespace legionclaw::interface;

    PurePursuitController::PurePursuitController(): name_("PurePursuit Lateral Controller")
    {
      // if (enable_log_debug_)
      // {
      //     steer_log_file_.open(GetLogFileName());
      //     steer_log_file_ << std::fixed;
      //     steer_log_file_ << std::setprecision(6);
      //     WriteHeaders(steer_log_file_);
      // }
      AINFO << "Using " << name_;
      simple_lateral_debug_ = std::make_shared<SimpleLateralDebug>();
    }

    PurePursuitController::~PurePursuitController() { CloseLogFile(); }

    legionclaw::common::Status PurePursuitController::Init(
        std::shared_ptr<DependencyInjector> injector,
        const ControlConf *control_conf)
    {
      injector_ = injector;
      control_conf_ = control_conf;

      // 加载配置参数
      if (!LoadPurePursuitControlConf(control_conf_))
      {
        AERROR << "failed to load control conf";
        return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                      "failed to load control_conf");
      }

      // 航向误差PID调节初始化
      heading_error_pid_controller_.Init(
          control_conf_->purepursuit_controller_conf().heading_error_pid_conf());
      heading_error_pid_controller_.SetPID(
          control_conf_->purepursuit_controller_conf().heading_error_pid_conf());
      // 横向误差PID调节初始化
      lat_error_pid_controller_.Init(
          control_conf_->purepursuit_controller_conf().lat_error_pid_conf());
      lat_error_pid_controller_.SetPID(
          control_conf_->purepursuit_controller_conf().lat_error_pid_conf());
      if (enable_digital_filter_)
      {
        // 初始化滤波器
        InitializeFilters(control_conf_);
      }

      return Status::Ok();
    }

    void PurePursuitController::InitializeFilters(const ControlConf *control_conf)
    {
      // Low pass filter
      std::vector<double> den(3, 0.0);
      std::vector<double> num(3, 0.0);
      LpfCoefficients(ts_,
                      control_conf_->purepursuit_controller_conf().cutoff_freq(),
                      &den, &num);
      digital_filter_f.set_coefficients(den, num);
      digital_filter_r.set_coefficients(den, num);
    }

    bool PurePursuitController::LoadPurePursuitControlConf(const ControlConf *control_conf)
    {
      if (control_conf == nullptr)
      {
        AERROR << "[LatController] control_conf == nullptr";
        return false;
      }
      vehicle_param_ = control_conf->vehicle_param();
      set_steer_limit_ = control_conf->set_steer_limit();
      enable_maximum_steer_rate_limit_ =
          control_conf->enable_maximum_steer_rate_limit();
      steer_mode_ = (VehicleParam::SteerMode)vehicle_param_.steer_mode();
      wheelbase_ = vehicle_param_.wheelbase();
      lf_ = vehicle_param_.lf();
      lr_ = vehicle_param_.lr();
      steer_ratio_ = vehicle_param_.steer_ratio();
      //方向盘最大转角（角度）
      front_steer_single_direction_max_degree_ = vehicle_param_.max_front_steer_angle() * 180 / M_PI;
      front_steer_single_direction_min_degree_ = vehicle_param_.min_front_steer_angle() * 180 / M_PI;
      rear_steer_single_direction_max_degree_ =
          vehicle_param_.max_rear_steer_angle() * 180 / M_PI;
      rear_steer_single_direction_min_degree_ =
          vehicle_param_.min_rear_steer_angle() * 180 / M_PI;
      max_lat_acc_ =
          control_conf_->purepursuit_controller_conf().max_lateral_acceleration();
      ts_ = control_conf_->purepursuit_controller_conf().ts();
      // 方向盘转角的最大变化量（度） = 车轮转角速率 × 时间间隔 × 转向比
      front_steer_diff_with_max_rate_ =
          (enable_maximum_steer_rate_limit_)
              ? vehicle_param_.max_front_steer_angle_rate() * ts_ * 180 / M_PI
              : front_steer_single_direction_max_degree_;
      rear_steer_diff_with_max_rate_ =
          (enable_maximum_steer_rate_limit_)
              ? vehicle_param_.max_rear_steer_angle_rate() * ts_ * 180 / M_PI
              : rear_steer_single_direction_max_degree_;
      max_front_steer_angle_rate_ =
          vehicle_param_.max_front_steer_angle_rate() * 180 / M_PI;
      query_relative_time_ = control_conf->query_relative_time();
      query_time_nearest_point_only_ =
          control_conf->query_time_nearest_point_only();
      query_forward_time_point_only_ =
          control_conf->query_forward_time_point_only();
      use_navigation_mode_ = control_conf->use_navigation_mode();
      enable_navigation_mode_position_update_ =
          control_conf->enable_navigation_mode_position_update();
      lookahead_station_ =
          control_conf_->purepursuit_controller_conf().lookahead_station();
      lookback_station_ =
          control_conf_->purepursuit_controller_conf().lookback_station();
      enable_log_debug_ =
          control_conf_->purepursuit_controller_conf().enable_log_debug();
      if (enable_log_debug_)
      {
        steer_log_file_.open(GetLogFileName());
        steer_log_file_ << std::fixed;
        steer_log_file_ << std::setprecision(6);
        WriteHeaders(steer_log_file_);
      }

      lat_error_threshold_ = control_conf_->purepursuit_controller_conf().lat_error_threshold();
      enable_lat_error_ =
          control_conf_->purepursuit_controller_conf().enable_lat_error();
      enable_heading_error_ =
          control_conf_->purepursuit_controller_conf().enable_heading_error();
      heading_error_gain_ =
          control_conf_->purepursuit_controller_conf().heading_error_gain();
      lat_error_gain_ =
          control_conf_->purepursuit_controller_conf().lat_error_gain();
      heading_error_threshold_ =
          control_conf_->purepursuit_controller_conf().has_heading_error_threshold()
              ? control_conf_->purepursuit_controller_conf().heading_error_threshold()
              : 17.19;  // 默认值约 0.3 弧度 (≈17.19度)
      // 加载偏差最小值阈值
      if (control_conf_->purepursuit_controller_conf().has_lat_error_min_threshold()) {
        lat_error_min_threshold_ =
            control_conf_->purepursuit_controller_conf().lat_error_min_threshold();
      } else {
        lat_error_min_threshold_ = 0.0;  // 默认值0表示不启用最小值阈值
      }
      if (control_conf_->purepursuit_controller_conf().has_heading_error_min_threshold()) {
        heading_error_min_threshold_ =
            control_conf_->purepursuit_controller_conf().heading_error_min_threshold();
      } else {
        heading_error_min_threshold_ = 0.0;  // 默认值0表示不启用最小值阈值
      }

      use_delay_compensation_ =
          control_conf->purepursuit_controller_conf().use_delay_compensation();
      use_interpolate_lookahead_point_ = control_conf->purepursuit_controller_conf()
                                             .use_interpolate_lookahead_point();
      minimum_lookahead_distance_ =
          control_conf->purepursuit_controller_conf().minimum_lookahead_distance();
      maximum_lookahead_distance_ =
          control_conf->purepursuit_controller_conf().maximum_lookahead_distance();
      speed_to_lookahead_ratio_ =
          control_conf->purepursuit_controller_conf().speed_to_lookahead_ratio();
      // 加载Pure Pursuit算法预瞄距离相关参数
      if (control_conf->purepursuit_controller_conf().has_purepursuit_minimum_lookahead_distance()) {
        purepursuit_minimum_lookahead_distance_ =
            control_conf->purepursuit_controller_conf().purepursuit_minimum_lookahead_distance();
      } else {
        purepursuit_minimum_lookahead_distance_ = minimum_lookahead_distance_;  // 默认使用目标点匹配的最小值
      }
      if (control_conf->purepursuit_controller_conf().has_purepursuit_maximum_lookahead_distance()) {
        purepursuit_maximum_lookahead_distance_ =
            control_conf->purepursuit_controller_conf().purepursuit_maximum_lookahead_distance();
      } else {
        purepursuit_maximum_lookahead_distance_ = maximum_lookahead_distance_;  // 默认使用目标点匹配的最大值
      }
      if (control_conf->purepursuit_controller_conf().has_purepursuit_speed_to_lookahead_ratio()) {
        purepursuit_speed_to_lookahead_ratio_ =
            control_conf->purepursuit_controller_conf().purepursuit_speed_to_lookahead_ratio();
      } else {
        purepursuit_speed_to_lookahead_ratio_ = speed_to_lookahead_ratio_;  // 默认使用目标点匹配的速度参数
      }
      enable_digital_filter_ =
          control_conf->purepursuit_controller_conf().enable_digital_filter();

      return true;
    }

    legionclaw::common::Status PurePursuitController::ComputeControlCommand(const legionclaw::interface::LocalizationEstimate *localization,
                                                                        const legionclaw::interface::Chassis *chassis,
                                                                        const legionclaw::interface::ADCTrajectory *trajectory,
                                                                        legionclaw::interface::ControlCommand *cmd,
                                                                        legionclaw::interface::ControlAnalysis *analysis)
    {
      int64_t compute_start_time = TimeTool::Now2Us();
      auto vehicle_state = injector_->vehicle_state();
      auto target_tracking_trajectory = *trajectory;
      // 轨迹分析器更新轨迹
      trajectory_analyzer_ = std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

      legionclaw::common::math::Vec2d pos;
      // 目前未用use_delay_compensation_
      if (use_delay_compensation_ && localization->header().has_stamp())
      {
        auto Location_time_diff = TimeTool::GetTimeDiffNow(localization->header().stamp());
        pos = vehicle_state->EstimateFuturePosition(Location_time_diff);
      }
      else
      {
        // 计算车辆的质心位置，输入从后轮到车辆的质心，输出车辆质心的位置。计算过程中使用到车辆位置信息(定位的坐标x,y)，姿态信息(定位的roll
        // pitch yaw)，档位信息 这里实质输出与定位坐标一致
        pos = vehicle_state->ComputeCOMPosition(0.0);
      }
      current_x_ = pos.x();
      current_y_ = pos.y();

      // 步骤1: 计算前视距离
      ComputeLookAheadDistance();

      // 步骤2: 计算目标点（完整轨迹点信息）
      // 使用目标点预瞄距离计算目标点（用于偏差计算等）
      auto target_point = ComputeTargetPoint(pos, vehicle_state->heading(), lookahead_distance_, trajectory);
      // 使用Pure Pursuit预瞄距离计算Pure Pursuit预瞄点（用于Pure Pursuit算法）
      auto purepursuit_target_point = ComputeTargetPoint(pos, vehicle_state->heading(), purepursuit_lookahead_distance_, trajectory);
      // 获取目标点坐标和轨迹曲率（使用target_point的曲率）
      double target_curvature = target_point.path_point().kappa();

      // 步骤3: 计算横向偏差和航向角偏差
      // 计算横向误差，vehicle_state中linear_velocity是底盘speed_mps
      // 使用ComputeTargetPoint的结果作为参数进行偏差计算
      ComputeLateralErrors(current_x_, current_y_, vehicle_state->heading(),
                           vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
                           vehicle_state->linear_acceleration(), target_point,
                           trajectory_analyzer_, simple_lateral_debug_.get(), chassis);
      // 航向误差标准化（使用配置的航向角误差阈值）
      auto error = simple_lateral_debug_->heading_error();

      auto front_heading_error_feedback = 0.0;
      // 后轮反馈调节值
      auto rear_feedback = 0.0;
      // 航向误差pid反馈调节值计算
      heading_pid_feedback_ = heading_error_pid_controller_.Control(error, ts_);
      // 前轮反馈赋值，后轮未使用反馈调节
      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        front_heading_error_feedback = heading_pid_feedback_;
      }
      else if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_DRIVE)
      {
        front_heading_error_feedback = -heading_pid_feedback_;
      }

      auto lat_error_feedback = lat_error_pid_controller_.Control(-simple_lateral_debug_->lateral_error_feedback(), ts_);

      // 步骤4: 计算转角
      auto curvature = target_curvature;

      AINFO << " 前视距离 :" << lookahead_distance_;
      AINFO << " 最终曲率 : " << curvature;
      AINFO <<"------------------------------------";
      // if( abs(error) > 0.1){
      //   front_heading_error_feedback = 0.75 * front_heading_error_feedback;
      // }

      // ofstream foutC("./points_vector.txt",ios::app);
      // foutC.precision(15);
      // foutC << vehicle_state->x() << " "
      //       << vehicle_state->y() << " "
      //       << target_point.x() << " "
      //       << target_point.y()
      //       << endl;
      // foutC.close();
      // PurePursuit计算结果
      // 前轮转角
      auto front_angle_target = 0.0;
      // 前轮方向盘转角
      auto front_steering_target = 0.0;
      // 后轮转角
      auto rear_angle_target = 0.0;

      // 区分前轮转向模式和四轮转向模式
      if (steer_mode_ == VehicleParam::STEER_MODE_2WS)
      {
        front_angle_target = atan(curvature * wheelbase_);
        if (!enable_heading_error_)
        {
          front_heading_error_feedback = 0.0;
        }
        if (!enable_lat_error_)
        {
          lat_error_feedback = 0.0;
        }

        double heading_error_gain = 0;
        double lat_error_gain = 0;
        if (abs(simple_lateral_debug_->lateral_error_feedback()) > lat_error_threshold_)
        {
          heading_error_gain = 0;
          lat_error_gain = 0.7;
        }
        else
        {
          heading_error_gain = heading_error_gain_;
          lat_error_gain = lat_error_gain_;
        }
        
        // 航向角偏差转换为前轮转角（度）
        // 使用车辆运动学模型：dθ/dt = (v/L) * tan(δ)
        // 假设需要在一定时间内消除航向角偏差，则：
        // heading_error / τ = (v/L) * tan(δ_heading)
        // δ_heading = atan((heading_error / τ) * L / v)
        // 对于小角度，可以近似：δ_heading ≈ (heading_error / τ) * L / v
        // 这里使用简化的转换：根据航向角偏差和车辆参数计算前轮转角
        double heading_error_wheel_angle_deg = 0.0;
        double lat_error_wheel_angle_deg = 0.0;
        double v = vehicle_state->linear_velocity();
        if (std::abs(v) > 0.1 && wheelbase_ > 1e-3)  // 避免除零
        {
          // 航向角偏差（度）转换为弧度
          double heading_error_rad = D2R(front_heading_error_feedback);
          // 使用车辆运动学模型转换为前轮转角（弧度）
          // 时间常数τ可以理解为期望的收敛时间，这里使用一个合理的值（如0.5秒）
          // 或者直接使用比例关系：δ = k * heading_error * L / v
          // 为了稳定性，使用atan限制输出范围
          double heading_error_wheel_angle_rad = std::atan(heading_error_rad * wheelbase_ / std::max(std::abs(v), 0.1));
          heading_error_wheel_angle_deg = R2D(heading_error_wheel_angle_rad);

          //横向偏差（度）转换为弧度
          double lat_error_deg = std::atan(lat_error_feedback / purepursuit_lookahead_distance_) * 180.0 / M_PI;
          double lat_error_rad = D2R(lat_error_deg);
          double lat_error_wheel_angle_rad = std::atan(lat_error_rad * wheelbase_ / std::max(std::abs(v), 0.1));
          lat_error_wheel_angle_deg = R2D(lat_error_wheel_angle_rad);
        }
        
        // 计算偏差贡献：前轮转角（度）乘以增益
        // 如果偏差小于最小值阈值，则贡献值清零
        double lat_error_contribution_deg = 0.0;
        if (std::abs(lat_error_wheel_angle_deg) >= lat_error_min_threshold_) {
          lat_error_contribution_deg = lat_error_wheel_angle_deg * lat_error_gain;
        }
        
        double heading_error_contribution_deg = 0.0;
        if (std::abs(heading_error_wheel_angle_deg) >= heading_error_min_threshold_) {
          heading_error_contribution_deg = heading_error_wheel_angle_deg * heading_error_gain;
        }
        
        // 计算前轮转角（度）= Pure Pursuit前馈 + 横向偏差贡献 + 航向角偏差贡献
        // 所有贡献值都是前轮转角（度）
        double front_angle_target_ff = R2D(front_angle_target);
        double front_angle_deg = front_angle_target_ff +
                                 lat_error_contribution_deg +
                                 heading_error_contribution_deg;
        // 前轮方向盘转角 (°) = 前轮转角 (°) * 转向比
        front_steering_target = front_angle_deg * steer_ratio_;
        ADEBUG<<"-------------------------------------------------------------------";
        ADEBUG << " 横向误差（米）：" << simple_lateral_debug_->lateral_error_feedback();
        ADEBUG << " 横向误差转前轮转角（度）：" << lat_error_wheel_angle_deg;
        ADEBUG << " 航向角误差转前轮转角（度）：" << heading_error_wheel_angle_deg;
        ADEBUG << " 横向误差权重:" << lat_error_gain;
        ADEBUG << " 航向误差权重:" << heading_error_gain;
        ADEBUG << " 前馈（前轮转角，度）：:" << front_angle_target_ff ;
        ADEBUG << " 横向转角贡献:" << lat_error_contribution_deg ;
        ADEBUG << " 航向转角贡献:" << heading_error_contribution_deg ;
        ADEBUG << " 前轮方向盘转角: " << front_steering_target;
      }

      // 3.3
      // 转向比16.5   时间0.2s
      //  else if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      //  {
      //    front_angle_target = atan(curvature * lf_);
      //    front_steering_target = - R2D(front_angle_target) * steer_ratio_ +
      //    R2D(front_heading_error_feedback) * steer_ratio_; rear_angle_target =
      //    R2D(atan(curvature * lr_));
      //  }

      // 横向加速度的前轮方向盘转角限制计算
      // Compute the steering command limit with the given maximum lateral acceleration
      //曲线半径r与车速v、侧向加速度a之间的关系：r = v^2 / a；车辆转向角α与轴距L、转弯半径r之间的关系：α = atan(L / r)；得出：α = atan(a*L / (v^2))
      //横向加速度与速度平方的比值,这一比值表示在给定线速度下，为了达到最大横向加速度所需的曲率半径的倒数(即曲率)
      //单位（°）
      const double front_steer_limit =(set_steer_limit_) ? 
                                                          std::atan(max_lat_acc_ * wheelbase_ /
                                                          (vehicle_state->linear_velocity() *
                                                          vehicle_state->linear_velocity())) *
                                                          steer_ratio_ * 180 / M_PI    //转为角度
                                                          : front_steer_single_direction_max_degree_;
      // 横向加速度的后轮转角限制计算
      const double rear_steer_limit = (set_steer_limit_) ? 
                                                          std::atan(max_lat_acc_ * wheelbase_ /
                                                          (vehicle_state->linear_velocity() *
                                                          vehicle_state->linear_velocity())) *
                                                          180 / M_PI
                                                          : rear_steer_single_direction_max_degree_;
      // 前轮方向盘转角进行转角限制
      double front_steer_angle_limited = common::math::Clamp(front_steering_target, 
                                                             -front_steer_limit, 
                                                             front_steer_limit);
      front_steering_target = front_steer_angle_limited;
      simple_lateral_debug_->set_front_steer_angle_limited(front_steer_angle_limited);
      // 方向盘转角滤波，减小抖动
      if (enable_digital_filter_)
      {
        front_steering_target = digital_filter_f.Filter(front_steering_target);
      }

      // 四轮转向模式
      // if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      // {
      //   // 后轮转角进行转角限制
      //   double rear_angle_limited = common::math::Clamp(
      //         rear_angle_target, -rear_steer_limit, rear_steer_limit);
      //   rear_angle_target = rear_angle_limited;
      //   simple_lateral_debug_->set_rear_steer_angle_limited(
      //           rear_angle_limited);
      //   // 后轮转角滤波
      //   if (enable_digital_filter_) {
      //     rear_angle_target = digital_filter_r.Filter(rear_angle_target);
      //   }
      //   // 后轮转角限制最大值、最小值
      //   rear_angle_target = common::math::Clamp(
      //         rear_angle_target, rear_steer_single_direction_min_degree_,
      //         rear_steer_single_direction_max_degree_);
      // }
      // 非泊车过程中，转角赋0
      if (chassis->steer_driving_mode() == legionclaw::common::DrivingMode::COMPLETE_MANUAL ||
          trajectory->behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE)
      // if (cmd->steer_driving_mode() ==
      // legionclaw::common::DrivingMode::COMPLETE_MANUAL ||
      //     trajectory->behaviour_state() ==
      //     legionclaw::interface::ADCTrajectory::BehaviourState::PARKING_FINISH_STATE
      //     || trajectory->behaviour_state() ==
      //     legionclaw::interface::ADCTrajectory::BehaviourState::FINISH_STATE)
      {
        front_steering_target = 0.0;
        rear_angle_target = 0.0;
      }
      // 前轮方向盘转角限制最大转向速率
      front_steering_target = math::Clamp(front_steering_target,
                                          pre_front_steer_angle_ - front_steer_diff_with_max_rate_,
                                          pre_front_steer_angle_ + front_steer_diff_with_max_rate_);
      // 后轮转角限制最大转向速率
      // rear_steering_target = math::Clamp(rear_angle_target, 
      //                                    pre_rear_steer_angle_ - rear_steer_diff_with_max_rate_,
      //                                    pre_rear_steer_angle_ + rear_steer_diff_with_max_rate_);
      // 方向盘转角转为车轮转角，赋给控制命令
      // 前轮方向盘转角限制最大值、最小值（°）
      front_steering_target = common::math::Clamp(front_steering_target, 
                                                  front_steer_single_direction_min_degree_,
                                                  front_steer_single_direction_max_degree_);
      pre_front_steer_angle_ = front_steering_target;
      cmd->set_front_steering_target(front_steering_target);
      cmd->set_rear_steering_target(0.0);
      // cmd->set_front_steering_target(60.0);
      // 防止输出nan值
      if (std::isnan(cmd->front_steering_target()))
      {
        cmd->set_front_steering_target(0.0);
      }
      if (std::isnan(cmd->rear_steering_target()))
      {
        cmd->set_rear_steering_target(0.0);
      }

      cmd->set_front_steering_rate(max_front_steer_angle_rate_);

      // cmd->set_steer_driving_mode(chassis->driving_mode());
      cmd->set_steer_driving_mode(trajectory->driving_mode());
      // std::cout << "first " <<cmd->steer_driving_mode() << endl;
      static double last_time = 0.0;
      static double hold_time = 0.0;
      // //方向盘回正逻辑
      // switch (steer_back_step_)
      // {
      // case 0:
      //   if(trajectory->driving_mode() ==
      //   legionclaw::common::DrivingMode::COMPLETE_MANUAL &&
      //   trajectory->behaviour_state() ==
      //   legionclaw::interface::ADCTrajectory::BehaviourState::PARKING_FINISH_STATE)
      //   {
      //     steer_back_step_ = 1;
      //     cmd->set_front_steering_target(0.0);
      //     cmd->set_rear_steering_target(0.0);
      //     cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
      //     last_time = TimeTool::NowToSeconds();
      //   }
      //   break;
      // case 1:
      //   {
      //     cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
      //     double current_time = TimeTool::NowToSeconds();
      //     double time_diff = current_time - last_time;
      //     if(std::fabs(chassis->front_steering_value()) < 2.0 &&
      //     std::fabs(chassis->rear_steering_value()) < 2.0)
      //     {
      //       hold_time = TimeTool::NowToSeconds();
      //       steer_back_step_ = 2;
      //     }
      //     if(time_diff >= 3.0)
      //     {
      //       steer_back_step_ = 3;
      //     }
      //     cmd->set_front_steering_target(0.0);
      //     cmd->set_rear_steering_target(0.0);
      //   }
      //   break;
      // case 2:
      //   {
      //     cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
      //     double current_time = TimeTool::NowToSeconds();
      //     double time_diff = current_time - hold_time;
      //     if(time_diff >= 0.5)
      //     {
      //       steer_back_step_ = 3;
      //     }
      //     cmd->set_front_steering_target(0.0);
      //     cmd->set_rear_steering_target(0.0);
      //   }
      //   break;
      // case 3:
      //   cmd->set_steer_driving_mode(trajectory->driving_mode());
      //   cmd->set_front_steering_target(0.0);
      //   cmd->set_rear_steering_target(0.0);
      //   if(trajectory->behaviour_state() !=
      //   legionclaw::interface::ADCTrajectory::BehaviourState::PARKING_FINISH_STATE
      //      && trajectory->behaviour_state() !=
      //      legionclaw::interface::ADCTrajectory::BehaviourState::INITIAL_STATE)
      //   {
      //     steer_back_step_ = 0;
      //   }
      //   break;
      // default:
      //   steer_back_step_ = 0;
      // }

      simple_lateral_debug_->set_heading(vehicle_state->heading());
      simple_lateral_debug_->set_front_steer_angle(cmd->front_steering_target());
      simple_lateral_debug_->set_front_steering_position(chassis->front_steering_value());
      simple_lateral_debug_->set_rear_steer_angle(cmd->rear_steering_target());
      simple_lateral_debug_->set_rear_steering_position(chassis->rear_steering_value());
      simple_lateral_debug_->set_ref_speed(vehicle_state->linear_velocity());
      simple_lateral_debug_->set_front_steer_angle_feedforward(-R2D(front_angle_target));
      simple_lateral_debug_->set_rear_steer_angle_feedforward(rear_angle_target);
      simple_lateral_debug_->set_front_steer_angle_feedback(R2D(front_heading_error_feedback));
      simple_lateral_debug_->set_rear_steer_angle_feedback(rear_feedback);
      pre_rear_steer_angle_ = cmd->rear_steering_target();

      // if (simple_lateral_debug_->current_target_point().is_steer_valid())
      // {
      //   cmd->set_front_steering_target(
      //       simple_lateral_debug_->current_target_point().front_steer() * 180 /
      //       M_PI * steer_ratio_);
      //   cmd->set_rear_steering_target(
      //       simple_lateral_debug_->current_target_point().rear_steer() * 180 /
      //       M_PI);
      // }
      ProcessLogs(cmd, simple_lateral_debug_.get(), chassis);
      SetAnalysis(cmd, simple_lateral_debug_.get(), chassis, analysis);
      calculate_time_ = TimeTool::Now2Us() - compute_start_time;
      calculate_time_max_ = max(calculate_time_, calculate_time_max_);
      analysis->set_lqr_calculate_time(calculate_time_);
      analysis->set_lqr_calculate_time_max(calculate_time_max_);
      return Status::Ok();
    }

    void PurePursuitController::ComputeLateralErrors(
        const double x, const double y, const double theta, const double linear_v,
        const double angular_v, const double linear_a,
        const legionclaw::interface::TrajectoryPoint &target_point,
        const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug,
        const legionclaw::interface::Chassis *chassis)
    {
      auto vehicle_state = injector_->vehicle_state();
      // 直接使用传入的目标点（来自ComputeTargetPoint）进行偏差计算
      // 车辆的坐标误差
      const double dx = x - target_point.path_point().x();
      const double dy = y - target_point.path_point().y();

      if (x == 0 && y == 0 && dx == 0)
      {
        AERROR << "Lat not received localization message error.";
        return;
      }

      debug->set_current_target_point(target_point);

      const double cos_target_heading = std::cos(target_point.path_point().theta());
      const double sin_target_heading = std::sin(target_point.path_point().theta());

      // double lateral_error = -(cos_target_heading * dy - sin_target_heading *
      // dx);
      // 正北方向 顺时针
      // double lateral_error = cos_target_heading * dx - sin_target_heading * dy;
      // 正东方向 逆时针
      // 横向误差计算
      double lateral_error = sin_target_heading * dx - cos_target_heading * dy;
      // std::cout << " target_point dx : " <<  dx << "  dy  : " << dy  << "
      // lateral_error  :  "<< lateral_error << endl;

      debug->set_lateral_error(lateral_error);

      // 输入参考的航向角
      debug->set_ref_heading(target_point.path_point().theta());
      // 正北方向 顺时针
      //  double heading_error =
      //      common::math::NormalizeAngle(theta - debug->ref_heading());

      // 航向误差计算，坐标系：正东方向、逆时针
      double heading_error = common::math::NormalizeAngle(debug->ref_heading() - theta);
      debug->set_heading_error(heading_error);
      // 航向误差结果输入
      // Estimate the heading error with look-ahead/look-back windows as feedback
      // signal for special driving scenarios
      double heading_error_feedback;

      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        heading_error_feedback = heading_error;
      }
      else
      {
        // 查看当前状态加上前向预测时间
        auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
            target_point.relative_time() +
                lookahead_station_ /
                    (std::max(std::fabs(linear_v), 0.1) * std::cos(heading_error)),
            query_forward_time_point_only_);
        heading_error_feedback = common::math::NormalizeAngle(
            heading_error + target_point.path_point().theta() -
            lookahead_point.path_point().theta());
      }
      debug->set_heading_error_feedback(heading_error_feedback);

      // Estimate the lateral error with look-ahead/look-back windows as feedback
      // signal for special driving scenarios
      double lateral_error_feedback;
      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        lateral_error_feedback =
            lateral_error - lookback_station_ * std::sin(heading_error);
      }
      else
      {
        lateral_error_feedback =
            lateral_error + lookahead_station_ * std::sin(heading_error);
      }
      debug->set_lateral_error_feedback(lateral_error_feedback);

      // 计算当前状态下的误差的一阶导数、二阶导数、三阶导数
      auto lateral_error_dot = linear_v * std::sin(heading_error);
      auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
      debug->set_lateral_error_rate(lateral_error_dot);
      debug->set_lateral_acceleration(lateral_error_dot_dot);
      debug->set_lateral_jerk(
          (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
      previous_lateral_acceleration_ = debug->lateral_acceleration();

      // 航向误差和参考航向的一阶导数、二阶导数、三阶导数
      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        debug->set_heading_rate(-angular_v);
      }
      else
      {
        debug->set_heading_rate(angular_v);
      }
      debug->set_ref_heading_rate(target_point.path_point().kappa() *
                                  target_point.v());
      debug->set_heading_error_rate(debug->heading_rate() -
                                    debug->ref_heading_rate());
      debug->set_heading_acceleration(
          (debug->heading_rate() - previous_heading_rate_) / ts_);
      debug->set_ref_heading_acceleration(
          (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
      debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                            debug->ref_heading_acceleration());
      debug->set_heading_jerk(
          (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
      debug->set_ref_heading_jerk(
          (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
          ts_);
      debug->set_heading_error_jerk(debug->heading_jerk() -
                                    debug->ref_heading_jerk());
      previous_heading_rate_ = debug->heading_rate();
      previous_ref_heading_rate_ = debug->ref_heading_rate();
      previous_heading_acceleration_ = debug->heading_acceleration();
      previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

      debug->set_curvature(target_point.path_point().kappa());
      // TODO  最终输出的各车辆的状态参数
    }

    void PurePursuitController::ComputeLookAheadDistance()
    {
      double v = injector_->vehicle_state()->linear_velocity();
      // 计算目标点预瞄距离（用于目标点匹配）
      lookahead_distance_ = common::math::Clamp(fabs(minimum_lookahead_distance_ + v * speed_to_lookahead_ratio_),
                                                minimum_lookahead_distance_,
                                                maximum_lookahead_distance_);
      // 计算Pure Pursuit算法预瞄距离
      purepursuit_lookahead_distance_ = common::math::Clamp(fabs(purepursuit_minimum_lookahead_distance_ + v * purepursuit_speed_to_lookahead_ratio_),
                                                            purepursuit_minimum_lookahead_distance_,
                                                            purepursuit_maximum_lookahead_distance_);
    }

    legionclaw::interface::TrajectoryPoint PurePursuitController::ComputeTargetPoint(
        const legionclaw::common::math::Vec2d pos, const double theta, const double s,
        const legionclaw::interface::ADCTrajectory *trajectory)
    {
      // 使用trajectory_analyzer的QueryProjectionPoint方法，直接根据位置、航向角和里程差s返回完整的TrajectoryPoint
      // 该方法已经实现了投影点索引查找和二分法查找目标里程的逻辑
      return trajectory_analyzer_.QueryProjectionPoint(pos, theta, s);
    }

    double PurePursuitController::ComputePointsDistanceSquared(
        const legionclaw::common::math::Vec2d pos1,
        const legionclaw::common::math::Vec2d pos2)
    {
      auto x = pos1.x() - pos2.x();
      auto y = pos1.y() - pos2.y();
      return x * x + y * y;
    }

    legionclaw::common::math::Vec2d PurePursuitController::ComputeRelativeXYOffset(
        const legionclaw::common::math::Vec2d pos1,
        const legionclaw::common::math::Vec2d pos2)
    {
      const auto diff_x = pos2.x() - pos1.x();
      const auto diff_y = pos2.y() - pos1.y();
      const auto heading = injector_->vehicle_state()->heading();
      const auto cos_heading = std::cos(heading);
      const auto sin_heading = std::sin(heading);
      const auto relative_x = (cos_heading * diff_x) + (sin_heading * diff_y);
      const auto relative_y = (-sin_heading * diff_x) + (cos_heading * diff_y);
      const legionclaw::common::math::Vec2d relative_xy(relative_x, relative_y);
      return relative_xy;
    }

    void PurePursuitController::ProcessLogs(
        const legionclaw::interface::ControlCommand *cmd,
        const SimpleLateralDebug *debug,
        const legionclaw::interface::Chassis *chassis)
    {
      // StrCat supports 9 arguments at most.
      if (enable_log_debug_)
      {
        stringstream sstream;
        sstream << TimeTool::Now2Ms() << std::fixed << "," << log_index_++ << ","
                << cmd->steer_driving_mode() << "," << cmd->front_steering_target()
                << "," << cmd->front_steering_rate() << ","
                << debug->lateral_error() << "," << debug->ref_heading() << ","
                << debug->heading() << "," << debug->heading_error() << ","
                << debug->heading_error_rate() << "," << debug->lateral_error_rate()
                << "," << debug->curvature() << "," << debug->front_steer_angle()
                << "," << debug->front_steer_angle_feedforward() << ","
                << debug->front_steer_angle_feedback_augment() << ","
                << debug->front_steer_angle_feedback() << ","
                << chassis->front_steering_value() << ","
                << injector_->vehicle_state()->linear_velocity() << ","
                << injector_->vehicle_state()->gear() << "," << calculate_time_
                << "," << debug->current_target_point().path_point().x() << ","
                << debug->current_target_point().path_point().y() << ","
                << current_x_ << "," << current_y_ << ",";

        // steer_log_file_ << sstream.str() << "\n";
      }
      //   AWARN<<sstream.str();
    }

    void PurePursuitController::SetAnalysis(const legionclaw::interface::ControlCommand *cmd,
                                            const SimpleLateralDebug *debug,
                                            const legionclaw::interface::Chassis *chassis,
                                            legionclaw::interface::ControlAnalysis *analysis)
    {
      analysis->set_ref_curvature(debug->curvature());
      analysis->set_ref_heading(debug->ref_heading() * 180 / M_PI);
      analysis->set_current_heading(debug->heading() * 180 / M_PI);
      analysis->set_heading_error(debug->heading_error() * 180 / M_PI);
      analysis->set_heading_error_rate(debug->heading_error_rate());
      analysis->set_lateral_error(debug->lateral_error());
      analysis->set_lateral_error_rate(debug->lateral_error_rate());
      analysis->set_front_steering_value_fd(chassis->front_steering_value() * 180.0 / M_PI);
      analysis->set_front_steering_target(cmd->front_steering_target());
      analysis->set_front_steering_rate(cmd->front_steering_rate());
      analysis->set_front_steer_angle_feedforward(debug->front_steer_angle_feedforward());
      analysis->set_front_steer_angle_feedback(debug->front_steer_angle_feedback());
      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        analysis->set_rear_steering_value_fd(chassis->rear_steering_value() * 180.0 / M_PI);
        analysis->set_rear_steering_target(cmd->rear_steering_target());
        analysis->set_rear_steering_rate(cmd->rear_steering_rate());
        analysis->set_rear_steer_angle_feedforward(debug->rear_steer_angle_feedforward());
        analysis->set_rear_steer_angle_feedback(debug->rear_steer_angle_feedback());
      }
      analysis->set_current_x(current_x_);
      analysis->set_current_y(current_y_);
      analysis->set_target_point_x(debug->current_target_point().path_point().x());
      analysis->set_target_point_y(debug->current_target_point().path_point().y());
      analysis->set_lat_target_point_s(debug->current_target_point().path_point().s());
    }

    std::string PurePursuitController::GetLogFileName()
    {
      time_t raw_time;
      char name_buffer[80];
      std::time(&raw_time);
      std::tm time_tm;
      localtime_r(&raw_time, &time_tm);
      strftime(name_buffer, 80, "./log/steer_log_simple_optimal_%F_%H%M%S.csv",
               &time_tm);
      return std::string(name_buffer);
    }

    void PurePursuitController::WriteHeaders(std::ofstream &file_stream)
    {
      file_stream << "time,"
                  << "index,"
                  << "steer_driving_mode,"
                  << "front_steering_target,"
                  << "front_steering_rate,"
                  << "current_lateral_error,"
                  << "current_ref_heading,"
                  << "current_heading,"
                  << "current_heading_error,"
                  << "heading_error_rate,"
                  << "lateral_error_rate,"
                  << "current_curvature,"
                  << "steer_angle,"
                  << "steer_angle_feedforward,"
                  << "steer_angle_feedback_augment,"
                  << "steer_angle_lateral_contribution,"
                  << "steer_angle_lateral_rate_contribution,"
                  << "steer_angle_heading_contribution,"
                  << "steer_angle_heading_rate_contribution,"
                  << "steer_angle_feedback,"
                  << "steering_position,"
                  << "v,"
                  << "gear,"
                  << "lqr_calculate_time,"
                  << "target_point_x,"
                  << "target_point_y,"
                  << "current_x,"
                  << "current_y," << "\n";
    }

    void PurePursuitController::CloseLogFile()
    {
      if (enable_log_debug_ && steer_log_file_.is_open())
      {
        steer_log_file_.close();
      }
    }

    void PurePursuitController::LogInitParameters()
    {
      AINFO << name_ << " begin.";
      AINFO << "[PurePursuitController parameters]"
            << " lf_: " << lf_ << ","
            << " lr_: " << lr_;
    }

    std::string PurePursuitController::Name() const { return name_; }

    void PurePursuitController::Stop() { CloseLogFile(); }

    legionclaw::common::Status PurePursuitController::Reset()
    {
      heading_error_pid_controller_.Reset();
      return Status::Ok();
    }
  } // namespace control
} // namespace legionclaw
