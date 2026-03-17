/**
 * @file lqr_lat_controller.cpp
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2018-07-07
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */
#include "lqr_lat_controller.h"

#include <time.h>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "modules/control/src/common/control_gflags.h"
#include "modules/control/src/common/filters/digital_filter_coefficients.h"
#include "modules/common/logging/logging.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/status/status.h"
#include "modules/common/time/time_tool.h"
#include "modules/control/src/common/trajectory_analyzer/trajectory_analyzer.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/quaternion.h"

namespace legionclaw
{
  namespace control
  {

    using namespace legionclaw::common;
    using namespace legionclaw::interface;

    std::string LQRLatController::GetLogFileName()
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

    void LQRLatController::WriteHeaders(std::ofstream &file_stream)
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

                  << "matrix_k_0,"
                  << "matrix_k_1,"
                  << "matrix_k_2,"
                  << "matrix_k_3,"

                  << "matrix_state_0,"
                  << "matrix_state_1,"
                  << "matrix_state_2,"
                  << "matrix_state_3,"
                  << "steering_position,"
                  << "v,"
                  << "gear,"
                  << "lqr_calculate_time,"
                  << "target_point_x,"
                  << "target_point_y,"
                  << "current_x,"
                  << "current_y," << "\n";
    }

    LQRLatController::LQRLatController() : name_("LQR-based Lateral Controller")
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

    LQRLatController::~LQRLatController() { CloseLogFile(); }

    legionclaw::common::Status LQRLatController::Init(
        std::shared_ptr<DependencyInjector> injector,
        const ControlConf *control_conf)
    {
      injector_ = injector;
      control_conf_ = control_conf;
      ts_ = control_conf_->lqr_controller_conf().ts();
      use_lqr_calibration_table_ =
          control_conf->lqr_controller_conf().use_lqr_calibration_table();

      if (!LoadLQRControlConf(control_conf_))
      {
        AERROR << "failed to load control conf";
        return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR,
                      "failed to load control_conf");
      }
      // Matrix init operations.
      // 矩阵大小初始化
      basic_state_size_ = control_conf_->lqr_controller_conf().matrix_q_size();
      // TODO      MPC
      const int r_matrix_size =
          control_conf_->mpc_controller_conf().matrix_r_size();

      const int matrix_size = basic_state_size_ + preview_window_;
      
      // 矩阵初始化
      matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
      matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
      matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
      matrix_a_(0, 1) = 1.0;
      matrix_a_(1, 2) = (cf_ + cr_) / mass_;
      matrix_a_(2, 3) = 1.0;
      matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

      matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
      matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
      matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
      matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
      matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
      // 区分两轮、四轮的b矩阵、k矩阵
      switch (steer_mode_)
      {
      case VehicleParam::STEER_MODE_2WS:
      {
        matrix_b_ = Matrix::Zero(basic_state_size_, 1);
        matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
        matrix_bdc_ = Matrix::Zero(matrix_size, 1);
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;

        matrix_k_ = Matrix::Zero(1, matrix_size);
      }
      break;
      case VehicleParam::STEER_MODE_4WS:
      {
        matrix_b_ = Matrix::Zero(basic_state_size_, 2);
        matrix_bd_ = Matrix::Zero(basic_state_size_, 2);
        matrix_bdc_ = Matrix::Zero(matrix_size, 2);
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_b_(1, 1) = cr_ / mass_;
        matrix_b_(3, 1) = -1.0 * lr_ * cr_ / iz_;

        matrix_k_ = Matrix::Zero(2, matrix_size);
      }
      break;
      default:
        AERROR << "steer mode error.";
        break;
      }

      matrix_state_ = Matrix::Zero(matrix_size, 1);
      // q、r矩阵用配置参数初始化
      matrix_q_ = Matrix::Zero(matrix_size, matrix_size);
      int q_param_size = control_conf_->lqr_controller_conf().matrix_q().size();
      int reverse_q_param_size =
          control_conf_->lqr_controller_conf().reverse_matrix_q_size();
      if (matrix_size != q_param_size || matrix_size != reverse_q_param_size)
      {
        stringstream sstream;
        sstream << "lateral controller error: matrix_q size: " << q_param_size
                << "lateral controller error: reverse_maxtrix_q size: "
                << reverse_q_param_size
                << " in parameter file not equal to matrix_size: " << matrix_size;
        AERROR << sstream.str();
        return Status(Status::ErrorCode::CONTROL_COMPUTE_ERROR, sstream.str());
      }

      for (int i = 0; i < q_param_size; ++i)
      {
        matrix_q_(i, i) = control_conf_->lqr_controller_conf().matrix_q().at(i);
      }

      matrix_q_updated_ = matrix_q_;

      // 区分两轮、四轮r矩阵
      switch (steer_mode_)
      {
      case VehicleParam::STEER_MODE_2WS:
      {
        matrix_r_ = Matrix::Identity(1, 1);
      }
      break;
      case VehicleParam::STEER_MODE_4WS:
      {
        matrix_r_ = Matrix::Zero(r_matrix_size, r_matrix_size);
        int r_param_size = control_conf_->mpc_controller_conf().matrix_r().size();
        for (int i = 0; i < r_param_size; ++i)
        {
          matrix_r_(i, i) = control_conf_->mpc_controller_conf().matrix_r().at(i);
        }
      }
      break;
      default:
        AERROR << "R matrix error.";
        break;
      }

      // 是否导入lqr离线标定表
      if (use_lqr_calibration_table_)
      {
        LoadLqrCalibrationTable(
            control_conf_->lqr_controller_conf().lqr_calibration_table());
      }
      InitializeFilters(control_conf_);
      auto lat_controller_conf = control_conf_->lqr_controller_conf();
      LoadLatGainScheduler(lat_controller_conf);

      // LogInitParameters();
      // 未使用
      bool enable_leadlag = control_conf_->lqr_controller_conf()
                                .enable_reverse_leadlag_compensation();
      if (enable_leadlag)
      {
        leadlag_controller_.Init(lat_controller_conf.reverse_leadlag_conf(), ts_);
      }
      // 未使用
      bool enable_mrac =
          control_conf_->lqr_controller_conf().enable_steer_mrac_control();
      if (enable_mrac)
      {
        mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(), ts_);
      }

      enable_look_ahead_back_control_ =
          control_conf_->lqr_controller_conf().enable_look_ahead_back_control();

      return Status::Ok();
    }

    bool LQRLatController::LoadLQRControlConf(const ControlConf *control_conf)
    {
      if (control_conf == nullptr)
      {
        AERROR << "[LatController] control_conf == nullptr";
        return false;
      }
      vehicle_param_ = control_conf->vehicle_param();

      steer_mode_ = (VehicleParam::SteerMode)vehicle_param_.steer_mode();

      ts_ = control_conf_->lqr_controller_conf().ts();
      if (ts_ <= 0.0)
      {
        AERROR << "[MPCController] Invalid control update interval.";
        return false;
      }

      use_new_planning_mode_ = control_conf->use_new_planning_mode();

      cf_ = vehicle_param_.cf();
      cr_ = vehicle_param_.cr();
      
      // 检查cf和cr是否有效（LQR控制器需要非零的侧偏刚度）
      if (std::abs(cf_) < 1e-6) {
        AERROR << "ERROR: VehicleParam cf (front cornering stiffness) is 0 or too small: " << cf_;
        AERROR << "LQR controller requires non-zero cf value (typically 150000-200000 N/rad)";
        AERROR << "Please check vehicle_param.json configuration file and set cf to a valid value.";
      }
      if (std::abs(cr_) < 1e-6) {
        AERROR << "ERROR: VehicleParam cr (rear cornering stiffness) is 0 or too small: " << cr_;
        AERROR << "LQR controller requires non-zero cr value (typically 150000-200000 N/rad)";
        AERROR << "Please check vehicle_param.json configuration file and set cr to a valid value.";
      }
      if (std::abs(cf_) < 1e-6 || std::abs(cr_) < 1e-6) {
        AERROR << "LQR controller will not work correctly with zero cf or cr values!";
        AERROR << "This will cause matrix_b_ to be zero, leading to uncontrollable system and zero feedback gain.";
      }
      
      preview_window_ = control_conf_->lqr_controller_conf().preview_window();
      lookahead_station_ = control_conf_->lqr_controller_conf().lookahead_station();
      lookback_station_ = control_conf_->lqr_controller_conf().lookback_station();
      wheelbase_ = vehicle_param_.wheelbase();
      steer_ratio_ = vehicle_param_.steer_ratio();
      // 弧度转角度
      front_steer_single_direction_max_degree_ =
          vehicle_param_.max_front_steer_angle() * 180 / M_PI;
      front_steer_single_direction_min_degree_ =
          vehicle_param_.min_front_steer_angle() * 180 / M_PI;
      rear_steer_single_direction_max_degree_ =
          vehicle_param_.max_rear_steer_angle() * 180 / M_PI;
      rear_steer_single_direction_min_degree_ =
          vehicle_param_.min_rear_steer_angle() * 180 / M_PI;
      max_lat_acc_ =
          control_conf_->lqr_controller_conf().max_lateral_acceleration();
      low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();

      use_navigation_mode_ = control_conf->use_navigation_mode();
      query_time_nearest_point_only_ =
          control_conf->query_time_nearest_point_only();
      query_forward_time_point_only_ =
          control_conf->query_forward_time_point_only();

      enable_navigation_mode_position_update_ =
          control_conf->enable_navigation_mode_position_update();
      enable_navigation_mode_error_filter_ =
          control_conf->enable_navigation_mode_error_filter();
      reverse_heading_control_ =
          control_conf_->lqr_controller_conf().reverse_heading_control();
      trajectory_transform_to_com_reverse_ =
          control_conf->trajectory_transform_to_com_reverse();
      trajectory_transform_to_com_drive_ =
          control_conf->trajectory_transform_to_com_drive();
      enable_gain_scheduler_ =
          control_conf_->lqr_controller_conf().enable_gain_scheduler();
      enable_feedback_augment_on_high_speed_ =
          control_conf->enable_feedback_augment_on_high_speed();
      set_steer_limit_ = control_conf->set_steer_limit();
      enable_maximum_steer_rate_limit_ =
          control_conf->enable_maximum_steer_rate_limit();
      lock_steer_speed_ = control_conf->lock_steer_speed();
      front_steer_angle_rate_ = control_conf->steer_angle_rate();

      enable_error_sigmoid_control_ =
          control_conf_->lqr_controller_conf().enable_error_sigmoid_control();
      lat_error_axis_ = control_conf_->lqr_controller_conf()
                            .error_sigmoid_conf()
                            .lat_error_axis();
      lat_error_slope_ = control_conf_->lqr_controller_conf()
                             .error_sigmoid_conf()
                             .lat_error_slope();
      heading_error_axis_ = control_conf_->lqr_controller_conf()
                                .error_sigmoid_conf()
                                .heading_error_axis();
      heading_error_slope_ = control_conf_->lqr_controller_conf()
                                 .error_sigmoid_conf()
                                 .heading_error_slope();

      enable_error_apc_scheduler_ =
          control_conf_->lqr_controller_conf().enable_error_apc_scheduler();
      lateral_error_threshold_ =
          control_conf_->lqr_controller_conf().lateral_error_threshold();
      heading_error_threshold_ =
          control_conf_->lqr_controller_conf().heading_error_threshold();
      q_lat_err_max_ = control_conf_->lqr_controller_conf().q_lat_err_max();
      q_heading_err_max_ = control_conf_->lqr_controller_conf().q_heading_err_max();

      const double mass_fl = vehicle_param_.mass_fl();
      const double mass_fr = vehicle_param_.mass_fr();
      const double mass_rl = vehicle_param_.mass_rl();
      const double mass_rr = vehicle_param_.mass_rr();
      const double mass_front = mass_fl + mass_fr;
      const double mass_rear = mass_rl + mass_rr;
      mass_ = mass_front + mass_rear;

      lf_ = wheelbase_ * (1.0 - mass_front / mass_);
      lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

      // moment of inertia
      iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

      lqr_eps_ = control_conf_->lqr_controller_conf().eps();
      lqr_max_iteration_ = control_conf_->lqr_controller_conf().max_iteration();

      query_relative_time_ = control_conf->query_relative_time();

      minimum_speed_protection_ = control_conf->minimum_speed_protection();

      enable_log_debug_ = control_conf_->lqr_controller_conf().enable_log_debug();
      lqr_print_enable_ = control_conf_->lqr_controller_conf().lqr_print_enable();
      if (enable_log_debug_)
      {
        steer_log_file_.open(GetLogFileName());
        steer_log_file_ << std::fixed;
        steer_log_file_ << std::setprecision(6);
        WriteHeaders(steer_log_file_);
      }

      return true;
    }

    void LQRLatController::UpdateState(
        const legionclaw::interface::ADCTrajectory *trajectory,
        const legionclaw::interface::LocalizationEstimate *localization,
        const legionclaw::interface::Chassis *chassis, SimpleLateralDebug *debug)
    {
      auto vehicle_state = injector_->vehicle_state();
      // 根据不同模式计算横向误差
      if (FLAGS_use_navigation_mode)
      {
        current_x_ = 0.0;
        current_y_ = 0.0;
        ComputeLateralErrors(
            0.0, 0.0, driving_orientation_, vehicle_state->linear_velocity(),
            vehicle_state->angular_velocity(), vehicle_state->linear_acceleration(),
            trajectory_analyzer_, debug, chassis);
      }
      else
      {
        if (use_new_planning_mode_)
          add_lr_ = 0.0;
        else
        {
          switch (state_machine)
          {
          case 0:
            add_lr_ = lr_;
            if (vehicle_state->gear() == legionclaw::common::GEAR_REVERSE)
            {
              state_machine = 1;
            }
            else
            {
              break;
            }
          case 1:
            add_lr_ = 0.0;
            if (vehicle_state->gear() == legionclaw::common::GEAR_DRIVE)
            {
              state_machine = 0;
            }
            else
            {
              break;
            }
          default:
            break;
          }
        }

        const auto &com = vehicle_state->ComputeCOMPosition(add_lr_);
        //auto Location_time_diff = TimeTool::GetTimeDiffNow(localization->header().stamp());
        //const auto &com = vehicle_state->EstimateFuturePosition(Location_time_diff);
        
        current_x_ = com.x();
        current_y_ = com.y();
        ComputeLateralErrors(com.x(), com.y(), driving_orientation_,
                             vehicle_state->linear_velocity(),
                             vehicle_state->angular_velocity(),
                             vehicle_state->linear_acceleration(),
                             trajectory_analyzer_, debug, chassis);
      }

      // State matrix update;
      // First four elements are fixed;
      if (control_conf_->lqr_controller_conf().enable_look_ahead_back_control())
      {
        matrix_state_(0, 0) = debug->lateral_error_feedback();
        matrix_state_(2, 0) = debug->heading_error_feedback();
      }
      else
      {
        matrix_state_(0, 0) = debug->lateral_error();
        matrix_state_(2, 0) = debug->heading_error();
      }

      // 未使用
      if (enable_error_sigmoid_control_)
      {
        double beta =
            1.0 / (1.0 + exp(-lat_error_slope_ * (fabs(debug->lateral_error()) -
                                                  lat_error_axis_))); // 0.0625
        matrix_state_(0, 0) = beta * debug->lateral_error();
        beta = 1.0 / (1.0 + exp(-heading_error_slope_ *
                                (fabs(debug->heading_error() * 180 / M_PI) -
                                 heading_error_axis_)));
        matrix_state_(2, 0) = beta * debug->heading_error();
      }

      // 状态矩阵横向误差、航向误差赋值
      matrix_state_(1, 0) = debug->lateral_error_rate();
      matrix_state_(3, 0) = debug->heading_error_rate();

      // 未使用
      // Next elements are depending on preview window size;
      for (int i = 0; i < preview_window_; ++i)
      {
        // std::cout<<"preview_window_: "<<preview_window_<<"\n";
        const double preview_time = ts_ * (i + 1);
        const auto preview_point =
            trajectory_analyzer_.QueryNearestPointByRelativeTime(
                preview_time, query_forward_time_point_only_);

        const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
            preview_point.path_point().x(), preview_point.path_point().y());

        const double dx =
            preview_point.path_point().x() - matched_point.path_point().x();
        const double dy =
            preview_point.path_point().y() - matched_point.path_point().y();

        const double cos_matched_theta =
            std::cos(matched_point.path_point().theta());
        const double sin_matched_theta =
            std::sin(matched_point.path_point().theta());
        const double preview_d_error =
            cos_matched_theta * dy - sin_matched_theta * dx;

        matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
      }
    }

    void LQRLatController::ComputeLateralErrors(
        const double x, const double y, const double theta, const double linear_v,
        const double angular_v, const double linear_a,
        const TrajectoryAnalyzer &trajectory_analyzer, SimpleLateralDebug *debug,
        const legionclaw::interface::Chassis *chassis)
    {
      auto vehicle_state = injector_->vehicle_state();
      legionclaw::interface::TrajectoryPoint target_point;

      if (query_time_nearest_point_only_)
      {
        target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
            TimeTool::NowToSeconds() + query_relative_time_,
            query_forward_time_point_only_);
      }
      else
      {
        if (use_navigation_mode_ && !enable_navigation_mode_position_update_)
        {
          target_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
              TimeTool::NowToSeconds() + query_relative_time_,
              query_forward_time_point_only_);
        }
        else
        {
          // target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
          target_point = trajectory_analyzer.QueryProjectionPoint({x, y}, theta);
          //double t = TimeTool::NowToSeconds();
          //target_point = trajectory_analyzer.QueryProjectionPoint({x, y}, theta,t + query_relative_time_,chassis->speed_mps());
          //AWARN<<fixed<<"TimeTool::Now2Seconds(): "<<t;
          //AWARN<<"query_relative_time_: "<<query_relative_time_;
        }
      }
      // std::cout <<"com.x()::" << x<<"  com.y()::" << y<< endl;
      // TODO  车辆的坐标误差
      const double dx = x - target_point.path_point().x();
      const double dy = y - target_point.path_point().y();
      if (x == 0 && y == 0 && dx == 0)
      {
        AERROR << "Lat not received localization message error.";
        return;
      }

      debug->set_current_target_point(target_point);

      //轨迹中的参考航向误差
      const double cos_target_heading = std::cos(target_point.path_point().theta());
      const double sin_target_heading = std::sin(target_point.path_point().theta());

      // double lateral_error = -(cos_target_heading * dy - sin_target_heading *
      // dx);
      //正北方向 顺时针
      // double lateral_error = cos_target_heading * dx - sin_target_heading * dy;
      //正东方向 逆时针
      // TODO 横向误差计算
      double lateral_error = sin_target_heading * dx - cos_target_heading * dy;
      double longitudinal_error = - cos_target_heading * dx - sin_target_heading * dy;

      if (enable_navigation_mode_error_filter_)
      {
        lateral_error = lateral_error_filter_.Update(lateral_error);
      }

      debug->set_lateral_error(lateral_error);
      debug->set_longitudinal_error(longitudinal_error);

      // TODO   输入参考的航向角
      debug->set_ref_heading(target_point.path_point().theta());
      //正北方向 顺时针
      // double heading_error =
      //     common::math::NormalizeAngle(theta - debug->ref_heading());
      //正东方向 逆时针
      double heading_error =
          common::math::NormalizeAngle(debug->ref_heading() - theta);
      if (enable_navigation_mode_error_filter_)
      {
        heading_error = heading_error_filter_.Update(heading_error);
      }
      debug->set_heading_error(heading_error);
      //  TODO  航向误差结果输入
      // Estimate the heading error with look-ahead/look-back windows as feedback
      // signal for special driving scenarios
      double heading_error_feedback;

      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        heading_error_feedback = heading_error;
      }
      else
      {
        // TODO 查看当前状态加上前向预测时间
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

      // TODO   计算当前状态下的误差的导数
      auto lateral_error_dot = linear_v * std::sin(heading_error);
      auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
      if (reverse_heading_control_)
      {
        if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
        {
          lateral_error_dot = -lateral_error_dot;
          lateral_error_dot_dot = -lateral_error_dot_dot;
        }
      }
      debug->set_lateral_error_rate(lateral_error_dot);
      debug->set_lateral_acceleration(lateral_error_dot_dot);
      debug->set_lateral_jerk(
          (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
      previous_lateral_acceleration_ = debug->lateral_acceleration();

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
      previous_heading_rate_ = debug->heading_rate();
      previous_ref_heading_rate_ = debug->ref_heading_rate();

      debug->set_heading_jerk(
          (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
      debug->set_ref_heading_jerk(
          (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
          ts_);
      debug->set_heading_error_jerk(debug->heading_jerk() -
                                    debug->ref_heading_jerk());
      previous_heading_acceleration_ = debug->heading_acceleration();
      previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

      debug->set_curvature(target_point.path_point().kappa());
      double steer_angle_feedforwardterm_rad = std::atan(wheelbase_ * target_point.path_point().kappa());
      // double steer_angle_feedforwardterm_deg = R2D(steer_angle_feedforwardterm_rad);
      debug->set_front_steer_angle_feedforward(steer_angle_feedforwardterm_rad);
      debug->set_rear_steer_angle_feedforward(target_point.rear_steer());
      // TODO  最终输出的各车辆的状态参数
    }

    void LQRLatController::UpdateMatrix(const legionclaw::interface::Chassis *chassis)
    {
      double v;
      // At reverse driving, replace the lateral translational motion dynamics with
      // the corresponding kinematic models
      if (injector_->vehicle_state()->gear() == legionclaw::common::GEAR_REVERSE)
      {
        v = std::min(injector_->vehicle_state()->linear_velocity(),
                     -minimum_speed_protection_);
      }
      else
      {
        v = std::max(injector_->vehicle_state()->linear_velocity(),
                     minimum_speed_protection_);
      }

      matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
      matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
      matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
      matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
      matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

      Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());

      matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
                   (matrix_i + ts_ * 0.5 * matrix_a_);
    }

    void LQRLatController::UpdateMatrixCompound()
    {
      // Initialize preview matrix

      matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;

      switch (steer_mode_)
      {
      case VehicleParam::STEER_MODE_2WS:
      {
        matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
      }
      break;
      case VehicleParam::STEER_MODE_4WS:
      {
        matrix_bdc_.block(0, 0, basic_state_size_, 2) = matrix_bd_;
      }
      break;
      default:
        AERROR << "B matrix error.";
        break;
      }

      if (preview_window_ > 0)
      {
        // std::cout << "preview_window_: " << preview_window_ << "\n";
        matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
        // Update A matrix;
        for (int i = 0; i < preview_window_ - 1; ++i)
        {
          matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
        }
      }
    }

    // TODO   计算前馈的值，前馈由轨迹规划给出
    double LQRLatController::ComputeFeedForward(
        double ref_curvature, const legionclaw::interface::Chassis *chassis) const
    {
      double steer_angle_feedforwardterm;
      // steer_angle_feedforwardterm =
      //     std::atan(wheelbase_ * ref_curvature) * 180 / M_PI * steer_ratio_;
      // std::cout << "steer_angle_feedforwardterm: " << steer_angle_feedforwardterm
      //           << "\n";
      // std::cout << "\n";
      return steer_angle_feedforwardterm;
    }

    // TODO  主要计算当前的
    legionclaw::common::Status LQRLatController::ComputeControlCommand(
        const legionclaw::interface::LocalizationEstimate *localization,
        const legionclaw::interface::Chassis *chassis,
        const legionclaw::interface::ADCTrajectory *trajectory,
        legionclaw::interface::ControlCommand *cmd,
        legionclaw::interface::ControlAnalysis *analysis)
    {
      int64_t current_time = legionclaw::common::TimeTool::Now2Us();
      auto vehicle_state = injector_->vehicle_state();

      auto target_tracking_trajectory = *trajectory;
      // TODO  判断是不是导航模式   不执行
      if (use_navigation_mode_ && enable_navigation_mode_position_update_)
      {
        auto time_stamp_diff = (double)TimeTool::TimeStruct2Ms(
                                   target_tracking_trajectory.header().stamp()) -
                               current_trajectory_timestamp_;

        auto curr_vehicle_x = localization->pose().position().x();
        auto curr_vehicle_y = localization->pose().position().y();

        double curr_vehicle_heading = 0.0;
        const auto &orientation = localization->pose().orientation();
        if (localization->pose().has_heading())
        {
          curr_vehicle_heading = localization->pose().heading();
        }
        else
        {
          curr_vehicle_heading =
              common::math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                                orientation.qy(), orientation.qz());
        }
        // new planning trajectory
        if (time_stamp_diff > 1.0e-6)
        {
          // init_vehicle_x_ = curr_vehicle_x;
          // init_vehicle_y_ = curr_vehicle_y;
          // init_vehicle_heading_ = curr_vehicle_heading;
          current_trajectory_timestamp_ = (double)TimeTool::TimeStruct2Ms(
              target_tracking_trajectory.header().stamp());
        }
      }

      // int64_t time_diff_5 = legionclaw::common::TimeTool::Now2Us() - current_time;
      //         std::cout << "time_diff5555:::" << time_diff_5 << endl;
      // 轨迹分析器更新轨迹
      trajectory_analyzer_ =
          std::move(TrajectoryAnalyzer(&target_tracking_trajectory));
      // int64_t time_diff_4 = legionclaw::common::TimeTool::Now2Us() - current_time;
      //         std::cout << "time_diff4444:::" << time_diff_4 << endl;

      // lqr计算过程，需要区分前进和后退
      // Transform the coordinate of the planning trajectory from the center of the
      // rear-axis to the center of mass, if conditions matched
      // 轨迹是否转化到质心，未使用
      if (((trajectory_transform_to_com_reverse_ &&
            vehicle_state->gear() == legionclaw::common::GEAR_REVERSE) ||
           (trajectory_transform_to_com_drive_ &&
            vehicle_state->gear() == legionclaw::common::GEAR_DRIVE)) &&
          enable_look_ahead_back_control_)
      {
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
      }

      // Re-build the vehicle dynamic models at reverse driving (in particular,
      // replace the lateral translational motion dynamics with the corresponding
      // kinematic models)
      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        cf_ = -vehicle_param_.cf();
        cr_ = -vehicle_param_.cr();
      }
      else
      {
        cf_ = vehicle_param_.cf();
        cr_ = vehicle_param_.cr();
      }

      switch (steer_mode_)
      {
      case VehicleParam::STEER_MODE_2WS:
      {
        matrix_a_(0, 1) = 0.0;
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
        matrix_a_coeff_(0, 2) = 1.0;
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        // matrix_bd_ = matrix_b_ * ts_;
      }
      break;
      case VehicleParam::STEER_MODE_4WS:
      {
        matrix_a_(0, 1) = 0.0;
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
        matrix_a_coeff_(0, 2) = 1.0;
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_b_(1, 1) = cr_ / mass_;
        matrix_b_(3, 1) = -1.0 * lr_ * cr_ / iz_;
        // matrix_bd_ = matrix_b_ * ts_;
      }
      break;
      default:
        AERROR << "matrix error.";
        break;
      }

      // TODO   更新驾驶方向，确定是倒车还是前进状态，更改YAW的正负号
      UpdateDrivingOrientation(localization, chassis);

      // TODO    更新车辆的当前状态
      UpdateState(trajectory, localization, chassis, simple_lateral_debug_.get());
      // int64_t time_diff_3 = legionclaw::common::TimeTool::Now2Us() - current_time;
      //         std::cout << "time_diff3333:::" << time_diff_3 << endl;

      // TODO    更新车辆矩阵的各参数值
      UpdateMatrix(chassis);

      // 更新车辆状态复合矩阵、控制复合矩阵
      UpdateMatrixCompound();

      // q矩阵区分前进、后退
      if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
      {
        if (control_conf_->lqr_controller_conf().reverse_matrix_q_size() ==
            control_conf_->lqr_controller_conf().matrix_q_size())
        {
          int temp_size = control_conf_->lqr_controller_conf().matrix_q_size();
          matrix_q_ = Matrix::Zero(temp_size, temp_size);
          for (int32_t i = 0;
               i < control_conf_->lqr_controller_conf().reverse_matrix_q_size();
               ++i)
          {
            matrix_q_(i, i) =
                control_conf_->lqr_controller_conf().reverse_matrix_q().at(i);
          }
        }
        else
        {
          AERROR << "reverse_matrix_q length error";
        }
      }
      else
      {
        int temp_size = control_conf_->lqr_controller_conf().matrix_q_size();
        matrix_q_ = Matrix::Zero(temp_size, temp_size);
        for (int32_t i = 0; i < temp_size; ++i)
        {
          matrix_q_(i, i) = control_conf_->lqr_controller_conf().matrix_q().at(i);
        }
      }

      // Add gain scheduler for higher speed steering
      if (enable_gain_scheduler_)
      {
        // q矩阵对速度自适应
        matrix_q_updated_(0, 0) =
            matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(
                                  std::fabs(vehicle_state->linear_velocity()));
        matrix_q_updated_(2, 2) =
            matrix_q_(2, 2) * heading_err_interpolation_->Interpolate(
                                  std::fabs(vehicle_state->linear_velocity()));

        if (enable_error_apc_scheduler_)
        {
          // q矩阵对误差自适应
          // 位置偏差和航向偏差对汽车转向方向相同，则q1增加，q3不变
          // 位置偏差和航向偏差对汽车转向方向相反，则q1不变，q3增加
          // 当位置偏差和航向偏差方向相反，且位置偏差在 0.02m 以内的振荡，不进行调节
          double k_lat_error_ = 1.0, k_heading_error_ = 1.0;
          if (simple_lateral_debug_->heading_error() >= 0)
          {
            if (simple_lateral_debug_->lateral_error() > 0)
            {
              k_lat_error_ += lat_err_apc_interpolation_->Interpolate(
                  std::fabs(simple_lateral_debug_->lateral_error()));
            }
            else
            {
              k_heading_error_ += heading_err_apc_interpolation_->Interpolate(
                  std::fabs(simple_lateral_debug_->heading_error()) / M_PI * 180.0);
            }
          }
          else
          {
            if (simple_lateral_debug_->lateral_error() > 0)
            {
              k_heading_error_ += heading_err_apc_interpolation_->Interpolate(
                  std::fabs(simple_lateral_debug_->heading_error()) / M_PI * 180.0);
            }
            else
            {
              k_lat_error_ += lat_err_apc_interpolation_->Interpolate(
                  std::fabs(simple_lateral_debug_->lateral_error()));
            }
          }
          // 加权矩阵改进，规则：当其对应误差较大时,系统会得到一个较大的控制量,实现快速调整;
          // 当其对应误差较小时,参数 q1,q3 随之减小,避免超调。
          if (fabs(simple_lateral_debug_->lateral_error()) <=
              lateral_error_threshold_)
          {
            matrix_q_updated_(0, 0) =
                matrix_q_updated_(0, 0) * k_lat_error_ *
                std::pow(2, fabs(simple_lateral_debug_->lateral_error()));
          }
          else
          {
            matrix_q_updated_(0, 0) = matrix_q_updated_(0, 0) * q_lat_err_max_;
          }
          if (fabs(simple_lateral_debug_->heading_error()) <=
              heading_error_threshold_ / 180.0 * M_PI)
          {
            matrix_q_updated_(2, 2) =
                matrix_q_updated_(2, 2) * k_heading_error_ *
                std::pow(2, fabs(simple_lateral_debug_->heading_error()));
          }
          else
          {
            matrix_q_updated_(2, 2) = matrix_q_updated_(2, 2) * q_heading_err_max_;
          }
        }

        // lqr使用离线计算结果，或实时计算
        if (use_lqr_calibration_table_)
        {
          UpdateMatrixKFromCalibration();
        }
        else
        {
          common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                        matrix_r_, lqr_eps_, lqr_max_iteration_,
                                        &matrix_k_);
        }
      }
      else
      {
        if (use_lqr_calibration_table_)
        {
          UpdateMatrixKFromCalibration();
        }
        else
        {
          common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                        matrix_r_, lqr_eps_, lqr_max_iteration_,
                                        &matrix_k_);
        }
      }

      // feedback = - K * state
      // Convert vehicle steer angle from rad to degree and then to steer degree
      // then to 100% ratio
      // lqr前轮转角反馈计算
      
      // 计算 matrix_k_ * matrix_state_
      double k_times_state_0 = (matrix_k_ * matrix_state_)(0, 0);
      
      // 速度插值检查
      double vehicle_speed = std::fabs(vehicle_state->linear_velocity());
      double speed_kp = lat_speed_kp_interpolation_->Interpolate(vehicle_speed);
      
      // 计算前轮转角反馈（度）
      // k_times_state_0 是前轮转角（弧度）
      // 转换为度，注意：不乘以转向比，转向比在最后统一处理
      double front_steer_angle_feedback =
          -k_times_state_0 * 180 / M_PI * speed_kp;

      if (std::isnan(front_steer_angle_feedback))
      {
        front_steer_angle_feedback = 0.0;
      }

      double rear_steer_angle_feedback = 0.0;
      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        rear_steer_angle_feedback = -(matrix_k_ * matrix_state_)(1, 0) * 180 / M_PI;
      }

      // TODO  计算轨迹输出前馈值是否满足需求

      double front_steer_angle_feedforward =
          R2D(simple_lateral_debug_->front_steer_angle_feedforward()) ;
      double rear_steer_angle_feedforward =
          R2D(simple_lateral_debug_->rear_steer_angle_feedforward());
      double front_steer_angle = 0.0;
      double rear_steer_angle = 0.0;
      double front_steer_angle_feedback_augment = 0.0;
      double rear_steer_angle_feedback_augment = 0.0;
      bool enable_leadlag = control_conf_->lqr_controller_conf()
                                .enable_reverse_leadlag_compensation();
      // Augment the feedback control on lateral error at the desired speed domain
      if (enable_leadlag)
      {
        if (enable_feedback_augment_on_high_speed_ ||
            std::fabs(vehicle_state->linear_velocity()) <= low_speed_bound_)
        {
          front_steer_angle_feedback_augment =
              leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI ;
        }
      }

      if (cmd->steer_driving_mode() ==
          legionclaw::common::DrivingMode::COMPLETE_MANUAL)
      {
        front_steer_angle_feedback = 0.0;
        rear_steer_angle_feedback = 0.0;
      }
      // std::cout << "front_steer_angle_feedforward:" << front_steer_angle_feedforward << "\n";
      // std::cout << "matrix_state_: " << "\n"
      //           << matrix_state_ << "\n";
      // std::cout << "matrix_k_: " << "\n"
      //           << matrix_k_ << "\n";
      // std::cout << "-(matrix_k_ * matrix_state_)(0, 0) * 180 / M_PI: " << "\n"
      //           << -(matrix_k_ * matrix_state_)(0, 0) * 180 / M_PI << "\n";

      // TODO  前轮
      // 计算前轮转角（度）= 反馈 + 前馈 + 反馈增强
      front_steer_angle = front_steer_angle_feedback +
                          front_steer_angle_feedforward +
                          front_steer_angle_feedback_augment;

      // 打印横向误差和航向角误差
      ADEBUG<<"==================================================";
      ADEBUG << "横向误差（米）：" << matrix_state_(0, 0);
      ADEBUG << "航向角误差（度）：" << (matrix_state_(2, 0) * 180 / M_PI);
      
      ADEBUG << "前馈（前轮转角，度）：" << front_steer_angle_feedforward;
      ADEBUG << "反馈（前轮转角，度）：" << front_steer_angle_feedback;
      ADEBUG << "反馈增强（前轮转角，度）：" << front_steer_angle_feedback_augment;
      ADEBUG << "前轮转角总和（度）：" << front_steer_angle;
      
      // 将前轮转角转换为方向盘转角（度）
      front_steer_angle = front_steer_angle * steer_ratio_;

      // front_steer_angle = front_steer_angle_feedforward +
      //                     front_steer_angle_feedback_augment;

      // Compute the steering command limit with the given maximum lateral
      // acceleration
      const double front_steer_limit =
          (set_steer_limit_) ? std::atan(max_lat_acc_ * wheelbase_ /
                                         (vehicle_state->linear_velocity() *
                                          vehicle_state->linear_velocity())) *
                                   steer_ratio_ * 180 / M_PI
                             : front_steer_single_direction_max_degree_;

      const double rear_steer_limit =
          (set_steer_limit_) ? std::atan(max_lat_acc_ * wheelbase_ /
                                         (vehicle_state->linear_velocity() *
                                          vehicle_state->linear_velocity())) *
                                   180 / M_PI
                             : rear_steer_single_direction_max_degree_;

      // 方向盘转角的最大变化量（度） = 车轮转角速率 × 时间间隔 × 转向比
      // 注意：因为 front_steer_angle 现在是方向盘转角，所以需要乘以转向比
      const double front_steer_diff_with_max_rate =
          (enable_maximum_steer_rate_limit_)
              ? vehicle_param_.max_front_steer_angle_rate() * ts_ * 180 / M_PI * steer_ratio_
              : front_steer_single_direction_max_degree_;

      const double rear_steer_diff_with_max_rate =
          (enable_maximum_steer_rate_limit_)
              ? vehicle_param_.max_rear_steer_angle_rate() * ts_ * 180 / M_PI
              : rear_steer_single_direction_max_degree_;

      const double front_steering_position = chassis->front_steering_value();
      const double rear_steering_position = chassis->rear_steering_value();

      // Re-compute the steering command if the MRAC control is enabled, with
      // steer angle limitation and steer rate limitation
      bool enable_mrac =
          control_conf_->lqr_controller_conf().enable_steer_mrac_control();
      if (enable_mrac)
      {
        const int mrac_model_order = control_conf_->lqr_controller_conf()
                                         .steer_mrac_conf()
                                         .mrac_model_order();
        Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
        steer_state(0, 0) = chassis->front_steering_value();
        if (mrac_model_order > 1)
        {
          steer_state(1, 0) =
              (front_steering_position - pre_front_steering_position_) / ts_;
        }
        front_steer_angle = mrac_controller_.Control(
            front_steer_angle, steer_state, front_steer_limit,
            front_steer_diff_with_max_rate / ts_);
      }
      pre_front_steering_position_ = front_steering_position;

      // Clamp the steer angle with steer limitations at current speed
      double front_steer_angle_limited = common::math::Clamp(
          front_steer_angle, -front_steer_limit, front_steer_limit);
      front_steer_angle = front_steer_angle_limited;
      simple_lateral_debug_->set_front_steer_angle_limited(
          front_steer_angle_limited);

      if (std::isnan(front_steer_angle))
      {
        front_steer_angle = 0;
      }
      // Limit the steering command with the designed digital filter
      front_steer_angle = digital_filter_.Filter(front_steer_angle);
      front_steer_angle = common::math::Clamp(
          front_steer_angle, front_steer_single_direction_min_degree_,
          front_steer_single_direction_max_degree_);

      // Check if the steer is locked and hence the previous steer angle should be
      // executed
      //TODO 行车泊车区分
      /* if (std::abs(vehicle_state->linear_velocity()) < lock_steer_speed_ &&
          (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_DRIVE ||
           vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE) &&
          chassis->steer_driving_mode() ==
              legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE)
      {
        front_steer_angle = pre_front_steer_angle_;
      } */

      if (cmd->steer_driving_mode() == legionclaw::common::DrivingMode::COMPLETE_MANUAL ||
          trajectory->behaviour_lat_state() ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE /*||
          trajectory->behaviour_lat_state() == 
              legionclaw::interface::ADCTrajectory::BehaviourLatState::FINISH_STATE*/)
      {
        front_steer_angle = 0.0;
        rear_steer_angle = 0.0;
      }

      cmd->set_front_steering_target(
          math::Clamp(front_steer_angle,
                      pre_front_steer_angle_ - front_steer_diff_with_max_rate,
                      pre_front_steer_angle_ + front_steer_diff_with_max_rate));

      if (std::isnan(cmd->front_steering_target()))
      {
        cmd->set_front_steering_target(0.0);
      }

      front_steer_angle_rate_ =
          (cmd->front_steering_target() - pre_front_steer_angle_) / ts_;

      cmd->set_front_steering_rate(front_steer_angle_rate_);
      //cmd->set_front_steering_rate(vehicle_param_.max_front_steer_angle_rate() * 180 / M_PI);

      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        // std::cout << "-(matrix_k_ * matrix_state_)(1, 0) * 180 / M_PI: " << "\n"
        //           << -(matrix_k_ * matrix_state_)(1, 0) * 180 / M_PI << "\n";
        // TODO   测试后轮转向反馈值的正负号  反馈是否影响输入的后轮转角前馈值

        rear_steer_angle = rear_steer_angle_feedback +
                           rear_steer_angle_feedforward +
                           rear_steer_angle_feedback_augment;

        // rear_steer_angle = rear_steer_angle_feedforward +
        //                    rear_steer_angle_feedback_augment;
        const double rear_steer_limit =
            (set_steer_limit_) ? std::atan(max_lat_acc_ * wheelbase_ /
                                           (vehicle_state->linear_velocity() *
                                            vehicle_state->linear_velocity())) *
                                     180 / M_PI
                               : rear_steer_single_direction_max_degree_;

        const double rear_steer_diff_with_max_rate =
            (enable_maximum_steer_rate_limit_)
                ? vehicle_param_.max_rear_steer_angle_rate() * ts_ * 180 / M_PI
                : rear_steer_single_direction_max_degree_;

        const double rear_steering_position = chassis->rear_steering_value();

        // Re-compute the steering command if the MRAC control is enabled, with
        // steer angle limitation and steer rate limitation
        bool enable_mrac =
            control_conf_->lqr_controller_conf().enable_steer_mrac_control();
        if (enable_mrac)
        {
          const int mrac_model_order = control_conf_->lqr_controller_conf()
                                           .steer_mrac_conf()
                                           .mrac_model_order();
          Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
          steer_state(0, 0) = chassis->rear_steering_value();
          if (mrac_model_order > 1)
          {
            steer_state(1, 0) =
                (rear_steering_position - pre_front_steering_position_) / ts_;
          }
          rear_steer_angle = mrac_controller_.Control(
              rear_steer_angle, steer_state, rear_steer_limit,
              rear_steer_diff_with_max_rate / ts_);
        }
        pre_front_steering_position_ = rear_steering_position;

        // Clamp the steer angle with steer limitations at current speed
        double rear_steer_angle_limited = common::math::Clamp(
            rear_steer_angle, -rear_steer_limit, rear_steer_limit);
        rear_steer_angle = rear_steer_angle_limited;
        simple_lateral_debug_->set_rear_steer_angle_limited(
            rear_steer_angle_limited);

        if (std::isnan(rear_steer_angle))
        {
          rear_steer_angle = 0;
        }
        // Limit the steering command with the designed digital filter
        // rear_steer_angle = digital_filter_.Filter(rear_steer_angle);
        rear_steer_angle = common::math::Clamp(
            rear_steer_angle, rear_steer_single_direction_min_degree_,
            rear_steer_single_direction_max_degree_);

        // Check if the steer is locked and hence the previous steer angle should
        // be executed
        if (std::abs(vehicle_state->linear_velocity()) < lock_steer_speed_ &&
            (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_DRIVE ||
             vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE) &&
            chassis->steer_driving_mode() ==
                legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE)
        {
          rear_steer_angle = pre_rear_steer_angle_;
        }

        if (cmd->steer_driving_mode() == legionclaw::common::DrivingMode::COMPLETE_MANUAL ||
            trajectory->behaviour_lat_state() ==
                legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE /*||
            trajectory->behaviour_lat_state() ==
                legionclaw::interface::ADCTrajectory::BehaviourLatState::FINISH_STATE*/)
        {
          rear_steer_angle = 0.0;
        }

        cmd->set_rear_steering_target(math::Clamp(
            rear_steer_angle, pre_rear_steer_angle_ - rear_steer_diff_with_max_rate,
            pre_rear_steer_angle_ + rear_steer_diff_with_max_rate));

        if (std::isnan(cmd->rear_steering_target()))
        {
          cmd->set_rear_steering_target(0.0);
        }

        rear_steer_angle_rate_ =
            (cmd->rear_steering_target() - pre_rear_steer_angle_) / ts_;

        cmd->set_rear_steering_rate(rear_steer_angle_rate_);
        //cmd->set_rear_steering_rate(vehicle_param_.max_rear_steer_angle_rate() * 180 / M_PI);
      }

      if(vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_NEUTRAL)
      {
        cmd->set_front_steering_target(pre_front_steer_angle_);
        cmd->set_rear_steering_target(pre_rear_steer_angle_);
      }

      cmd->set_steer_driving_mode(trajectory->driving_mode());
      static double last_time = 0.0;
      static double hold_time = 0.0;

      // 转向回正逻辑
      switch (steer_back_step_)
      {
      case 0:
        if(trajectory->driving_mode() == legionclaw::common::DrivingMode::COMPLETE_MANUAL &&
        trajectory->behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE)
        {
          steer_back_step_ = 1;
          cmd->set_front_steering_target(0.0);
          cmd->set_rear_steering_target(0.0);
          cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
          last_time = TimeTool::NowToSeconds();
        }
        break;
      case 1:
        {
          cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
          double current_time = TimeTool::NowToSeconds();
          double time_diff = current_time - last_time;
          if(std::fabs(chassis->front_steering_value()) < 2.0 && std::fabs(chassis->rear_steering_value()) < 2.0)
          {
            hold_time = TimeTool::NowToSeconds();
            steer_back_step_ = 2;
          }
          if(time_diff >= 3.0)
          {
            steer_back_step_ = 3;
          }
          cmd->set_front_steering_target(0.0);
          cmd->set_rear_steering_target(0.0);
        }
        break;
      case 2:
        {
          cmd->set_steer_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
          double current_time = TimeTool::NowToSeconds();
          double time_diff = current_time - hold_time;
          if(time_diff >= 0.5)
          {
            steer_back_step_ = 3;
          }
          cmd->set_front_steering_target(0.0);
          cmd->set_rear_steering_target(0.0);
        }
        break;
      case 3:
        cmd->set_steer_driving_mode(trajectory->driving_mode());
        cmd->set_front_steering_target(0.0);
        cmd->set_rear_steering_target(0.0);
        if(trajectory->behaviour_lat_state() != legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE
           /*&& trajectory->behaviour_lat_state() != legionclaw::interface::ADCTrajectory::BehaviourLatState::INITIAL_STATE*/)
        {
          steer_back_step_ = 0;
        }
        break;
      default:
        steer_back_step_ = 0;
      }

      // compute extra information for logging and debugging
      // 计算横向误差、航向误差、横向误差率、航向误差率对车轮转角度的贡献值
      const double front_steer_angle_lateral_contribution =
          -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_;

      const double front_steer_angle_lateral_rate_contribution =
          -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_ratio_;

      const double front_steer_angle_heading_contribution =
          -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_;

      const double front_steer_angle_heading_rate_contribution =
          -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_ratio_;

      const double rear_steer_angle_lateral_contribution = 0.0;

      const double rear_steer_angle_lateral_rate_contribution = 0.0;

      const double rear_steer_angle_heading_contribution = 0.0;

      const double rear_steer_angle_heading_rate_contribution = 0.0;

      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        const double front_steer_angle_lateral_contribution =
            -matrix_k_(1, 0) * matrix_state_(0, 0) * 180 / M_PI;

        const double front_steer_angle_lateral_rate_contribution =
            -matrix_k_(1, 1) * matrix_state_(1, 0) * 180 / M_PI;

        const double front_steer_angle_heading_contribution =
            -matrix_k_(1, 2) * matrix_state_(2, 0) * 180 / M_PI;

        const double front_steer_angle_heading_rate_contribution =
            -matrix_k_(1, 3) * matrix_state_(3, 0) * 180 / M_PI;
      }

      simple_lateral_debug_->set_heading(driving_orientation_);
      simple_lateral_debug_->set_front_steer_angle(front_steer_angle);
      simple_lateral_debug_->set_front_steer_angle_lateral_contribution(
          front_steer_angle_lateral_contribution);
      simple_lateral_debug_->set_front_steer_angle_lateral_rate_contribution(
          front_steer_angle_lateral_rate_contribution);
      simple_lateral_debug_->set_front_steer_angle_heading_contribution(
          front_steer_angle_heading_contribution);
      simple_lateral_debug_->set_front_steer_angle_heading_rate_contribution(
          front_steer_angle_heading_rate_contribution);
      simple_lateral_debug_->set_front_steer_angle_feedback(
          front_steer_angle_feedback);
      simple_lateral_debug_->set_front_steer_angle_feedback_augment(
          front_steer_angle_feedback_augment);
      simple_lateral_debug_->set_front_steering_position(front_steering_position);

      simple_lateral_debug_->set_rear_steer_angle(rear_steer_angle);
      // simple_lateral_debug_->set_rear_steer_angle_feedforward(
      //     rear_steer_angle_feedforward);
      simple_lateral_debug_->set_rear_steer_angle_lateral_contribution(
          rear_steer_angle_lateral_contribution);
      simple_lateral_debug_->set_rear_steer_angle_lateral_rate_contribution(
          rear_steer_angle_lateral_rate_contribution);
      simple_lateral_debug_->set_rear_steer_angle_heading_contribution(
          rear_steer_angle_heading_contribution);
      simple_lateral_debug_->set_rear_steer_angle_heading_rate_contribution(
          rear_steer_angle_heading_rate_contribution);
      simple_lateral_debug_->set_rear_steer_angle_feedback(
          rear_steer_angle_feedback);
      simple_lateral_debug_->set_rear_steer_angle_feedback_augment(
          rear_steer_angle_feedback_augment);
      simple_lateral_debug_->set_rear_steering_position(rear_steering_position);
      simple_lateral_debug_->set_ref_speed(vehicle_state->linear_velocity());

      if (lqr_print_enable_)
      {
        static int index_display = 0;
        if (index_display == 1) //
        {
          index_display = 0;

          printf("\033[2J");
          std::cout.precision(5);
          std::cout << "--------------lqr conroller info---------" << endl;
          std::cout << "steer_driving_mode: " << cmd->steer_driving_mode()
                    << "   lqr_calculate_time: " << lqr_calculate_time_ << endl;
          std::cout << endl;
          std::cout << "steer_angle_fd_augment deg: "
                    << simple_lateral_debug_->front_steer_angle_feedback_augment()
                    << endl;
          std::cout << "steering_angle_fb deg: "
                    << simple_lateral_debug_->front_steer_angle_feedback() << endl;
          std::cout << "steering_angle_ff deg: "
                    << simple_lateral_debug_->front_steer_angle_feedforward()
                    << endl;
          std::cout << "front_steering_target deg: " << cmd->front_steering_target()
                    << endl;
          std::cout << endl;
          std::cout << "head_error_ deg: "
                    << simple_lateral_debug_->heading_error() * 180 / M_PI << endl;
          std::cout << "head_error_rate_ deg/s: "
                    << simple_lateral_debug_->heading_error_rate() * 180 / M_PI
                    << endl;
          std::cout << "lateral_error_ m: "
                    << simple_lateral_debug_->lateral_error() << endl;
          std::cout << "lateral_error_rate_ m/s: "
                    << simple_lateral_debug_->lateral_error_rate() << endl;
          std::cout << endl;
          std::cout
              << "front_steer_angle_heading_contribution_: "
              << simple_lateral_debug_->front_steer_angle_heading_contribution()
              << endl;
          std::cout << "front_steer_angle_heading_rate_contribution_ : "
                    << simple_lateral_debug_
                           ->front_steer_angle_heading_rate_contribution()
                    << endl;
          std::cout
              << "front_steer_angle_lateral_contribution_: "
              << simple_lateral_debug_->front_steer_angle_lateral_contribution()
              << endl;
          std::cout << "front_steer_angle_lateral_rate_contribution_ m/s: "
                    << simple_lateral_debug_
                           ->front_steer_angle_lateral_rate_contribution()
                    << endl;
          if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
          {
            std::cout
                << "rear_steer_angle_heading_contribution_: "
                << simple_lateral_debug_->rear_steer_angle_heading_contribution()
                << endl;
            std::cout << "rear_steer_angle_heading_rate_contribution_ : "
                      << simple_lateral_debug_
                             ->rear_steer_angle_heading_rate_contribution()
                      << endl;
            std::cout
                << "rear_steer_angle_lateral_contribution_: "
                << simple_lateral_debug_->rear_steer_angle_lateral_contribution()
                << endl;
            std::cout << "rear_steer_angle_lateral_rate_contribution_ m/s: "
                      << simple_lateral_debug_
                             ->rear_steer_angle_lateral_rate_contribution()
                      << endl;
          }
          std::cout << "gear_location: " << cmd->gear_location()
                    << "  epb_level_: " << cmd->epb_level() << endl;
          std::cout << "current_x_ : " << current_x_
                    << "  current_y_ : " << current_y_ << endl;
          std::cout
              << "target_point_x_ : "
              << simple_lateral_debug_->current_target_point().path_point().x()
              << "  target_point_y_ : "
              << simple_lateral_debug_->current_target_point().path_point().y()
              << endl;
          std::cout << endl;
        }
        index_display++;
      }

      //直接采用规划的推荐的轮子转角
      if (simple_lateral_debug_->current_target_point().is_steer_valid())
      {
        cmd->set_front_steering_target(
            simple_lateral_debug_->current_target_point().front_steer() * 180 /
            M_PI * steer_ratio_);
        cmd->set_rear_steering_target(
            simple_lateral_debug_->current_target_point().rear_steer() * 180 /
            M_PI);
      }
      pre_front_steer_angle_ = cmd->front_steering_target();
      pre_rear_steer_angle_ = cmd->rear_steering_target();

      ProcessLogs(cmd, simple_lateral_debug_.get(), chassis);
      SetAnalysis(cmd, simple_lateral_debug_.get(), chassis, analysis);
      lqr_calculate_time_ = legionclaw::common::TimeTool::Now2Us() - current_time;
      lqr_calculate_time_max_ = max(lqr_calculate_time_, lqr_calculate_time_max_);
      analysis->set_lqr_calculate_time(lqr_calculate_time_);
      analysis->set_lqr_calculate_time_max(lqr_calculate_time_max_);
      return Status::Ok();
    }

    void LQRLatController::ProcessLogs(const legionclaw::interface::ControlCommand *cmd,
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
                << debug->front_steer_angle_lateral_contribution() << ","
                << debug->front_steer_angle_lateral_rate_contribution() << ","
                << debug->front_steer_angle_heading_contribution() << ","
                << debug->front_steer_angle_heading_rate_contribution() << ","
                << debug->front_steer_angle_feedback()
                << ","

                //  << matrix_a_(1, 1) <<","
                //  << matrix_a_(1, 3) <<","
                //  << matrix_a_(3, 1) <<","
                //  << matrix_a_(3, 3) <<","

                << matrix_k_(0, 0) << "," << matrix_k_(0, 1) << ","
                << matrix_k_(0, 2) << "," << matrix_k_(0, 3) << ","

                << matrix_state_(0, 0) << "," << matrix_state_(1, 0) << ","
                << matrix_state_(2, 0) << "," << matrix_state_(3, 0) << ","
                << chassis->front_steering_value() << ","
                << injector_->vehicle_state()->linear_velocity() << ","
                << injector_->vehicle_state()->gear() << "," << lqr_calculate_time_
                << "," << debug->current_target_point().path_point().x() << ","
                << debug->current_target_point().path_point().y() << ","
                << current_x_ << "," << current_y_ << ",";

        // steer_log_file_ << sstream.str() << "\n";
      }
      //   AWARN<<sstream.str();
    }

    void LQRLatController::SetAnalysis(
        const legionclaw::interface::ControlCommand *cmd,
        const SimpleLateralDebug *debug, const legionclaw::interface::Chassis *chassis,
        legionclaw::interface::ControlAnalysis *analysis)
    {
      analysis->set_ref_curvature(debug->curvature());
      analysis->set_ref_heading(debug->ref_heading() * 180 / M_PI);
      analysis->set_current_heading(debug->heading() * 180 / M_PI);
      analysis->set_heading_error(debug->heading_error());
      analysis->set_heading_error_rate(debug->heading_error_rate());
      analysis->set_lateral_error(debug->lateral_error());
      analysis->set_lateral_error_rate(debug->lateral_error_rate());
      analysis->set_lon_error(debug->longitudinal_error());
      analysis->set_front_steering_value_fd(chassis->front_steering_value() *
                                            180.0 / M_PI);
      analysis->set_front_steering_target(cmd->front_steering_target());
      analysis->set_front_steering_rate(cmd->front_steering_rate());
      analysis->set_front_steer_angle_feedforward(
          debug->front_steer_angle_feedforward());
      analysis->set_front_steer_angle_feedback(debug->front_steer_angle_feedback());
      analysis->set_front_steer_angle_lateral_contribution(
          debug->front_steer_angle_lateral_contribution());
      analysis->set_front_steer_angle_lateral_rate_contribution(
          debug->front_steer_angle_lateral_rate_contribution());
      analysis->set_front_steer_angle_heading_contribution(
          debug->front_steer_angle_heading_contribution());
      analysis->set_front_steer_angle_heading_rate_contribution(
          debug->front_steer_angle_heading_rate_contribution());
      analysis->set_matrix_k_00(matrix_k_(0, 0));
      analysis->set_matrix_k_01(matrix_k_(0, 1));
      analysis->set_matrix_k_02(matrix_k_(0, 2));
      analysis->set_matrix_k_03(matrix_k_(0, 3));
      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        analysis->set_rear_steering_value_fd(chassis->rear_steering_value() *
                                             180.0 / M_PI);
        analysis->set_rear_steering_target(cmd->rear_steering_target());
        analysis->set_rear_steering_rate(cmd->rear_steering_rate());
        analysis->set_rear_steer_angle_feedforward(
            debug->rear_steer_angle_feedforward());
        analysis->set_rear_steer_angle_feedback(debug->rear_steer_angle_feedback());
        analysis->set_rear_steer_angle_lateral_contribution(
            debug->rear_steer_angle_lateral_contribution());
        analysis->set_rear_steer_angle_lateral_rate_contribution(
            debug->rear_steer_angle_lateral_rate_contribution());
        analysis->set_rear_steer_angle_heading_contribution(
            debug->rear_steer_angle_heading_contribution());
        analysis->set_rear_steer_angle_heading_rate_contribution(
            debug->rear_steer_angle_heading_rate_contribution());
        analysis->set_matrix_k_10(matrix_k_(1, 0));
        analysis->set_matrix_k_11(matrix_k_(1, 1));
        analysis->set_matrix_k_12(matrix_k_(1, 2));
        analysis->set_matrix_k_13(matrix_k_(1, 3));
      }
      analysis->set_matrix_state_0(matrix_state_(0, 0));
      analysis->set_matrix_state_1(matrix_state_(1, 0));
      analysis->set_matrix_state_2(matrix_state_(2, 0));
      analysis->set_matrix_state_3(matrix_state_(3, 0));
      analysis->set_matrix_q_updated_0(matrix_q_updated_(0, 0));
      analysis->set_matrix_q_updated_1(matrix_q_updated_(1, 1));
      analysis->set_matrix_q_updated_2(matrix_q_updated_(2, 2));
      analysis->set_matrix_q_updated_3(matrix_q_updated_(3, 3));
      analysis->set_current_x(current_x_);
      analysis->set_current_y(current_y_);
      analysis->set_target_point_x(debug->current_target_point().path_point().x());
      analysis->set_target_point_y(debug->current_target_point().path_point().y());
      analysis->set_lat_target_point_s(
          debug->current_target_point().path_point().s());
    }

    void LQRLatController::InitializeFilters(const ControlConf *control_conf)
    {
      // Low pass filter
      std::vector<double> den(3, 0.0);
      std::vector<double> num(3, 0.0);
      LpfCoefficients(ts_, control_conf_->lqr_controller_conf().cutoff_freq(), &den,
                      &num);
      digital_filter_.set_coefficients(den, num);
      lateral_error_filter_ = MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf_->lqr_controller_conf().mean_filter_window_size()));
      heading_error_filter_ = MeanFilter(static_cast<std::uint_fast8_t>(
          control_conf_->lqr_controller_conf().mean_filter_window_size()));
    }

    void LQRLatController::CloseLogFile()
    {
      if (enable_log_debug_ && steer_log_file_.is_open())
      {
        steer_log_file_.close();
      }
    }

    void LQRLatController::LoadLatGainScheduler(
        const LQRControllerConf &lqr_controller_conf)
    {
      // 从配置文件加载q矩阵增益参数
      const auto &lat_err_gain_scheduler =
          lqr_controller_conf.lat_err_gain_scheduler();
      const auto &heading_err_gain_scheduler =
          lqr_controller_conf.heading_err_gain_scheduler();
      const auto &lat_err_apc_scheduler =
          lqr_controller_conf.lat_err_apc_scheduler();
      const auto &heading_err_apc_scheduler =
          lqr_controller_conf.heading_err_apc_scheduler();
      const auto &lat_speed_kp_scheduler =
          lqr_controller_conf.lat_speed_kp_scheduler();
      AINFO << "Lateral control gain scheduler loaded";
      Interpolation1D::DataType xy1, xy2, xy3, xy4, xy5;
      for (const auto &scheduler : lat_err_gain_scheduler)
      {
        xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }
      for (const auto &scheduler : heading_err_gain_scheduler)
      {
        xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }
      for (const auto &scheduler : lat_err_apc_scheduler)
      {
        xy3.push_back(std::make_pair(scheduler.error(), scheduler.ratio()));
      }
      for (const auto &scheduler : heading_err_apc_scheduler)
      {
        xy4.push_back(std::make_pair(scheduler.error(), scheduler.ratio()));
      }
      for (const auto &scheduler : lat_speed_kp_scheduler)
      {
        xy5.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
      }

      lat_err_interpolation_.reset(new Interpolation1D);
      if (lat_err_interpolation_->Init(xy1) == false)
        AERROR << "Fail to load lateral error gain scheduler";

      heading_err_interpolation_.reset(new Interpolation1D);
      if (heading_err_interpolation_->Init(xy2) == false)
        AERROR << "Fail to load heading error gain scheduler";

      lat_err_apc_interpolation_.reset(new Interpolation1D);
      if (lat_err_apc_interpolation_->Init(xy3) == false)
        AERROR << "Fail to load lateral error apc scheduler";

      heading_err_apc_interpolation_.reset(new Interpolation1D);
      if (heading_err_apc_interpolation_->Init(xy4) == false)
        AERROR << "Fail to load heading error apc scheduler";

      lat_speed_kp_interpolation_.reset(new Interpolation1D);
      if (lat_speed_kp_interpolation_->Init(xy5) == false)
        AERROR << "Fail to load lat speed kp gain scheduler";
    }

    void LQRLatController::LogInitParameters()
    {
      AINFO << name_ << " begin.";
      AINFO << "[LatController parameters]"
            << " mass_: " << mass_ << ","
            << " iz_: " << iz_ << ","
            << " lf_: " << lf_ << ","
            << " lr_: " << lr_;
    }

    void LQRLatController::UpdateDrivingOrientation(
        const legionclaw::interface::LocalizationEstimate *localization,
        const legionclaw::interface::Chassis *chassis)
    {
      auto vehicle_state = injector_->vehicle_state();
      driving_orientation_ = vehicle_state->heading();
      matrix_bd_ = matrix_b_ * ts_;
      // Reverse the driving direction if the vehicle is in reverse mode
      if (reverse_heading_control_)
      {
        if (vehicle_state->gear() == legionclaw::common::GearPosition::GEAR_REVERSE)
        {
          driving_orientation_ =
              common::math::NormalizeAngle(driving_orientation_ + M_PI);
          // Update Matrix_b for reverse mode
          matrix_bd_ = -matrix_b_ * ts_;
          ADEBUG << "Matrix_b changed due to gear direction";
        }
      }
    }

    void LQRLatController::LoadLqrCalibrationTable(
        const LqrCalibrationTable &lqr_calibration_table)
    {
      // 从配置文件加载lqr离线计算结果
      const auto &lqr_table = lqr_calibration_table;
      AINFO << "LQR calibration table loaded";
      AINFO << "LQR calibration table size is "
            << lqr_table.calibration_table().size();
      std::vector<Interpolation1D::DataType> xy_vec;
      xy_vec.resize(lqr_table.calibration_table().begin()->k_size());
      for (const auto &calibration : lqr_table.calibration_table())
      {
        auto v = calibration.v();
        for (int i = 0; i < calibration.k().size(); i++)
        {
          xy_vec[i].push_back(std::make_pair(v, calibration.k(i)));
        }
      }
      lqr_k_interpolation_vector.resize(xy_vec.size());
      for (uint32_t i = 0; i < xy_vec.size(); i++)
      {
        lqr_k_interpolation_vector[i].reset(new Interpolation1D);
        if (lqr_k_interpolation_vector[i]->Init(xy_vec[i]) == false)
          AERROR << "Fail to load lqr calibration table";
      }
    }

    void LQRLatController::UpdateMatrixKFromCalibration()
    {
      // 用插值法从lqr离线计算结果中，获得当前速度对应的k矩阵
      double v;
      if (injector_->vehicle_state()->gear() == legionclaw::common::GEAR_REVERSE)
      {
        v = std::min(
            -1.0 * std::fabs(injector_->vehicle_state()->linear_velocity()),
            -minimum_speed_protection_);
      }
      else
      {
        v = std::max(injector_->vehicle_state()->linear_velocity(),
                     minimum_speed_protection_);
      }
      // std::cout << "v_calibration: " << v << endl;
      matrix_k_(0, 0) = lqr_k_interpolation_vector[0]->Interpolate(v);
      matrix_k_(0, 1) = lqr_k_interpolation_vector[1]->Interpolate(v);
      matrix_k_(0, 2) = lqr_k_interpolation_vector[2]->Interpolate(v);
      matrix_k_(0, 3) = lqr_k_interpolation_vector[3]->Interpolate(v);
      if (steer_mode_ == VehicleParam::STEER_MODE_4WS)
      {
        matrix_k_(1, 0) = lqr_k_interpolation_vector[4]->Interpolate(v);
        matrix_k_(1, 1) = lqr_k_interpolation_vector[5]->Interpolate(v);
        matrix_k_(1, 2) = lqr_k_interpolation_vector[6]->Interpolate(v);
        matrix_k_(1, 3) = lqr_k_interpolation_vector[7]->Interpolate(v);
      }
    }

    std::string LQRLatController::Name() const { return name_; }

    void LQRLatController::Stop() { CloseLogFile(); }

    legionclaw::common::Status LQRLatController::Reset() { return Status::Ok(); }
  } // namespace control
} // namespace legionclaw
