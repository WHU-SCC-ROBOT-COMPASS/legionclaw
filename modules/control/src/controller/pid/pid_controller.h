
/**
 * @file pid_controller.h
 * @brief Defines the PIDController class.
 */

#pragma once

#include "modules/control/src/proto/pid_conf.pb.h"
// #include "modules/common/enum/enum.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

/**
 * @class PIDController
 * @brief 用于速度和转向的比例积分微分控制器
 */
class PIDController {
 public:
  /**
   * @brief 初始化 pid controller
   * @param pid_conf pid配置
   */
  void Init(const PIDConf &pid_conf);

  /**
   * @brief 设置比例、积分和微分的 PID 控制器系数
   * @param pid_conf pid配置
   */
  void SetPID(const PIDConf &pid_conf);

  /**
   * @brief 重置pid变量
   */
  void Reset();

  /**
   * @brief 基于误差计算控制值
   * @param error 误差值
   * @param dt 采样时间间隔
   * @return 基于PID的控制值
   */
  virtual double Control(const double error, const double dt);
  

  /**
   * @brief 获取饱和状态
   * @return 饱和状态
   */
  int IntegratorSaturationStatus() const;

  /**
   * @brief get status that if integrator is hold
   * @return if integrator is hold return true
   */
  //bool IntegratorHold() const;

  /**
   * @brief set whether to hold integrator component at its current value.
   * @param hold
   */
  //void SetIntegratorHold(bool hold);

 protected:
  /// @brief p值
  double kp_ = 0.0;
  /// @brief i值
  double ki_ = 0.0;
  /// @brief d值
  double kd_ = 0.0;
  /// @brief 抗积分饱和系数
  double kaw_ = 0.0;
  /// @brief 上次误差
  double previous_error_ = 0.0;
  /// @brief 上次输出
  double previous_output_ = 0.0;
  /// @brief 积分
  double integral_ = 0.0;
  /// @brief 积分上限
  double integrator_saturation_high_ = 0.0;
  /// @brief 积分下限
  double integrator_saturation_low_ = 0.0;
  /// @brief 是否首次执行
  bool first_hit_ = false;
  /// @brief 是否使用积分
  bool integrator_enabled_ = false;
  //bool integrator_hold_ = false;
  /// @brief 积分状态（1达到上限，-1达到下限，0未达到上下限）
  int integrator_saturation_status_ = 0;
  // Only used for pid_BC_controller and pid_IC_controller
  /// @brief 输出上限
  double output_saturation_high_ = 0.0;
  /// @brief 输出下限
  double output_saturation_low_ = 0.0;
  /// @brief 输出状态
  int output_saturation_status_ = 0;
  /// @brief 是否初始化
  bool init_;
};

}  // namespace control
}  // namespace legionclaw
