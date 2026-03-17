

#include "pid_controller.h"

#include <cmath>
#include <iostream>

namespace legionclaw {
namespace control {

double PIDController::Control(const double error, const double dt) {
  if (init_ == false || std::isnan(error)) return 0.0;

  if (dt <= 0) {
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  // 第一次进入不计算微分
  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }
  // 积分计算
  // integral hold
  if (!integrator_enabled_) {
    integral_ = 0;
  } else {
    integral_ += error * dt * ki_;
    // apply Ki before integrating to avoid steps when change Ki at steady state
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
      integrator_saturation_status_ = 1;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
      integrator_saturation_status_ = -1;
    } else {
      integrator_saturation_status_ = 0;
    }
  }
  if (integral_ > 0 && error < 0)  //快速进入刹车
    integral_ = 0;
  if (integral_ < 0 && error > 1e-6)  //快速进入刹车
    integral_ = 0;
  if (fabs(error) > 1.0 || fabs(error) < 0.05)   //稳态调节在允许速度差不进行积分调节，或者速度差足够大
    integral_ = 0;
  // if (epb_level != legionclaw::common::EPBLevel::RELEASED)
  //   integral_ = 0;

  previous_error_ = error;
  output = error * kp_ + integral_ + diff * kd_;  // Ki already applied
  previous_output_ = output;
  // std::cout<<"kp_: "<<kp_<<"\n";
  // std::cout<<"ki_: "<<ki_<<"\n";
  // std::cout<<"error: "<<error<<"\n";
  // std::cout<<"integral_: "<<integral_<<"\n";
  // std::cout<<"diff: "<<diff<<"\n";
  // std::cout<<"output: "<<output<<"\n";
  return output;
}

void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  output_saturation_status_ = 0;
}

void PIDController::Init(const PIDConf &pid_conf) {
  init_ = false;

  // if (!get_key()) return;

  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  //integrator_hold_ = false;
  output_saturation_status_ = 0;
  //TODO 此处未完成初始化，此处set无用
  SetPID(pid_conf);
  init_ = true;
}

void PIDController::SetPID(const PIDConf &pid_conf) {
  if (init_ == false) return;
  integrator_enabled_ = pid_conf.integrator_enable();
  //积分器饱和限幅的水平值。当PID控制器中的积分部分（I-term）在累积误差时，如果这个累积量达到设定的饱和极限，则积分器输出将不再继续增加或减少
  integrator_saturation_high_ = std::fabs(pid_conf.integrator_saturation_level());
  integrator_saturation_low_ = -std::fabs(pid_conf.integrator_saturation_level());
  output_saturation_high_ = std::fabs(pid_conf.output_saturation_level());
  output_saturation_low_ = -std::fabs(pid_conf.output_saturation_level());
  kp_ = pid_conf.kp();
  ki_ = pid_conf.ki();
  kd_ = pid_conf.kd();
  kaw_ = pid_conf.kaw();
}

int PIDController::IntegratorSaturationStatus() const {
  return integrator_saturation_status_;
}

// bool PIDController::IntegratorHold() const { return integrator_hold_; }

// void PIDController::SetIntegratorHold(bool hold) { integrator_hold_ = hold; }

}  // namespace control
}  // namespace legionclaw
