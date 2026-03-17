
/**
 * @file lon_dynamics.h
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2019-11-21
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#pragma once

#include <cmath>
#include "common/status/status.h"
#include "modules/control/src/proto/control_conf.pb.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

/**
 * @class LonDynamics
 *
 * @brief LonDynamics.
 */
class LonDynamics {
 public:
  /**
   * @brief constructor
   */
  LonDynamics() = default;

  /**
   * @brief destructor
   */
  ~LonDynamics() = default;

  /**
   * @brief init.
   * @param[in] control_conf control configurations
   * @return Status initialization status.
   */
  legionclaw::common::Status Init(const ControlConf *control_conf) {
    air_cd_ = control_conf->vehicle_param().air_damping_coefficient();
    air_density_ = control_conf->vehicle_param().air_density();
    rolling_cr_ = control_conf->vehicle_param().rolling_coefficient();
    face_area_ = control_conf->vehicle_param().length() *
                 control_conf->vehicle_param().width();

    const double mass_front = control_conf->vehicle_param().mass_fl() +
                              control_conf->vehicle_param().mass_fr();
    const double mass_rear = control_conf->vehicle_param().mass_rl() +
                             control_conf->vehicle_param().mass_rr();
    wheelbase_ = control_conf->vehicle_param().wheelbase();
    mass_ = mass_front + mass_rear;

    lf_ = wheelbase_ * (1.0 - mass_front / mass_);
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

    return legionclaw::common::Status::Ok();
  }

  double f_aero(double current_speed) {
    f_aero_ = 0.5 * air_density_ * face_area_ * std::abs(current_speed) *
              std::abs(current_speed);
    return f_aero_;
  }

//根据给定的空气密度、物体横截面积、速度和质量计算出物体在空气阻力作用下的加速度
  double acc_aero(double current_speed) {
    return (0.5 * air_density_ * face_area_ * std::abs(current_speed) *
            std::abs(current_speed) / mass_);
  }

  double f_grade(double pitch) {
    f_grade_ = mass_ * gra_acc_ * std::sin(pitch);
    return f_grade_;
  }

  double acc_grade(double pitch) { return (gra_acc_ * std::sin(pitch)); }

  double f_rolling(double pitch) {
    f_rolling_ = mass_ * gra_acc_ * rolling_cr_ * std::cos(pitch);
    return f_rolling_;
  }

  double acc_rolling(double pitch) {
    // std::cout<<"gra_acc_"<<gra_acc_<<std::endl;
    // std::cout<<"rolling_cr_"<<rolling_cr_<<std::endl;
    // std::cout<<"pitch"<<pitch<<std::endl;
    return (gra_acc_ * rolling_cr_ * std::cos(pitch));
  }

  double f_turning(double current_speed, double steering_angle) {
    turning_radius(current_speed, steering_angle);

    f_turning_ = mass_ * lr_ / wheelbase_ * current_speed * current_speed *
                 std::abs(std::tan(steering_angle) / turning_radius_);
    return f_turning_;
  }

//根据车辆当前的速度、转向角度以及车辆的尺寸特性（如前后轴距）来计算车辆在转弯时的加速度
  double acc_turning(double current_speed, double steering_angle) {
    // std::cout << " steering_angle :" << steering_angle<<endl;
    turning_radius(current_speed, steering_angle);
    // std::cout << "wheelbase_ :" << wheelbase_ << " lr_ :" << lr_<< "
    // turning_radius_ :" << turning_radius_<<endl;
    return (lr_ / wheelbase_ * current_speed * current_speed *
            std::abs(std::tan(steering_angle) / turning_radius_));
  }

  double f_accel(double acc) {
    f_accel_ = mass_ * acc;
    return f_accel_;
  }

  double turning_radius(double current_speed, double steering_angle) {
    double wf = 0.0, wr = 0.0, kug = 0.0;
    wf = lf_ / wheelbase_ * mass_ * gra_acc_;
    wr = lr_ / wheelbase_ * mass_ * gra_acc_;
    kug = wf - wr;
    turning_radius_ =
        (wheelbase_ + kug * current_speed * current_speed / gra_acc_) /
        steering_angle;
    return turning_radius_;
  }

  const double gra_acc_ = 9.8;

 protected:
  double mass_;
  double air_cd_;
  double air_density_;
  double rolling_cr_;
  double face_area_;
  double lf_;
  double lr_;
  double wheelbase_;
  double turning_radius_;
  double f_aero_;
  double f_grade_;
  double f_rolling_;
  double f_turning_;
  double f_accel_;
};  // namespace control
}  // namespace control
}  // namespace legionclaw
