/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "Eigen/Core"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/logging/logging.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/status/status.h"
#include "modules/common/time/time_tool.h"

namespace legionclaw {
namespace control {
namespace common {

// VehicleStateProvider::VehicleStateProvider() {}

Status VehicleStateProvider::Update(
    const legionclaw::interface::LocalizationEstimate &localization,
    const legionclaw::interface::Chassis &chassis) {
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    // std::string msg = absl::StrCat(
    //     "Fail to update because ConstructExceptLinearVelocity error.",
    //     "localization:\n", localization.DebugString());
    //     std::string msg = absl::StrCat(
    // "Fail to update because ConstructExceptLinearVelocity error.",
    // "localization:\n");
    std::string msg = "";
    return Status(Status::ErrorCode::LOCALIZATION_ERROR, msg);
  }
  // if (localization.has_measurement_time()) {
  //   vehicle_state_.set_stamp(localization.measurement_time());
  // } else if (localization.header().has_timestamp()) {
  //   vehicle_state_.set_stamp((double)TimeTool::TimeStruct2Ms(localization.header().stamp()));
  // } else if (chassis.has_header() && chassis.header().has_timestamp()) {
  //   AERROR << "Unable to use location timestamp for vehicle state. Use
  //   chassis "
  //             "time instead.";
  //   vehicle_state_.set_stamp((double)TimeTool::TimeStruct2Ms(chassis.header().stamp()));
  // }
  vehicle_state_.set_header(chassis.header());

  // if (chassis.has_gear_location()) {
  vehicle_state_.set_gear(chassis.gear_location());
  // } else {
  //   vehicle_state_.set_gear(legionclaw::interface::Chassis::GEAR_NONE);
  // }

  // if (chassis.has_speed_mps()) {
  vehicle_state_.set_linear_velocity(chassis.speed_mps());
  if (!FLAGS_reverse_heading_vehicle_state &&
      vehicle_state_.gear() == legionclaw::common::GEAR_REVERSE) {
    vehicle_state_.set_linear_velocity(-vehicle_state_.linear_velocity());
  }
  //}

  // if (chassis.has_steering_percentage()) {
  vehicle_state_.set_front_steering_value(chassis.front_steering_value());
  //}

  static constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  vehicle_state_.set_driving_mode(chassis.steer_driving_mode());

  return Status::Ok();
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const legionclaw::interface::LocalizationEstimate &localization) {
  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  vehicle_state_.set_pose(localization.pose());

  vehicle_state_.set_x(localization.pose().position().x());
  vehicle_state_.set_y(localization.pose().position().y());
  vehicle_state_.set_z(localization.pose().position().z());

  const auto &orientation = localization.pose().orientation();

  vehicle_state_.set_heading(localization.pose().heading());

  vehicle_state_.set_angular_velocity(
      localization.pose().angular_velocity().z());

  vehicle_state_.set_linear_acceleration(
      localization.pose().linear_acceleration().y());

  math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                    orientation.qy(), orientation.qz());
  vehicle_state_.set_roll(euler_angle.roll());
  vehicle_state_.set_pitch(euler_angle.pitch());
  vehicle_state_.set_yaw(euler_angle.yaw());

  return true;
}

double VehicleStateProvider::x() const { return vehicle_state_.x(); }

double VehicleStateProvider::y() const { return vehicle_state_.y(); }

double VehicleStateProvider::z() const { return vehicle_state_.z(); }

double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading();
}

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa(); }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity();
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity();
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration();
}

double VehicleStateProvider::gear() const { return vehicle_state_.gear(); }

double VehicleStateProvider::front_steering_value() const {
  return vehicle_state_.front_steering_value();
}

double VehicleStateProvider::rear_steering_value() const {
  return vehicle_state_.rear_steering_value();
}

Header VehicleStateProvider::header() const { return vehicle_state_.header(); }

const Pose VehicleStateProvider::pose() const { return vehicle_state_.pose(); }

const Pose VehicleStateProvider::original_pose() const {
  return original_localization_.pose();
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /
                      vehicle_state_.angular_velocity();
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                            vehicle_state_.z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + vehicle_state_.x(),
                     vec_distance[1] + vehicle_state_.y());
}

//计算车辆的质心位置，输入从后轮到车辆的质心，输出车辆质心的位置。计算过程中使用到车辆位置信息(定位的坐标x,y)，姿态信息(定位的roll pitch yaw)，档位信息
math::Vec2d VehicleStateProvider::ComputeCOMPosition(const double rear_to_com_distance) const 
{
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  if ((FLAGS_state_transform_to_com_reverse &&
       vehicle_state_.gear() == legionclaw::common::GEAR_REVERSE) ||
      (FLAGS_state_transform_to_com_drive &&
       (vehicle_state_.gear() == legionclaw::common::GEAR_DRIVE ||
        vehicle_state_.gear() == legionclaw::common::GEAR_NEUTRAL))) {
    v << 0.0, rear_to_com_distance, 0.0;
    // v << 0.0, 0.0, 0.0;
  } else {
    v << 0.0, 0.0, 0.0;
  }

  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),vehicle_state_.z());

  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // std::cout<<"pos_vec: "<<pos_vec<<std::endl;
  //  std::cout<<"com_pos_3d.x(: "<<com_pos_3d.x()<<std::endl;
  // std::cout<<"com_pos_3d.y()): "<<com_pos_3d.y()<<std::endl;
  //    std::cout<<"com_pos_3d.z()): "<<com_pos_3d.z()<<std::endl;
  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) 
  {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),orientation.qy(), orientation.qz());

    // Update the COM position with rotation
    com_pos_3d = v.transpose() * quaternion.toRotationMatrix() + pos_vec.transpose();
    if (std::isnan(com_pos_3d.y())) 
    {
      com_pos_3d.z() = 0;
      com_pos_3d.y() = 0;
      com_pos_3d.x() = 0;
    }
  }
  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace common
}  // namespace control
}  // namespace legionclaw
