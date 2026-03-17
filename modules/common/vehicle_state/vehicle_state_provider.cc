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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "Eigen/Core"
// #include "absl/strings/str_cat.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/logging/logging.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time_tool.h"

namespace legionclaw {
namespace common {
Status VehicleStateProvider::Update(const Location &localization,
                                    const Chassis &chassis) {
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    // std::string msg = absl::StrCat(
    //     "Fail to update because ConstructExceptLinearVelocity error.",
    //     "localization:\n", localization.DebugString());
    return Status(
        Status::ErrorCode::LOCALIZATION_ERROR,
        "Fail to update because ConstructExceptLinearVelocity error.");
  }
  double measurement_time =
      TimeTool::TimeStruct2S(localization.header().stamp());
  vehicle_state_.set_timestamp(measurement_time);
  vehicle_state_.set_gear(chassis.gear_location());
  // if (chassis.has_speed_mps()) {
  vehicle_state_.set_linear_velocity(chassis.speed_mps());
  if (!FLAGS_reverse_heading_vehicle_state &&
      vehicle_state_.gear() == legionclaw::common::GEAR_REVERSE) {
    vehicle_state_.set_linear_velocity(-vehicle_state_.linear_velocity());
  }
  // }
  vehicle_state_.set_front_steering_value(chassis.front_steering_value());
  vehicle_state_.set_rear_steering_value(chassis.rear_steering_value());

  static constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  Pose pose;
  legionclaw::interface::PointENU position;
  position.set_x(localization.utm_position().x());
  position.set_y(localization.utm_position().y());
  position.set_z(localization.utm_position().z());
  pose.set_position(position);

  math::EulerAnglesZXYd a(localization.roll(),
                          localization.pitch(),
                          localization.heading());
  legionclaw::interface::Quaternion orientation;
  auto q = a.ToQuaternion();
  orientation.set_qw(q.w());
  orientation.set_qx(q.x());
  orientation.set_qy(q.y());
  orientation.set_qz(q.z());
  pose.set_orientation(orientation);

  pose.set_linear_velocity(localization.linear_velocity());

  pose.set_linear_acceleration(localization.linear_acceleration());

  pose.set_angular_velocity(localization.angular_velocity());
  double heading = localization.heading();
  pose.set_heading(heading);

  legionclaw::interface::Point3D euler_angles;
  euler_angles.set_x(localization.heading());
  euler_angles.set_y(localization.pitch());
  euler_angles.set_z(localization.roll());

  vehicle_state_.set_pose(pose);

  vehicle_state_.set_driving_mode(chassis.driving_mode());

  return Status::Ok();
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const Location &localization) {
  // if (!localization.has_pose()) {
  //   AERROR << "Invalid localization input.";
  //   return false;
  // }

  // skip localization update when it is in use_navigation_mode.
  if (FLAGS_use_navigation_mode) {
    ADEBUG << "Skip localization update when it is in use_navigation_mode.";
    return true;
  }

  // vehicle_state_.mutable_pose()->CopyFrom(localization.pose());
  // if (localization.pose().has_position()) {
  //   vehicle_state_.set_x(localization.pose().position().x());
  //   vehicle_state_.set_y(localization.pose().position().y());
  //   vehicle_state_.set_z(localization.pose().position().z());
  // }
  vehicle_state_.set_x(localization.utm_position().x());
  vehicle_state_.set_y(localization.utm_position().y());
  vehicle_state_.set_z(localization.utm_position().z());

  // const auto &orientation = localization.pose().orientation();

  vehicle_state_.set_heading(localization.heading());
  // if (localization.pose().has_heading()) {
  //   vehicle_state_.set_heading(localization.pose().heading());
  // } else {
  //   vehicle_state_.set_heading(
  //       math::QuaternionToHeading(orientation.qw(), orientation.qx(),
  //                                 orientation.qy(), orientation.qz()));
  // }

  // if (FLAGS_enable_map_reference_unify) {
  //   if (!localization.pose().has_angular_velocity_vrf()) {
  //     AERROR << "localization.pose().has_angular_velocity_vrf() must be true
  //     "
  //               "when FLAGS_enable_map_reference_unify is true.";
  //     return false;
  //   }
  //   vehicle_state_.set_angular_velocity(
  //       localization.pose().angular_velocity_vrf().z());

  //   if (!localization.pose().has_linear_acceleration_vrf()) {
  //     AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
  //               "true when FLAGS_enable_map_reference_unify is true.";
  //     return false;
  //   }
  //   vehicle_state_.set_linear_acceleration(
  //       localization.pose().linear_acceleration_vrf().y());
  // } else {
  //   if (!localization.pose().has_angular_velocity()) {
  //     AERROR << "localization.pose() has no angular velocity.";
  //     return false;
  //   }
  //   vehicle_state_.set_angular_velocity(
  //       localization.pose().angular_velocity().z());

  //   if (!localization.pose().has_linear_acceleration()) {
  //     AERROR << "localization.pose() has no linear acceleration.";
  //     return false;
  //   }
  //   vehicle_state_.set_linear_acceleration(
  //       localization.pose().linear_acceleration().y());
  // }
  vehicle_state_.set_angular_velocity(localization.angular_velocity().z());

  // TODO:需要跟定位同时确认
  vehicle_state_.set_linear_acceleration(
      localization.linear_acceleration().y());

  vehicle_state_.set_roll(localization.roll());
  vehicle_state_.set_pitch(localization.pitch());
  vehicle_state_.set_yaw(localization.heading());

  // if (localization.pose().has_euler_angles()) {
  //   vehicle_state_.set_roll(localization.pose().euler_angles().y());
  //   vehicle_state_.set_pitch(localization.pose().euler_angles().x());
  //   vehicle_state_.set_yaw(localization.pose().euler_angles().z());
  // } else {
  //   math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
  //                                     orientation.qy(), orientation.qz());
  //   vehicle_state_.set_roll(euler_angle.roll());
  //   vehicle_state_.set_pitch(euler_angle.pitch());
  //   vehicle_state_.set_yaw(euler_angle.yaw());
  // }

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

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp();
}

const Pose &VehicleStateProvider::pose() const { return vehicle_state_.pose(); }

// const Pose &VehicleStateProvider::original_pose() const {
//   return original_localization_.pose();
// }

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
  const auto &orientation = vehicle_state_.pose().orientation();
  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                       orientation.qy(), orientation.qz());
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());
  const Eigen::Vector3d future_pos_3d =
      quaternion.toRotationMatrix() * vec_distance + pos_vec;
  return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
}

math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  if ((FLAGS_state_transform_to_com_reverse &&
       vehicle_state_.gear() == legionclaw::common::GEAR_REVERSE) ||
      (FLAGS_state_transform_to_com_drive &&
       vehicle_state_.gear() == legionclaw::common::GEAR_DRIVE)) {
    v << 0.0, rear_to_com_distance, 0.0;
  } else {
    v << 0.0, 0.0, 0.0;
  }
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.

  const auto &orientation = vehicle_state_.pose().orientation();
  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                       orientation.qy(), orientation.qz());
  // Update the COM position with rotation
  com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;

  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace common
}  // namespace legionclaw
