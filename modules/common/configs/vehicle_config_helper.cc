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

#include "modules/common/configs/vehicle_config_helper.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/logging/logging.h"

namespace legionclaw {
namespace common {

legionclaw::interface::VehicleConfig VehicleConfigHelper::vehicle_config_;
json VehicleConfigHelper::vehicle_param_json_;
bool VehicleConfigHelper::is_init_ = false;

VehicleConfigHelper::VehicleConfigHelper() {}

void VehicleConfigHelper::Init() { Init(FLAGS_vehicle_config_path); }

void VehicleConfigHelper::Init(const std::string &config_file) {
  std::ifstream in(config_file);
  AINFO << "vehicle_param_file_path: " << std::endl;
  in >> vehicle_param_json_;

  if (vehicle_param_json_.is_null()) {
    AERROR << "vehicle_param_json_ is null";
    return;
  }

  legionclaw::interface::VehicleConfig vehicle_conf;
  legionclaw::interface::VehicleParam vehicle_param;

  vehicle_param.set_brand(vehicle_param_json_["brand"]);
  vehicle_param.set_steer_mode(vehicle_param_json_["steer_mode"]);
  vehicle_param.set_length(vehicle_param_json_["length"]);
  vehicle_param.set_width(vehicle_param_json_["width"]);
  vehicle_param.set_height(vehicle_param_json_["height"]);
  vehicle_param.set_mass_fl(vehicle_param_json_["mass_fl"]);
  vehicle_param.set_mass_fr(vehicle_param_json_["mass_fr"]);
  vehicle_param.set_mass_rl(vehicle_param_json_["mass_rl"]);
  vehicle_param.set_mass_rr(vehicle_param_json_["mass_rr"]);
  vehicle_param.set_min_turning_radius(
      vehicle_param_json_["min_turning_radius"]);
  vehicle_param.set_wheel_radius(vehicle_param_json_["wheel_radius"]);
  vehicle_param.set_wheelbase(vehicle_param_json_["wheelbase"]);
  vehicle_param.set_front_edge_to_center(
      vehicle_param_json_["front_edge_to_center"]);
  vehicle_param.set_back_edge_to_center(
      vehicle_param_json_["back_edge_to_center"]);
  vehicle_param.set_lf(vehicle_param_json_["lf"]);
  vehicle_param.set_lr(vehicle_param_json_["lr"]);
  vehicle_param.set_cf(vehicle_param_json_["cf"]);
  vehicle_param.set_cr(vehicle_param_json_["cr"]);
  vehicle_param.set_steer_ratio(vehicle_param_json_["steer_ratio"]);
  vehicle_param.set_rolling_coefficient(
      vehicle_param_json_["rolling_coefficient"]);
  vehicle_param.set_air_density(vehicle_param_json_["air_density"]);
  vehicle_param.set_air_damping_coefficient(
      vehicle_param_json_["air_damping_coefficient"]);

  vehicle_param.set_max_front_steer_angle(vehicle_param_json_["max_front_steer_angle"]);
  vehicle_param.set_min_front_steer_angle(vehicle_param_json_["min_front_steer_angle"]);
  vehicle_param.set_max_rear_steer_angle(vehicle_param_json_["max_rear_steer_angle"]);
  vehicle_param.set_min_rear_steer_angle(vehicle_param_json_["min_rear_steer_angle"]);
  vehicle_param.set_speed_limit(vehicle_param_json_["speed_limit"]);
  vehicle_param.set_max_brake_value(vehicle_param_json_["max_brake_value"]);
  vehicle_param.set_min_brake_value(vehicle_param_json_["min_brake_value"]);
  vehicle_param.set_max_accel_value(vehicle_param_json_["max_accel_value"]);
  vehicle_param.set_min_accel_value(vehicle_param_json_["min_accel_value"]);
  vehicle_param.set_speed_deadzone(vehicle_param_json_["speed_deadzone"]);
  vehicle_param.set_accel_deadzone(vehicle_param_json_["accel_deadzone"]);
  vehicle_param.set_brake_deadzone(vehicle_param_json_["brake_deadzone"]);
  vehicle_param.set_acceleration_in_idle(
      vehicle_param_json_["acceleration_in_idle"]);
  vehicle_param.set_deceleration_in_idle(
      vehicle_param_json_["deceleration_in_idle"]);
  vehicle_param.set_max_acceleration_jerk(
      vehicle_param_json_["max_acceleration_jerk"]);
  vehicle_param.set_standstill_acceleration(
      vehicle_param_json_["standstill_acceleration"]);
  vehicle_param.set_max_front_steer_angle_rate(
      vehicle_param_json_["max_front_steer_angle_rate"]);
  vehicle_param.set_max_rear_steer_angle_rate(
      vehicle_param_json_["max_rear_steer_angle_rate"]);
  vehicle_param.set_max_abs_speed_when_stopped(
      vehicle_param_json_["max_abs_speed_when_stopped"]);
  vehicle_param.set_max_abs_speed_when_stopped_duration(
      vehicle_param_json_["max_abs_speed_when_stopped_duration"]);
  vehicle_param.set_brake_value_when_gear_transitioning(
      vehicle_param_json_["brake_value_when_gear_transitioning"]);
  vehicle_param.set_max_acceleration(vehicle_param_json_["max_acceleration"]);
  vehicle_param.set_max_deceleration(vehicle_param_json_["max_deceleration"]);

  vehicle_conf.set_vehicle_param(vehicle_param);
  Init(vehicle_conf);
}

void VehicleConfigHelper::Init(
    const legionclaw::interface::VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const legionclaw::interface::VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

double VehicleConfigHelper::MinSafeTurnRadius() {
  const auto &param = vehicle_config_.vehicle_param();
  double lat_edge_to_center = std::max(param.width() / 2, param.width() / 2);
  double lon_edge_to_center =
      std::max(param.front_edge_to_center(), param.back_edge_to_center());
  return std::sqrt((lat_edge_to_center + param.min_turning_radius()) *
                       (lat_edge_to_center + param.min_turning_radius()) +
                   lon_edge_to_center * lon_edge_to_center);
}

common::math::Box2d VehicleConfigHelper::GetBoundingBox(
    const legionclaw::interface::PathPoint &path_point) {
  const auto &vehicle_param = vehicle_config_.vehicle_param();
  double diff_truecenter_and_pointX = (vehicle_param.front_edge_to_center() -
                                       vehicle_param.back_edge_to_center()) /
                                      2.0;
  common::math::Vec2d true_center(
      path_point.x() +
          diff_truecenter_and_pointX * std::cos(path_point.theta()),
      path_point.y() +
          diff_truecenter_and_pointX * std::sin(path_point.theta()));

  return common::math::Box2d(true_center, path_point.theta(),
                             vehicle_param.length(), vehicle_param.width());
}

common::math::Box2d VehicleConfigHelper::GetSafeBoundingBox(
    const legionclaw::interface::PathPoint &path_point, double lon, double lat) {
  const auto &vehicle_param = vehicle_config_.vehicle_param();
  double diff_truecenter_and_pointX = (vehicle_param.front_edge_to_center() -
                                       vehicle_param.back_edge_to_center()) /
                                      2.0;
  common::math::Vec2d true_center(
      path_point.x() +
          diff_truecenter_and_pointX * std::cos(path_point.theta()),
      path_point.y() +
          diff_truecenter_and_pointX * std::sin(path_point.theta()));

  return common::math::Box2d(true_center, path_point.theta(),
                             vehicle_param.length() + lon,
                             vehicle_param.width() + lat);
}

common::math::Box2d VehicleConfigHelper::GetDirSafeBoundingBox(
    const legionclaw::interface::PathPoint &path_point, double lon, double lat,double lat_err) {
  const auto &vehicle_param = vehicle_config_.vehicle_param();
  double diff_truecenter_and_pointX = (vehicle_param.front_edge_to_center() -
                                       vehicle_param.back_edge_to_center()) /
                                      2.0;
  double theta = path_point.theta();
  common::math::Vec2d true_center(
      path_point.x() + diff_truecenter_and_pointX * std::cos(theta),
      path_point.y() + diff_truecenter_and_pointX * std::sin(theta));
  theta = lat_err > 0.0 ? theta - M_PI_2:theta + M_PI_2;
  true_center = common::math::Vec2d(path_point.x() + 0.5*lat_err * std::cos(theta),
                                  path_point.y() + 0.5*lat_err * std::sin(theta));

  return common::math::Box2d(true_center, path_point.theta(),
                             vehicle_param.length() + lon,
                             vehicle_param.width() + lat + std::fabs(lat_err * 2));
}

}  // namespace common
}  // namespace legionclaw
