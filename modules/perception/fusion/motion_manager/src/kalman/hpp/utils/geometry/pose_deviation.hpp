// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_

// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Pose.h>
#include "common/tf2/include/geometry_msgs/transform_stamped.h"

namespace utils
{
struct PoseDeviation
{
  double lateral{0.0};
  double longitudinal{0.0};
  double yaw{0.0};
};

double calcLateralDeviation(
  const tf2_geometry_msgs::Pose & base_pose, const tf2_geometry_msgs::Point & target_point);

double calcLongitudinalDeviation(
  const tf2_geometry_msgs::Pose & base_pose, const tf2_geometry_msgs::Point & target_point);

double calcYawDeviation(
  const tf2_geometry_msgs::Pose & base_pose, const tf2_geometry_msgs::Pose & target_pose);

PoseDeviation calcPoseDeviation(
  const tf2_geometry_msgs::Pose & base_pose, const tf2_geometry_msgs::Pose & target_pose);

}  // namespace utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
