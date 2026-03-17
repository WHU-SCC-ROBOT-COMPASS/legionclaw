// Copyright 2022 Tier IV, Inc.
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

#ifndef INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
#define INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_

#include "interpolation/interpolation_utils.hpp"

#include "common/interface/quaternion.hpp"
#include "common/tf2/include/geometry_msgs/transform_stamped.h"
// #include "common/tf2/include/geometry_msgs/transform_datatypes.h"
#include "common/tf2/include/tf2/utils.h"
#include "common/tf2/include/tf2/impl/utils.h"
// #include <geometry_msgs/Quaternion.h>

// #include <tf2/utils.h>

// #ifdef ROS_DISTRO_GALACTIC
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif

#include <vector>

namespace interpolation
{
tf2_geometry_msgs::Quaternion slerp(
  const tf2_geometry_msgs::Quaternion & src_quat, const tf2_geometry_msgs::Quaternion & dst_quat,
  const double ratio);

std::vector<tf2_geometry_msgs::Quaternion> slerp(
  const std::vector<double> & base_keys,
  const std::vector<tf2_geometry_msgs::Quaternion> & base_values,
  const std::vector<double> & query_keys);

tf2_geometry_msgs::Quaternion lerpOrientation(
  const tf2_geometry_msgs::Quaternion & o_from, const tf2_geometry_msgs::Quaternion & o_to,
  const double ratio);
}  // namespace interpolation

#endif  // INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
