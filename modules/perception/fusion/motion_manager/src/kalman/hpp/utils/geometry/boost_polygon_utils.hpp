// Copyright 2022 TIER IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_

#include "utils/geometry/geometry.hpp"
#include "utils/geometry/boost_geometry.hpp"

#include "common/interface/detected_object.hpp"
#include "common/interface/predicted_object.hpp"
#include "common/interface/tracked_object.hpp"
// #include "common/interface/pose.hpp"
#include "modules/common/tf2/include/geometry_msgs/transform_stamped.h"

#include <vector>

namespace utils
{
bool isClockwise(const Polygon2d & polygon);
Polygon2d inverseClockwise(const Polygon2d & polygon);
tf2_geometry_msgs::Polygon rotatePolygon(
  const tf2_geometry_msgs::Polygon & polygon, const double & angle);
/// @brief rotate a polygon by some angle around the origin
/// @param[in] polygon input polygon
/// @param[in] angle angle of rotation [rad]
/// @return rotated polygon
Polygon2d rotatePolygon(const Polygon2d & polygon, const double angle);
Polygon2d toPolygon2d(
  const tf2_geometry_msgs::Pose & pose, const motion_manager::interface::Shape & shape);
Polygon2d toPolygon2d(const motion_manager::interface::DetectedObject & object);
Polygon2d toPolygon2d(const motion_manager::interface::TrackedObject & object);
Polygon2d toPolygon2d(const motion_manager::interface::PredictedObject & object);
Polygon2d toFootprint(
  const tf2_geometry_msgs::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width);
double getArea(const motion_manager::interface::Shape & shape);
Polygon2d expandPolygon(const Polygon2d & input_polygon, const double offset);
}  // namespace utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
