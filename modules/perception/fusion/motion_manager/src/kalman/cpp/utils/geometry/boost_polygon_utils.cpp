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

#include "utils/geometry/boost_polygon_utils.hpp"

// #include "utils/geometry/geometry.hpp"

#include <boost/geometry/geometry.hpp>

#include "common/tf2/include/geometry_msgs/transform_stamped.h"
#include "common/enum/enum.h"

// #include <tf2/utils.h>



namespace
{
namespace bg = boost::geometry;
using utils::Point2d;
using utils::Polygon2d;

void appendPointToPolygon(Polygon2d & polygon, const tf2_geometry_msgs::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

void appendPointToPolygon(Polygon2d & polygon, const Point2d & point)
{
  bg::append(polygon.outer(), point);
}

/*
 * NOTE: Area is negative when footprint.points is clock wise.
 *       Area is positive when footprint.points is anti clock wise.
 */
double getPolygonArea(const tf2_geometry_msgs::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double getRectangleArea(const tf2_geometry_msgs::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double getCircleArea(const tf2_geometry_msgs::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}
}  // namespace

namespace utils
{
bool isClockwise(const Polygon2d & polygon)
{
  const int n = polygon.outer().size();
  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }

  return sum < 0.0;
}

Polygon2d inverseClockwise(const Polygon2d & polygon)
{
  auto output_polygon = polygon;
  boost::geometry::reverse(output_polygon);
  return output_polygon;
}

tf2_geometry_msgs::Polygon rotatePolygon(
  const tf2_geometry_msgs::Polygon & polygon, const double & angle)
{
  const double cos = std::cos(angle);
  const double sin = std::sin(angle);
  tf2_geometry_msgs::Polygon rotated_polygon;
  for (const auto & point : polygon.points) {
    auto rotated_point = point;
    rotated_point.x = cos * point.x - sin * point.y;
    rotated_point.y = sin * point.x + cos * point.y;
    rotated_polygon.points.push_back(rotated_point);
  }
  return rotated_polygon;
}

Polygon2d rotatePolygon(const Polygon2d & polygon, const double angle)
{
  Polygon2d rotated_polygon;
  const boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::radian, double, 2, 2>
    rotation(-angle);
  boost::geometry::transform(polygon, rotated_polygon, rotation);
  return rotated_polygon;
}

Polygon2d toPolygon2d(
  const tf2_geometry_msgs::Pose & pose, const motion_manager::interface::Shape & shape)
{
  Polygon2d polygon;

  if (shape.type() == motion_manager::common::ShapeType::S_BOUNDING_BOX) {
    const auto point0 = utils::calcOffsetPose(
                          pose, shape.dimensions().x() / 2.0, shape.dimensions().y() / 2.0, 0.0)
                          .position;
    const auto point1 = utils::calcOffsetPose(
                          pose, -shape.dimensions().x() / 2.0, shape.dimensions().y() / 2.0, 0.0)
                          .position;
    const auto point2 = utils::calcOffsetPose(
                          pose, -shape.dimensions().x() / 2.0, -shape.dimensions().y() / 2.0, 0.0)
                          .position;
    const auto point3 = utils::calcOffsetPose(
                          pose, shape.dimensions().x() / 2.0, -shape.dimensions().y() / 2.0, 0.0)
                          .position;

    appendPointToPolygon(polygon, point0);
    appendPointToPolygon(polygon, point1);
    appendPointToPolygon(polygon, point2);
    appendPointToPolygon(polygon, point3);
  } else if (shape.type() == motion_manager::common::ShapeType::S_CYLINDER) {
    const double radius = shape.dimensions().x() / 2.0;
    constexpr int circle_discrete_num = 6;
    for (int i = 0; i < circle_discrete_num; ++i) {
      tf2_geometry_msgs::Point point;
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.x;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(circle_discrete_num)) *
                  radius +
                pose.position.y;
      appendPointToPolygon(polygon, point);
    }
  } else if (shape.type() == motion_manager::common::ShapeType::S_POLYGON) {
    const double poly_yaw = tf2::getYaw(pose.orientation);
    // TODO dcw: motion_manager::interface::Polygon trans to Polygon2d                     Done
    Polygon2d poly_tmp;
    for (auto p:shape.footprint().points())
    {
      boost::geometry::append(poly_tmp.outer(),bg::model::point<double, 2, bg::cs::cartesian>(p.x(),p.y()));
    }
    
    const auto rotated_footprint = rotatePolygon(poly_tmp, poly_yaw);
    // tf2_geometry_msgs::Polygon rotated_footprint;
    for (const auto rel_point : rotated_footprint.outer()) {
      tf2_geometry_msgs::Point abs_point;
      abs_point.x = pose.position.x + bg::get<0>(rel_point);
      abs_point.y = pose.position.y + bg::get<1>(rel_point);

      appendPointToPolygon(polygon, abs_point);
    }
  } else {
    throw std::logic_error("The shape type is not supported in utils.");
  }

  // NOTE: push back the first point in order to close polygon
  if (!polygon.outer().empty()) {
    appendPointToPolygon(polygon, polygon.outer().front());
  }

  return isClockwise(polygon) ? polygon : inverseClockwise(polygon);
}


// TODO dcw: motion_manager::interface::Pose trans to tf2_geometry_msgs::Pose    Done
utils::Polygon2d toPolygon2d(
  const motion_manager::interface::DetectedObject & object)
{
  // Polygon2d polygon;
  // return polygon;
  tf2_geometry_msgs::Pose pose;
  pose.position.x = object.kinematics().pose_with_covariance().pose().position().x();
  pose.position.y = object.kinematics().pose_with_covariance().pose().position().y();
  pose.position.z = object.kinematics().pose_with_covariance().pose().position().z();
  pose.orientation.x = object.kinematics().pose_with_covariance().pose().orientation().qx();
  pose.orientation.y = object.kinematics().pose_with_covariance().pose().orientation().qy();
  pose.orientation.z = object.kinematics().pose_with_covariance().pose().orientation().qz();
  pose.orientation.w = object.kinematics().pose_with_covariance().pose().orientation().qw();
  return utils::toPolygon2d(
    pose, object.shape());
}

utils::Polygon2d toPolygon2d(
  const motion_manager::interface::TrackedObject & object)
{
 // Polygon2d polygon;
  // return polygon;
  tf2_geometry_msgs::Pose pose;
  pose.position.x = object.kinematics().pose_with_covariance().pose().position().x();
  pose.position.y = object.kinematics().pose_with_covariance().pose().position().y();
  pose.position.z = object.kinematics().pose_with_covariance().pose().position().z();
  pose.orientation.x = object.kinematics().pose_with_covariance().pose().orientation().qx();
  pose.orientation.y = object.kinematics().pose_with_covariance().pose().orientation().qy();
  pose.orientation.z = object.kinematics().pose_with_covariance().pose().orientation().qz();
  pose.orientation.w = object.kinematics().pose_with_covariance().pose().orientation().qw();
  return utils::toPolygon2d(
    pose, object.shape());
}

utils::Polygon2d toPolygon2d(
  const motion_manager::interface::PredictedObject & object)
{
  // Polygon2d polygon;
  // return polygon;
  tf2_geometry_msgs::Pose pose;
  pose.position.x = object.kinematics().initial_pose_with_covariance().pose().position().x();
  pose.position.y = object.kinematics().initial_pose_with_covariance().pose().position().y();
  pose.position.z = object.kinematics().initial_pose_with_covariance().pose().position().z();
  pose.orientation.x = object.kinematics().initial_pose_with_covariance().pose().orientation().qx();
  pose.orientation.y = object.kinematics().initial_pose_with_covariance().pose().orientation().qy();
  pose.orientation.z = object.kinematics().initial_pose_with_covariance().pose().orientation().qz();
  pose.orientation.w = object.kinematics().initial_pose_with_covariance().pose().orientation().qw();
  return utils::toPolygon2d(
    pose, object.shape());
}

utils::Polygon2d toFootprint(
  const tf2_geometry_msgs::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width)
{
  Polygon2d polygon;
  const auto point0 =
    utils::calcOffsetPose(base_link_pose, base_to_front, width / 2.0, 0.0).position;
  const auto point1 =
    utils::calcOffsetPose(base_link_pose, base_to_front, -width / 2.0, 0.0).position;
  const auto point2 =
    utils::calcOffsetPose(base_link_pose, -base_to_rear, -width / 2.0, 0.0).position;
  const auto point3 =
    utils::calcOffsetPose(base_link_pose, -base_to_rear, width / 2.0, 0.0).position;

  appendPointToPolygon(polygon, point0);
  appendPointToPolygon(polygon, point1);
  appendPointToPolygon(polygon, point2);
  appendPointToPolygon(polygon, point3);
  appendPointToPolygon(polygon, point0);

  return isClockwise(polygon) ? polygon : inverseClockwise(polygon);
}

double getArea(const motion_manager::interface::Shape & shape)
{
  // TODO dcw : point3d --> vector3d
  if (shape.type() == motion_manager::common::ShapeType::S_BOUNDING_BOX) {
    tf2_geometry_msgs::Vector3 vector3;
    vector3.x = shape.dimensions().x();
    vector3.y = shape.dimensions().y();
    vector3.z = shape.dimensions().z();
    return getRectangleArea(vector3);
  } else if (shape.type() == motion_manager::common::ShapeType::S_CYLINDER) {
    tf2_geometry_msgs::Vector3 vector3;
    vector3.x = shape.dimensions().x();
    vector3.y = shape.dimensions().y();
    vector3.z = shape.dimensions().z();
    return getCircleArea(vector3);
  } else if (shape.type() == motion_manager::common::ShapeType::S_POLYGON) {
    // TODO dcw : motion_manager::interface::Polygon --> tf2_geometry_msgs::Polygon      done
    tf2_geometry_msgs::Polygon poly_tmp;
    for (auto p:shape.footprint().points())
    {
      // boost::geometry::append(poly_tmp.outer(),bg::model::point<double, 2, bg::cs::cartesian>(p.x(),p.y()),p.z());
      tf2_geometry_msgs::Point32 this_p;
      this_p.x = p.x();
      this_p.y = p.y();
      this_p.z = p.z();
      poly_tmp.points.push_back(this_p);
    }
    return getPolygonArea(poly_tmp);
    // return 0.0;
  }

  throw std::logic_error("The shape type is not supported in utils.");
}

// NOTE: The number of vertices on the expanded polygon by boost::geometry::buffer
//       is larger than the original one.
//       This function fixes the issue.
Polygon2d expandPolygon(const Polygon2d & input_polygon, const double offset)
{
  // NOTE: input_polygon is supposed to have a duplicated point.
  const size_t num_points = input_polygon.outer().size() - 1;

  Polygon2d expanded_polygon;
  for (size_t i = 0; i < num_points; ++i) {
    const auto & curr_p = input_polygon.outer().at(i);
    const auto & next_p = input_polygon.outer().at(i + 1);
    const auto & prev_p =
      i == 0 ? input_polygon.outer().at(num_points - 1) : input_polygon.outer().at(i - 1);

    Eigen::Vector2d current_to_next(next_p.x() - curr_p.x(), next_p.y() - curr_p.y());
    Eigen::Vector2d current_to_prev(prev_p.x() - curr_p.x(), prev_p.y() - curr_p.y());
    current_to_next.normalize();
    current_to_prev.normalize();

    const Eigen::Vector2d offset_vector = (-current_to_next - current_to_prev).normalized();
    const double theta = std::acos(offset_vector.dot(current_to_next));
    const double scaled_offset = offset / std::sin(theta);
    const Eigen::Vector2d offset_point =
      Eigen::Vector2d(curr_p.x(), curr_p.y()) + offset_vector * scaled_offset;

    expanded_polygon.outer().push_back(Point2d(offset_point.x(), offset_point.y()));
  }

  boost::geometry::correct(expanded_polygon);
  return expanded_polygon;
}
}  // namespace utils
