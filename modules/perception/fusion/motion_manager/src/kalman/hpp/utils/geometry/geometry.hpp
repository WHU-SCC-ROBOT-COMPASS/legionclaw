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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_

#include "utils/geometry/boost_geometry.hpp"
#include "utils/math/constants.hpp"
#include "utils/math/normalization.hpp"
// #include "tier4_autoware_utils/ros/msg_covariance.hpp"

#include <exception>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include "common/interface/planning_path.hpp"
#include "common/interface/path_point_with_lane_id.hpp"
#include "common/interface/planning_trajectory.hpp"
#include "common/tf2/include/geometry_msgs/transform_stamped.h"
// #include "common/tf2/include/geometry_msgs/transform_datatypes.h"
#include "common/tf2/include/tf2/utils.h"

// #include <geometry_msgs/Point32.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TwistWithCovariance.h>
// #include <geometry_msgs/Vector3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <tf2/utils.h>

namespace utils
{
template <class T>
tf2_geometry_msgs::Point getPoint(const T & p)
{
  tf2_geometry_msgs::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
  return point;
}

template <>
inline tf2_geometry_msgs::Point getPoint(const tf2_geometry_msgs::Point & p)
{
  return p;
}

template <>
inline tf2_geometry_msgs::Point getPoint(const tf2_geometry_msgs::Pose & p)
{
  return p.position;
}

// template <>
// inline tf2_geometry_msgs::Point getPoint(const tf2_geometry_msgs::PoseStamped & p)
// {
//   return p.pose.position;
// }

template <>
inline tf2_geometry_msgs::Point getPoint(const tf2_geometry_msgs::PoseWithCovarianceStamped & p)
{
  return p.pose.pose.position;
}

template <>
inline tf2_geometry_msgs::Point getPoint(const motion_manager::interface::GeometryPose & p)
{
    tf2_geometry_msgs::Point point;
    point.x = p.position().x();
    point.y = p.position().y();
    point.z = p.position().z();
  return point;
}

template <>
inline tf2_geometry_msgs::Point getPoint(const motion_manager::interface::PlanningPathPoint & p)
{
    tf2_geometry_msgs::Point point;
    point.x = p.pose().position().x();
    point.y = p.pose().position().y();
    point.z = p.pose().position().z();
  return point;
}

template <>
inline tf2_geometry_msgs::Point getPoint(
  const motion_manager::interface::PathPointWithLaneId & p)
{
    tf2_geometry_msgs::Point point;
    point.x = p.point().pose().position().x();
    point.y = p.point().pose().position().y();
    point.z = p.point().pose().position().z();
  return point;
}

template <>
inline tf2_geometry_msgs::Point getPoint(
  const motion_manager::interface::PlanningTrajectoryPoint & p)
{
    tf2_geometry_msgs::Point point;
    point.x = p.pose().position().x();
    point.y = p.pose().position().y();
    point.z = p.pose().position().z();
  return point;
}

// template <class T>
// tf2_geometry_msgs::Pose getPose([[maybe_unused]] const T & p)
// {
//   static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
//   throw std::logic_error("Only specializations of getPose can be used.");
// }

template <class T>
tf2_geometry_msgs::Pose getPose(const T & p)
{
  tf2_geometry_msgs::Pose pose;
    pose.position.x = p.position().x();
    pose.position.y = p.position().y();
    pose.position.z = p.position().z();
    pose.orientation.x = p.orientation().qx();
    pose.orientation.y = p.orientation().qy();
    pose.orientation.z = p.orientation().qz();
    pose.orientation.w = p.orientation().qw();
  return pose;
}

template <>
inline tf2_geometry_msgs::Pose getPose(const tf2_geometry_msgs::Pose & p)
{
  return p;
}

// template <>
// inline tf2_geometry_msgs::Pose getPose(const tf2_geometry_msgs::PoseStamped & p)
// {
//   return p.pose;
// }

template <>
inline tf2_geometry_msgs::Pose getPose(const motion_manager::interface::PlanningPathPoint & p)
{
    tf2_geometry_msgs::Pose pose;
    pose.position.x = p.pose().position().x();
    pose.position.y = p.pose().position().y();
    pose.position.z = p.pose().position().z();
    pose.orientation.x = p.pose().orientation().qx();
    pose.orientation.y = p.pose().orientation().qy();
    pose.orientation.z = p.pose().orientation().qz();
    pose.orientation.w = p.pose().orientation().qz();
  return pose;
}

template <>
inline tf2_geometry_msgs::Pose getPose(
  const motion_manager::interface::PathPointWithLaneId & p)
{
    tf2_geometry_msgs::Pose pose;
    pose.position.x = p.point().pose().position().x();
    pose.position.y = p.point().pose().position().y();
    pose.position.z = p.point().pose().position().z();
    pose.orientation.x = p.point().pose().orientation().qx();
    pose.orientation.y = p.point().pose().orientation().qy();
    pose.orientation.z = p.point().pose().orientation().qz();
    pose.orientation.w = p.point().pose().orientation().qz();
  return pose;
}

template <>
inline tf2_geometry_msgs::Pose getPose(const motion_manager::interface::PlanningTrajectoryPoint & p)
{
    tf2_geometry_msgs::Pose pose;
    pose.position.x = p.pose().position().x();
    pose.position.y = p.pose().position().y();
    pose.position.z = p.pose().position().z();
    pose.orientation.x = p.pose().orientation().qx();
    pose.orientation.y = p.pose().orientation().qy();
    pose.orientation.z = p.pose().orientation().qz();
    pose.orientation.w = p.pose().orientation().qz();
  return pose;
}

template <class T>
double getLongitudinalVelocity([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getVelocity can be used.");
  throw std::logic_error("Only specializations of getVelocity can be used.");
}

template <>
inline double getLongitudinalVelocity(const motion_manager::interface::PlanningPathPoint & p)
{
  return p.longitudinal_velocity_mps();
}

template <>
inline double getLongitudinalVelocity(
  const motion_manager::interface::PathPointWithLaneId & p)
{
  return p.point().longitudinal_velocity_mps();
}

template <>
inline double getLongitudinalVelocity(const motion_manager::interface::PlanningTrajectoryPoint & p)
{
  return p.longitudinal_velocity_mps();
}

template <class T>
void setPose([[maybe_unused]] const tf2_geometry_msgs::Pose & pose, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

template <>
inline void setPose(const tf2_geometry_msgs::Pose & pose, tf2_geometry_msgs::Pose & p)
{
  p = pose;
}

// template <>
// inline void setPose(const tf2_geometry_msgs::Pose & pose, tf2_geometry_msgs::PoseStamped & p)
// {
//   p.pose = pose;
// }

template <>
inline void setPose(
  const tf2_geometry_msgs::Pose & pose, motion_manager::interface::PlanningPathPoint & p)
{
    p.mutable_pose()->mutable_position()->set_x(pose.position.x);
    p.mutable_pose()->mutable_position()->set_y(pose.position.y);
    p.mutable_pose()->mutable_position()->set_z(pose.position.z);
    p.mutable_pose()->mutable_orientation()->set_qx(pose.orientation.x);
    p.mutable_pose()->mutable_orientation()->set_qy(pose.orientation.y);
    p.mutable_pose()->mutable_orientation()->set_qz(pose.orientation.z);
    p.mutable_pose()->mutable_orientation()->set_qw(pose.orientation.w);
}

template <>
inline void setPose(
  const tf2_geometry_msgs::Pose & pose, motion_manager::interface::PathPointWithLaneId & p)
{
    p.mutable_point()->mutable_pose()->mutable_position()->set_x(pose.position.x);
    p.mutable_point()->mutable_pose()->mutable_position()->set_y(pose.position.y);
    p.mutable_point()->mutable_pose()->mutable_position()->set_z(pose.position.z);
    p.mutable_point()->mutable_pose()->mutable_orientation()->set_qx(pose.orientation.x);
    p.mutable_point()->mutable_pose()->mutable_orientation()->set_qy(pose.orientation.y);
    p.mutable_point()->mutable_pose()->mutable_orientation()->set_qz(pose.orientation.z);
    p.mutable_point()->mutable_pose()->mutable_orientation()->set_qw(pose.orientation.w);  
}

template <>
inline void setPose(
  const tf2_geometry_msgs::Pose & pose, motion_manager::interface::PlanningTrajectoryPoint & p)
{
    p.mutable_pose()->mutable_position()->set_x(pose.position.x);
    p.mutable_pose()->mutable_position()->set_y(pose.position.y);
    p.mutable_pose()->mutable_position()->set_z(pose.position.z);
    p.mutable_pose()->mutable_orientation()->set_qx(pose.orientation.x);
    p.mutable_pose()->mutable_orientation()->set_qy(pose.orientation.y);
    p.mutable_pose()->mutable_orientation()->set_qz(pose.orientation.z);
    p.mutable_pose()->mutable_orientation()->set_qw(pose.orientation.w);
}

template <class T>
inline void setOrientation(const tf2_geometry_msgs::Quaternion & orientation, T & p)
{
  auto pose = getPose(p);
  pose.orientation = orientation;
  setPose(pose, p);
}

template <class T>
void setLongitudinalVelocity([[maybe_unused]] const float velocity, [[maybe_unused]] T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getLongitudinalVelocity can be used.");
  throw std::logic_error("Only specializations of getLongitudinalVelocity can be used.");
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, motion_manager::interface::PlanningTrajectoryPoint & p)
{
  p.set_longitudinal_velocity_mps(velocity);
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, motion_manager::interface::PlanningPathPoint & p)
{
  p.set_longitudinal_velocity_mps(velocity);
}

template <>
inline void setLongitudinalVelocity(
  const float velocity, motion_manager::interface::PathPointWithLaneId & p)
{
  p.mutable_point()->set_longitudinal_velocity_mps(velocity);
}

inline tf2_geometry_msgs::Point createPoint(const double x, const double y, const double z)
{
  tf2_geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

tf2_geometry_msgs::Vector3 getRPY(const tf2_geometry_msgs::Quaternion & quat);

tf2_geometry_msgs::Vector3 getRPY(const tf2_geometry_msgs::Pose & pose);

// tf2_geometry_msgs::Vector3 getRPY(const tf2_geometry_msgs::PoseStamped & pose);

tf2_geometry_msgs::Vector3 getRPY(const tf2_geometry_msgs::PoseWithCovarianceStamped & pose);

tf2_geometry_msgs::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w);

tf2_geometry_msgs::Vector3 createTranslation(const double x, const double y, const double z);

// Revival of tf::createQuaternionFromRPY
// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
// tf2_geometry_msgs::Quaternion createQuaternionFromRPY(
//   const double roll, const double pitch, const double yaw);

tf2_geometry_msgs::Quaternion createQuaternionFromYaw(const double yaw);

template <class Point1, class Point2>
double calcDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class Point1, class Point2>
double calcSquaredDistance2d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;
  return dx * dx + dy * dy;
}

template <class Point1, class Point2>
double calcDistance3d(const Point1 & point1, const Point2 & point2)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  // To be replaced by std::hypot(dx, dy, dz) in C++17
  return std::hypot(std::hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

/**
 * @brief calculate elevation angle of two points.
 * @details This function returns the elevation angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If the two points are in the same position, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi/2 <= elevation angle <= pi/2
 */
double calcElevationAngle(
  const tf2_geometry_msgs::Point & p_from, const tf2_geometry_msgs::Point & p_to);

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
double calcAzimuthAngle(
  const tf2_geometry_msgs::Point & p_from, const tf2_geometry_msgs::Point & p_to);

tf2_geometry_msgs::Pose transform2pose(const tf2_geometry_msgs::Transform & transform);

// tf2_geometry_msgs::PoseStamped transform2pose(
//   const tf2_geometry_msgs::TransformStamped & transform);

tf2_geometry_msgs::Transform pose2transform(const tf2_geometry_msgs::Pose & pose);

// tf2_geometry_msgs::TransformStamped pose2transform(
//   const tf2_geometry_msgs::PoseStamped & pose, const std::string & child_frame_id);

template <class Point1, class Point2>
tf2::Vector3 point2tfVector(const Point1 & src, const Point2 & dst)
{
  const auto src_p = getPoint(src);
  const auto dst_p = getPoint(dst);

  double dx = dst_p.x - src_p.x;
  double dy = dst_p.y - src_p.y;
  double dz = dst_p.z - src_p.z;
  return tf2::Vector3(dx, dy, dz);
}

Point3d transformPoint(const Point3d & point, const tf2_geometry_msgs::Transform & transform);

Point2d transformPoint(const Point2d & point, const tf2_geometry_msgs::Transform & transform);

Eigen::Vector3d transformPoint(
  const Eigen::Vector3d & point, const tf2_geometry_msgs::Pose & pose);

tf2_geometry_msgs::Point transformPoint(
  const tf2_geometry_msgs::Point & point, const tf2_geometry_msgs::Pose & pose);

// tf2_geometry_msgs::Point32 transformPoint(
//   const tf2_geometry_msgs::Point32 & point32, const tf2_geometry_msgs::Pose & pose);

template <class T>
T transformVector(const T & points, const tf2_geometry_msgs::Transform & transform)
{
  T transformed;
  for (const auto & point : points) {
    transformed.push_back(transformPoint(point, transform));
  }
  return transformed;
}

tf2_geometry_msgs::Pose transformPose(
  const tf2_geometry_msgs::Pose & pose, const tf2_geometry_msgs::TransformStamped & transform);

tf2_geometry_msgs::Pose transformPose(
  const tf2_geometry_msgs::Pose & pose, tf2_geometry_msgs::Transform & transform);

tf2_geometry_msgs::Pose transformPose(
  const tf2_geometry_msgs::Pose & pose, const tf2_geometry_msgs::Pose & pose_transform);

// Transform pose in world coordinates to local coordinates
/*
tf2_geometry_msgs::Pose inverseTransformPose(
  const tf2_geometry_msgs::Pose & pose, const tf2_geometry_msgs::TransformStamped & transform);
*/

// Transform pose in world coordinates to local coordinates
tf2_geometry_msgs::Pose inverseTransformPose(
  const tf2_geometry_msgs::Pose & pose, const tf2_geometry_msgs::Transform & transform);

// Transform pose in world coordinates to local coordinates
tf2_geometry_msgs::Pose inverseTransformPose(
  const tf2_geometry_msgs::Pose & pose, const tf2_geometry_msgs::Pose & transform_pose);

// Transform point in world coordinates to local coordinates
Eigen::Vector3d inverseTransformPoint(
  const Eigen::Vector3d & point, const tf2_geometry_msgs::Pose & pose);

// Transform point in world coordinates to local coordinates
tf2_geometry_msgs::Point inverseTransformPoint(
  const tf2_geometry_msgs::Point & point, const tf2_geometry_msgs::Pose & pose);

double calcCurvature(
  const tf2_geometry_msgs::Point & p1, const tf2_geometry_msgs::Point & p2,
  const tf2_geometry_msgs::Point & p3);

template <class Pose1, class Pose2>
bool isDrivingForward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(getPose(src_pose).orientation);
  const double pose_direction_yaw = calcAzimuthAngle(getPoint(src_pose), getPoint(dst_pose));
  return std::fabs(normalizeRadian(src_yaw - pose_direction_yaw)) < pi / 2.0;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
tf2_geometry_msgs::Pose calcOffsetPose(
  const tf2_geometry_msgs::Pose & p, const double x, const double y, const double z,
  const double yaw = 0.0);

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
tf2_geometry_msgs::Point calcInterpolatedPoint(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = getPoint(src);
  const auto dst_point = getPoint(dst);

  tf2::Vector3 src_vec;
  src_vec.setX(src_point.x);
  src_vec.setY(src_point.y);
  src_vec.setZ(src_point.z);

  tf2::Vector3 dst_vec;
  dst_vec.setX(dst_point.x);
  dst_vec.setY(dst_point.y);
  dst_vec.setZ(dst_point.z);

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = tf2::lerp(src_vec, dst_vec, clamped_ratio);

  tf2_geometry_msgs::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();

  return point;
}

/**
 * @brief Calculate a pose by linear interpolation.
 * Note that if dist(src_pose, dst_pose)<=0.01
 * the orientation of the output pose is same as the orientation
 * of the dst_pose
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @param set_orientation_from_position_direction set position by spherical interpolation if false
 * @return interpolated point
 */
template <class Pose1, class Pose2>
tf2_geometry_msgs::Pose calcInterpolatedPose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  tf2_geometry_msgs::Pose output_pose;
  output_pose.position =
    calcInterpolatedPoint(getPoint(src_pose), getPoint(dst_pose), clamped_ratio);

  if (set_orientation_from_position_direction) {
    const double input_poses_dist = calcDistance2d(getPoint(src_pose), getPoint(dst_pose));
    const bool is_driving_forward = isDrivingForward(src_pose, dst_pose);

    // Get orientation from interpolated point and src_pose
    if ((is_driving_forward && clamped_ratio > 1.0 - (1e-6)) || input_poses_dist < 1e-3) {
      output_pose.orientation = getPose(dst_pose).orientation;
    } else if (!is_driving_forward && clamped_ratio < 1e-6) {
      output_pose.orientation = getPose(src_pose).orientation;
    } else {
      const auto & base_pose = is_driving_forward ? dst_pose : src_pose;
      const double pitch = calcElevationAngle(getPoint(output_pose), getPoint(base_pose));
      const double yaw = calcAzimuthAngle(getPoint(output_pose), getPoint(base_pose));
      tf2::Quaternion quat_vehicle = tf2::createQuaternionFromRPY(0.0, pitch, yaw);
      tf2::quaternionTFToMsg(quat_vehicle, output_pose.orientation);
    }
  } else {
    // Get orientation by spherical linear interpolation
    tf2::Transform src_tf;
    tf2::Transform dst_tf;
    tf2::poseMsgToTF(getPose(src_pose), src_tf);
    tf2::poseMsgToTF(getPose(dst_pose), dst_tf);
    const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    // tf2_geometry_msgs::Quaternion orientation;
    // output_pose.orientation.x = quaternion.getX();
    // output_pose.orientation.y = quaternion.getY();
    // output_pose.orientation.z = quaternion.getZ();
    // output_pose.orientation.w = quaternion.getW();
    tf2::quaternionTFToMsg(quaternion, output_pose.orientation);
    // output_pose.orientation = tf2::toMsg(quaternion);
  }

  return output_pose;
}

inline tf2_geometry_msgs::Vector3 createVector3(const double x, double y, double z)
{
  tf2_geometry_msgs::Vector3 vector3;
  vector3.x = x;
  vector3.y = y;
  vector3.z = z;
  return vector3;
}

// inline tf2_geometry_msgs::Twist createTwist(
//   const tf2_geometry_msgs::Vector3 & velocity, tf2_geometry_msgs::Vector3 & angular)
// {
//   tf2_geometry_msgs::Twist twist;
//   twist.linear = velocity;
//   twist.angular = angular;
//   return twist;
// }

inline double calcNorm(const tf2_geometry_msgs::Vector3 & v)
{
  return std::hypot(v.x, v.y, v.z);
}

/**
 * @brief Judge whether twist covariance is valid.
 *
 * @param twist_with_covariance source twist with covariance
 * @return If all element of covariance is 0, return false.
 */
//
// bool isTwistCovarianceValid(const tf2_geometry_msgs::TwistWithCovariance & twist_with_covariance);

// NOTE: much faster than boost::geometry::intersects()
// std::optional<tf2_geometry_msgs::Point> intersect(
//   const tf2_geometry_msgs::Point & p1, const tf2_geometry_msgs::Point & p2,
//   const tf2_geometry_msgs::Point & p3, const tf2_geometry_msgs::Point & p4);

}  // namespace utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__GEOMETRY_HPP_
