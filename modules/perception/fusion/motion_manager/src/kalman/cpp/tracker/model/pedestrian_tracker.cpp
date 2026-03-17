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
//
//
// Author: v1.0 Yukihiro Saito
//

#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <utils/geometry/boost_polygon_utils.hpp>
#include <utils/math/normalization.hpp>
#include <utils/math/unit_conversion.hpp>
#include <utils/uuid_helper.hpp>

#include <bits/stdc++.h>
#include "common/tf2/include/tf2/LinearMath/Matrix3x3.h"
#include "common/tf2/include/tf2/LinearMath/Quaternion.h"
#include "common/tf2/include/tf2/utils.h"
#include "common/tf2/include/geometry_msgs/transform_stamped.h"

#include "object_recognition_utils/object_classification.hpp"
#include "object_recognition_utils/conversion.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

// using Label = motion_manager::interface::ObjectClassification;
namespace bg = boost::geometry;

PedestrianTracker::PedestrianTracker(
  const motion_manager::interface::Time & time, const motion_manager::interface::DetectedObject & object,
  const tf2_geometry_msgs::Transform & /*self_transform*/)
: Tracker(time, object.classification()),
  // logger_(rclcpp::get_logger("PedestrianTracker")),
  last_update_time_(time),
  z_(object.kinematics().pose_with_covariance().pose().position().z())
{
  object_ = object;

  // Initialize parameters
  // measurement noise covariance
  float r_stddev_x = 0.4;                                  // [m]
  float r_stddev_y = 0.4;                                  // [m]
  float r_stddev_yaw = utils::deg2rad(30);  // [rad]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  // initial state covariance
  float p0_stddev_x = 2.0;                                    // [m]
  float p0_stddev_y = 2.0;                                    // [m]
  float p0_stddev_yaw = utils::deg2rad(1000);  // [rad]
  float p0_stddev_vx = utils::kmph2mps(120);   // [m/s]
  float p0_stddev_wz = utils::deg2rad(360);    // [rad/s]
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_wz = std::pow(p0_stddev_wz, 2.0);
  // process noise covariance
  float q_stddev_x = 0.4;                                  // [m/s]
  float q_stddev_y = 0.4;                                  // [m/s]
  float q_stddev_yaw = utils::deg2rad(20);  // [rad/s]
  float q_stddev_vx = utils::kmph2mps(5);   // [m/(s*s)]
  float q_stddev_wz = utils::deg2rad(30);   // [rad/(s*s)]
  ekf_params_.q_cov_x = std::pow(q_stddev_x, 2.0);
  ekf_params_.q_cov_y = std::pow(q_stddev_y, 2.0);
  ekf_params_.q_cov_yaw = std::pow(q_stddev_yaw, 2.0);
  ekf_params_.q_cov_vx = std::pow(q_stddev_vx, 2.0);
  ekf_params_.q_cov_wz = std::pow(q_stddev_wz, 2.0);
  // limitations
  max_vx_ = utils::kmph2mps(10);  // [m/s]
  max_wz_ = utils::deg2rad(30);   // [rad/s]

  // initialize state vector X
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  X(IDX::X) = object.kinematics().pose_with_covariance().pose().position().x();
  X(IDX::Y) = object.kinematics().pose_with_covariance().pose().position().y();
    tf2::Quaternion orientation;
    orientation.setX(object.kinematics().pose_with_covariance().pose().orientation().qx());
    orientation.setY(object.kinematics().pose_with_covariance().pose().orientation().qy());
    orientation.setZ(object.kinematics().pose_with_covariance().pose().orientation().qz());
    orientation.setW(object.kinematics().pose_with_covariance().pose().orientation().qw());
  X(IDX::YAW) = tf2::getYaw(orientation);
  if (object.kinematics().has_twist()) {
    X(IDX::VX) = object.kinematics().twist_with_covariance().twist().linear().x();
    X(IDX::WZ) = object.kinematics().twist_with_covariance().twist().angular().z();
  } else {
    X(IDX::VX) = 0.0;
    X(IDX::WZ) = 0.0;
  }

  // initialize state covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  if (!object.kinematics().has_position_covariance()) {
    const double cos_yaw = std::cos(X(IDX::YAW));
    const double sin_yaw = std::sin(X(IDX::YAW));
    const double sin_2yaw = std::sin(2.0f * X(IDX::YAW));
    // Rotate the covariance matrix according to the vehicle yaw
    // because p0_cov_x and y are in the vehicle coordinate system.
    P(IDX::X, IDX::X) =
      ekf_params_.p0_cov_x * cos_yaw * cos_yaw + ekf_params_.p0_cov_y * sin_yaw * sin_yaw;
    P(IDX::X, IDX::Y) = 0.5f * (ekf_params_.p0_cov_x - ekf_params_.p0_cov_y) * sin_2yaw;
    P(IDX::Y, IDX::Y) =
      ekf_params_.p0_cov_x * sin_yaw * sin_yaw + ekf_params_.p0_cov_y * cos_yaw * cos_yaw;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::YAW, IDX::YAW) = ekf_params_.p0_cov_yaw;
    P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
    P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
  } else {
    P(IDX::X, IDX::X) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::Y_X];
    P(IDX::YAW, IDX::YAW) =
      object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::YAW_YAW];
    if (object.kinematics().has_twist_covariance()) {
      P(IDX::VX, IDX::VX) =
        object.kinematics().twist_with_covariance().covariance()[utils::MSG_COV_IDX::X_X];
      P(IDX::WZ, IDX::WZ) =
        object.kinematics().twist_with_covariance().covariance()[utils::MSG_COV_IDX::YAW_YAW];
    } else {
      P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
      P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;
    }
  }

  bounding_box_ = {0.5, 0.5, 1.7};
  cylinder_ = {0.3, 1.7};
  if (object.shape().type() == motion_manager::common::ShapeType::S_BOUNDING_BOX) {
    bounding_box_ = {
      object.shape().dimensions().x(), object.shape().dimensions().y(), object.shape().dimensions().z()};
  } else if (object.shape().type() == motion_manager::common::ShapeType::S_CYLINDER) {
    cylinder_ = {object.shape().dimensions().x(), object.shape().dimensions().z()};
  }

  ekf_.init(X, P);
}

bool PedestrianTracker::predict(const motion_manager::interface::Time & time)
{
  const double dt = TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  bool ret = predict(dt, ekf_);
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool PedestrianTracker::predict(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: Constant turn-rate motion model (CTRV)
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * vx_{k+1}  = vx_k
   * wz_{k+1}  = wz_k
   *
   */

  /*  Jacobian Matrix
   *
   * A = [ 1, 0, -vx*sin(yaw)*dt, cos(yaw)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw)*dt, sin(yaw)*dt,  0]
   *     [ 0, 0,               1,           0, dt]
   *     [ 0, 0,               0,           1,  0]
   *     [ 0, 0,               0,           0,  1]
   */

  // Current state vector X t
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
  ekf.getX(X_t);
  const double cos_yaw = std::cos(X_t(IDX::YAW));
  const double sin_yaw = std::sin(X_t(IDX::YAW));
  const double sin_2yaw = std::sin(2.0f * X_t(IDX::YAW));

  // Predict state vector X t+1
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);                // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VX) * cos_yaw * dt;  // dx = v * cos(yaw)
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VX) * sin_yaw * dt;  // dy = v * sin(yaw)
  X_next_t(IDX::YAW) = X_t(IDX::YAW) + (X_t(IDX::WZ)) * dt;      // dyaw = omega
  X_next_t(IDX::VX) = X_t(IDX::VX);
  X_next_t(IDX::WZ) = X_t(IDX::WZ);

  // State transition matrix A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
  A(IDX::X, IDX::YAW) = -X_t(IDX::VX) * sin_yaw * dt;
  A(IDX::X, IDX::VX) = cos_yaw * dt;
  A(IDX::Y, IDX::YAW) = X_t(IDX::VX) * cos_yaw * dt;
  A(IDX::Y, IDX::VX) = sin_yaw * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  // Process noise covariance Q
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) =
    (ekf_params_.q_cov_x * cos_yaw * cos_yaw + ekf_params_.q_cov_y * sin_yaw * sin_yaw) * dt * dt;
  Q(IDX::X, IDX::Y) = (0.5f * (ekf_params_.q_cov_x - ekf_params_.q_cov_y) * sin_2yaw) * dt * dt;
  Q(IDX::Y, IDX::Y) =
    (ekf_params_.q_cov_x * sin_yaw * sin_yaw + ekf_params_.q_cov_y * cos_yaw * cos_yaw) * dt * dt;
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::YAW, IDX::YAW) = ekf_params_.q_cov_yaw * dt * dt;
  Q(IDX::VX, IDX::VX) = ekf_params_.q_cov_vx * dt * dt;
  Q(IDX::WZ, IDX::WZ) = ekf_params_.q_cov_wz * dt * dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  if (!ekf.predict(X_next_t, A, Q)) {
    // RCLCPP_WARN(logger_, "Cannot predict");
    AWARN << "Cannot predict";
  }

  return true;
}

bool PedestrianTracker::measureWithPose(
  const motion_manager::interface::DetectedObject & object)
{
  constexpr int dim_y = 2;  // pos x, pos y depending on Pose output
  // double measurement_yaw =
  //   utils::normalizeRadian(tf2::getYaw(object.state.pose_covariance.pose.orientation));
  // {
  //   Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
  //   ekf_.getX(X_t);
  //   while (M_PI_2 <= X_t(IDX::YAW) - measurement_yaw) {
  //     measurement_yaw = measurement_yaw + M_PI;
  //   }
  //   while (M_PI_2 <= measurement_yaw - X_t(IDX::YAW)) {
  //     measurement_yaw = measurement_yaw - M_PI;
  //   }
  //   float theta = std::acos(
  //     std::cos(X_t(IDX::YAW)) * std::cos(measurement_yaw) +
  //     std::sin(X_t(IDX::YAW)) * std::sin(measurement_yaw));
  //   if (utils::deg2rad(60) < std::fabs(theta)) return false;
  // }

  // Set measurement matrix C and observation vector Y
  Eigen::MatrixXd Y(dim_y, 1);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, ekf_params_.dim_x);
  Y << object.kinematics().pose_with_covariance().pose().position().x(),
    object.kinematics().pose_with_covariance().pose().position().y();
  C(0, IDX::X) = 1.0;  // for pos x
  C(1, IDX::Y) = 1.0;  // for pos y
  // C(2, IDX::YAW) = 1.0;  // for yaw

  // Set noise covariance matrix R
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (!object.kinematics().has_position_covariance()) {
    R(0, 0) = ekf_params_.r_cov_x;  // x - x
    R(0, 1) = 0.0;                  // x - y
    R(1, 1) = ekf_params_.r_cov_y;  // y - y
    R(1, 0) = R(0, 1);              // y - x
    // R(2, 2) = ekf_params_.r_cov_yaw;                        // yaw - yaw
  } else {
    R(0, 0) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::X_X];
    R(0, 1) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::X_Y];
    // R(0, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_YAW];
    R(1, 0) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::Y_X];
    R(1, 1) = object.kinematics().pose_with_covariance().covariance()[utils::MSG_COV_IDX::Y_Y];
    // R(1, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_YAW];
    // R(2, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_X];
    // R(2, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_Y];
    // R(2, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
  }

  // ekf update
  if (!ekf_.update(Y, C, R)) {
    // RCLCPP_WARN(logger_, "Cannot update");
    AWARN << "Cannot update";
  }

  // normalize yaw and limit vx, wz
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    Eigen::MatrixXd P_t(ekf_params_.dim_x, ekf_params_.dim_x);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    X_t(IDX::YAW) = utils::normalizeRadian(X_t(IDX::YAW));
    if (!(-max_vx_ <= X_t(IDX::VX) && X_t(IDX::VX) <= max_vx_)) {
      X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -max_vx_ : max_vx_;
    }
    if (!(-max_wz_ <= X_t(IDX::WZ) && X_t(IDX::WZ) <= max_wz_)) {
      X_t(IDX::WZ) = X_t(IDX::WZ) < 0 ? -max_wz_ : max_wz_;
    }
    ekf_.init(X_t, P_t);
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics().pose_with_covariance().pose().position().z();

  return true;
}

bool PedestrianTracker::measureWithShape(
  const motion_manager::interface::DetectedObject & object)
{
  constexpr float gain = 0.9;
  if (object.shape().type() == motion_manager::common::ShapeType::S_BOUNDING_BOX) {
    bounding_box_.length = gain * bounding_box_.length + (1.0 - gain) * object.shape().dimensions().x();
    bounding_box_.width = gain * bounding_box_.width + (1.0 - gain) * object.shape().dimensions().y();
    bounding_box_.height = gain * bounding_box_.height + (1.0 - gain) * object.shape().dimensions().z();
  } else if (object.shape().type() == motion_manager::common::ShapeType::S_CYLINDER) {
    cylinder_.width = gain * cylinder_.width + (1.0 - gain) * object.shape().dimensions().x();
    cylinder_.height = gain * cylinder_.height + (1.0 - gain) * object.shape().dimensions().z();
  } else {
    return false;
  }

  return true;
}

bool PedestrianTracker::measure(
  const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
  const tf2_geometry_msgs::Transform & self_transform)
{
  const auto & current_classification = getClassification();
  object_ = object;
  if (object_recognition_utils::getHighestProbLabel(object.classification()) == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
    setClassification(current_classification);
  }

  if (0.01 /*10msec*/ < std::fabs(TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_))) {
    // RCLCPP_WARN(
    //   logger_, "There is a large gap between predicted time and measurement time. (%f)",
    //   TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_));
    AWARN << "There is a large gap between predicted time and measurement time. (%f)",
      TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  }

  measureWithPose(object);
  measureWithShape(object);

  (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool PedestrianTracker::getTrackedObject(
  const motion_manager::interface::Time & time, motion_manager::interface::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.set_object_id(getUUID());
  object.set_classification(getClassification());

  // predict state
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  const double dt = TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  if (0.001 /*1msec*/ < dt) {
    predict(dt, tmp_ekf_for_no_update);
  }
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);                // predicted state
  Eigen::MatrixXd P(ekf_params_.dim_x, ekf_params_.dim_x);  // predicted state
  tmp_ekf_for_no_update.getX(X_t);
  tmp_ekf_for_no_update.getP(P);

  /*  put predicted pose and twist to output object  */
  auto & pose_with_cov = object.kinematics().pose_with_covariance();
  auto & twist_with_cov = object.kinematics().twist_with_covariance();

  // position
  pose_with_cov.mutable_pose()->mutable_position()->set_x(X_t(IDX::X));
  pose_with_cov.mutable_pose()->mutable_position()->set_y(X_t(IDX::Y));
  pose_with_cov.mutable_pose()->mutable_position()->set_z(z_);
  // quaternion
  {
    double roll, pitch, yaw;
    tf2::Quaternion original_quaternion;
    original_quaternion.setX(object_.kinematics().pose_with_covariance().pose().orientation().qx());
    original_quaternion.setY(object_.kinematics().pose_with_covariance().pose().orientation().qy());
    original_quaternion.setZ(object_.kinematics().pose_with_covariance().pose().orientation().qz());
    original_quaternion.setW(object_.kinematics().pose_with_covariance().pose().orientation().qw());
    tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
    tf2::Quaternion filtered_quaternion;
    filtered_quaternion.setRPY(roll, pitch, X_t(IDX::YAW));
    // filtered_quaternion.setRPY(roll, pitch, yaw);
    pose_with_cov.mutable_pose()->mutable_orientation()->set_qx(filtered_quaternion.x());
    pose_with_cov.mutable_pose()->mutable_orientation()->set_qy(filtered_quaternion.y());
    pose_with_cov.mutable_pose()->mutable_orientation()->set_qz(filtered_quaternion.z());
    pose_with_cov.mutable_pose()->mutable_orientation()->set_qw(filtered_quaternion.w());
    object.mutable_kinematics()->set_orientation_availability(
      utils::ORIENTATION_AVAILABILITY::SIGN_UNKNOWN);
  }
  // position covariance
  double z_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  double r_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  double p_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_with_cov.covariance().resize(36);
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::X_X, P(IDX::X, IDX::X));
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::X_Y, P(IDX::X, IDX::Y));
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::Y_X, P(IDX::Y, IDX::X));
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::Y_Y, P(IDX::Y, IDX::Y));
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::Z_Z, z_cov);
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::ROLL_ROLL, r_cov);
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::PITCH_PITCH, p_cov);
  pose_with_cov.set_covariance(utils::MSG_COV_IDX::YAW_YAW, P(IDX::YAW, IDX::YAW));

  // twist
  // twist_with_cov.twist.linear.x = X_t(IDX::VX);
  twist_with_cov.mutable_twist()->mutable_linear()->set_x(X_t(IDX::VX) * std::cos(X_t(IDX::YAW)));
  twist_with_cov.mutable_twist()->mutable_linear()->set_y(X_t(IDX::VX) * std::sin(X_t(IDX::YAW)));
  twist_with_cov.mutable_twist()->mutable_angular()->set_z(X_t(IDX::WZ));
  // twist covariance
  double vy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative

  twist_with_cov.covariance().resize(36);
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::X_X, P(IDX::VX, IDX::VX));
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::Y_Y, vy_cov);
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::Z_Z, vz_cov);
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::X_YAW, P(IDX::VX, IDX::WZ));
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::YAW_X, P(IDX::WZ, IDX::VX));
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::ROLL_ROLL, wx_cov);
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::PITCH_PITCH, wy_cov);
  twist_with_cov.set_covariance(utils::MSG_COV_IDX::YAW_YAW, P(IDX::WZ, IDX::WZ));

  // set shape
  if (object.shape().type() == motion_manager::common::ShapeType::S_BOUNDING_BOX) {
    object.mutable_shape()->mutable_dimensions()->set_x(bounding_box_.length);
    object.mutable_shape()->mutable_dimensions()->set_y(bounding_box_.width);
    object.mutable_shape()->mutable_dimensions()->set_z(bounding_box_.height);
  } else if (object.shape().type() == motion_manager::common::ShapeType::S_CYLINDER) {
    object.mutable_shape()->mutable_dimensions()->set_x(cylinder_.width);
    object.mutable_shape()->mutable_dimensions()->set_y(cylinder_.width);
    object.mutable_shape()->mutable_dimensions()->set_z(cylinder_.height);
  } else if (object.shape().type() == motion_manager::common::ShapeType::S_POLYGON) {
  tf2::Quaternion object_orientation;
  object_orientation.setX(object_.kinematics().pose_with_covariance().pose().orientation().qx());
  object_orientation.setY(object_.kinematics().pose_with_covariance().pose().orientation().qy());
  object_orientation.setZ(object_.kinematics().pose_with_covariance().pose().orientation().qz());
  object_orientation.setW(object_.kinematics().pose_with_covariance().pose().orientation().qw());
const auto origin_yaw = tf2::getYaw(object_orientation);
  tf2::Quaternion orientation;
  orientation.setX(pose_with_cov.pose().orientation().qx());
  orientation.setY(pose_with_cov.pose().orientation().qy());
  orientation.setZ(pose_with_cov.pose().orientation().qz());
  orientation.setW(pose_with_cov.pose().orientation().qw());
const auto ekf_pose_yaw = tf2::getYaw(orientation);
  // TODO dcw : Polygon2d to motion_manager::interface::Ploygon        done
    // object.shape.footprint =
    //   utils::rotatePolygon(object.shape().footprint(), origin_yaw - ekf_pose_yaw);
    utils::Polygon2d poly_tmp1, poly_tmp2;    
  motion_manager::interface::Polygon poly_out;
  for(auto p:object.shape().footprint().points())
  {
    boost::geometry::append(poly_tmp1.outer(),bg::model::point<double, 2, bg::cs::cartesian>(p.x(),p.y()));
  } 
  poly_tmp2 =  utils::rotatePolygon(poly_tmp1, origin_yaw - ekf_pose_yaw);

  vector<motion_manager::interface::Point32> point_vec;
  for (auto p:poly_tmp2.outer()){
    motion_manager::interface::Point32 this_point;
    this_point.set_x(bg::get<0>(p));
    this_point.set_y(bg::get<1>(p));
    point_vec.push_back(this_point);
  }
  poly_out.set_points(point_vec);
  object.mutable_shape()->set_footprint(poly_out);
  }

  return true;
}
