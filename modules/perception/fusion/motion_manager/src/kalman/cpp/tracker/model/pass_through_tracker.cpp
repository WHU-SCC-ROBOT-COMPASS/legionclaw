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
//
//
// Author: v1.0 Yutaka Shimizu
//

#include <bits/stdc++.h>
#include "common/tf2/include/tf2/LinearMath/Matrix3x3.h"
#include "common/tf2/include/tf2/LinearMath/Quaternion.h"
#include "common/tf2/include/tf2/utils.h"
#include "common/tf2/include/geometry_msgs/transform_stamped.h"

#include "object_recognition_utils/object_classification.hpp"
#include "object_recognition_utils/conversion.hpp"

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/tracker/model/pass_through_tracker.hpp"
#include "multi_object_tracker/utils/utils.hpp"

#include <utils/uuid_helper.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

PassThroughTracker::PassThroughTracker(
  const motion_manager::interface::Time & time, const motion_manager::interface::DetectedObject & object,
  const tf2_geometry_msgs::Transform & /*self_transform*/)
: Tracker(time, object.classification()),
  // logger_(rclcpp::get_logger("PassThroughTracker")),
  last_update_time_(time)
{
  object_ = object;
  prev_observed_object_ = object;
}

bool PassThroughTracker::predict(const motion_manager::interface::Time & time)
{
  if (0.5 /*500msec*/ < std::fabs(TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_))) {
    // RCLCPP_WARN(
    //   logger_, "There is a large gap between predicted time and measurement time. (%f)",
    //   TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_));
    AWARN << "There is a large gap between predicted time and measurement time. (%f)",
      TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  }

  return true;
}

bool PassThroughTracker::measure(
  const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
  const tf2_geometry_msgs::Transform & self_transform)
{
  prev_observed_object_ = object_;
  object_ = object;

  // Update Velocity if the observed object does not have twist information
  const double dt = TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  if (!object_.kinematics().has_twist() && dt > 1e-6) {
    const double dx = object_.kinematics().pose_with_covariance().pose().position().x() -
                      prev_observed_object_.kinematics().pose_with_covariance().pose().position().x();
    const double dy = object_.kinematics().pose_with_covariance().pose().position().y() -
                      prev_observed_object_.kinematics().pose_with_covariance().pose().position().y();
    object_.mutable_kinematics()->mutable_twist_with_covariance()->mutable_twist()->mutable_linear()->set_x(std::hypot(dx, dy) / dt);
  }
  last_update_time_ = time;

  (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool PassThroughTracker::getTrackedObject(
  const motion_manager::interface::Time & time, motion_manager::interface::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.set_object_id(getUUID());
  object.set_classification(getClassification());

  double pose_covariance = 0.0;
  object.kinematics().pose_with_covariance().covariance().resize(36);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::X_X, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::X_Y, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::Y_X, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::Y_Y, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::Z_Z, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::ROLL_ROLL, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::PITCH_PITCH, pose_covariance);
  object.mutable_kinematics()->mutable_pose_with_covariance()->set_covariance(utils::MSG_COV_IDX::YAW_YAW, pose_covariance);

  // twist covariance
  double twist_covariance = 0.0;
  object.kinematics().twist_with_covariance().covariance().resize(36);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::X_X, twist_covariance);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::Y_Y, twist_covariance);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::Z_Z, twist_covariance);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::ROLL_ROLL, twist_covariance);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::PITCH_PITCH, twist_covariance);
  object.mutable_kinematics()->mutable_twist_with_covariance()->set_covariance(utils::MSG_COV_IDX::YAW_YAW, twist_covariance);

  const double dt = TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  if (0.5 /*500msec*/ < dt) {
    // RCLCPP_WARN(
    //   logger_, "There is a large gap between last updated time and current time. (%f)",
    //   TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_));
    AWARN << "There is a large gap between predicted time and measurement time. (%f)",
      TimeTool::TimeStruct2S(time)-TimeTool::TimeStruct2S(last_update_time_);
  }

  return true;
}
