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

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <algorithm>
#include <random>

Tracker::Tracker(
  const motion_manager::interface::Time & time,
  const std::vector<motion_manager::interface::ObjectClassification> & classification)
: classification_(classification),
  no_measurement_count_(0),
  total_no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time)
{
  // Generate random number
  uuid_.uuid().resize(16);
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.mutable_uuid()->begin(), uuid_.mutable_uuid()->end(), bit_eng);

}

bool Tracker::updateWithMeasurement(
  const motion_manager::interface::DetectedObject & object,
  const motion_manager::interface::Time & measurement_time, const tf2_geometry_msgs::Transform & self_transform)
{
  no_measurement_count_ = 0;
  ++total_measurement_count_;
  last_update_with_measurement_time_ = measurement_time;
  measure(object, measurement_time, self_transform);
  return true;
}

bool Tracker::updateWithoutMeasurement()
{
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  return true;
}

tf2_geometry_msgs::PoseWithCovariance Tracker::getPoseWithCovariance(
  const motion_manager::interface::Time & time) const
{
  motion_manager::interface::TrackedObject object;
  getTrackedObject(time, object); 
  // TODO dcw:      done
  // return object.kinematics.pose_with_covariance;
  
  tf2_geometry_msgs::PoseWithCovariance covariance_out;
  covariance_out.pose.position.x = object.kinematics().pose_with_covariance().pose().position().x();
  covariance_out.pose.position.y = object.kinematics().pose_with_covariance().pose().position().y();
  covariance_out.pose.position.z = object.kinematics().pose_with_covariance().pose().position().z();
  
  for(int i = 0; i <  object.kinematics().pose_with_covariance().covariance_size(); i++)
  {
    covariance_out.covariance[i] = object.kinematics().pose_with_covariance().covariance()[i];
  }

  return covariance_out;

}
