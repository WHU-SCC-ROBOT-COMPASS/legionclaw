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

#include "multi_object_tracker/tracker/model/multiple_vehicle_tracker.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include <utils/uuid_helper.hpp>

using Label = motion_manager::interface::ObjectClassification;

MultipleVehicleTracker::MultipleVehicleTracker(
  const motion_manager::interface::Time & time, const motion_manager::interface::DetectedObject & object,
  const tf2_geometry_msgs::Transform & self_transform)
: Tracker(time, object.classification()),
  normal_vehicle_tracker_(time, object, self_transform),
  big_vehicle_tracker_(time, object, self_transform)
{
}

bool MultipleVehicleTracker::predict(const motion_manager::interface::Time & time)
{
  big_vehicle_tracker_.predict(time);
  normal_vehicle_tracker_.predict(time);
  return true;
}

bool MultipleVehicleTracker::measure(
  const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
  const tf2_geometry_msgs::Transform & self_transform)
{
  big_vehicle_tracker_.measure(object, time, self_transform);
  normal_vehicle_tracker_.measure(object, time, self_transform);
  if (object_recognition_utils::getHighestProbLabel(object.classification()) != motion_manager::common::DetectedObjectLabel::DO_UNKNOWN)
    setClassification(object.classification());
  return true;
}

bool MultipleVehicleTracker::getTrackedObject(
  const motion_manager::interface::Time & time, motion_manager::interface::TrackedObject & object) const
{
  using Label = motion_manager::interface::ObjectClassification;
  const uint8_t label = getHighestProbLabel();

  if (label == motion_manager::common::DetectedObjectLabel::DO_CAR) {
    normal_vehicle_tracker_.getTrackedObject(time, object);
  } else if (utils::isLargeVehicleLabel(label)) {
    big_vehicle_tracker_.getTrackedObject(time, object);
  }
  object.set_object_id(getUUID());
  object.set_classification(getClassification());
  return true;
}
