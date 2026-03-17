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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
// #include "multi_object_tracker/utils/utils.hpp"
// #include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>

#include "common/interface/detected_objects.hpp"
#include "common/interface/tracked_objects.hpp"
#include "common/interface/point_3d.hpp"
#include "common/interface/uuid.hpp"
#include "kalman/hpp/utils/time_tool.h"
#include "common/tf2/include/geometry_msgs/transform_stamped.h"
#include "object_recognition_utils/object_classification.hpp"

#include "common/logging/logging.h"

#include <vector>
using namespace motion_manager::common;
class Tracker
{
protected:
  motion_manager::interface::UUID getUUID() const { return uuid_; }
  void setClassification(
    const std::vector<motion_manager::interface::ObjectClassification> & classification)
  {
    classification_ = classification;
  }

private:
  motion_manager::interface::UUID uuid_;
  std::vector<motion_manager::interface::ObjectClassification> classification_;
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  motion_manager::interface::Time last_update_with_measurement_time_;

public:
  Tracker(
    const motion_manager::interface::Time & time,
    const std::vector<motion_manager::interface::ObjectClassification> & classification);
  virtual ~Tracker() {}
  bool updateWithMeasurement(
    const motion_manager::interface::DetectedObject & object,
    const motion_manager::interface::Time & measurement_time, const tf2_geometry_msgs::Transform & self_transform);
  bool updateWithoutMeasurement();
  std::vector<motion_manager::interface::ObjectClassification> getClassification() const
  {
    return classification_;
  }
  std::uint8_t getHighestProbLabel() const
  {
    return object_recognition_utils::getHighestProbLabel(classification_);
  }
  int getNoMeasurementCount() const { return no_measurement_count_; }
  int getTotalNoMeasurementCount() const { return total_no_measurement_count_; }
  int getTotalMeasurementCount() const { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate(const motion_manager::interface::Time & current_time) const
  {
    return TimeTool::TimeStruct2S(current_time)-TimeTool::TimeStruct2S(last_update_with_measurement_time_);
  }
  virtual tf2_geometry_msgs::PoseWithCovariance getPoseWithCovariance(
    const motion_manager::interface::Time & time) const;

  /*
   * Pure virtual function
   */

protected:
  virtual bool measure(
    const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
    const tf2_geometry_msgs::Transform & self_transform) = 0;

public:
  virtual bool getTrackedObject(
    const motion_manager::interface::Time & time,
    motion_manager::interface::TrackedObject & object) const = 0;
  virtual bool predict(const motion_manager::interface::Time & time) = 0;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
