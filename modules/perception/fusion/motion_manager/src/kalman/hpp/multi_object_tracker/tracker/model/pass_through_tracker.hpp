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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_

#include "tracker_base.hpp"

#include <kalman_filter/kalman_filter.hpp>

class PassThroughTracker : public Tracker
{
private:
  motion_manager::interface::DetectedObject object_;
  motion_manager::interface::DetectedObject prev_observed_object_;
  // rclcpp::Logger logger_;
  motion_manager::interface::Time last_update_time_;

public:
  PassThroughTracker(
    const motion_manager::interface::Time & time, const motion_manager::interface::DetectedObject & object,
    const tf2_geometry_msgs::Transform & self_transform);
  bool predict(const motion_manager::interface::Time & time) override;
  bool measure(
    const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
    const tf2_geometry_msgs::Transform & self_transform) override;
  bool getTrackedObject(
    const motion_manager::interface::Time & time,
    motion_manager::interface::TrackedObject & object) const override;
  virtual ~PassThroughTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_
