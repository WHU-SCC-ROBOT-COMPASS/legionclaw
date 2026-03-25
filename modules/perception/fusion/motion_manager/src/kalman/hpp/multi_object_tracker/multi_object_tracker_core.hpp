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
///

#ifndef MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_
#define MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_

#include "multi_object_tracker/data_association/data_association.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"

// #include <ros/ros.h>
// #include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <utils/system/stop_watch.hpp>
#include <utils/uuid_helper.hpp>

#include "common/interface/detected_object.hpp"
#include "common/interface/tracked_object.hpp"
// #include <motion_manager_perception_msgs/DetectedObject.h>
// #include <motion_manager_perception_msgs/TrackedObjects.h>
// #include <geometry_msgs/PoseStamped.h>

#include "common/tf2/include/tf2/LinearMath/Transform.h"
#include "common/tf2/include/tf2/convert.h"
#include "common/tf2/include/tf2/transform_datatypes.h"

// #ifdef ROS_DISTRO_GALACTIC
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif

// #include <diagnostic_updater/diagnostic_updater.hpp>
// #include <diagnostic_updater/publisher.hpp>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>
// #include <lcm/lcm-cpp.hpp>
// #include <lcm/lcm.h>
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/location.hpp"
// #include "ros_interface/ObstacleList.h"
#include "converter/convert.hpp"

#include <fstream>
#include "kalman/hpp/utils/time_tool.h"
#include "common/json/json.hpp"
#include "common/enum/enum.h"

using namespace std;

class MultiObjectTracker
{
public:
  MultiObjectTracker(){};
  void parseConfig(std::string config_path);
  ~MultiObjectTracker()=default;
  motion_manager::interface::TrackedObjects onMeasurement(const std::shared_ptr<motion_manager::interface::DetectedObjects> input_objects_msg,legionclaw::interface::Location location);
private:
  // ros::NodeHandle nh_;
  // ros::Publisher tracked_objects_pub_;
  // ros::Subscriber detected_object_sub_;

  // std::shared_ptr<lcm::LCM> lcm_;

  nlohmann::json multi_object_tracker_json_;

  motion_manager::interface::DetectedObjects objects_;
  // motion_manager::interface::TimerBase::SharedPtr publish_timer_;  // publish timer

  // debugger class
  // TODO TrackerDebugger
  // std::unique_ptr<TrackerDebugger> debugger_;


  std::map<std::uint8_t, std::string> tracker_map_;

  
  

  std::string world_frame_id_;  // tracking frame
  std::list<std::shared_ptr<Tracker>> list_tracker_;
  std::unique_ptr<DataAssociation> data_association_;

  void checkTrackerLifeCycle(
    std::list<std::shared_ptr<Tracker>> & list_tracker, const motion_manager::interface::Time & time,
    const tf2_geometry_msgs::Transform & self_transform);
  void sanitizeTracker(
    std::list<std::shared_ptr<Tracker>> & list_tracker, const motion_manager::interface::Time & time);
  std::shared_ptr<Tracker> createNewTracker(
    const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
    const tf2_geometry_msgs::Transform & self_transform) const;

  inline bool shouldTrackerPublish(const std::shared_ptr<const Tracker> tracker) const;

  
};

#endif  // MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_
