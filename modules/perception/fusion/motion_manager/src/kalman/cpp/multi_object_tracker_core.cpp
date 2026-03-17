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

#include <boost/optional.hpp>

#include <glog/logging.h>

// #include "tf/transform_datatypes.h"
#include "common/tf2/include/tf2/transform_datatypes.h"


#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/multi_object_tracker_core.hpp"
// #include "multi_object_tracker/utils/utils.hpp"
#include "object_recognition_utils/matching.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>


// using Label = motion_manager::interface::ObjectClassification;

void MultiObjectTracker::parseConfig(std::string config_path)
{
  std::cout << "start parse config ....." << "\n"; 
  std::ifstream in(config_path);
  in >> multi_object_tracker_json_;
  if (multi_object_tracker_json_.is_null())
  {
    std::cout << "multi_object_tracker_json_ is null" << "\n";
    return;
  }
  in.close();

  std::cout << "finishs parse config ....." << "\n"; 


  std::vector<int> can_assign_matrix = multi_object_tracker_json_.at("can_assign_matrix").get<std::vector<int>>();
  std::vector<double> max_dist_matrix = multi_object_tracker_json_.at("max_dist_matrix").get<std::vector<double>>();
  std::vector<double> max_area_matrix = multi_object_tracker_json_.at("max_area_matrix").get<std::vector<double>>();
  std::vector<double> min_area_matrix = multi_object_tracker_json_.at("min_area_matrix").get<std::vector<double>>();
  std::vector<double> max_rad_matrix = multi_object_tracker_json_.at("max_rad_matrix").get<std::vector<double>>();
  std::vector<double> min_iou_matrix = multi_object_tracker_json_.at("min_iou_matrix").get<std::vector<double>>();

  // tracker map
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_CAR, multi_object_tracker_json_["car_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_TRUCK, multi_object_tracker_json_["truck_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_BUS, multi_object_tracker_json_["bus_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_TRAILER, multi_object_tracker_json_["trailer_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_PEDESTRIAN, multi_object_tracker_json_["pedestrian_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_BICYCLE, multi_object_tracker_json_["bicycle_tracker"]));
  tracker_map_.insert(
    std::make_pair(motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE, multi_object_tracker_json_["motorcycle_tracker"]));

  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix,
    min_iou_matrix);


  std::cout << "get  config param ....." << "\n"; 
}

motion_manager::interface::TrackedObjects MultiObjectTracker::onMeasurement(
  const std::shared_ptr<motion_manager::interface::DetectedObjects> input_objects_msg, legion::interface::Location location)
{

  double time_begin=motion_manager::common::TimeTool::Now2Ms();

  tf2_geometry_msgs::Transform self_transform;
  self_transform.translation.x = location.utm_position().x();
  self_transform.translation.y = location.utm_position().y();    
  tf2_geometry_msgs::Quaternion quat;
  // quat=tf::createQuaternionMsgFromRollPitchYaw(location.roll(),location.pitch(),location.heading());
  tf2::Quaternion q;
  q.setRPY(location.roll(),location.pitch(),location.heading());
  quat.w = q.getW();
  quat.x = q.getX();
  quat.y = q.getY();
  quat.z = q.getZ();
  self_transform.rotation=quat;


  // //转换到世界坐标系
  // motion_manager::interface::DetectedObjects transformed_objects;
  // for (auto object:input_objects_msg->objects())
  // {
    
  //   std::vector<double> values = motion_manager::preprocessor::coordinate::convert_point(location.utm_position().x(),
  //                                                                                 location.utm_position().y(),
  //                                                                                 location.utm_position().z(),
  //                                                                                 location.roll(),
  //                                                                                 location.pitch(),
  //                                                                                 location.heading(),
  //                                                                                 object.kinematics().pose_with_covariance().pose().position().x(),
  //                                                                                 object.kinematics().pose_with_covariance().pose().position().y(),
  //                                                                                 object.kinematics().pose_with_covariance().pose().position().z());
  //   object.mutable_kinematics()->mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_x(values[0]);
  //   object.mutable_kinematics()->mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_y(values[1]);
  //   object.mutable_kinematics()->mutable_pose_with_covariance()->mutable_pose()->mutable_position()->set_z(0);

  //   transformed_objects.add_objects(object);
  // }

  motion_manager::interface::Time measurement_time;
  measurement_time = input_objects_msg->header().stamp();
  // measurement_time.set_sec(input_objects_msg->header.stamp.sec);
  // measurement_time.set_nsec(input_objects_msg->header.stamp.nsec);

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    *input_objects_msg, list_tracker_);  // row : tracker, col : measurement
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      (*(tracker_itr))
        ->updateWithMeasurement(
          input_objects_msg->objects().at(direct_assignment.find(tracker_idx)->second),
          measurement_time, self_transform);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, measurement_time, self_transform);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, measurement_time);

  /* new tracker */
  for (size_t i = 0; i < input_objects_msg->objects().size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    std::shared_ptr<Tracker> tracker =
      createNewTracker(input_objects_msg->objects().at(i), measurement_time, self_transform);
    if (tracker) list_tracker_.push_back(tracker);
  }

  motion_manager::interface::TrackedObjects output_msg, tentative_objects_msg;
  output_msg.mutable_header()->set_frame_id(world_frame_id_);
  output_msg.mutable_header()->set_stamp(measurement_time);
  // output_msg.header.stamp.sec = measurement_time.sec();
  // output_msg.header.stamp.nsec = measurement_time.nsec();
  tentative_objects_msg.set_header(output_msg.header());

  

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if (!shouldTrackerPublish(*itr)) 
    {
      // motion_manager::interface::TrackedObject object1;
      // (*itr)->getTrackedObject(measurement_time, object1);
      
      // std::string object_id = tier4_autoware_utils::toHexString(object1.object_id);
      // int decimalNumber = std::stoi(object_id, nullptr, 16);
      // std::cout << "no publish object_id " << decimalNumber  << " object type  " << object1.classification[0] << "\n";
      continue;
    } 
    motion_manager::interface::TrackedObject object;
    (*itr)->getTrackedObject(measurement_time, object);
    
    output_msg.add_objects(object);
  }

  // publish(measurement_time);

  double time_end=motion_manager::common::TimeTool::Now2Ms();

  std::cout << "track time : " << time_end- time_begin << " ms " <<"\n";
  return output_msg;
}

std::shared_ptr<Tracker> MultiObjectTracker::createNewTracker(
  const motion_manager::interface::DetectedObject & object, const motion_manager::interface::Time & time,
  const tf2_geometry_msgs::Transform & self_transform) const
{
  int label=object.classification()[0].label();
  // const std::uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (tracker_map_.count(label) != 0) {
    const auto tracker = tracker_map_.at(label);

    if (tracker == "bicycle_tracker") {
      return std::make_shared<BicycleTracker>(time, object, self_transform);
    } else if (tracker == "big_vehicle_tracker") {
      return std::make_shared<BigVehicleTracker>(time, object, self_transform);
    } else if (tracker == "multi_vehicle_tracker") {
      return std::make_shared<MultipleVehicleTracker>(time, object, self_transform);
    } else if (tracker == "normal_vehicle_tracker") {
      return std::make_shared<NormalVehicleTracker>(time, object, self_transform);
    } else if (tracker == "pass_through_tracker") {
      return std::make_shared<PassThroughTracker>(time, object, self_transform);
    } else if (tracker == "pedestrian_and_bicycle_tracker") {
      return std::make_shared<PedestrianAndBicycleTracker>(time, object, self_transform);
    } else if (tracker == "pedestrian_tracker") {
      return std::make_shared<PedestrianTracker>(time, object, self_transform);
    }
  }
  return std::make_shared<UnknownTracker>(time, object, self_transform);
}


void MultiObjectTracker::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const motion_manager::interface::Time & time,
  [[maybe_unused]] const tf2_geometry_msgs::Transform & self_transform)
{
  /* params */
  constexpr float max_elapsed_time = 0;

  /* delete tracker */
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    const bool is_old = max_elapsed_time < (*itr)->getElapsedTimeFromLastUpdate(time);
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker.erase(erase_itr);
    }
  }
}

void MultiObjectTracker::sanitizeTracker(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const motion_manager::interface::Time & time)
{
  constexpr float min_iou = 0.0;
  constexpr float min_iou_for_unknown_object = 0.001;
  constexpr double distance_threshold = 20.0;
  /* delete collision tracker */
  for (auto itr1 = list_tracker.begin(); itr1 != list_tracker.end(); ++itr1) {
    motion_manager::interface::TrackedObject object1;
    (*itr1)->getTrackedObject(time, object1);
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      motion_manager::interface::TrackedObject object2;
      (*itr2)->getTrackedObject(time, object2);
      const double distance = std::hypot(
        object1.kinematics().pose_with_covariance().pose().position().x() -
          object2.kinematics().pose_with_covariance().pose().position().x(),
        object1.kinematics().pose_with_covariance().pose().position().y() -
          object2.kinematics().pose_with_covariance().pose().position().y());
      if (distance_threshold < distance) {
        continue;
      }

      const double min_union_iou_area = 1e-2;
      const auto iou = object_recognition_utils::get2dIoU(object1, object2, min_union_iou_area);
      const auto & label1 = (*itr1)->getHighestProbLabel();
      const auto & label2 = (*itr2)->getHighestProbLabel();
      bool should_delete_tracker1 = false;
      bool should_delete_tracker2 = false;

      // If at least one of them is UNKNOWN, delete the one with lower IOU. Because the UNKNOWN
      // objects are not reliable.
      if (label1 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN || label2 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
        if (min_iou_for_unknown_object < iou) {
          if (label1 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN && label2 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
            if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          } else if (label1 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
            should_delete_tracker1 = true;
          } else if (label2 == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
            should_delete_tracker2 = true;
          }
        }
      } else {  // If neither is UNKNOWN, delete the one with lower IOU.
        if (min_iou < iou) {
          if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
            should_delete_tracker1 = true;
          } else {
            should_delete_tracker2 = true;
          }
        }
      }

      if (should_delete_tracker1) {
        itr1 = list_tracker.erase(itr1);
        --itr1;
        break;
      } else if (should_delete_tracker2) {
        itr2 = list_tracker.erase(itr2);
        --itr2;
      }
    }
  }
}



inline bool MultiObjectTracker::shouldTrackerPublish(
  const std::shared_ptr<const Tracker> tracker) const
{
  constexpr int measurement_count_threshold = 1;
  // std::cout << " track num : " << tracker->getTotalMeasurementCount() <<"\n";
  if (tracker->getTotalMeasurementCount() < measurement_count_threshold)
  {
    return false;
  }
  return true;
}





