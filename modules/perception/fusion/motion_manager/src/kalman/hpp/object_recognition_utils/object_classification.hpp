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

#ifndef OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_
#define OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_

#include "common/interface/object_classification.hpp"
#include "common/enum/enum.h"

#include <string>
#include <vector>

namespace object_recognition_utils
{
using motion_manager::interface::ObjectClassification;

inline ObjectClassification getHighestProbClassification(
  const std::vector<ObjectClassification> & object_classifications)
{
  if (object_classifications.empty()) {
    return ObjectClassification{};
  }
  return *std::max_element(
    std::begin(object_classifications), std::end(object_classifications),
    [](const auto & a, const auto & b) { return a.probability() < b.probability(); });
}

inline std::uint8_t getHighestProbLabel(
  const std::vector<ObjectClassification> & object_classifications)
{
  auto classification = getHighestProbClassification(object_classifications);
  return classification.label();
}

inline bool isVehicle(const uint8_t label)
{
  return label == motion_manager::common::DetectedObjectLabel::DO_BICYCLE || label == motion_manager::common::DetectedObjectLabel::DO_BUS ||
         label == motion_manager::common::DetectedObjectLabel::DO_CAR || label == motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE ||
         label == motion_manager::common::DetectedObjectLabel::DO_TRAILER || label == motion_manager::common::DetectedObjectLabel::DO_TRUCK;
}

inline bool isVehicle(const ObjectClassification & object_classification)
{
  return isVehicle(object_classification.label());
}

inline bool isVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isVehicle(highest_prob_label);
}

inline bool isCarLikeVehicle(const uint8_t label)
{
  return label == motion_manager::common::DetectedObjectLabel::DO_BUS || label == motion_manager::common::DetectedObjectLabel::DO_CAR ||
         label == motion_manager::common::DetectedObjectLabel::DO_TRAILER || label == motion_manager::common::DetectedObjectLabel::DO_TRUCK;
}

inline bool isCarLikeVehicle(const ObjectClassification & object_classification)
{
  return isCarLikeVehicle(object_classification.label());
}

inline bool isCarLikeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isCarLikeVehicle(highest_prob_label);
}

inline bool isLargeVehicle(const uint8_t label)
{
  return label == motion_manager::common::DetectedObjectLabel::DO_BUS || label == motion_manager::common::DetectedObjectLabel::DO_TRAILER ||
         label == motion_manager::common::DetectedObjectLabel::DO_TRUCK;
}

inline bool isLargeVehicle(const ObjectClassification & object_classification)
{
  return isLargeVehicle(object_classification.label());
}

inline bool isLargeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isLargeVehicle(highest_prob_label);
}

inline motion_manager::common::DetectedObjectLabel toLabel(const std::string & class_name)
{
  if (class_name == "UNKNOWN") {
    return motion_manager::common::DetectedObjectLabel::DO_UNKNOWN;
  } else if (class_name == "CAR") {
    return motion_manager::common::DetectedObjectLabel::DO_CAR;
  } else if (class_name == "TRUCK") {
    return motion_manager::common::DetectedObjectLabel::DO_TRUCK;
  } else if (class_name == "BUS") {
    return motion_manager::common::DetectedObjectLabel::DO_BUS;
  } else if (class_name == "TRAILER") {
    return motion_manager::common::DetectedObjectLabel::DO_TRAILER;
  } else if (class_name == "MOTORCYCLE") {
    return motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE;
  } else if (class_name == "BICYCLE") {
    return motion_manager::common::DetectedObjectLabel::DO_BICYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return motion_manager::common::DetectedObjectLabel::DO_PEDESTRIAN;
  } else {
    throw std::runtime_error("Invalid Classification label.");
  }
}

inline ObjectClassification toObjectClassification(
  const std::string & class_name, float probability)
{
  ObjectClassification classification;
  classification.set_label(toLabel(class_name));
  classification.set_probability(probability);
  return classification;
}

inline std::vector<ObjectClassification> toObjectClassifications(
  const std::string & class_name, float probability)
{
  std::vector<ObjectClassification> classifications;
  classifications.push_back(toObjectClassification(class_name, probability));
  return classifications;
}

inline std::string convertLabelToString(const uint8_t label)
{
  if (label == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN) {
    return "UNKNOWN";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_CAR) {
    return "CAR";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_TRUCK) {
    return "TRUCK";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_BUS) {
    return "BUS";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_TRAILER) {
    return "TRAILER";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE) {
    return "MOTORCYCLE";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_BICYCLE) {
    return "BICYCLE";
  } else if (label == motion_manager::common::DetectedObjectLabel::DO_PEDESTRIAN) {
    return "PEDESTRIAN";
  } else {
    return "UNKNOWN";
  }
}

inline std::string convertLabelToString(const ObjectClassification object_classification)
{
  return convertLabelToString(object_classification.label());
}

inline std::string convertLabelToString(
  const std::vector<ObjectClassification> object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return convertLabelToString(highest_prob_label);
}

}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_
