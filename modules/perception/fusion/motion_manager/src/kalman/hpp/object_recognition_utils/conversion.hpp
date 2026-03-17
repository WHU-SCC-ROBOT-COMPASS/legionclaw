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

#ifndef OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
#define OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_

#include "common/interface/detected_objects.hpp"
#include "common/interface/tracked_objects.hpp"

namespace object_recognition_utils
{
using motion_manager::interface::DetectedObject;
using motion_manager::interface::DetectedObjects;
using motion_manager::interface::TrackedObject;
using motion_manager::interface::TrackedObjects;

DetectedObject toDetectedObject(const TrackedObject & tracked_object);
DetectedObjects toDetectedObjects(const TrackedObjects & tracked_objects);
TrackedObject toTrackedObject(const DetectedObject & detected_object);
TrackedObjects toTrackedObjects(const DetectedObjects & detected_objects);
}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
