/******************************************************************************
 * Copyright 2018 The LegionClaw Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief This file provides the implementation of the class "ParkingPlanner".
 */
#include "modules/planning/src/planner/parking_planner/parking_planner.h"

#include <memory>
#include <utility>

#include "modules/planning/src/common/planning_gflags.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

namespace {
constexpr uint32_t KDestLanePriority = 0;
constexpr double kPathOptimizationFallbackClost = 2e4;
constexpr double kSpeedOptimizationFallbackClost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

legionclaw::common::Status ParkingPlanner::Init(const legionclaw::planning::PlanningConf *planning_conf)
{
  parking_manager_.Init(planning_conf);
  return Status::Ok();
}

bool ParkingPlanner::Reset() { return parking_manager_.Reset(); }

legionclaw::common::Status ParkingPlanner::IsParkingOk(Frame *frame)
{
  return parking_manager_.IsGeometryOK(frame);
}

legionclaw::common::Status ParkingPlanner::Plan(Frame *frame,
                                           legionclaw::interface::ADCTrajectory *ptr_adc_trajectory,
                                           legionclaw::interface::PlanningAnalysis *ptr_analysis,
                                           legionclaw::interface::PlanningCmd *ptr_planning_cmd,
                                           legionclaw::interface::TrajectoryArray *ptr_trajectory_array)
{
  return parking_manager_.Process(frame, ptr_adc_trajectory, ptr_analysis);
}
}  // namespace planning
}  // namespace legionclaw
