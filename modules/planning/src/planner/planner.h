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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/planning_analysis.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/trajectory_array.hpp"
#include "modules/common/status/status.h"
#include "modules/planning/src/common/frame.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Planner {
 public:
  Planner() = default;

  virtual ~Planner() = default;

  virtual legionclaw::common::Status Init(const PlanningConf *planning_conf) = 0;

  virtual std::string Name() const = 0;

  virtual bool Reset() = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual legionclaw::common::Status Plan(Frame *frame,
                                     legionclaw::interface::ADCTrajectory *ptr_adc_trajectory,
                                     legionclaw::interface::PlanningAnalysis *ptr_analysis,
                                     legionclaw::interface::PlanningCmd *ptr_planning_cmd,
                                     legionclaw::interface::TrajectoryArray *ptr_trajectory_array) = 0;

  virtual void Stop() = 0;
};
}  // namespace planning
}  // namespace legionclaw
