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
 * @brief This file provides the declaration of the class "LatticePlanner".
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

// #include "modules/common/proto/pnc_point.pb.h"
// #include "modules/common/status/status.h"
// #include "modules/common/util/factory.h"
// #include "modules/planning/common/reference_line_info.h"
// #include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
// #include "modules/planning/lattice/decider/lattice_task.h"
// #include "modules/planning/planner/planner.h"
// #include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/proto/planning_config.pb.h"
// #include "modules/planning/reference_line/reference_line.h"
// #include "modules/planning/reference_line/reference_point.h"

#include "modules/common/enum/enum.h"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/path.hpp"
#include "modules/common/math/math_utils.h"
#include "modules/planning/src/common/local_view.h"
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"
#include "modules/planning/src/planner/lattice_planner/trajectory_evaluator/trajectory_evaluator.h"
#include "modules/planning/src/planner/lattice_planner/trajectory_generator/trajectory_generator.h"
#include "modules/planning/src/planner/planner.h"

// using legionclaw::planning::Planning;
/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

using namespace legionclaw::common;
// class Planning;

/**
 * @class LatticePlanner
 * @brief LatticePlanner is a planner based on
 */
class LatticePlanner : public Planner {
 public:
  LatticePlanner() = default;

  virtual ~LatticePlanner() = default;

  std::string Name() const override { return "LATTICE"; }

  common::Status Init(const legionclaw::planning::PlanningConf *planning_conf) override;

  bool Reset() override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(Frame *frame, legionclaw::interface::ADCTrajectory *ptr_adc_trajectory,
                      legionclaw::interface::PlanningAnalysis *ptr_analysis,
                      legionclaw::interface::PlanningCmd *ptr_planning_cmd,
                      legionclaw::interface::TrajectoryArray *ptr_trajectory_array) override;

  int JudgeReplanFlag(const LocalView &local_view);

  int GetADCTrajectory(const PlanningTrajectory &trajectory, const BehaviourState &behaviour,
                       legionclaw::interface::ADCTrajectory *ptr_adc_trajectory);

  void SetTrajectoryArray(const vector<vector<PlanningTrajectory>> &trajectory_array,
                          legionclaw::interface::TrajectoryArray *ptr_trajectory_array);

  void Stop() override {}

 private:
  const planning::PlanningConf* planning_conf_;  // 参数

  Frame frame_;
  bool is_need_replan_ = false;
  bool is_new_goal_ = false;
  TrajectoryGenerator trajectory_generator_;  // 轨迹生成器
  TrajectoryEvaluator trajectory_evaluator_;  // 轨迹评估器
  BehaviourSelector behaviour_selector_;      // 行为选择器
};

}  // namespace planning
}  // namespace legionclaw
