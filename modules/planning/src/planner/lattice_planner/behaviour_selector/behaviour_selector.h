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
 * @brief This file provides the declaration of the class "BehaviourSelector".
 */

#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/path.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/stop_point.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/state_machine/state_context.hpp"
#include "modules/common/status/status.h"
#include "modules/planning/src/common/calculated_conditions.h"
#include "modules/planning/src/common/frame.h"
#include "modules/planning/src/common/math/include/spline/cubic_spline.h"
#include "modules/planning/src/common/planning_gflags.h"
#include "modules/planning/src/common/reference_line/reference_line.h"
#include "modules/planning/src/proto/driving/lattice_planner_conf.pb.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/planning/src/interface/planning_trajectory.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

using namespace legionclaw::reference_line;
using namespace legionclaw::common;
using namespace legionclaw::interface;
using namespace std;

/**
 * @class  BehaviourSelector
 * @brief
 */
class BehaviourSelector {
 public:
  BehaviourSelector() = default;

  virtual ~BehaviourSelector() = default;

  /********************************
   * Init Funtion
   ********************************/
  /**
   * @brief . 初始化函数
   * @param .
   * @param .
   * @return .
   */
  void Init(const legionclaw::planning::PlanningConf *planning_conf);

  /**
   * @brief   轨迹选择器入口功能函数.
   * @param[in]		&trajectory_array 轨迹评估器产生的信息.
   * @param[in]   &frame &is_new_goal.
   * @return    Status.
   */
  Status SelectBehaviour(const vector<vector<PlanningTrajectory>> &trajectory_array, Frame *frame,
                         bool &is_new_goal);

 private:
  /**
   * @brief   参数初始化函数.
   * @param .
   * @param .
   * @return void.
   */
  void BehaviourParamsInit(const legionclaw::planning::PlanningConf *planning_conf);

  /**
   * @brief   全局变量初始化函数.
   * @param .
   * @param .
   * @return void.
   */
  void BehaviourVariableInit();

  /********************************
   * Behaviour Application Funtion
   ********************************/

  /**
   * @brief   根据evaluator的结果trajectory_array_，
   *          选择出cost最小的轨迹optimal_trajectory_.
   * @param .
   * @param .
   * @return .
   */
  void SelectMinCostTrajectory();

  /**
   * @brief .
   * @param .
   * @param .
   * @return .
   */
  PlanningTrajectory FindStoreTrajectory(const std::pair<int, int> &center_index);

  /**
   * @brief   选择目标轨迹.
   * @return .
   */
  void SelectTargetTrajectory(Frame *frame);

  /**
   * @brief   更新目标速度.
   * @return .
   */
  void UpdateTargetSpeed();

  /**
   * @brief   存储备选轨迹.
   * @return .
   */
  void SelectStoreTrajectory();

  /**
   * @brief   更新交通信号并根据交通信号计算停车距离.
   * @return .
   */
  void UpdateTrafficSignalAndStopDis();

  /**
   * @brief   更新汇出过程不同状态
   * @return  .
   */
  void UpdateInwardOutwardStatus();

  /**
   * @brief   计算最终选择的轨迹起始点和终点距离车当前车道中心线的横向偏差.
   * @return .
   */
  void CalculateLatOffset();

  /**
   * @brief   更新最近障碍物信息.
   * @return .
   */
  void UpdateClosestObjInfo();

  /**
   * @brief   计算决策条件参数.
   * @param .
   * @param .
   * @return .
   */
  void CalculateImportantParameterForDecisionMaking();

 public:
  /**
   * @brief . 调用横纵向状态机，获取行为状态
   * @param .
   * @param .
   * @return .
   */
  void GenerateBehaviorState();

  /**
   * @brief . 计算规划速度和目标加速度信息
   * @param .
   * @param .
   * @return .
   */
  double UpdatePlanSpeedAndTargetAccel();

  /**
   * @brief . 以轨迹为尺度计算站点的距离
   * @param .
   * @param .
   * @return .
   */
  bool ComputeGoalDistanceOnTrajectory(const std::vector<TrajectoryPoint> &trajectory,
                                       const math::Vec2d &position, const double yaw,
                                       double &goal_distance);

  /**
   * @brief . 轨迹曲率限制速度
   * @param .
   * @param .
   * @return .
   */
  double ComputeLimitSpeedByKappa(const std::vector<TrajectoryPoint> &trajectory,
                                  const double &cur_speed, const double &search_length,
                                  const double &limit_lat_acc, const double &min_tar_speed);

  /**
   * @brief . 计算轨迹速度、挡位信息，时间采样抽稀
   * @param .
   * @param .
   * @return .
   */
  bool ComputeTrajectoryLongitudinalInfo(const double &start_speed, const double &accel,
                                         const double &limit_speed,
                                         std::vector<TrajectoryPoint> &trajectory);

  /**
   * @brief . 计算轨迹纵向信息
   * @param .
   * @param .
   * @return .
   */
  bool ComputeLongitudinalInfoSpline(const double &current_velocity, const double &current_acc,
                                     const double &tar_length, const double &tar_speed,
                                     const double &limit_speed,
                                     std::vector<TrajectoryPoint> &trajectory);

  /**
   * @brief . ACC计算：目标速度、目标距离、目标加速度
   * @param .
   * @param .
   * @return .
   */
  bool AdaptiveCruiseControl(const double &final_length, const double &delta_length,
                             const double &car_speed, const double &obj_speed, const double &l_min,
                             const double &h, const double &k1, const double &k2,
                             const double &t_pre, double &motion_speed, double &motion_length,
                             double &motion_accel);

  /**
   * @brief 计算本车道绕障、跨车道借道时，切换绕行轨迹时车辆距最近障碍物的距离
   * @param closest_obj_velocity 障碍物速度 引用
   * @param vehicle_velocity 自动驾驶车辆速度 引用
   * @return avoidance_distance 触发距离
   */
  double ComputeAvoidanceDistance(const double &closest_obj_velocity,
                                  const double &vehicle_velocity);

  /**
   * @brief . 车辆灯光控制
   * @param .
   * @param .
   * @return .
   */
  void SetTrunLamp(const std::vector<TrajectoryPoint> &trajectory);

  /********************************
   * State Machine Callback Funtion
   ********************************/
  void StateMachineSpin(void *param);
  /**
   * @brief . 状态机初始化函数
   * @param .
   * @param .
   * @return .
   */
  void BehaviorStateMachineInit();
  bool StateMachineReset();
  //
  void LatNotActiveStateUpdate(const std::string &state_name, int state);
  void ForwardStateUpdate(const std::string &state_name, int state);
  void StationStateUpdate(const std::string &state_name, int state);
  void StationStopStateUpdate(const std::string &state_name, int state);
  void StationWaitStateUpdate(const std::string &state_name, int state);
  void StationArrivedStateUpdate(const std::string &state_name, int state);
  void ChangeLaneStateUpdate(const std::string &state_name, int state);
  void ChangeLaneLeftPreStateUpdate(const std::string &state_name, int state);
  void ChangeLaneRightPreStateUpdate(const std::string &state_name, int state);
  void ChangeLaneLeftActStateUpdate(const std::string &state_name, int state);
  void ChangeLaneRightActStateUpdate(const std::string &state_name, int state);
  //
  void AvoidanceStateUpdate(const std::string &state_name, int state);
  void AvoidancePreStateUpdate(const std::string &state_name, int state);
  void AvoidanceKeepStateUpdate(const std::string &state_name, int state);
  void AvoidanceBackPreStateUpdate(const std::string &state_name, int state);
  void AvoidanceActStateUpdate(const std::string &state_name, int state);
  void AvoidanceBackActStateUpdate(const std::string &state_name, int state);
  //
  void LonNotActiveStateUpdate(const std::string &state_name, int state);
  void NormalStateUpdate(const std::string &state_name, int state);
  void PreciseStopStateUpdate(const std::string &state_name, int state);
  void StopStateUpdate(const std::string &state_name, int state);
  //
  void TrafficSignStopStateUpdate(const std::string &state_name, int state);
  void TrafficSignWaitStateUpdate(const std::string &state_name, int state);
  void TrafficLightStopStateUpdate(const std::string &state_name, int state);
  void TrafficLightWaitStateUpdate(const std::string &state_name, int state);
  //
  void ContinueStateUpdate(const std::string &state_name, int state);
  void FollowStateUpdate(const std::string &state_name, int state);
  void ACCStateUpdate(const std::string &state_name, int state);
  void TJPStateUpdate(const std::string &state_name, int state);
  void CutInStateUpdate(const std::string &state_name, int state);
  void OnComingStateUpdate(const std::string &state_name, int state);
  void CrossingStateUpdate(const std::string &state_name, int state);
  // TODO汇入汇出
  void InwardStateUpdate(const std::string &state_name, int state);
  void OutwardStateUpdate(const std::string &state_name, int state);

  void ResetTimer() { state_timer_ = TimeTool::NowToSeconds(); }  // TODO:check it

  double DistStopToNormal()
  {
    return 0.5 * common_params()->norm_acceleration() *
               pow(selector_params().stop_to_acceleration_min_time(), 2) +
           selector_params().additional_braking_distance();
  }
  //
  bool GoalDistanceLess()
  {
    // TODO:优化条件
    if (ptr_conditions_->goal_distance_ > 0.0 &&
        ptr_conditions_->goal_distance_ < ptr_conditions_->min_stopping_distance_) return true;
    return false;
  }

  bool StationGoalDistanceLess()
  {
    // TODO:优化条件
    if (ptr_conditions_->goal_distance_ > 0.0 &&
        ptr_conditions_->goal_distance_ < selector_params().distance_to_goal() &&
        ptr_conditions_->goal_distance_ < ptr_conditions_->min_stopping_distance_)
      return true;
    return false;
  }
  //
  bool IsSameLane()
  {
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) == 0)
      return true;
    return false;
  }
  //
  bool AvoidanceToForwardState()
  {
    if (!common_params()->enable_swerving()) {
      // ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (!IsSameLane()) {
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) >= selector_params().threshold_lat_offset()) {
      return false;
    }
    // if (fabs(ptr_conditions_->start_lat_offset_) >=
    //     selector_params().threshold_lat_offset()) {
    //   return false;
    // }
    return true;
  }
  //
  bool AvoidanceToChangeLaneLeftState()
  {
    if (!common_params()->enable_swerving()) {  // TODO:配置
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) >= 0) {
      // ADEBUG << "Vehicle  ";
      return false;
    }
    return true;
  }
  //
  bool AvoidanceToChangeLaneRightState()
  {
    if (!common_params()->enable_swerving()) {
      // ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) <= 0) {
      // ADEBUG << "Vehicle  ";
      return false;
    }
    return true;
  }
  //
  bool ChangeToGoal()
  {
    if (ptr_conditions_->goal_distance_trajectory_ < selector_params().goal_min_distance()) {
      return true;
    }
    if (ptr_conditions_->goal_distance_trajectory_ < selector_params().goal_max_distance() &&
        ptr_conditions_->is_static_) {
      return true;
    }
    return false;
  }

  bool StationArrived()
  {
    if (ptr_conditions_->goal_distance_trajectory_ < selector_params().goal_max_distance() &&
        ptr_conditions_->is_static_) {
      return true;
    }
    return false;
  }

  /********************************
   * Return Funtion
   ********************************/
  CalculatedConditions *GetConditions()
  {
    if (ptr_conditions_ == nullptr) ptr_conditions_ = new CalculatedConditions();
    return ptr_conditions_;
  }
  PlanningTrajectory FinalTrajectory() { return final_trajectory_; }
  BehaviourState CurrentBehaviour() { return current_behaviour_; }
  std::vector<std::vector<PlanningTrajectory>> TrajectoryArray() { return trajectory_array_; }
  legionclaw::interface::PlanningCmd PlanningCommand() { return planning_cmd_; }
  //
  const legionclaw::planning::PlanningConf *common_params() const { return planning_params_; }
  BehaviourSelectorConf selector_params() const { return selector_params_; }

 protected:
  CalculatedConditions *ptr_conditions_ = GetConditions();
  SelectorLatFlag lat_state_;
  SelectorLonFlag lon_state_;
  BehaviourState current_behaviour_;  // 最终决策的行为
  std::shared_ptr<state_machine::StateContext> behaviour_lat_state_sm_;
  std::shared_ptr<state_machine::StateContext> behaviour_lon_state_sm_;

  double state_timer_;
  double decision_making_time_;
  int decision_making_count_;
  double pre_acc_to_tjp_time_;

 private:
  legionclaw::interface::VehicleParam vehicle_param_;  // 车辆参数
  const legionclaw::planning::PlanningConf *planning_params_;                  // planning 参数类
  BehaviourSelectorConf selector_params_;         // selector参数类

  Frame frame_;
  StopPoint terminal_stop_point_;
  PlanningTrajectory final_trajectory_;
  PlanningTrajectory store_trajectory_;
  PlanningTrajectory optimal_trajectory_;
  PlanningTrajectory min_cost_trajectory_;
  PlanningTrajectory target_trajectory_;
  std::vector<std::vector<PlanningTrajectory>> trajectory_array_;
  legionclaw::interface::PlanningCmd planning_cmd_;

  bool is_new_goal_;
  bool lc_dis_complete_ = false;
  bool lc_displacement_state_ = false;
  int replan_flag_;
  int origin_lane_id_;
  double ahead_map_limit_speed_ = 0.0;  // 前瞻地图限速
  double tar_accel_ = 0.0;
  bool stop_earlier_ = false;
  double current_velocity_ = 0.0;
  double target_velocity_ = 0.0;
  double THRESHOLD_CURVATURE_FOR_TURN = 0.03;  // 判断转向的曲率阈值，TODO:配置
  double remain_dist_to_change_left_ = 0.0;
  double remain_dist_to_change_right_ = 0.0;
  double relative_length_smooth_ = 9999;
};

}  // namespace planning
}  // namespace legionclaw
