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
 * @brief This file provides the implementation of the class "
 * BehaviourSelector".
 */
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"

#include <functional>
#include <iostream>
#include <memory>
#include <utility>

#include "modules/planning/src/common/map_matcher/map_matcher.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
using namespace std;

void BehaviourSelector::Init(const legionclaw::planning::PlanningConf *planning_conf)
{
  BehaviourParamsInit(planning_conf);
  BehaviourVariableInit();
  BehaviorStateMachineInit();
}

void BehaviourSelector::BehaviourParamsInit(const legionclaw::planning::PlanningConf *planning_conf)
{
  if (planning_conf == nullptr) return;
  vehicle_param_ = planning_conf->vehicle_param();
  planning_params_ = planning_conf;
  selector_params_ = planning_conf->lattice_planner_conf().behaviour_selector_conf();
}

void BehaviourSelector::BehaviourVariableInit()
{
  state_timer_ = TimeTool::NowToSeconds();
  decision_making_time_ = selector_params_.decision_making_time();
  decision_making_count_ = 0;
  pre_acc_to_tjp_time_ = TimeTool::NowToSeconds();
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  lon_state_ = SelectorLonFlag::LON_INVALID;
  ptr_conditions_->Init();
  is_new_goal_ = false;
  origin_lane_id_ = -1;
  replan_flag_ = 0;
  ahead_map_limit_speed_ = 0.0;
  tar_accel_ = 0.0;
  stop_earlier_ = false;
  current_velocity_ = 0.0;
  target_velocity_ = 0.0;
  THRESHOLD_CURVATURE_FOR_TURN = selector_params_.turn_lamp_kappa();
  relative_length_smooth_ = 9999;
  planning_cmd_ = PlanningCmd();
  final_trajectory_ = PlanningTrajectory();
}

Status BehaviourSelector::SelectBehaviour(
    const vector<vector<PlanningTrajectory>> &trajectory_array, Frame *frame, bool &is_new_goal)
{
  int count = 0;
  for (unsigned int i = 0; i < trajectory_array.size(); i++) {
    if (trajectory_array.at(i).size() > 0) {
      count += 1;
    }
  }
  if (frame->ReferenceLines().size() == 0 || frame->MapMatchInfo().current_lane_id < 0 ||
      frame->MapMatchInfo().lon_index < 0 || (count == 0))
    return Status(Status::ErrorCode::PLANNING_ERROR, "Input error.");

  trajectory_array_ = trajectory_array;
  frame_ = *frame;
  is_new_goal_ = is_new_goal;
  is_new_goal = false;
  terminal_stop_point_ = frame_.terminal_stop_point();

  // 获取目标轨迹
  SelectTargetTrajectory(frame);

  // 计算决策参数
  CalculateImportantParameterForDecisionMaking();

  UpdateTargetSpeed();
  // 根据车辆状态，获取决策状态
  GenerateBehaviorState();
  // 针对轨迹，进行速度规划计算
  double plan_speed = UpdatePlanSpeedAndTargetAccel();
  // 纵向速度赋值
  ComputeTrajectoryLongitudinalInfo(plan_speed, tar_accel_, target_velocity_,
                                    *final_trajectory_.mutable_trajectory_points());
  // 设置最终选择的轨迹为last_trajectory为下一次plan做准备
  frame->SetLastPlanningTrajectory(final_trajectory_);
  // 设置上一次决策行为，目前还没用
  frame->SetLastBehaviour(current_behaviour_);
  frame->SetAheadMapLimitSpeed(ahead_map_limit_speed_);
  // 完成到站动作，清除对应站点信息
  //  if (current_behaviour_.lat_state ==
  //      legionclaw::interface::ADCTrajectory::BehaviourLatState::
  //          STATION_ARRIVED_STATE &&
  //          frame_.VehicleState().gear ==
  //          legionclaw::common::GearPosition::GEAR_PARKING) {
  //    frame->SetTerminalType(-1);
  //  }

  return Status(Status::ErrorCode::OK);
}

void BehaviourSelector::SelectMinCostTrajectory()
{
  double min_cost = DBL_MAX;
  unsigned int i = 0;
  PlanningTrajectory mid_trajectory;
  for (auto paths : trajectory_array_) {
    unsigned int j = 0;
    for (auto path : paths) {
      if (i == 1 && trajectory_array_.at(i).at(j).lon_id() == 0 &&
          trajectory_array_.at(i).at(j).lat_id() == 0) {
        mid_trajectory = trajectory_array_.at(i).at(j);
      }
      if (min_cost > path.trajectory_cost().cost) {
        min_cost = path.trajectory_cost().cost;
        min_cost_trajectory_ = trajectory_array_.at(i).at(j);
        optimal_trajectory_ = trajectory_array_.at(i).at(j);
      }
      j++;
    }
    i++;
  }
  // TODO需优化:若当前车道较近处有静止障碍物，其它车道较远处有静止障碍物，针对该场景，目前没办法连续让障
  // 检查min cost轨迹是否有静态障碍物碰撞，有，则选择当前车道中间的轨迹
  if (optimal_trajectory_.trajectory_cost().blocked_cost == 1000 ||
      optimal_trajectory_.trajectory_cost().is_passage_safe_f == false ||
      optimal_trajectory_.trajectory_cost().is_passage_safe_b == false) {
    optimal_trajectory_ = mid_trajectory;
  }
  return;
}

PlanningTrajectory BehaviourSelector::FindStoreTrajectory(const std::pair<int, int> &center_index)
{
  PlanningTrajectory trajectory;
  bool index_available = false;
  if (frame_.LastPlanningTrajectory().trajectory_points_size() > 0 &&
      trajectory_array_.at(frame_.LastPlanningTrajectory().global_index()).size() > 0) {
    for (auto path : trajectory_array_.at(frame_.LastPlanningTrajectory().global_index())) {
      if (path.lat_id() == frame_.LastPlanningTrajectory().lat_id() &&
          path.lon_id() == frame_.LastPlanningTrajectory().lon_id()) {
        index_available = true;
        trajectory = path;
        break;
      }
    }
  } else {  // 有轨迹簇生成时，使用中心轨迹ptr_conditions->central_trajectory_index_
    trajectory = trajectory_array_.at(center_index.first).at(center_index.second);
  }

  if (!index_available) {
    trajectory = trajectory_array_.at(center_index.first).at(center_index.second);
  }

  return trajectory;
}

void BehaviourSelector::SelectTargetTrajectory(Frame *frame)
{
  CalculatedConditions *ptr_conditions = GetConditions();
  ptr_conditions->stop_to_wait_change_lane_ = false;
  int current_lane_index = frame->MapMatchInfo().current_lane_index;
  // 换道中
  bool is_lane_changing_exit = false;
  if (current_behaviour_.lat_state ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE ||
      current_behaviour_.lat_state ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
    SelectMinCostTrajectory();
    if (current_behaviour_.lat_state ==
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE) {
      // 非最优轨迹，只针对后方来车场景
      if (optimal_trajectory_.global_index() != frame->LastPlanningTrajectory().global_index() &&
          trajectory_array_.at(frame->LastPlanningTrajectory().global_index())
                  .front()
                  .trajectory_cost()
                  .is_passage_safe_b == false) {
        // 计算侵占程度
        auto reference_point =
            frame->ReferenceLine(1).ReferencePoints(frame->MapMatchInfo().lon_index);
        double angle_diff =
            NormalizeAngle(frame->VehicleState().pose.theta() - reference_point.theta());
        double lane_occupancy = frame->MapMatchInfo().lat_offset +
                                vehicle_param_.front_edge_to_center() * sin(angle_diff) +
                                0.5 * vehicle_param_.width() * cos(angle_diff);
        // case1:侵占程度很小,取消换道
        if (lane_occupancy <
            reference_point.left_road_width() - planning_params_->safe_width_lower_limit()) {
          lc_dis_complete_ = true;
        } else if (lane_occupancy <
                   reference_point.left_road_width() - 0.1) {  // case2:侵占程度未压线
          if (frame->VehicleState().speed < 1.5 &&
              optimal_trajectory_.trajectory_cost().cost >
                  trajectory_array_.at(frame->LastPlanningTrajectory().global_index())
                      .front()
                      .trajectory_cost()
                      .cost) {  // 车速慢，若本车道代价值大于换道代价值，且速度较慢，则停车让行继续等待换道时机
            ptr_conditions->stop_to_wait_change_lane_ = true;
          } else {  // 非最优/车速较快 换道撤回
            lc_dis_complete_ = true;
          }
        } else {  // case2:侵占程度压线，继续完成换道动作
        }
      }
      if (origin_lane_id_ - 1 == frame->MapMatchInfo().current_lane_id ||
          lc_dis_complete_ == true) {
        lc_dis_complete_ = false;
        lc_displacement_state_ = false;
        ptr_conditions->target_lane_index_ = current_lane_index;
        frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
        frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_COMPLETE);
        if (trajectory_array_.at(current_lane_index).size() > 0) {
          target_trajectory_ = trajectory_array_.at(current_lane_index).front();
          return;
        }
      } else {
        auto target_trajectorys =
            trajectory_array_.at(frame->LastPlanningTrajectory().global_index());
        if (target_trajectorys.size() > 0) {
          ptr_conditions->target_lane_index_ = frame->LastPlanningTrajectory().global_index();
          target_trajectory_ = target_trajectorys.front();
          return;
        } else {
          is_lane_changing_exit = true;
        }
      }
    } else if (current_behaviour_.lat_state ==
               legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
      // 非最优轨迹
      if (optimal_trajectory_.global_index() != frame->LastPlanningTrajectory().global_index() &&
          trajectory_array_.at(frame->LastPlanningTrajectory().global_index())
                  .front()
                  .trajectory_cost()
                  .is_passage_safe_b == false) {
        // 计算侵占程度
        auto reference_point =
            frame->ReferenceLine(1).ReferencePoints(frame->MapMatchInfo().lon_index);
        double angle_diff =
            NormalizeAngle(reference_point.theta() - frame->VehicleState().pose.theta());
        double lane_occupancy = -frame->MapMatchInfo().lat_offset +
                                vehicle_param_.front_edge_to_center() * sin(angle_diff) +
                                0.5 * vehicle_param_.width() * cos(angle_diff);
        // case1:侵占程度很小,取消换道
        if (lane_occupancy <
            reference_point.right_road_width() - planning_params_->safe_width_lower_limit()) {
          lc_dis_complete_ = true;
        } else if (lane_occupancy <
                   reference_point.right_road_width() - 0.1) {  // case2:侵占程度未压线
          if (frame->VehicleState().speed < 1.5 &&
              optimal_trajectory_.trajectory_cost().cost >
                  trajectory_array_.at(frame->LastPlanningTrajectory().global_index())
                      .front()
                      .trajectory_cost()
                      .cost) {  // 车速慢，若本车道代价值大于换道代价值，且速度较慢，则停车让行继续等待换道时机
            ptr_conditions->stop_to_wait_change_lane_ = true;
          } else {  // 非最优/车速较快 换道撤回
            lc_dis_complete_ = true;
          }
        } else {  // case2:侵占程度压线，继续完成换道动作
        }
      }
      if (origin_lane_id_ + 1 == frame->MapMatchInfo().current_lane_id ||
          lc_dis_complete_ == true) {
        lc_dis_complete_ = false;
        lc_displacement_state_ = false;
        ptr_conditions->target_lane_index_ = current_lane_index;
        frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
        frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_COMPLETE);
        if (trajectory_array_.at(current_lane_index).size() > 0) {
          target_trajectory_ = trajectory_array_.at(current_lane_index).front();
          return;
        }
      } else {
        auto target_trajectorys =
            trajectory_array_.at(frame->LastPlanningTrajectory().global_index());
        if (target_trajectorys.size() > 0) {
          ptr_conditions->target_lane_index_ = frame->LastPlanningTrajectory().global_index();
          target_trajectory_ = target_trajectorys.front();
          return;
        } else {
          is_lane_changing_exit = true;
        }
      }
    }
  }
  ptr_conditions->stop_to_wait_change_lane_ = false;

  // case1 :左拨杆
  if (frame->LaneChangeDirection() == common::Direction::LEFT) {
    auto target_trajectorys = trajectory_array_.at(current_lane_index - 1);
    if (target_trajectorys.size() > 0) {
      if (target_trajectorys.front().trajectory_cost().blocked_cost != 1000 &&
          target_trajectorys.front().trajectory_cost().is_passage_safe_dclc == true) {
        ptr_conditions->target_lane_index_ = current_lane_index - 1;
        target_trajectory_ = target_trajectorys.front();
        return;
      } else {
        // TODO 驳回event  设置成Direction invalid
        frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
        frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_INVALID);
        // AWARN << " Not Safe, Left Manual LC Reject !";
      }
    } else {
      // TODO 驳回event  设置成Direction invalid
      frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
      frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_INVALID);
      // AWARN << " No Left Trajectory, Left Manual LC Reject !";
    }
    ptr_conditions->target_lane_index_ = current_lane_index;
    if (trajectory_array_.at(current_lane_index).size() == 0) {
      AERROR << " No Target Trajectory Can Be Selected !";
    } else {
      target_trajectory_ = trajectory_array_.at(current_lane_index).at(0);
    }
    return;
  }

  // case2 :右拨杆
  if (frame->LaneChangeDirection() == common::Direction::RIGHT) {
    auto target_trajectorys = trajectory_array_.at(current_lane_index + 1);
    if (target_trajectorys.size() > 0) {
      if (target_trajectorys.front().trajectory_cost().blocked_cost != 1000 &&
          target_trajectorys.front().trajectory_cost().is_passage_safe_dclc == true) {
        ptr_conditions->target_lane_index_ = current_lane_index + 1;
        target_trajectory_ = target_trajectorys.front();
        return;
      } else {
        // 驳回event  设置成Direction invalid
        frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
        frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_INVALID);
        // AWARN << " Not Safe, Right Manual LC Reject !";
      }
    } else {
      // 驳回event  设置成Direction invalid
      frame->SetLaneChangeDirection(common::Direction::DIR_INVALID);
      frame->SetLaneChangeState(Lane_Change_State::LANE_CHANGE_INVALID);
      // AWARN << " No Right Trajectory, Right Manual LC Reject !";
    }
    ptr_conditions->target_lane_index_ = current_lane_index;
    if (trajectory_array_.at(current_lane_index).size() == 0) {
      AERROR << " No Target Trajectory Can Be Selected !";
    } else {
      target_trajectory_ = trajectory_array_.at(current_lane_index).at(0);
    }
    return;
  }

  if (is_lane_changing_exit == true) {
    // case :lKA
    ptr_conditions->target_lane_index_ = current_lane_index;
    if (trajectory_array_.at(current_lane_index).size() == 0) {
      target_trajectory_ = PlanningTrajectory();
      AERROR << " Current Lane No Target Trajectory Can Be Selected !";
    } else {
      // 每车道有轨迹，便仅有中间一条
      target_trajectory_ = trajectory_array_.at(current_lane_index).at(0);
      return;
    }
  }

  // case3 :自主
  if (frame->LaneChangingMode() == Lane_Changing_Mode::Autonomous) {
    // 根据evaluator的结果trajectory_array_，选择出cost最小的轨迹optimal_trajectory_
    SelectMinCostTrajectory();
    ptr_conditions->target_lane_index_ = optimal_trajectory_.global_index();
    target_trajectory_ = optimal_trajectory_;
    if (!optimal_trajectory_.has_trajectory_points()) {
      AERROR << " min_cost_trajectory  no  trajectory_points!!!!";
    }
    return;
  }

  // case :lKA
  ptr_conditions->target_lane_index_ = current_lane_index;
  if (trajectory_array_.at(current_lane_index).size() == 0) {
    target_trajectory_ = PlanningTrajectory();
    AERROR << " Current Lane No Target Trajectory Can Be Selected !";
  } else {
    // 筛选最中间的轨迹
    for (auto trajectory : trajectory_array_.at(current_lane_index)) {
      if (trajectory.lat_id() == 0) {
        target_trajectory_ = trajectory;
        break;
      }
      target_trajectory_ = trajectory;
    }
  }
  return;
}

void BehaviourSelector::UpdateTargetSpeed()
{
  // 计算地图限速
  auto reference_line = frame_.ReferenceLines().at(ptr_conditions_->target_lane_index_);
  int car_index = frame_.MapMatchInfo().match_index[ptr_conditions_->target_lane_index_];
  if (car_index < 0) {
    AERROR << "car_index is abnormal : " << car_index << endl;
    return;
  }
  double cur_map_limit_speed = reference_line.ReferencePoints(car_index).limit_speed();

  double ahead_length = cur_map_limit_speed * planning_params_->speedcoeff_collision_check() +
                        planning_params_->baselen_collison_check();
  // 搜索ahead_length区域内的最小速度，所以会提前减速，不会提前加速
  ahead_map_limit_speed_ =
      MapMatcher::GetVelocityAhead(reference_line.ReferencePoints(), car_index, ahead_length);
  // 配置文件限速，主要用于调试测试
  ahead_map_limit_speed_ = (ahead_map_limit_speed_ > vehicle_param_.speed_limit())
                               ? vehicle_param_.speed_limit()
                               : ahead_map_limit_speed_;
  // 动态限速
  legionclaw::interface::TrafficEvents traffic_events = frame_.GetTrafficEvents();
  if (traffic_events.limit_speed_info().limitspeed_valid_flag() == common::IsValid::VALID &&
      traffic_events.limit_speed_info().limit_speed() < ahead_map_limit_speed_) {
    ahead_map_limit_speed_ = traffic_events.limit_speed_info().limit_speed();
  }
  // 曲率限速只用它的减速度，避免出现速度阶跃较大产生的刹车
  double search_length = planning_params_->baselen_collison_check() +
                         current_velocity_ * planning_params_->speedcoeff_collision_check();
  double limit_speed_kappa = ComputeLimitSpeedByKappa(
      target_trajectory_.trajectory_points(), current_velocity_, search_length,
      selector_params().max_lat_acc(), selector_params().min_limit_speed());
  if (limit_speed_kappa < current_velocity_) {
    // target_velocity_ = limit_speed_kappa;
    ptr_conditions_->is_limit_speed_kappa_ = true;
  } else {
    target_velocity_ = ahead_map_limit_speed_;
    ptr_conditions_->is_limit_speed_kappa_ = false;
  }

  auto guide_info = frame_.GetGuideInfo();
  double intersection_limit_speed = selector_params().intersection_limit_speed();

  // 交叉路口左转/右转/掉头限速
  if (guide_info.current_road().road_type() == 2 &&
      (guide_info.current_road().turn_type() == 1 || guide_info.current_road().turn_type() == 3)) {
    if (intersection_limit_speed < target_velocity_) {
      target_velocity_ = intersection_limit_speed;
    }
  }

  // TODO 待优化 左转/右转/掉头提前减速
  if (guide_info.next_dis() <= selector_params().min_turn_distance() &&
      guide_info.current_road().road_type() != 1) {
    if (guide_info.current_road().turn_type() == 1 || guide_info.current_road().turn_type() == 3 ||
        guide_info.next_road().turn_type() == 1 || guide_info.next_road().turn_type() == 3) {
      double speed_t = sqrt(pow(intersection_limit_speed, 2) -
                            2 * guide_info.next_dis() * planning_params_->norm_deceleration());
      if (speed_t < target_velocity_) {
        target_velocity_ = speed_t;
      }
    }
  }

  // 轨迹长度过短，速度限制保护
  if (frame_.LastPlanningTrajectorySize() > 0) {
    double trajectory_length = frame_.LastTrajectoryPoints().back().path_point().s();
    if (trajectory_length <= selector_params().min_turn_distance()) {
      double length_limit_speed =
          sqrt(-2 * trajectory_length * planning_params_->norm_deceleration());
      if (length_limit_speed < target_velocity_) {
        target_velocity_ = length_limit_speed;
      }
    }
  }
  return;
}

void BehaviourSelector::SelectStoreTrajectory()
{
  CalculatedConditions *ptr_conditions = GetConditions();
  int max_lon_id = -9999;
  ptr_conditions->central_trajectory_index_ = make_pair(-1, -1);
  int path_index = 0;
  int trajectory_array_current_lane_size =
      trajectory_array_.at(frame_.MapMatchInfo().current_lane_index).size();
  if (trajectory_array_current_lane_size > 0) {
    for (auto cur_lane_path : trajectory_array_.at(frame_.MapMatchInfo().current_lane_index)) {
      if (cur_lane_path.lat_id() == 0 ||
          path_index == (trajectory_array_current_lane_size - 1))  // 选中间
      {
        if (cur_lane_path.lon_id() > max_lon_id)  // 选最长
        {
          max_lon_id = cur_lane_path.lon_id();
          ptr_conditions->central_trajectory_index_ =
              make_pair(frame_.MapMatchInfo().current_lane_index, path_index);
        }
      }
      path_index++;
    }
  } else {  // current_lane_id上没有轨迹簇生成时，使用最右边车道的中心轨迹
    for (auto cur_lane_path : trajectory_array_.back()) {
      if (cur_lane_path.lat_id() == 0)  // 选中间
      {
        if (cur_lane_path.lon_id() > max_lon_id)  // 选最长
        {
          max_lon_id = cur_lane_path.lon_id();
          ptr_conditions->central_trajectory_index_ =
              make_pair(trajectory_array_.size() - 1, path_index);
        }
      }
      path_index++;
    }
  }
  // 从轨迹簇中挑选出延续上帧轨迹id的轨迹，并更新该轨迹对应最近障碍物信息
  if (ptr_conditions->central_trajectory_index_ != make_pair(-1, -1)) {
    store_trajectory_ = FindStoreTrajectory(ptr_conditions->central_trajectory_index_);
  }
}

void BehaviourSelector::UpdateTrafficSignalAndStopDis()
{
  CalculatedConditions *ptr_conditions = GetConditions();
  // 计算正常制动减速度下最小停车距离
  if (planning_params_->norm_deceleration() != 0) {
    ptr_conditions->min_stopping_distance_ =
        selector_params_.additional_braking_distance() -
        0.5 * pow(current_velocity_, 2) / (planning_params_->norm_deceleration());
  }

  ptr_conditions->stopping_distances_.clear();
  ptr_conditions->stopping_distances_.shrink_to_fit();
  // 交通信号获取
  // legionclaw::interface::TrafficEvents traffic_events = frame_.GetTrafficEvents();
  // ptr_conditions->is_trafficIs_red_ = false;
  // ptr_conditions->distance_to_stop_line_ = DBL_MAX;
  // ptr_conditions->cur_traffic_light_id_ = traffic_events.junction_info().id();
  // if (traffic_events.junction_info().light_flag() ==
  //     legionclaw::common::IsValid::VALID) {
  //   if (traffic_events.junction_info().light_color() ==
  //           legionclaw::common::TrafficLightColor::RED ||
  //       traffic_events.junction_info().light_color() ==
  //           legionclaw::common::TrafficLightColor::YELLOW) {
  //     ptr_conditions->is_trafficIs_red_ = true;
  //     ptr_conditions->distance_to_stop_line_ = traffic_events.junction_info().distance_to_stop();
  //     std::vector<legionclaw::interface::Point3D> stop_line =
  //         traffic_events.junction_info().stop_line();
  //     int stop_line_points_size =
  //         traffic_events.junction_info().stop_line_size();
  //     double distance_to_stop = ptr_conditions->distance_to_stop_line_;
  //     if(distance_to_stop < 0){
  //       distance_to_stop = DBL_MAX;
  //     }
  //     for (int i = 0; i < stop_line_points_size; ++i) {
  //       double delta_x = frame_.VehicleState().pose.x() - stop_line.at(i).x();
  //       double delta_y = frame_.VehicleState().pose.y() - stop_line.at(i).y();
  //       double distance = sqrt(delta_x * delta_x + delta_y * delta_y);
  //       if (distance < distance_to_stop) {
  //         distance_to_stop = distance;
  //       }
  //     }

  //     // 停止线后推车头到定位点的距离
  //     distance_to_stop =
  //         (distance_to_stop -
  //          vehicle_param_.front_edge_to_center()) > 0
  //             ? (distance_to_stop -
  //                vehicle_param_.front_edge_to_center())
  //             : 0;
  //     ptr_conditions->distance_to_stop_line_ = distance_to_stop;
  //     ptr_conditions->stopping_distances_.push_back(distance_to_stop);
  //   } else if (traffic_events.junction_info().light_color() ==
  //              legionclaw::common::TrafficLightColor::GREEN) {
  //     ptr_conditions->is_trafficIs_red_ = false;
  //   }
  // }
  // 障碍物停车距离
  // 虚拟障碍物停车距离更远，确保能预留换道空间
  // 旁边车道有通行空间当前车道有静止障碍物占道停车，确保能预留换道空间
  if (final_trajectory_.trajectory_cost().closest_obj_id < -1 ||
      (final_trajectory_.global_index() != min_cost_trajectory_.global_index() &&
       final_trajectory_.trajectory_cost().blocked_cost == 1000)) {
    ptr_conditions_->stopping_distances_.push_back(
        final_trajectory_.trajectory_cost().closest_obj_collision_dis -
        selector_params_.additional_braking_distance() - 10);

  } else if (final_trajectory_.trajectory_cost().id == "greenbelt_occlusion_stop_wall") {
    ptr_conditions_->stopping_distances_.push_back(
        final_trajectory_.trajectory_cost().closest_obj_collision_dis);
  } else {
    ptr_conditions_->stopping_distances_.push_back(
        final_trajectory_.trajectory_cost().closest_obj_collision_dis -
        selector_params_.additional_braking_distance());
  }
  // 更新到站信息 TODO: 基于目标车道参考线去计算参数
  ReferenceLine current_reference_line =
      frame_.ReferenceLine(frame_.MapMatchInfo().current_lane_index);
  int car_index = frame_.MapMatchInfo().lon_index;
  ptr_conditions->goal_distance_ = DBL_MAX;
  // TODO: is_new_goal_的必要性
  if (is_new_goal_)  // bNewGoalPos==true:收到新的站点
  {
    ptr_conditions->goal_distance_ = DBL_MAX;
    ptr_conditions->goal_distance_trajectory_ = DBL_MAX;
    ptr_conditions->current_goal_id_ = 1;
  }
  double match_distance = DBL_MAX;
  int goal_index_global = -1;
  if (terminal_stop_point_.type() != -1) {
    goal_index_global = MapMatcher::QueryNearestPointWithBuffer(
        current_reference_line.ReferencePoints(),
        {terminal_stop_point_.point().x(),
         terminal_stop_point_.point().y()},  // TODO:暂时认为只有一个停车点
        terminal_stop_point_.theta(), 1.0e-6,
        match_distance);  // 站点与参考线匹配
  }
  if (goal_index_global > -1 && fabs(match_distance) < selector_params().goal_match_error()) {
    ptr_conditions->goal_distance_ = current_reference_line.ReferencePoints(goal_index_global).s() -
                                     current_reference_line.ReferencePoints(car_index).s();
  }
  ptr_conditions_->stopping_distances_.push_back(ptr_conditions->goal_distance_);
}

// TODO路口汇入汇出信号获取
void BehaviourSelector::UpdateInwardOutwardStatus()
{
  if (!common_params()->enable_inward_outward_behavior()) {
    ADEBUG << "Not allow enable inward_outward behavior ";
    return;
  }
  CalculatedConditions *ptr_conditions = GetConditions();
  legionclaw::interface::TrafficEvents traffic_events = frame_.GetTrafficEvents();

  int current_lane_index = frame_.MapMatchInfo().current_lane_index;
  if (trajectory_array_.at(current_lane_index).size() <= 0) {
    ADEBUG << "current trajectory is null ";
    return;
  }
  ptr_conditions->is_outward_ = false;
  ptr_conditions->is_inward_ = false;
  ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Invalid;
  ptr_conditions->inwardstatus_ = InwardStatus::Inward_Invalid;
  ptr_conditions->distance_to_junction_ = traffic_events.junction_info().distance_to_junction();
  LaneInfoType ego_lane_type = frame_.MapMatchInfo().ego_lane_type[current_lane_index];
  if (traffic_events.junction_info().direction_flag() == legionclaw::common::IsValid::VALID &&
      ptr_conditions->distance_to_junction_ < selector_params().outwardcreep_distance()) {
    if ((trajectory_array_.at(current_lane_index - 1).size() <= 0 &&
         trajectory_array_.at(current_lane_index + 1).size() <= 0) ||
        (final_trajectory_.trajectory_cost().junction_close_cost == 0 &&
         final_trajectory_.trajectory_cost().horizon_cost < 1000)) {
      ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Complete;
      ptr_conditions->inwardstatus_ = InwardStatus::Inward_Complete;
      return;
    }
    // 汇入汇出状态，计算当前车道，左边两边实线的位置
    // 暂不考虑该范围内有实线虚线实线变化区域(比如有两个隔得很近的上下匝道口)
    double outwardstop_distance = selector_params().outwardstop_distance();
    double remain_mileage_max = std::min(trajectory_array_.at(current_lane_index)
                                             .front()
                                             .trajectory_points()
                                             .back()
                                             .path_point()
                                             .s(),
                                         frame_.ReferenceLine(current_lane_index).remain_mileage());
    remain_dist_to_change_left_ = remain_mileage_max;
    remain_dist_to_change_right_ = remain_mileage_max;
    // 轨迹剩余长度过短，进行减速策略和停车等待策略
    auto current_reference_points = frame_.ReferenceLine(current_lane_index).ReferencePoints();
    if (traffic_events.junction_info().direction() == legionclaw::common::Direction::RIGHT &&
        (!(ego_lane_type == LaneInfoType::RIGHT_TURN ||
           ego_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN) ||
         remain_dist_to_change_right_ <= outwardstop_distance)) {
      // 汇出，完成执行正常的加减速
      if ((trajectory_array_.at(current_lane_index + 1).size() <= 0 &&
           frame_.MapMatchInfo().ego_lane_type[current_lane_index] ==
               LaneInfoType::LANE_TYPE_UNKNOWN) ||
          current_behaviour_.lat_state ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
        ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Complete;
        // AWARN << "自车在右车道，无需换道";
        return;
      } else {
        // 不同阶段
        if (ptr_conditions->distance_to_junction_ < 150.0) {
          for (size_t it = frame_.MapMatchInfo().lon_index; it < current_reference_points.size();
               it++) {
            if (current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::CURB ||
                current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::WHITE_SOLID ||
                current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::YELLOW_SOLID) {
              remain_dist_to_change_right_ =
                  current_reference_points.at(it).s() -
                  current_reference_points.at(frame_.MapMatchInfo().lon_index).s();
              break;
            }
          }
        }
        remain_dist_to_change_right_ = std::min(remain_dist_to_change_right_, remain_mileage_max);
        // 向右汇出需要连续变道场景，预留更多变道空间
        LaneInfoType right_lane_type = frame_.MapMatchInfo().ego_lane_type[2];
        if (!(right_lane_type == LaneInfoType::RIGHT_TURN ||
              right_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN)) {
          outwardstop_distance = 2 * outwardstop_distance;
        }
        if (ptr_conditions->distance_to_junction_ <= outwardstop_distance ||
            remain_dist_to_change_right_ <= outwardstop_distance) {
          remain_dist_to_change_right_ -= selector_params().outwardstop_distance();
          ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Stop;
          AWARN << "Outward Stop!!!";
        } else {
          ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Creep;
          AWARN << "Outward Creep!!!";
        }
      }
      ptr_conditions->is_outward_ = true;

    } else if (traffic_events.junction_info().direction() == legionclaw::common::Direction::LEFT &&
               (!(ego_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
                  ego_lane_type == LaneInfoType::STRAIGHT_AND_LEFT_TURN ||
                  ego_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND ||
                  ego_lane_type == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND) ||
                remain_dist_to_change_left_ <= outwardstop_distance)) {
      // 汇，完成执行正常的加减速
      if ((trajectory_array_.at(current_lane_index - 1).size() <= 0 &&
           frame_.MapMatchInfo().ego_lane_type[current_lane_index] ==
               LaneInfoType::LANE_TYPE_UNKNOWN) ||
          current_behaviour_.lat_state ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE) {
        ptr_conditions->inwardstatus_ = InwardStatus::Inward_Complete;
        // AWARN << "自车在左车道，无需换道";
        return;
      } else {
        // 不同阶段
        if (ptr_conditions->distance_to_junction_ < 150.0) {
          for (size_t it = frame_.MapMatchInfo().lon_index; it < current_reference_points.size();
               it++) {
            if (current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::CURB ||
                current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::WHITE_SOLID ||
                current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::YELLOW_SOLID) {
              remain_dist_to_change_left_ =
                  current_reference_points.at(it).s() -
                  current_reference_points.at(frame_.MapMatchInfo().lon_index).s();
              break;
            }
          }
        }
        remain_dist_to_change_left_ = std::min(remain_dist_to_change_left_, remain_mileage_max);
        // 向左汇入需要连续变道场景，预留更多变道空间
        LaneInfoType left_lane_type = frame_.MapMatchInfo().ego_lane_type[0];
        if (!(left_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
              left_lane_type == LaneInfoType::STRAIGHT_AND_LEFT_TURN ||
              left_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND ||
              left_lane_type == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND)) {
          outwardstop_distance = 2 * outwardstop_distance;
        }
        if (ptr_conditions->distance_to_junction_ <= outwardstop_distance ||
            remain_dist_to_change_left_ <= outwardstop_distance) {
          remain_dist_to_change_left_ -= selector_params().outwardstop_distance();
          ptr_conditions->inwardstatus_ = InwardStatus::Inward_Stop;
          AWARN << "Inward Stop!!!";
        } else {
          ptr_conditions->inwardstatus_ = InwardStatus::Inward_Creep;
          // AWARN << "Inward Creep!!!";
        }
      }
      ptr_conditions->is_inward_ = true;
    } else if (traffic_events.junction_info().direction() == legionclaw::common::Direction::UP &&
               !(ego_lane_type == LaneInfoType::STRAIGHT ||
                 ego_lane_type == LaneInfoType::STRAIGHT_AND_LEFT_TURN ||
                 ego_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN ||
                 ego_lane_type == LaneInfoType::LANE_TYPE_UNKNOWN)) {
      // 自车在右转道
      if (ego_lane_type == RIGHT_TURN) {
        if (current_behaviour_.lat_state ==
            legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE) {
          ptr_conditions->inwardstatus_ = InwardStatus::Inward_Complete;
          return;
        }
        if (ptr_conditions->distance_to_junction_ < 150.0) {
          for (size_t it = frame_.MapMatchInfo().lon_index; it < current_reference_points.size();
               it++) {
            if (current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::CURB ||
                current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::WHITE_SOLID ||
                current_reference_points.at(it).left_line_type() ==
                    legionclaw::common::LaneLineType::YELLOW_SOLID) {
              remain_dist_to_change_left_ =
                  current_reference_points.at(it).s() -
                  current_reference_points.at(frame_.MapMatchInfo().lon_index).s();
              break;
            }
          }
        }
        remain_dist_to_change_left_ = std::min(remain_dist_to_change_left_, remain_mileage_max);
        if (ptr_conditions->distance_to_junction_ <= outwardstop_distance ||
            remain_dist_to_change_left_ <= outwardstop_distance) {
            remain_dist_to_change_left_ -= selector_params().outwardstop_distance();
          ptr_conditions->inwardstatus_ = InwardStatus::Inward_Stop;
          AWARN << "Inward Stop!!!";
        } else {
          ptr_conditions->inwardstatus_ = InwardStatus::Inward_Creep;
          // AWARN << "Inward Creep!!!";
        }
        ptr_conditions->is_inward_ = true;
      } else {  // 自车在左转道
        if (current_behaviour_.lat_state ==
            legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
          ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Complete;
          // AWARN << "自车在右车道，无需换道";
          return;
        }
        if (ptr_conditions->distance_to_junction_ < 150.0) {
          for (size_t it = frame_.MapMatchInfo().lon_index; it < current_reference_points.size();
               it++) {
            if (current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::CURB ||
                current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::WHITE_SOLID ||
                current_reference_points.at(it).right_line_type() ==
                    legionclaw::common::LaneLineType::YELLOW_SOLID) {
              remain_dist_to_change_right_ =
                  current_reference_points.at(it).s() -
                  current_reference_points.at(frame_.MapMatchInfo().lon_index).s();
              break;
            }
          }
        }
        remain_dist_to_change_right_ = std::min(remain_dist_to_change_right_, remain_mileage_max);
        if (ptr_conditions->distance_to_junction_ <= outwardstop_distance ||
            remain_dist_to_change_right_ <= outwardstop_distance) {
          remain_dist_to_change_right_ -= selector_params().outwardstop_distance();
          ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Stop;
          AWARN << "Outward Stop!!!";
        } else {
          ptr_conditions->outwardstatus_ = OutwardStatus::Outward_Creep;
          AWARN << "Outward Creep!!!";
        }
        ptr_conditions->is_outward_ = true;
      }
    }
  }
}

void BehaviourSelector::CalculateLatOffset()
{
  CalculatedConditions *ptr_conditions = GetConditions();
  if (target_trajectory_.trajectory_points_size() > 0) {
    auto current_reference_points =
        frame_.ReferenceLine(frame_.MapMatchInfo().current_lane_index).ReferencePoints();
    // 目标轨迹起始点相对于地图参考中心线的横向偏移量
    if (current_reference_points.size() == 0) {
      return;
    }
    auto start_point = target_trajectory_.trajectory_points().front().path_point();
    MapMatcher::QueryVerticalDistanceWithBuffer(
        current_reference_points, {start_point.x(), start_point.y()}, start_point.theta(), 1.0e-6,
        ptr_conditions->start_lat_offset_);
    // 目标轨迹终点相对于地图参考中心线的横向偏移量
    auto end_point = target_trajectory_.trajectory_points().back().path_point();
    MapMatcher::QueryVerticalDistanceWithBuffer(current_reference_points,
                                                {end_point.x(), end_point.y()}, end_point.theta(),
                                                1.0e-6, ptr_conditions->end_lat_offset_);
  } else {
    ptr_conditions->start_lat_offset_ = 0.0;
    ptr_conditions->end_lat_offset_ = 0.0;
  }

  return;
}

void BehaviourSelector::UpdateClosestObjInfo()
{
  CalculatedConditions *ptr_conditions = GetConditions();
  // 更新当前车行驶路线的障碍物信息
  ptr_conditions->current_lane_closest_obj_distance_ =
      store_trajectory_.trajectory_cost().closest_obj_distance;
  // 更新目标轨迹行驶路线障碍物信息
  ptr_conditions->closest_obj_collision_dis_ =
      target_trajectory_.trajectory_cost().closest_obj_collision_dis;
  ptr_conditions->closest_obj_velocity_ = target_trajectory_.trajectory_cost().closest_obj_velocity;
  ptr_conditions->closest_obj_theta_abs_ =
      target_trajectory_.trajectory_cost().closest_obj_theta_abs;
  ptr_conditions->closest_obj_theta_veh_ =
      target_trajectory_.trajectory_cost().closest_obj_theta_veh;
  ptr_conditions->target_lane_closest_obj_distance_ =
      target_trajectory_.trajectory_cost().closest_obj_distance;
  ptr_conditions->avoidance_distance_ = ComputeAvoidanceDistance(
      target_trajectory_.trajectory_cost().closest_obj_velocity, current_velocity_);
  return;
}

void BehaviourSelector::CalculateImportantParameterForDecisionMaking()
{
  CalculatedConditions *ptr_conditions = GetConditions();
  // 更新车辆静止状态
  current_velocity_ = frame_.VehicleState().speed;
  if (current_velocity_ < planning_params_->threshold_static_speed_self()) {
    ptr_conditions->is_static_ = true;
  } else {
    ptr_conditions->is_static_ = false;
  }

  // 获得当前车道在固定三车道的排序
  ptr_conditions->current_lane_index_ = frame_.MapMatchInfo().current_lane_index;

  // 存储备选轨迹
  SelectStoreTrajectory();

  // 计算最终选择的轨迹起始点和终点距离车当前车道中心线的横向偏差
  CalculateLatOffset();

  // 更新对应目标车道和当前车道障碍物信息
  UpdateClosestObjInfo();

  // 更新交通信号并根据交通信号计算停车距离// TODO 代码待梳理
  // UpdateTrafficSignalAndStopDis();

  // // 更新汇入汇出状态    //TODO
  // UpdateInwardOutwardStatus();
}

void BehaviourSelector::GenerateBehaviorState()
{
  // 但有些状态下，需要重新选择final_trajectory_（根据current_behaviour_.state来约束）
  final_trajectory_ = target_trajectory_;
  // 横向状态机更新
  behaviour_lat_state_sm_->OnUpdate();
  // 根据当前横向状态进行约束
  switch (current_behaviour_.lat_state) {
    case ADCTrajectory::LAT_NOT_ACTIVE_STATE:
      // do something
      break;
    case ADCTrajectory::WAITING_STATE:
      // do something
      break;
    case ADCTrajectory::FORWARD_STATE:
      // do something
      break;
    case ADCTrajectory::STOPPING_STATE:
      // do something
      break;
    case ADCTrajectory::STATION_STATE:
      final_trajectory_ = store_trajectory_;
      break;
    case ADCTrajectory::STATION_ARRIVED_STATE:
      final_trajectory_ = store_trajectory_;
      break;
    case ADCTrajectory::AVOIDANCE_PRE_STATE:
      final_trajectory_ = store_trajectory_;
      break;
    case ADCTrajectory::AVOIDANCE_BACK_PRE_STATE:
      final_trajectory_ = store_trajectory_;
      break;
    case ADCTrajectory::OBSTACLE_AVOIDANCE_STATE:
      // do something
      break;
    case ADCTrajectory::LANE_CHANGE_LEFT_PRE_STATE:
      final_trajectory_ = store_trajectory_;
      break;
    case ADCTrajectory::LANE_CHANGE_RIGHT_PRE_STATE:
      final_trajectory_ = store_trajectory_;
      break;

    default:
      // do something
      break;
  }

  // 更新交通信号并根据交通信号计算停车距离// TODO 代码待梳理
  UpdateTrafficSignalAndStopDis();
  // 更新汇入汇出状态    //TODO
  UpdateInwardOutwardStatus();
  // 跟车距离
  current_behaviour_.collision_distance =
      final_trajectory_.trajectory_cost().closest_obj_collision_dis;
  // relative距离
  current_behaviour_.relative_distance = final_trajectory_.trajectory_cost().closest_obj_distance;
  // 跟车速度
  current_behaviour_.follow_velocity = final_trajectory_.trajectory_cost().closest_obj_velocity;
  // 跟车加速度
  current_behaviour_.follow_acceleration =
      final_trajectory_.trajectory_cost().closest_obj_acceleration;
  // if (final_trajectory_.trajectory_cost().closest_obj_id != -1) {
  //   std::cout << ":::closest_obj_id : " <<
  //   final_trajectory_.trajectory_cost().closest_obj_id<<endl; std::cout << ":::follow_velocity :
  //   " << current_behaviour_.follow_velocity<<endl; std::cout << ":::follow_acceleration : " <<
  //   current_behaviour_.follow_acceleration<<endl;
  // }

  // 纵向状态机更新
  behaviour_lon_state_sm_->OnUpdate();
  current_behaviour_.lon_state_name = behaviour_lon_state_sm_->GetCurrentStateName();

  // 红绿灯决策
  if (common_params()->enable_traffic_light_behavior() &&
      final_trajectory_.trajectory_cost().id == "traffic_light_stop_wall") {
    current_behaviour_.lon_state =
        legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_STOP_STATE;
  }
  // 绿化带决策
  if (final_trajectory_.trajectory_cost().id == "greenbelt_occlusion_stop_wall") {
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::EMYIELD;
  }

  // 根据状态开启转向灯
  SetTrunLamp(final_trajectory_.trajectory_points());
  return;
}

double BehaviourSelector::UpdatePlanSpeedAndTargetAccel()
{
  double update_speed = current_velocity_;
  // 地图速度梯度变化较大，产生的急减速
  double speed_diff = current_velocity_ - target_velocity_;
  if (tar_accel_ > planning_params_->norm_deceleration() && speed_diff < 0.3)
    update_speed = min(target_velocity_, current_velocity_);

  double last_acc = 0.0;

  // 速度和加速度尽量帧间连续
  // 如果存在上一次轨迹，使用当前计算轨迹首点匹配上一次轨迹，获取对应速度和加速度
  if (frame_.LastPlanningTrajectorySize() > 0) {
    double matched_distance;
    int matched_index = MapMatcher::QueryNearestPointWithBuffer(
        frame_.LastTrajectoryPoints(),
        {final_trajectory_.trajectory_points().front().path_point().x(),
         final_trajectory_.trajectory_points().front().path_point().y()},
        final_trajectory_.trajectory_points().front().path_point().theta(), 1.0e-6,
        matched_distance);
    if (matched_index < 0) {
      ;
    } else {
      if ((fabs(update_speed - frame_.LastTrajectoryPoints(matched_index).v()) <
               selector_params().replan_speed_diff() &&
           !GoalDistanceLess()) &&
          (current_behaviour_.relative_distance > 10.0)) {
        update_speed = frame_.LastTrajectoryPoints(matched_index).v();
        // stop状态下修复加速度和速度计算规则不匹配的bug
        if (tar_accel_ <= 0 &&
            (current_behaviour_.lon_state == ADCTrajectory::STOP ||
             current_behaviour_.lon_state == ADCTrajectory::TRAFFIC_LIGHT_STOP_STATE) &&
            current_behaviour_.follow_velocity <= planning_params_->threshold_static_speed()) {
          double stop_length = ptr_conditions_->MinDistanceToStop();
          if (stop_length <= 0) stop_length = 0.1;
          tar_accel_ = -0.5 * update_speed * update_speed / stop_length;
        }
      }

      last_acc = frame_.LastTrajectoryPoints(matched_index).a();

      // 若需刹车并且离障碍物相对距离特别近，速度直接降为0.
      // if (tar_accel_ < 0 &&
      //         current_behaviour_.relative_distance <
      //             selector_params_.acc_min_length() &&
      //     current_behaviour_.follow_velocity <
      //         selector_params_.acc_activate_obj_speed()) {
      //   update_speed = 0.0;
      // }
    }
  }

  // 动态限速
  legionclaw::interface::TrafficEvents traffic_events = frame_.GetTrafficEvents();
  if (traffic_events.limit_speed_info().limitspeed_valid_flag() == common::IsValid::VALID &&
      update_speed > traffic_events.limit_speed_info().limit_speed()) {
    double limit_speed_accel =
        (traffic_events.limit_speed_info().limit_speed() - update_speed) * 0.2;
    if (limit_speed_accel < vehicle_param_.max_deceleration() * 0.2) {
      limit_speed_accel = vehicle_param_.max_deceleration() * 0.2;
    }
    if (tar_accel_ > limit_speed_accel) {
      tar_accel_ = limit_speed_accel;
    }
  }

  // 曲率限速加速度
  if (ptr_conditions_->is_limit_speed_kappa_ &&
      current_behaviour_.lon_state != ADCTrajectory::BehaviourLonState::EMYIELD) {
    tar_accel_ = max(ptr_conditions_->limit_speed_kappa_deceleration_, tar_accel_);
  }

  // TODO 加速度平滑待优化
  //  加速度限制
  if (tar_accel_ > vehicle_param_.max_acceleration()) {
    tar_accel_ = vehicle_param_.max_acceleration();
  }
  if (tar_accel_ < vehicle_param_.max_deceleration()) {
    tar_accel_ = vehicle_param_.max_deceleration();
  }

  // if (tar_accel_ > 0 && tar_accel_ - last_acc > vehicle_param_.max_acceleration_jerk() * 0.1) {
  //   if (last_acc < 0) {
  //     tar_accel_ = vehicle_param_.max_acceleration_jerk() * 0.1;
  //   } else {
  //     tar_accel_ = last_acc + vehicle_param_.max_acceleration_jerk() * 0.1;
  //   }
  // }

  // if ((tar_accel_ - last_acc) > -4)
  {
    if (tar_accel_ >= 0.0) {
      if (last_acc < 0.0) {
        tar_accel_ = std::min(0.0, (last_acc + 0.1));
      } else if (last_acc >= 0.0) {
        tar_accel_ =
            common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.1,
                                last_acc + vehicle_param_.max_acceleration_jerk() * 0.1);
      }

    } else if (tar_accel_ < 0.0) {
      if (last_acc > 0.0) {
        if(tar_accel_ >= vehicle_param_.deceleration_in_idle()) {
          tar_accel_ =
            common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.1,
                                last_acc + vehicle_param_.max_acceleration_jerk() * 0.1);
        } else if(tar_accel_ < vehicle_param_.deceleration_in_idle()) {
          tar_accel_ = std::max(0.0, (last_acc - 0.1));
        }
      }

      // else if ((last_acc <= 0.0) && (last_acc >= -1.0)) {
      //   tar_accel_ =
      //       common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.2,
      //                           last_acc + vehicle_param_.max_acceleration_jerk() * 0.2);
      // } else if(last_acc >= -2.0) {
      //   tar_accel_ =
      //       common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.3,
      //                           last_acc + vehicle_param_.max_acceleration_jerk() * 0.3);
      // } else if(last_acc >= -3.0) {
      //   tar_accel_ =
      //       common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.4,
      //                           last_acc + vehicle_param_.max_acceleration_jerk() * 0.4);
      // } else {
      //   tar_accel_ =
      //       common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.5,
      //                           last_acc + vehicle_param_.max_acceleration_jerk() * 0.5);
      // }

      else if((last_acc <= 0.0) && (last_acc >= -1.0)) {
        tar_accel_ =
            common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.2,
                                last_acc + vehicle_param_.max_acceleration_jerk() * 0.2);
      } else if (last_acc >= -3.0) {
        tar_accel_ =
            common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.3,
                                last_acc + vehicle_param_.max_acceleration_jerk() * 0.3);
      } else {
        tar_accel_ =
            common::math::Clamp(tar_accel_, last_acc - vehicle_param_.max_acceleration_jerk() * 0.2,
                                last_acc + vehicle_param_.max_acceleration_jerk() * 0.2);
      }
    }
  }

  // 速度几乎静止时，速度置零
  if (tar_accel_ < 0.0 && update_speed <= planning_params_->threshold_static_speed_self()) {
    update_speed = 0.0;
  }

  return update_speed;
}

bool BehaviourSelector::ComputeGoalDistanceOnTrajectory(
    const std::vector<TrajectoryPoint> &trajectory, const math::Vec2d &position, const double yaw,
    double &goal_distance)
{
  goal_distance = DBL_MAX;
  if (trajectory.size() == 0) return false;

  double goal_match_distance = DBL_MAX;
  double match_distance = DBL_MAX;
  int goal_index_global = MapMatcher::QueryNearestPointWithBuffer(trajectory, position, yaw, 1.0e-6,
                                                                  goal_match_distance);

  TrajectoryPoint point;
  double diff_pose = 0.0, diff_yaw = 0.0;
  int step = (int)(1 / planning_params_->path_density());  // 搜索步长1米
  // 匹配到第一点时，有可能车已经超过站点（match_distance较大），也需要进入goal状态
  if (goal_index_global == 0 || fabs(goal_match_distance) < selector_params().goal_match_error()) {
    // 从站点匹配点开始沿轨迹向前搜索，判断轨迹上的停车点与道路是否平行，且与道路参考线的横向位置差不大
    for (size_t i = goal_index_global; i < trajectory.size(); i += step) {
      point = trajectory.at(i);
      int match_index = MapMatcher::QueryNearestPointWithBuffer(
          frame_.ReferenceLines().back().ReferencePoints(),
          {point.path_point().x(),
           point.path_point().y()},  // TODO:是当前所在的参考线，还是最右的参考线
          point.path_point().theta(), 1.0e-6, match_distance);
      if (match_index >= 0) {
        diff_pose = fabs(match_distance);
        using namespace math;
        diff_yaw = fabs(
            math::AngleDiff(frame_.ReferenceLines().back().ReferencePoints(match_index).theta(),
                            point.path_point().theta()));
        if (diff_pose < 1 && diff_yaw < 5.0 * M_PI / 180.0) {
          goal_distance = MapMatcher::GetExactDistanceOnTrajectory(trajectory, 0, i);
          return true;
        }
      }
    }
    // 如果整体轨迹都找不到满足条件的点，则仍按站点匹配点停车
    goal_distance = MapMatcher::GetExactDistanceOnTrajectory(trajectory, 0, goal_index_global);
    return true;
  } else
    return false;
}

double BehaviourSelector::ComputeLimitSpeedByKappa(const std::vector<TrajectoryPoint> &trajectory,
                                                   const double &cur_speed,
                                                   const double &search_length,
                                                   const double &limit_lat_acc,
                                                   const double &min_tar_speed)
{
  // 搜索最大曲率的点
  double max_kappa = -DBL_MAX;
  int max_kappa_index = -1;
  double s = 0;
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    s += math::Norm(trajectory.at(i).path_point().x() - trajectory.at(i - 1).path_point().x(),
                    trajectory.at(i).path_point().y() - trajectory.at(i - 1).path_point().y());
    if (s > search_length) break;
    if (fabs(trajectory.at(i).path_point().kappa()) > max_kappa) {
      max_kappa = fabs(trajectory.at(i).path_point().kappa());
      max_kappa_index = i;
    }
  }

  if (fabs(max_kappa) < 1e-9) return DBL_MAX;

  // 离心加速度公式:a=v^2/R
  double limit_speed =
      sqrt(fabs(limit_lat_acc / max_kappa)) * selector_params_.curve_speed_adjuster();
  if (limit_speed < min_tar_speed)  // 防止速度过小
    limit_speed = min_tar_speed;
  if (max_kappa_index >= 0 && cur_speed > limit_speed) {
    double length = max(trajectory.at(max_kappa_index).path_point().s() - cur_speed * 0.2, 0.5);
    ptr_conditions_->limit_speed_kappa_deceleration_ =
        0.5 * (limit_speed * limit_speed - cur_speed * cur_speed) / length;
    if (limit_speed - current_velocity_ > -0.56) {
      ptr_conditions_->limit_speed_kappa_deceleration_ =
          max(ptr_conditions_->limit_speed_kappa_deceleration_,
              (limit_speed - current_velocity_) / selector_params().norm_dec_time());
    }
  } else {
    ptr_conditions_->limit_speed_kappa_deceleration_ = 0;
  }
  // double length =
  //     MapMatcher::GetExactDistanceOnTrajectory(trajectory, 0, max_kappa_index);
  // return (0.5 * (limit_speed * limit_speed - cur_speed * cur_speed) / length);
  // AWARN << "tar v : " << target_velocity_ << " , lim v : " << limit_speed << std::endl;
  return limit_speed;
}

bool BehaviourSelector::ComputeTrajectoryLongitudinalInfo(const double &start_speed,
                                                          const double &accel,
                                                          const double &limit_speed,
                                                          std::vector<TrajectoryPoint> &trajectory)
{
  if (trajectory.size() <= 0) return false;

  double temp_velocity = 0.0;
  double init_mileage = trajectory.front().path_point().s();
  double delta_mileage = 0.0;
  double relative_time = 0.0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    delta_mileage = trajectory.at(i).path_point().s() - init_mileage;
    // TODO 2.0 * accel * 0.1 轨迹密度并不是固定的0.1
    // if (stop_earlier_ && delta_mileage >= 0.5 * ptr_conditions_->MinDistanceToStop()) {
    //   temp_velocity = temp_velocity * temp_velocity - 2.0 * accel *
    //   planning_params_->path_density(); trajectory.at(i).set_a(-1.0 * accel);
    // } else
    {
      // target speed^2 of current point.
      temp_velocity = start_speed * start_speed + 2.0 * accel * delta_mileage;
      trajectory.at(i).set_a(accel);
    }

    if (temp_velocity <= 0.0) {
      temp_velocity = 0.0;
      if (accel <= 0.0)
        trajectory.at(i).set_gear(legionclaw::common::GEAR_PARKING);
      else
        trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);
    } else {
      temp_velocity = sqrt(temp_velocity);
      trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);
      // if (accel >= 0.0 && temp_velocity > limit_speed) {
      //   temp_velocity = limit_speed;
      //   trajectory.at(i).set_a(0.0);
      // }
    }
    trajectory.at(i).set_v(temp_velocity);

    // set time cost
    if (i == 0) {
      trajectory.at(i).set_relative_time(0.0);
    } else {
      if (trajectory.at(i - 1).a() == 0.0) {
        if (trajectory.at(i - 1).v() == 0.0) {
          relative_time = 0;
        } else {
          relative_time =
              trajectory.at(i - 1).relative_time() +
              (trajectory.at(i).path_point().s() - trajectory.at(i - 1).path_point().s()) /
                  trajectory.at(i - 1).v();
        }
        trajectory.at(i).set_relative_time(relative_time);
      } else {
        relative_time =
            trajectory.at(i - 1).relative_time() +
            (trajectory.at(i).v() - trajectory.at(i - 1).v()) / trajectory.at(i - 1).a();
        trajectory.at(i).set_relative_time(relative_time);
      }
    }
  }
  stop_earlier_ = false;
  // 轨迹按相对时间采样抽稀
  std::vector<TrajectoryPoint> trajectory_temp;
  double time_adder = 0;
  double s_adder = 0;
  for (unsigned int j = 0; j < trajectory.size(); ++j) {
    // 轨迹抽稀站点停车优化
    if (trajectory.at(j).relative_time() < 1 ||
        trajectory.at(j).path_point().s() < selector_params().goal_max_distance()) {
      trajectory_temp.emplace_back(trajectory.at(j));
      continue;
    }
    double delta_t = trajectory.at(j).relative_time() - trajectory.at(j - 1).relative_time();
    time_adder += delta_t;
    // 速度为0的点按2m进行抽稀,且速度为0的第一个点不能过滤
    if (delta_t == 0) {
      if (time_adder > 0 && time_adder <= 0.1) {
        trajectory_temp.emplace_back(trajectory.at(j - 1));
        time_adder = 0;
      }
      double delta_s = trajectory.at(j).path_point().s() - trajectory.at(j - 1).path_point().s();
      s_adder += delta_s;
      if (s_adder > 2.0) {
        trajectory_temp.emplace_back(trajectory.at(j));
        s_adder = 0;
      }
    }
    // 速度非0的点按0.1s进行抽稀
    if (time_adder > 0.1) {
      trajectory_temp.emplace_back(trajectory.at(j));
      time_adder = 0;
    }
  }
  trajectory_temp.emplace_back(trajectory.back());
  trajectory = std::move(trajectory_temp);

  return true;
}

bool BehaviourSelector::ComputeLongitudinalInfoSpline(
    const double &current_velocity, const double &current_acc, const double &tar_length,
    const double &tar_speed, const double &limit_speed, std::vector<TrajectoryPoint> &trajectory)
{
  if (trajectory.size() < 2) return false;
  spline::CubicSpline cubic_spline;
  double cur_mileage = trajectory.front().path_point().s();
  cubic_spline.set_points(cur_mileage, current_velocity, cur_mileage + tar_length, tar_speed);
  cubic_spline.set_boundary(current_acc / current_velocity, 0.0);
  cubic_spline.compute_coef();
  double relative_time = 0.0;

  int tar_num = min(int(tar_length * 10), int(trajectory.size() - 1));

  for (int i = 1; i < tar_num; i++) {
    trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);

    double mileage = trajectory.at(i).path_point().s();
    double speed = cubic_spline(mileage);
    double acc = cubic_spline.compute_accelerate(mileage);

    trajectory.at(i).set_a(acc);
    if (acc >= 0.0 && speed > limit_speed) {
      speed = limit_speed;
      trajectory.at(i).set_a(0.0);
    }
    trajectory.at(i).set_v(speed);

    // set time cost
    if (i == 0) {
      trajectory.at(i).set_relative_time(0.0);
    } else {
      if (trajectory.at(i - 1).a() == 0.0) {
        relative_time =
            trajectory.at(i - 1).relative_time() +
            (trajectory.at(i).path_point().s() - trajectory.at(i - 1).path_point().s()) /
                trajectory.at(i - 1).v();
        trajectory.at(i).set_relative_time(relative_time);
      } else {
        relative_time =
            trajectory.at(i - 1).relative_time() +
            (trajectory.at(i).v() - trajectory.at(i - 1).v()) / trajectory.at(i - 1).a();
        trajectory.at(i).set_relative_time(relative_time);
      }
    }
  }

  // 轨迹至少有两个点，第一个点的速度信息=第二个点
  trajectory.at(0).set_v(trajectory.at(1).v());
  trajectory.at(0).set_a(trajectory.at(1).a());

  for (size_t i = tar_num; i < trajectory.size(); i++) {
    if (tar_speed > 0.0)
      trajectory.at(i).set_gear(legionclaw::common::GEAR_DRIVE);
    else
      trajectory.at(i).set_gear(legionclaw::common::GEAR_PARKING);
    trajectory.at(i).set_v(tar_speed);
    trajectory.at(i).set_a(0.0);

    // set time cost
    if (i == 0) {
      trajectory.at(i).set_relative_time(0.0);
    } else {
      relative_time = trajectory.at(i - 1).relative_time() +
                      (trajectory.at(i).path_point().s() - trajectory.at(i - 1).path_point().s()) /
                          trajectory.at(i - 1).v();
      trajectory.at(i).set_relative_time(relative_time);
    }
  }
  return true;
}

bool BehaviourSelector::AdaptiveCruiseControl(const double &final_length,
                                              const double &delta_length, const double &car_speed,
                                              const double &obj_speed, const double &l_min,
                                              const double &h, const double &k1, const double &k2,
                                              const double &t_pre, double &motion_speed,
                                              double &motion_length, double &motion_accel)
{
  // if (delta_length > selector_params().obstacle_speed_multiplier() * obj_speed) return 0;
  // double l_min = ACC_MIN_LENGTH;//最短距离
  // double h = ACC_COLLISION_TIME; //时距
  // double k1 = ACC_DELTA_LENGTH_GAIN;//距离差参数
  // double k2 = ACC_DELTA_SPEED_GAIN;//速度差参数
  // double t_pre = ACC_PREDICTION_TIME;//预测时间
  double normal_acc = 0;
  if (target_velocity_ > car_speed) {
    normal_acc = (target_velocity_ - car_speed) / selector_params().norm_acc_time();
    if (normal_acc > planning_params_->norm_acceleration())
      normal_acc = planning_params_->norm_acceleration();
  }
  double dv = obj_speed - car_speed;
  double l_tar = max(final_length, l_min);   // l_min + h*car_speed;
  double length_err = delta_length - l_tar;  // 当前距离 - 期望距离

  double beta_k1 = 1.0 / (1 + exp(-2.0 * (fabs(length_err) - 3.0)));
  double beta_k2 = 1.0 / (1 + exp(-3.0 * (fabs(dv) - 1.0)));

  double tar_accel = 0.0;
  if (length_err > 0)
    tar_accel = 1 / h * (beta_k2 * k2 * dv + beta_k1 * 0.5 * k1 * length_err);
  else
    tar_accel = 1 / h * (beta_k2 * k2 * dv + beta_k1 * k1 * length_err);
  if (tar_accel > 0) {
    tar_accel = min(tar_accel * 0.5, normal_acc);
  }
  // if(tar_accel_ > MAX_ACCELERATION) tar_accel_ = MAX_ACCELERATION;
  // if(tar_accel_ < -MAX_DECELERATION) tar_accel_ = -MAX_DECELERATION;

  motion_length = obj_speed * t_pre + delta_length - l_tar;
  motion_speed = car_speed + tar_accel * t_pre;
  if (motion_length < 0) {
    motion_length = 0;
  }
  motion_accel = tar_accel;

  // test 目前刹车平缓，这里修改看看效果，以防突然切入减速导致我车刹不住
  if (dv > 5 && delta_length > l_min &&
      motion_accel < 0.0)  // 对车辆突然切入且距离大于l_min时，采取匀速行驶
  {
    motion_accel = 0;
    motion_length = obj_speed * t_pre + delta_length - l_tar;
    motion_speed = car_speed + motion_accel * t_pre;
  }

  // 参照ISO15622 ACC加减速性能要求
  // 减速度及减速度变化率限制
  double previous_acc = std::min(frame_.LastTrajectoryPoints().front().a(), 0.0);
  if (motion_accel < 0.0) {
    if (car_speed <= 5.0) {
      motion_accel = common::math::Clamp(motion_accel, previous_acc - 0.5, previous_acc + 0.5);
      motion_accel = std::max(motion_accel, -5.0);
    } else if (car_speed >= 20.0) {
      motion_accel = common::math::Clamp(motion_accel, previous_acc - 0.25, previous_acc + 0.25);
      motion_accel = std::max(motion_accel, -3.5);
    } else {
      motion_accel = common::math::Clamp(motion_accel,
                                         previous_acc + ((car_speed - 5) * 2.5 / 15.0 - 5.0) * 0.1,
                                         previous_acc - ((car_speed - 5) * 2.5 / 15.0 - 5.0) * 0.1);
      motion_accel = std::max(motion_accel, (car_speed - 5) * 0.1 - 5.0);
    }
  }

  // if(current_behaviour_.lon_state ==
  //     legionclaw::interface::ADCTrajectory::BehaviourLonState::ACC) {
  //       std::cout << std::fixed;
  //       std::cout << std::setprecision(3)
  //                 << TimeTool::NowToSeconds() << "\t"
  //                 << "obj_v: " << obj_speed << "\t"
  //                 << "car_v: " << car_speed << "\t"
  //                 << "dv: " << dv << "\t"
  //                 << "tar_l: " << l_tar << "\t"
  //                 << "rel_l: " << delta_length << "\t"
  //                 << "dl: " << length_err << "\t"
  //                 << "v_gain: " << k2 << "\t"
  //                 << "l_gain: " << k1 << "\t"
  //                 // << " beta_k1: " << beta_k1
  //                 // << " beta_k2: " << beta_k2
  //                 << "tar_a: " << tar_accel << "\t"
  //                 << "motion_a: " << motion_accel << "\t"
  //                 << "pre_a: " << previous_acc
  //                 << std::endl;
  // }

  return true;
}

double BehaviourSelector::ComputeAvoidanceDistance(const double &closest_obj_velocity,
                                                   const double &vehicle_velocity)
{
  if (closest_obj_velocity < 0) {
    return -1;
  }
  double avoidance_distance;
  double relative_velocity = vehicle_velocity - closest_obj_velocity;
  if (closest_obj_velocity >= 0 && closest_obj_velocity < 0.56) {
    if (relative_velocity > 0 && relative_velocity < 2.7) {
      avoidance_distance = relative_velocity * selector_params_.avoidance_distance_time() +
                           planning_params_->vehicle_param().length() * 2;
    } else {
      avoidance_distance = relative_velocity * selector_params_.avoidance_distance_time();
    }
  } else if (closest_obj_velocity >= 0.56 && closest_obj_velocity < 2.7) {
    if (relative_velocity < 1) {
      avoidance_distance = -1;
    } else {
      avoidance_distance = relative_velocity * selector_params_.avoidance_distance_time() +
                           planning_params_->vehicle_param().length() * 2;
    }
  } else if (closest_obj_velocity >= 2.7 && closest_obj_velocity < 8.3) {
    if (relative_velocity < 2) {
      avoidance_distance = -1;
    } else {
      avoidance_distance = relative_velocity * 15;
    }
  } else if (closest_obj_velocity >= 8.3) {
    if (relative_velocity < 3) {
      avoidance_distance = -1;
    } else {
      avoidance_distance = relative_velocity * 18;
    }
  }
  return avoidance_distance;
}

void BehaviourSelector::SetTrunLamp(const std::vector<TrajectoryPoint> &trajectory)
{
  planning_cmd_.set_turn_lamp_ctrl(TURN_INACTIVE_SIGNAL);
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    if ((trajectory.at(i).path_point().kappa()) > THRESHOLD_CURVATURE_FOR_TURN) {
      planning_cmd_.set_turn_lamp_ctrl(TURN_LEFT_SIGNAL);
      return;
    } else if ((trajectory.at(i).path_point().kappa()) < -THRESHOLD_CURVATURE_FOR_TURN) {
      planning_cmd_.set_turn_lamp_ctrl(TURN_RIGHT_SIGNAL);
      return;
    } else {
      planning_cmd_.set_turn_lamp_ctrl(TURN_INACTIVE_SIGNAL);
      // return;
    }
  }
  // TODO::弯道变道
  switch (current_behaviour_.lat_state) {
    case ADCTrajectory::LAT_NOT_ACTIVE_STATE:
      // do something
      break;
    case ADCTrajectory::WAITING_STATE:
      // do something
      break;
    case ADCTrajectory::LANE_CHANGE_LEFT_PRE_STATE:
      planning_cmd_.set_turn_lamp_ctrl(TURN_LEFT_SIGNAL);
      break;
    case ADCTrajectory::LANE_CHANGE_LEFT_STATE:
      planning_cmd_.set_turn_lamp_ctrl(TURN_LEFT_SIGNAL);
      break;
    case ADCTrajectory::LANE_CHANGE_RIGHT_PRE_STATE:
      planning_cmd_.set_turn_lamp_ctrl(TURN_RIGHT_SIGNAL);
      break;
    case ADCTrajectory::LANE_CHANGE_RIGHT_STATE:
      planning_cmd_.set_turn_lamp_ctrl(TURN_RIGHT_SIGNAL);
      break;
    default:
      break;
  }
}

}  // namespace planning
}  // namespace legionclaw
