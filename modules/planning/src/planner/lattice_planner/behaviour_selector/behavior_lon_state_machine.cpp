/// \file behavior_lon_state_machine_planning.cpp
/// \author Hatem Darweesh
/// \brief OpenPlanner's state machine implementation for different driving
/// behaviors \date Jun 19, 2016

#include <iostream>

#include "modules/common/time/time_tool.h"
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"

namespace legionclaw {
namespace planning {

using namespace legionclaw::common;

void BehaviourSelector::LonNotActiveStateUpdate(const std::string &state_name, int state)
{
  // somthing to do
}

void BehaviourSelector::NormalStateUpdate(const std::string &state_name, int state)
{
  auto IsTrafficLightState = [=]() -> bool {
    if (!common_params()->enable_traffic_light_behavior()) {
      ADEBUG << "Not allow enable traffic light behavior ";
      return false;
    }
    if (!ptr_conditions_->is_trafficIs_red_) {
      ADEBUG << "Traffic lights are not red lights ";
      return false;
    }
    return true;
  };
  auto IsTrafficSignState = [=]() -> bool {
    if (!common_params()->enable_stop_sign_behavior()) {
      ADEBUG << "Not allow enable stop sign behavior ";
      return false;
    }
    if (ptr_conditions_->current_stop_sign_id_ == 0) {
      ADEBUG << "Current stop sign id = 0 ";
      return false;
    }
    if (ptr_conditions_->current_stop_sign_id_ == ptr_conditions_->prev_stop_sign_id_) {
      ADEBUG << "Current stop sign id == prev stop sign id ";
      return false;
    }
    return true;
  };

  // TODO汇出状态判断  使能+events
  auto IsOutwardState = [=]() -> bool {
    if (!common_params()->enable_inward_outward_behavior()) {
      ADEBUG << "Not allow enable inward_outward behavior ";
      return false;
    }
    if (!ptr_conditions_->is_outward_) {
      ADEBUG << "Condition is not in outward status";
      return false;
    }
    return true;
  };

  // TODO汇入状态判断 使能—+events
  auto IsIntwardState = [=]() -> bool {
    if (!common_params()->enable_inward_outward_behavior()) {
      ADEBUG << "Not allow enable inward_outward behavior ";
      return false;
    }
    if (!ptr_conditions_->is_inward_) {
      ADEBUG << "Condition is not in inward status";
      return false;
    }
    return true;
  };

  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
  lon_state_ = SelectorLonFlag::LON_INVALID;

  // 触发条件： 1.使能开启；2.ID非默认值；3.红绿灯为红色；4.红绿灯的ID有刷新
  if (IsTrafficLightState()) {
    behaviour_lon_state_sm_->NextState("TrafficLightStop");
    current_behaviour_.lon_state =
        legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_STOP_STATE;
    return;
  }
  // 触发条件： 1.使能开启；2.ID非默认值；3.ID有刷新
  if (IsTrafficSignState()) {
    behaviour_lon_state_sm_->NextState("TrafficSignStop");
    current_behaviour_.lon_state =
        legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_SIGN_STOP_STATE;
    return;
  }

  // 静止障碍物或站点或变道状态后方来车让行减速
  if ((final_trajectory_.trajectory_cost().is_fully_block &&
       final_trajectory_.trajectory_cost().closest_obj_velocity <=
           planning_params_->threshold_static_speed()) ||
      (GoalDistanceLess() && ptr_conditions_->goal_distance_ > 10) ||
      (ptr_conditions_->stop_to_wait_change_lane_ == true)) {
    behaviour_lon_state_sm_->NextState("Stop");
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::STOP;
    return;
  }

  // 精准到站停车
  if (GoalDistanceLess() && ptr_conditions_->goal_distance_ < 10) {
    behaviour_lon_state_sm_->NextState("PreciseStop");
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::PRECISE_STOP;
    return;
  }

  // 汇出处理
  if (IsOutwardState()) {
    behaviour_lon_state_sm_->NextState("Outward");
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::OUTWARD;
    return;
  }

  // 汇入处理
  if (IsIntwardState()) {
    behaviour_lon_state_sm_->NextState("Inward");
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::INWARD;
    return;
  }

  // 动态响应
  if (final_trajectory_.trajectory_cost().is_fully_block &&
      final_trajectory_.trajectory_cost().closest_obj_velocity >
          planning_params_->threshold_static_speed()) {
    if ((final_trajectory_.trajectory_cost().dynamic_objects_pose ==
         DynamicObjectsPose::D_OBJ_FORWARD) ||
        (final_trajectory_.trajectory_cost().dynamic_objects_pose ==
         DynamicObjectsPose::D_OBJ_LEFTUP) ||
        (final_trajectory_.trajectory_cost().dynamic_objects_pose ==
         DynamicObjectsPose::D_OBJ_RIGHTUP)) {
      // double delta_lateral_dis = 0.0;
      // if (frame_.LastPlanningTrajectory()
      //         .trajectory_cost()
      //         .closest_obj_lateral_distance < 999) {
      //   delta_lateral_dis =
      //       frame_.LastPlanningTrajectory()
      //           .trajectory_cost()
      //           .closest_obj_lateral_distance -
      //       final_trajectory_.trajectory_cost().closest_obj_lateral_distance;
      // }
      if (planning_params_->enable_cutin()) {
        if ((fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) <
             M_PI / 6.0) &&
            (final_trajectory_.trajectory_cost().closest_obj_lateral_distance <=
                 selector_params().cut_in_state_boundary() &&
             final_trajectory_.trajectory_cost().closest_obj_lateral_distance >
                 selector_params().acc_state_boundary()) &&
            (current_behaviour_.relative_distance <
             selector_params().obstacle_speed_multiplier() *
                 current_behaviour_.follow_velocity ||
                 current_behaviour_.relative_distance <
             selector_params().obstacle_speed_multiplier() *
                 current_velocity_)/*&&
            (delta_lateral_dis >= 0 && delta_lateral_dis < 0.5)*/) {
          behaviour_lon_state_sm_->NextState("Cut_In");
          relative_length_smooth_ = current_velocity_ * selector_params().acc_collision_time();
          current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::CUT_IN;
          return;
        }
      }
      if (planning_params_->enable_following()) {
        if ((fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) < M_PI / 6.0) &&
            (final_trajectory_.trajectory_cost().closest_obj_lateral_distance <=
                 selector_params().acc_state_boundary() &&
             final_trajectory_.trajectory_cost().closest_obj_lateral_distance >= 0.0)) {
          if ((current_velocity_ / final_trajectory_.trajectory_cost().closest_obj_velocity) <
                  0.7 &&
              current_behaviour_.collision_distance >= vehicle_param_.length()) {
            tar_accel_ =
                (final_trajectory_.trajectory_cost().closest_obj_velocity - current_velocity_) /
                selector_params().norm_acc_time();
            if (tar_accel_ > planning_params_->norm_acceleration())
              tar_accel_ = planning_params_->norm_acceleration();
            return;
          } else {
            // std::cout << "Follow1111111111111111111" << endl;
            behaviour_lon_state_sm_->NextState("Follow");
            // current_behaviour_.lon_state =
            //     legionclaw::interface::ADCTrajectory::BehaviourLonState::FOLLOW;
            return;
          }
        }
      }
      if (fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) > 0.75 * M_PI) {
        behaviour_lon_state_sm_->NextState("OnComing");
        current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::ONCOMING;
        return;
      }
      if (fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) >= M_PI / 6.0 &&
          fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) <= 0.75 * M_PI) {
        behaviour_lon_state_sm_->NextState("Crossing");
        current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::CROSSING;
        return;
      }
      // 切出逻辑,切出距离小于最小安全时距进行减速保护
      //  double delta_v = current_behaviour_.follow_velocity - current_velocity_;
      if (current_behaviour_.collision_distance <
              current_velocity_ * (selector_params_.acc_collision_time() -
                                   selector_params_.acc_t_immerse())/* &&
          delta_v < selector_params_.acc_delta_0_v() / 2*/) {
        double stop_length = current_behaviour_.collision_distance;
        if (stop_length <= 0) stop_length = 0.1;
        tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
        // TODO:优化
        // 处理距离停止线比较远，车速低甚至反馈车速为零时，车辆停车问题
        double min_speed = sqrt(-2 * planning_params_->norm_deceleration() * stop_length);
        if (current_velocity_ <= min_speed &&
            stop_length > selector_params().distance_stop_earlier()) {
          tar_accel_ =
              planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
          stop_earlier_ = true;
        }
        return;
      }
    } else {
    }
  }
  // 加速度计算
  if ((common_params()->enable_traffic_light_behavior() && ptr_conditions_->is_trafficIs_red_) ||
      (common_params()->enable_stop_sign_behavior())) {
    double stop_length = ptr_conditions_->MinDistanceToStop();
    if (stop_length <= 0) stop_length = 0.1;
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
    // TODO:优化 处理距离停止线比较远，车速低甚至反馈车速为零时，车辆停车问题
    double min_speed = sqrt(-2 * planning_params_->norm_deceleration() * stop_length);
    if (current_velocity_ <= min_speed && stop_length > selector_params().distance_stop_earlier()) {
      tar_accel_ =
          planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
      stop_earlier_ = true;
    }
  } else {
    if (target_velocity_ > current_velocity_) {
      tar_accel_ = (target_velocity_ - current_velocity_) / selector_params().norm_acc_time();
      if (tar_accel_ > planning_params_->norm_acceleration())
        tar_accel_ = planning_params_->norm_acceleration();
    } else if (target_velocity_ < current_velocity_) {
      tar_accel_ = (target_velocity_ - current_velocity_) / selector_params().norm_dec_time();
      if (tar_accel_ < planning_params_->norm_deceleration())
        tar_accel_ = planning_params_->norm_deceleration();
      if (ptr_conditions_->is_limit_speed_kappa_) {
        tar_accel_ = max(ptr_conditions_->limit_speed_kappa_deceleration_,
                         planning_params_->norm_deceleration());
      }
    } else {
      tar_accel_ = 0.0;
    }
  }
}

void BehaviourSelector::PreciseStopStateUpdate(const std::string &state_name, int state)
{
  //
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::PRECISE_STOP;
  double trajectory_length = final_trajectory_.trajectory_points().back().path_point().s() -
                             final_trajectory_.trajectory_points().front().path_point().s();
  if (ptr_conditions_->goal_distance_ > trajectory_length)  // 站点在轨迹的前面
    ptr_conditions_->goal_distance_trajectory_ = ptr_conditions_->goal_distance_;
  else
    BehaviourSelector::ComputeGoalDistanceOnTrajectory(
        final_trajectory_.trajectory_points(),
        {terminal_stop_point_.point().x(), terminal_stop_point_.point().y()},
        terminal_stop_point_.theta(),  // TODO:暂时认为只有一个停车点
        ptr_conditions_->goal_distance_trajectory_);
  double stop_length =
      std::min(ptr_conditions_->MinDistanceToStop(), ptr_conditions_->goal_distance_trajectory_);
  // 匿名函数
  auto PreciseStopToNormal = [=]() -> bool {
    if (GoalDistanceLess()) {
      if (frame_.VehicleState().speed >= common_params()->threshold_static_speed_self()) {
        return false;
      }
      if (stop_length <= selector_params().distance_stop_earlier()) {
        return false;
      }
      // 上述两个case应该不是GoalDistanceLess()为true下的全集？
    }
    return true;
  };

  // 站点停车刹车计算
  tar_accel_ =
      -0.5 * current_velocity_ * current_velocity_ / std::max(ptr_conditions_->goal_distance_, 0.1);
  // 站点停车状态下，有障碍物侵入
  if (stop_length <= 0) stop_length = 0.1;
  if (ptr_conditions_->closest_obj_velocity_ <= planning_params_->threshold_static_speed()) {
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  } else {
    tar_accel_ = std::min(tar_accel_, (ptr_conditions_->closest_obj_velocity_ - current_velocity_) /
                                          selector_params().norm_acc_time());
  }
  // TODO往前摆正过程中，存在障碍物的场景待优化
  // 若前方车辆停不正，纵向控制策略：固定速度行驶怠速5码，满足车身停正条件后固定刹车值停车
  if (stop_length < selector_params().additional_braking_distance() &&
      (ptr_conditions_->goal_distance_ < ptr_conditions_->closest_obj_collision_dis_ -
                                             selector_params().additional_braking_distance())) {
    // case1:自车前方安全范围内，无静止/同向运动的障碍物
    if (ptr_conditions_->closest_obj_collision_dis_ >=
        selector_params().additional_braking_distance()) {
      // 计算偏离目标车道的横向偏移量
      if ((frame_.MapMatchInfo().lat_offset >= 0.2 ||
           frame_.ReferenceLines().back().ReferencePoints().size() > 0) &&
          final_trajectory_.trajectory_points().back().path_point().s() >
              planning_params_->emergency_lower_limit_length() + 4) {
        double speed_in_idle = 1.4;
        if (speed_in_idle > current_velocity_) {
          tar_accel_ = vehicle_param_.acceleration_in_idle();
        } else if (speed_in_idle < current_velocity_) {
          tar_accel_ = vehicle_param_.deceleration_in_idle();
        } else {
          tar_accel_ = 0.0;
        }
      } else {
        tar_accel_ = vehicle_param_.standstill_acceleration();
      }
    } else {  // case2:自车前方存在危险障碍物
      tar_accel_ = std::min(-0.5 * current_velocity_ * current_velocity_ / 0.1,
                            vehicle_param_.standstill_acceleration());
    }
  }
  if (PreciseStopToNormal()) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
}

void BehaviourSelector::StopStateUpdate(const std::string &state_name, int state)
{
  //
  double min_stopping_distance =
      selector_params_.additional_braking_distance() -
      0.5 * pow(ahead_map_limit_speed_, 2) / (planning_params_->norm_deceleration());
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::STOP;
  double stop_length = ptr_conditions_->MinDistanceToStop();
  auto StopToNormal = [=]() -> bool {
    // case1:到站状态但非精准到站禁止切Normal
    if (ptr_conditions_->goal_distance_ < min_stopping_distance &&
        ptr_conditions_->goal_distance_ > 10) {
      return false;
    }
    // case2:后方来车让行减速禁止切Normal
    if (ptr_conditions_->stop_to_wait_change_lane_ == true) {
      return false;
    }
    // case3:无碰撞风险，切normal状态
    if (!final_trajectory_.trajectory_cost().is_fully_block) {
      return true;
    }
    // case4: 有碰撞风险后方障碍物禁止切normal
    if (final_trajectory_.trajectory_cost().dynamic_objects_pose ==
        DynamicObjectsPose::D_OBJ_LEFTDOWN) {
      return false;
    }
    if (final_trajectory_.trajectory_cost().dynamic_objects_pose ==
        DynamicObjectsPose::D_OBJ_RIGHTDOWN) {
      return false;
    }
    // case5: 有碰撞风险，但是前方障碍物速度大于阈值且拉开一定距离则切normal,进入跟车状态
    if (final_trajectory_.trajectory_cost().closest_obj_velocity >
            planning_params_->threshold_static_speed() &&
        current_behaviour_.relative_distance > DistStopToNormal()) {
      return true;
    }
    // TODO
    // 有碰撞风险，但停的较远场景下，优化停车距离,暂时用stop_early去调整stop状态下的停车距离
    if (current_velocity_ == 0 && current_behaviour_.relative_distance > DistStopToNormal()) {
    }
    return false;
  };
  if (ptr_conditions_->stop_to_wait_change_lane_ == true) {
    tar_accel_ = vehicle_param_.max_deceleration() / 2.0;
    return;
  }

  // 站点停车刹车计算
  if (GoalDistanceLess()) {
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ /
                 std::max(ptr_conditions_->goal_distance_, 0.1);
  }
  // 站点停车状态下，有障碍物侵入
  if (stop_length <= 0) stop_length = 0.1;
  if (ptr_conditions_->closest_obj_velocity_ <= planning_params_->threshold_static_speed()) {
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  } else {
    tar_accel_ = std::min(tar_accel_, (ptr_conditions_->closest_obj_velocity_ - current_velocity_) /
                                          selector_params().norm_acc_time());
  }
  // TODO:车速低甚至反馈车速为零时，车辆停车问题
  double min_speed = sqrt(-2 * planning_params_->norm_deceleration() * stop_length);
  // if (current_velocity_ <= min_speed &&
  //     stop_length > selector_params().additional_braking_distance()) {
  //   tar_accel_ = planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
  //   stop_earlier_ = true;
  // }
  if((current_velocity_ <= min_speed) && (stop_length > selector_params().additional_braking_distance())) {
    if(current_velocity_ <= planning_params_->threshold_static_speed()) {
      tar_accel_ = planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
    } else {
      tar_accel_ = 0.0;
    }

    stop_earlier_ = true;
  }

  // if (final_trajectory_.trajectory_cost().id == "greenbelt_occlusion_stop_wall") {
  //   if (stop_length <= selector_params().additional_braking_distance() && stop_length >= 1) {
  //     if (min_speed > current_velocity_) {
  //       tar_accel_ = vehicle_param_.acceleration_in_idle();
  //     } else if (min_speed < current_velocity_) {
  //       tar_accel_ = vehicle_param_.deceleration_in_idle();
  //     } else {
  //       tar_accel_ = 0.0;
  //     }
  //   }
  // }

  if (StopToNormal()) {
    tar_accel_ = frame_.LastTrajectoryPoints(0).a();
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
}

void BehaviourSelector::TrafficSignStopStateUpdate(const std::string &state_name, int state)
{
  if (lon_state_ == SelectorLonFlag::NORMAL) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  // TODO: LON_INVALID的必要性
  lon_state_ = SelectorLonFlag::LON_INVALID;
  current_behaviour_.lon_state =
      legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_SIGN_STOP_STATE;
  // 匿名函数
  auto TrafficSignStopToWaitState = [=]() -> bool {
    if (!common_params()->enable_stop_sign_behavior()) {
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (ptr_conditions_->current_stop_sign_id_ == 0) {
      ADEBUG << "  ";
      return false;
    }
    if (ptr_conditions_->current_stop_sign_id_ != ptr_conditions_->prev_stop_sign_id_) {
      ADEBUG << "  ";
      return false;
    }
    if (!ptr_conditions_->is_static_) {
      ADEBUG << "  ";
      return false;
    }
    return true;
  };
  if (ptr_conditions_->current_stop_sign_id_ == 0) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  if (TrafficSignStopToWaitState()) {
    behaviour_lon_state_sm_->NextState("TrafficSignWait");
    current_behaviour_.lon_state =
        legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_SIGN_WAIT_STATE;
    return;
  }
}

void BehaviourSelector::TrafficSignWaitStateUpdate(const std::string &state_name, int state)
{
  //
  // if (TimeTool::GetTimeDiffNow(state_timer_) < decision_making_time_) return;
  current_behaviour_.lon_state =
      legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_SIGN_WAIT_STATE;
  ptr_conditions_->prev_stop_sign_id_ = ptr_conditions_->current_stop_sign_id_;
  lon_state_ = SelectorLonFlag::NORMAL;
  behaviour_lon_state_sm_->NextState("Normal");
  return;
}

void BehaviourSelector::TrafficLightStopStateUpdate(const std::string &state_name, int state)
{
  if (lon_state_ == SelectorLonFlag::NORMAL) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  lon_state_ = SelectorLonFlag::LON_INVALID;
  current_behaviour_.lon_state =
      legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_STOP_STATE;
  // 匿名函数
  auto TrafficLightStopToWaitState = [=]() -> bool {
    if (ptr_conditions_->distance_to_stop_line_ >= selector_params().distance_satisfy_wait()) {
      // ADEBUG << "  ";
      return false;
    }
    if (!ptr_conditions_->is_trafficIs_red_) {
      // ADEBUG << "  ";
      return false;
    }
    if (!ptr_conditions_->is_static_) {
      // ADEBUG << "  ";
      return false;
    }
    return true;
  };
  if (!ptr_conditions_->is_trafficIs_red_) {
    ptr_conditions_->pre_traffic_light_id_ = ptr_conditions_->cur_traffic_light_id_;
    current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  if (TrafficLightStopToWaitState()) {
    behaviour_lon_state_sm_->NextState("TrafficLightWait");
    current_behaviour_.lon_state =
        legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_WAIT_STATE;
    return;
  }
  // TODO 添加虚拟障碍物
  double stop_length = ptr_conditions_->MinDistanceToStop();
  if (stop_length <= 0) stop_length = 0.1;
  tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  // if (fabs(stop_length - ptr_conditions_->distance_to_stop_line_) < 0.5 ||
  //     ptr_conditions_->closest_obj_velocity_ < current_velocity_ ||
  //     ptr_conditions_->closest_obj_velocity_<0.1) {
  //   tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  // } else {
  //   tar_accel_ = frame_.LastTrajectoryPoints(0).a();
  // }

  // stop状态减速度保护
  if (current_velocity_ == 0.0) tar_accel_ = vehicle_param_.standstill_acceleration();

  // TODO:优化 处理距离停止线比较远，车速低甚至反馈车速为零时，车辆停车问题
  double min_speed = sqrt(-2 * planning_params_->norm_deceleration() * stop_length);
  if (current_velocity_ <= min_speed && stop_length > selector_params().distance_stop_earlier()) {
    tar_accel_ = planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
    stop_earlier_ = true;
  }

  // 红绿灯状态跟车起步场景，距离障碍物距离较远，障碍物跟车距离调整
  if (current_behaviour_.relative_distance < ptr_conditions_->distance_to_stop_line_ - 0.2) {
    double final_length = selector_params_.tjp_base_length() +
                          selector_params_.tjp_collision_time() * current_velocity_;
    if (current_velocity_ < current_behaviour_.follow_velocity &&
        current_behaviour_.relative_distance > final_length) {
      tar_accel_ =
          planning_params_->norm_acceleration() * selector_params().norm_acceleration_rate();
    }
  }

  return;
}

void BehaviourSelector::TrafficLightWaitStateUpdate(const std::string &state_name, int state)
{
  //
  current_behaviour_.lon_state =
      legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_WAIT_STATE;
  if (!ptr_conditions_->is_trafficIs_red_) {
    ptr_conditions_->pre_traffic_light_id_ = ptr_conditions_->cur_traffic_light_id_;
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("TrafficLightStop");
    return;
  }
  double stop_length = ptr_conditions_->MinDistanceToStop();
  if (stop_length <= 0) stop_length = 0.1;
  tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  // 停车状态减速度保护
  if (current_velocity_ == 0.0) tar_accel_ = vehicle_param_.standstill_acceleration();
}

void BehaviourSelector::FollowStateUpdate(const std::string &state_name, int state)
{
  auto ToTJPState = [=]() -> bool {
    // 等待3s，开启TJP功能
    if (TimeTool::GetTimeDiffNow(pre_acc_to_tjp_time_) <= selector_params().pre_acc_to_tjp_time()) {
      // AWARN << "ACC to TJP time is too short";
      return false;
    }
    return true;
  };
  if (lon_state_ == SelectorLonFlag::NORMAL) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  if (!(current_velocity_ < selector_params().acc_activate_speed() &&
        current_behaviour_.follow_velocity < selector_params().acc_activate_obj_speed())) {
    pre_acc_to_tjp_time_ = TimeTool::NowToSeconds();
    behaviour_lon_state_sm_->NextState("ACC");
    return;
  } else if (!ToTJPState()) {
    behaviour_lon_state_sm_->NextState("ACC");
    return;
  } else {
    behaviour_lon_state_sm_->NextState("TJP");
    return;
  }
}

void BehaviourSelector::ACCStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::ACC;
  if (ptr_conditions_->is_outward_) {
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }

  if (ptr_conditions_->is_trafficIs_red_ &&
      ptr_conditions_->distance_to_stop_line_ <= current_behaviour_.relative_distance) {
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }
  if (final_trajectory_.trajectory_cost().closest_obj_velocity <=
          planning_params_->threshold_static_speed() ||
      (final_trajectory_.trajectory_cost().closest_obj_lateral_distance >
       selector_params().acc_state_boundary())) {
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }

  double free_length = current_behaviour_.collision_distance;
  // 碰撞距离free_length转换成相对距离relative_length_
  double relative_length = current_behaviour_.relative_distance;
  double follow_v = current_behaviour_.follow_velocity;
  double delta_v = follow_v - current_velocity_;
  if (relative_length <= 0) relative_length = free_length;
  // 期望跟车时距
  double final_time = selector_params().acc_collision_time();
  double final_length = current_velocity_ * final_time;
  double l_gain = selector_params_.acc_delta_length_gain();
  double v_gain = selector_params_.acc_delta_speed_gain();

  // // 速度分阶
  // double temp_v0 = 10.0;
  // double temp_v1 = 20.0;
  // // 不同速度下的期望时距(时间)
  // double temp_t0 = 2.0;
  // double temp_t1 = 1.6;
  // double temp_t2 = 1.3;
  // double temp_dt = 0.0;
  // // 计算期望距离
  // if (current_velocity_ <= temp_v0) {
  //   final_length = current_velocity_ * temp_t0;
  //   final_time = temp_t0;
  // } else if (current_velocity_ <= temp_v1) {
  //   final_length = temp_v0 * temp_t0 + (current_velocity_ - temp_v0) * temp_t1;
  //   final_time = (temp_t0 + temp_t1) / 2.0;
  // } else {
  //   final_length =
  //       temp_v0 * temp_t0 + (temp_v1 - temp_v0) * temp_t1 + (current_velocity_ - temp_v1) * temp_t2;
  //   final_time = (temp_t0 + temp_t1 + temp_t2) / 3.0;
  // }

  // if (delta_v <= -selector_params_.acc_delta_0_v()) {
  //   final_length += current_velocity_ * selector_params_.acc_t_0_compensate();
  // }
  // // else if (delta_v <= (-selector_params_.acc_delta_0_v()/2.0)) {

  // // }
  // else if (delta_v <= 0) {
  //   final_length += current_velocity_ * selector_params_.acc_t_0_compensate() *
  //                   (-delta_v / selector_params_.acc_delta_0_v());
  // }
  // // else if(delta_v <= (selector_params_.acc_delta_0_v() / 2.0)) {

  // // }
  // else if (delta_v <= selector_params_.acc_delta_0_v()) {
  //   final_length -= current_velocity_ * selector_params_.acc_t_0_compensate() *
  //                   (delta_v / selector_params_.acc_delta_0_v());
  // } else {
  //   final_length -= current_velocity_ * selector_params_.acc_t_0_compensate();
  // }

  // final_time = -0.1 * current_velocity_ + 3;
  // final_length = -0.1 * 0.5 * current_velocity_ * current_velocity_ + 3 * current_velocity_;

  final_length = selector_params().acc_min_length() + final_length -
                 ((current_velocity_ * delta_v) / (2 * std::sqrt(5 * 2)));

  // if ((delta_v / current_velocity_) >= -0.1 && (delta_v / current_velocity_) < 0) {
  //   final_length = selector_params().acc_min_length() + final_length;
  // }
  // // else if ((delta_v / current_velocity_) >= -0.5 && (delta_v / current_velocity_) < -0.1) {
  // //   final_length = selector_params().acc_min_length() + final_length -
  // //                  ((current_velocity_ * delta_v) / (2 * std::sqrt(5 * 2)));
  // // }
  // else if ((delta_v / current_velocity_) < -0.7) {
  //   final_length = selector_params().acc_min_length() -
  //                  ((current_velocity_ * delta_v) / (2 * std::sqrt(5 * 2)));
  // }

  // v_gain = v_gain + 0.2 * (current_velocity_  - follow_v)/ current_velocity_;
  // l_gain = l_gain + 0.2 * (final_length - relative_length) / final_length;

  /*
  // case1： 跟车距离很近
  if (relative_length > 0 && relative_length <= selector_params_.acc_min_length()) {
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_time += selector_params_.acc_t_0_compensate();
      final_length = current_velocity_ * final_time;
      l_gain = selector_params_.acc_delta_3_length_gain();
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      l_gain = selector_params_.acc_delta_3_length_gain();
      final_time +=
          selector_params_.acc_t_0_compensate() * (-delta_v / selector_params_.acc_delta_0_v());
      final_length = current_velocity_ * final_time;
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_time -=
          selector_params_.acc_t_1_compensate() * (delta_v / selector_params_.acc_delta_0_v());
      final_length = current_velocity_ * final_time;
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快
      final_time -= selector_params_.acc_t_2_compensate();
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_2_compensate();
    }
  } else if (relative_length > selector_params_.acc_min_length() &&
             relative_length <= current_velocity_) {
    // case2： 跟车距离小于1s时距
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_time += selector_params_.acc_t_00_compensate();
      final_length = current_velocity_ * final_time;
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      final_time +=
          selector_params_.acc_t_00_compensate() * (-delta_v / selector_params_.acc_delta_0_v());
      final_length = current_velocity_ * final_time;
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_time -=
          selector_params_.acc_t_11_compensate() * (delta_v / selector_params_.acc_delta_0_v());
      final_length = current_velocity_ * final_time;
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快
      final_time -= selector_params_.acc_t_22_compensate();
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_22_compensate();
    }
  } else if (relative_length > current_velocity_ &&
             relative_length <= current_velocity_ * selector_params_.acc_collision_time() -
                                    current_velocity_ * selector_params_.acc_t_immerse()) {
    // case3： 跟车距离大于1s时距，小于1.5s时距
    double final_length_max = current_velocity_ * (selector_params_.acc_collision_time() -
                                                   selector_params_.acc_t_immerse());
    // 前车速度较慢场景距离调节因子平滑优化
    if (delta_v <= 0) {
      if (follow_v > 16) {
      } else if (follow_v > 8 && follow_v <= 16) {
        l_gain += (selector_params_.acc_delta_3_length_gain() - l_gain) * 8.0 / follow_v;
      } else {
        l_gain = selector_params_.acc_delta_3_length_gain();
      }
    }
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_length += current_velocity_ * selector_params_.acc_t_000_compensate();
    } else if (delta_v <= -selector_params_.acc_delta_0_v() / 2.0 &&
               delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      if (follow_v > 16) {
        final_length =
            relative_length + current_velocity_ * (selector_params_.acc_t_000_compensate() *
                                                   (-delta_v / selector_params_.acc_delta_0_v()));
      } else if (follow_v > 8 && follow_v <= 16) {
        final_length =
            relative_length + current_velocity_ * selector_params_.acc_t_000_compensate() *
                                  (-delta_v / selector_params_.acc_delta_0_v()) * 8.0 / follow_v;
      } else {
        final_length = relative_length + current_velocity_ *
                                             selector_params_.acc_t_000_compensate() *
                                             (-delta_v / selector_params_.acc_delta_0_v());
      }
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v() / 2.0) {
      // c：前车速度较慢
      if (follow_v > 16) {
        final_length = relative_length + current_velocity_ *
                                             selector_params_.acc_t_111_compensate() *
                                             (-delta_v / selector_params_.acc_delta_0_v());
      } else if (follow_v > 8 && follow_v <= 16) {
        final_time += selector_params_.acc_t_111_compensate() *
                      (-delta_v / selector_params_.acc_delta_0_v()) * 8.0 / follow_v;
        final_length = current_velocity_ * final_time;
      } else {
        final_time +=
            selector_params_.acc_t_111_compensate() * (-delta_v / selector_params_.acc_delta_0_v());
        final_length = current_velocity_ * final_time;
      }
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_length =
          relative_length + current_velocity_ * (selector_params_.acc_t_111_compensate() *
                                                 (1 - delta_v / selector_params_.acc_delta_0_v()));
      if (final_length > final_length_max) {
        final_length = final_length_max;
      }
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快;
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_222_compensate() *
                                           delta_v / selector_params_.acc_delta_0_v();
    }
  } else if (relative_length > current_velocity_ * selector_params_.acc_collision_time() -
                                   current_velocity_ * selector_params_.acc_t_immerse() &&
             relative_length <= current_velocity_ * final_time) {
    // case4： 跟车距离大于1.5s时距，小于final_time
    if (delta_v < 0) {
      // TODO 按比例去调节
      if (delta_v > -selector_params_.acc_delta_0_v() / 2.0) {
        // 前车较慢
        if (follow_v > 16) {
          final_length =
              relative_length + current_velocity_ * (selector_params_.acc_t_111_compensate() *
                                                     (-delta_v / selector_params_.acc_delta_0_v()));
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else if (follow_v > 8 && follow_v <= 16) {
          final_length = current_velocity_ * final_time;
        } else {
          final_time += selector_params_.acc_t_111_compensate() *
                        (-delta_v / selector_params_.acc_delta_0_v());
          final_length = current_velocity_ * final_time;
        }
      } else if (delta_v <= -selector_params_.acc_delta_0_v() / 2.0 &&
                 delta_v > -selector_params_.acc_delta_0_v()) {
        // 前车较慢
        if (follow_v > 16) {
          v_gain = selector_params_.acc_delta_1_speed_gain();
          final_length = relative_length;
        } else {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
          final_length = relative_length + current_velocity_ *
                                               selector_params_.acc_t_111_compensate() *
                                               (-delta_v / selector_params_.acc_delta_0_v());
        }
      } else {
        // 前车很慢
        if (follow_v > 16) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
          final_length = relative_length;
        } else {
          l_gain = selector_params_.acc_delta_3_length_gain();
          v_gain = selector_params_.acc_delta_3_speed_gain();
          final_length =
              relative_length + current_velocity_ * selector_params_.acc_t_000_compensate();
        }
      }
    } else {
      // 前车较快
      l_gain = selector_params_.acc_delta_1_length_gain();
      v_gain = selector_params_.acc_delta_1_speed_gain();
      final_length = relative_length;
    }
  } else if (relative_length > current_velocity_ * final_time &&
             relative_length <= current_velocity_ * 3) {
    // case5： 跟车距离大于final_time时距，小于3s时距
    if (delta_v < 0) {
      if (follow_v > 0 && follow_v <= 8) {
        // final_length = relative_length;
        if (delta_v >= -1) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else if (delta_v >= -2 && delta_v < -1) {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        } else {
          l_gain = selector_params_.acc_delta_3_length_gain();
          v_gain = selector_params_.acc_delta_3_speed_gain();
        }
      } else if (follow_v > 8 && follow_v <= 16) {
        if (delta_v > -selector_params_.acc_delta_0_v()) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        }
      } else if (follow_v > 16) {
        v_gain = selector_params_.acc_delta_1_speed_gain();
      }
    } else {
      final_length = relative_length;
      v_gain = selector_params_.acc_delta_1_speed_gain();
    }
  } else if (relative_length > current_velocity_ * 3) {
    // case6： 跟车距离大于3s时距
    if (delta_v < 0) {
      if (follow_v > 0 && follow_v <= 8) {
        // final_length = relative_length;
        if (delta_v >= -selector_params_.acc_delta_0_v()) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else if (delta_v >= -selector_params_.acc_delta_0_v() * 2 &&
                   delta_v < -selector_params_.acc_delta_0_v()) {
          l_gain = selector_params_.acc_delta_1_length_gain() +
                   (selector_params_.acc_delta_3_length_gain() -
                    selector_params_.acc_delta_1_length_gain()) /
                       selector_params_.acc_delta_0_v() *
                       (-delta_v - selector_params_.acc_delta_0_v());
          v_gain = selector_params_.acc_delta_1_speed_gain() +
                   (selector_params_.acc_delta_3_speed_gain() -
                    selector_params_.acc_delta_1_speed_gain()) /
                       selector_params_.acc_delta_0_v() *
                       (-delta_v - selector_params_.acc_delta_0_v());
        } else {
          l_gain = selector_params_.acc_delta_3_length_gain();
          v_gain = selector_params_.acc_delta_3_speed_gain();
        }
      } else if (follow_v > 8) {
        if (delta_v > -selector_params_.acc_delta_0_v() * 2) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_length_gain();
        } else {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        }
      }
    } else {
      final_length = relative_length;
      v_gain = selector_params_.acc_delta_1_speed_gain();
    }
  }
  */
  // 自适应巡航(跟车):根据前车速度、与前车距离计算的规划推荐加速度等信息
  bool b_acc;
  double tar_speed, tar_length;
  // double acc_min_length = current_behaviour_.follow_velocity * 3.6;
  b_acc = AdaptiveCruiseControl(
      final_length, relative_length, current_velocity_, current_behaviour_.follow_velocity,
      selector_params().acc_min_length(), final_time, l_gain, v_gain,
      selector_params().acc_prediction_time(), tar_speed, tar_length, tar_accel_);
  if (b_acc != true) {
    // AERROR << "<ACC State>: Accelation Computed Error !";
  }
  // TODO 根据障碍物的减速度进行划分计算方法
  if (current_behaviour_.follow_acceleration < -1.0 && current_behaviour_.follow_velocity <= 3 &&
      delta_v < -2 * selector_params_.acc_delta_0_v()) {
    // 按静止障碍物停车计算方法进行处理
    double stop_length =
        current_behaviour_.relative_distance - selector_params_.additional_braking_distance();
    if (stop_length <= 0) stop_length = 0.1;
    tar_accel_ = std::min(tar_accel_, -0.5 * current_velocity_ * current_velocity_ / stop_length);
  }
  if (current_velocity_ < selector_params().acc_activate_speed() &&
      current_behaviour_.follow_velocity < selector_params().acc_activate_obj_speed()) {
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }
}

void BehaviourSelector::TJPStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::TJP;
  if (ptr_conditions_->is_trafficIs_red_ &&
      ptr_conditions_->distance_to_stop_line_ <= current_behaviour_.relative_distance) {
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }
  double final_length = selector_params_.tjp_base_length() +
                        selector_params_.tjp_collision_time() * current_velocity_;
  if (final_length < selector_params_.tjp_min_length())
    final_length = selector_params_.tjp_min_length();
  double relative_length = current_behaviour_.relative_distance;
  double free_length = current_behaviour_.collision_distance;
  double delta_v = current_behaviour_.follow_velocity - current_velocity_;
  // 碰撞距离free_length转换成相对距离relative_length_
  if (relative_length <= 0) relative_length = free_length;
  bool b_acc;
  double tar_speed, tar_length;
  double l_gain = selector_params_.tjp_delta_length_gain();
  double v_gain = selector_params_.tjp_delta_speed_gain();
  if (final_trajectory_.trajectory_cost().dynamic_objects_pose !=
      DynamicObjectsPose::D_OBJ_FORWARD) {
    l_gain = selector_params_.acc_delta_1_length_gain();
    v_gain = selector_params_.acc_delta_1_speed_gain();
  }
  if (delta_v < -selector_params_.acc_delta_0_v() / 2) {
    if (current_behaviour_.follow_velocity <= 3) {
      final_length += current_velocity_ * selector_params_.acc_t_0_compensate() *
                      (-delta_v / selector_params_.acc_delta_0_v());
    }
  }
  b_acc = AdaptiveCruiseControl(
      final_length, relative_length, current_velocity_, current_behaviour_.follow_velocity,
      selector_params_.tjp_min_length(), selector_params_.tjp_collision_time(), l_gain, v_gain,
      selector_params_.acc_prediction_time(), tar_speed, tar_length, tar_accel_);
  if (b_acc != true) {
    // AERROR << "<TJP State>: Accelation Computed Error !";
  }
  if (final_trajectory_.trajectory_cost().closest_obj_velocity <=
          planning_params_->threshold_static_speed() ||
      (final_trajectory_.trajectory_cost().closest_obj_lateral_distance >
       selector_params_.acc_state_boundary())) {
    lon_state_ = SelectorLonFlag::NORMAL;
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }
  if (!(current_velocity_ < selector_params_.acc_activate_speed() &&
        current_behaviour_.follow_velocity < selector_params().acc_activate_obj_speed())) {
    behaviour_lon_state_sm_->NextState("Follow");
    return;
  }
}

void BehaviourSelector::CutInStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::CUT_IN;
  double relative_length = current_behaviour_.relative_distance;
  double follow_v = current_behaviour_.follow_velocity;
  double delta_v = follow_v - current_velocity_;
  if (current_behaviour_.follow_velocity <= 0) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  if (delta_v > 0 && relative_length >= selector_params().obstacle_speed_multiplier() * follow_v) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }

  if ((!(relative_length < selector_params().obstacle_speed_multiplier() * follow_v ||
         relative_length < selector_params().obstacle_speed_multiplier() * current_velocity_)) ||
      (final_trajectory_.trajectory_cost().closest_obj_lateral_distance >
           selector_params_.cut_in_state_boundary() ||
       final_trajectory_.trajectory_cost().closest_obj_lateral_distance <=
           selector_params_.acc_state_boundary())) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }

  double free_length = current_behaviour_.collision_distance;
  // 碰撞距离free_length转换成相对距离relative_length_
  if (relative_length <= 0 && delta_v > selector_params_.acc_delta_0_v())
    relative_length = free_length;
  // 期望跟车时距
  double final_time = selector_params().acc_collision_time();
  double final_length = current_velocity_ * final_time;
  double l_gain = selector_params_.acc_delta_length_gain();
  double v_gain = selector_params_.acc_delta_speed_gain();
  // case0:  相对距离很近，但是碰撞距离较远
  if (relative_length < 0) {
    if (free_length > final_length) {
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_0_compensate() *
                                           (1 - delta_v / selector_params_.acc_delta_0_v());
    }
  }
  // case1： 跟车距离很近
  if (relative_length > 0 && relative_length <= selector_params_.acc_min_length()) {
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_time += selector_params_.acc_t_0_compensate();
      final_length = current_velocity_ * final_time;
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_1_compensate() *
                                           (-delta_v / selector_params_.acc_delta_0_v());
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_length = relative_length +
                     current_velocity_ * (selector_params_.acc_t_0_compensate() +
                                          selector_params_.acc_t_1_compensate() *
                                              (1 - delta_v / selector_params_.acc_delta_0_v()));
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_2_compensate();
    }
  } else if (relative_length > selector_params_.acc_min_length() &&
             relative_length <= current_velocity_) {
    // case2： 跟车距离小于1s时距
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_length = relative_length + current_velocity_ * (selector_params_.acc_t_00_compensate() +
                                                            selector_params_.acc_t_11_compensate() +
                                                            selector_params_.acc_t_22_compensate());
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      final_length =
          relative_length + current_velocity_ * (selector_params_.acc_t_11_compensate() +
                                                 selector_params_.acc_t_22_compensate() *
                                                     (-delta_v / selector_params_.acc_delta_0_v()));
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_length = relative_length +
                     current_velocity_ * (selector_params_.acc_t_11_compensate() +
                                          selector_params_.acc_t_22_compensate() *
                                              (1 - delta_v / selector_params_.acc_delta_0_v()));
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_22_compensate();
    }
  } else if (relative_length > current_velocity_ &&
             relative_length <= current_velocity_ * selector_params_.acc_collision_time() -
                                    current_velocity_ * selector_params_.acc_t_immerse()) {
    // case3： 跟车距离大于1s时距，小于1.5s时距
    if (delta_v <= -selector_params_.acc_delta_0_v()) {
      // a：前车速度很低
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_000_compensate();
    } else if (delta_v <= -selector_params_.acc_delta_0_v() / 2.0 &&
               delta_v > -selector_params_.acc_delta_0_v()) {
      // b：前车速度较慢
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_111_compensate() *
                                           (-delta_v / selector_params_.acc_delta_0_v());
    } else if (delta_v <= 0 && delta_v > -selector_params_.acc_delta_0_v() / 2.0) {
      // b：前车速度较慢
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_111_compensate() *
                                           (-delta_v / selector_params_.acc_delta_0_v());
    } else if (delta_v > 0 && delta_v <= selector_params_.acc_delta_0_v()) {
      // c：前车速度较快
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_222_compensate();
    } else if (delta_v > selector_params_.acc_delta_0_v()) {
      // d：前车速度很快
      final_length = relative_length + current_velocity_ * selector_params_.acc_t_222_compensate() *
                                           delta_v / selector_params_.acc_delta_0_v();
    }
  } else if (relative_length > current_velocity_ * selector_params_.acc_collision_time() -
                                   current_velocity_ * selector_params_.acc_t_immerse() &&
             relative_length <= current_velocity_ * final_time) {
    // case4： 跟车距离大于1.5s时距，小于final_time时距
    if (delta_v < 0) {
      // TODO 按比例去调节
      if (delta_v > -1.0) {
        // 前车较慢
        l_gain = selector_params_.acc_delta_1_length_gain();
        v_gain = selector_params_.acc_delta_1_speed_gain();
        final_length = relative_length + current_velocity_ *
                                             selector_params_.acc_t_111_compensate() *
                                             (-delta_v / selector_params_.acc_delta_0_v());
      } else if (delta_v <= -1.0 && delta_v > -selector_params_.acc_delta_0_v()) {
        // 前车较慢
        l_gain = selector_params_.acc_delta_2_length_gain();
        v_gain = selector_params_.acc_delta_2_speed_gain();
        final_length = relative_length + current_velocity_ *
                                             selector_params_.acc_t_111_compensate() *
                                             (-delta_v / selector_params_.acc_delta_0_v());
      } else {
        // 前车很慢
        l_gain = selector_params_.acc_delta_3_length_gain();
        v_gain = selector_params_.acc_delta_3_speed_gain();
        final_length =
            relative_length + current_velocity_ * selector_params_.acc_t_000_compensate();
      }
    } else {
      // 前车较快
      l_gain = selector_params_.acc_delta_1_length_gain();
      v_gain = selector_params_.acc_delta_1_speed_gain();
      final_length = relative_length;
    }
  } else if (relative_length > current_velocity_ * final_time &&
             relative_length <= current_velocity_ * 3) {
    // case5： 跟车距离大于final_time时距，小于3s时距
    if (delta_v < 0) {
      if (follow_v > 0 && follow_v <= 8) {
        if (delta_v >= -1) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else if (delta_v >= -2 && delta_v < -1) {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        } else {
          l_gain = selector_params_.acc_delta_3_length_gain();
          v_gain = selector_params_.acc_delta_3_speed_gain();
        }
      } else if (follow_v > 8) {
        if (delta_v > -selector_params_.acc_delta_0_v()) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_length_gain();
        } else {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        }
      }
    } else {
      final_length = relative_length;
      v_gain = selector_params_.acc_delta_1_speed_gain();
    }
  } else if (relative_length > current_velocity_ * 3) {
    // case6： 跟车距离大于3s时距
    if (delta_v < 0) {
      if (follow_v > 0 && follow_v <= 8) {
        if (delta_v >= -4) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_speed_gain();
        } else if (delta_v >= -6 && delta_v < -4) {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        } else {
          l_gain = selector_params_.acc_delta_3_length_gain();
          v_gain = selector_params_.acc_delta_3_speed_gain();
        }
      } else if (follow_v > 8) {
        if (delta_v > -selector_params_.acc_delta_0_v() * 2) {
          l_gain = selector_params_.acc_delta_1_length_gain();
          v_gain = selector_params_.acc_delta_1_length_gain();
        } else {
          l_gain = selector_params_.acc_delta_2_length_gain();
          v_gain = selector_params_.acc_delta_2_speed_gain();
        }
      }
    } else {
      final_length = relative_length;
      v_gain = selector_params_.acc_delta_1_speed_gain();
    }
  }

  // 相对距离平滑
  if (relative_length < final_length && relative_length_smooth_ > relative_length) {
    relative_length_smooth_ -= (final_length - relative_length) * 0.05;
    relative_length_smooth_ = std::min(relative_length_smooth_, final_length);
  } else {
    relative_length_smooth_ = relative_length;
  }

  double acc_speed, acc_length, acc_accel;
  bool is_acc = AdaptiveCruiseControl(
      final_length, relative_length_smooth_, current_velocity_, current_behaviour_.follow_velocity,
      4.0, selector_params_.acc_collision_time(), l_gain, v_gain,
      selector_params_.acc_prediction_time(), acc_speed, acc_length, acc_accel);
  if (is_acc != true) {
    // AERROR<< "<CUT IN State>:  ACC's Accelation Computed Error !";
  }
  tar_accel_ = std::min(acc_accel, 0.0);
  // AERROR << "1.5 .acc_accel. : " << acc_accel;
}

void BehaviourSelector::OnComingStateUpdate(const std::string &state_name, int state)
{
  //
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::ONCOMING;
  // oncoming场景根据航向进行优化
  double stop_length = final_trajectory_.trajectory_cost().closest_obj_distance -
                       selector_params().additional_braking_distance();
  if (fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) > M_PI * 5 / 6) {
    stop_length -= selector_params().oncoming_brake_distance_buffer() *
                   final_trajectory_.trajectory_cost().closest_obj_velocity;
  }
  if (stop_length <= 0) stop_length = 0.1;
  tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
  // 停车状态减速度保护
  if (current_velocity_ == 0.0 && stop_length <= 0.1)
    tar_accel_ = vehicle_param_.standstill_acceleration();
  if (fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) <= 0.75 * M_PI) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
}

void BehaviourSelector::CrossingStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::CROSSING;
  double stop_length_crossing = final_trajectory_.trajectory_cost().closest_obj_collision_dis -
                                selector_params().crossing_brake_distance_buffer() *
                                    selector_params().additional_braking_distance();
  double stop_length = std::min(ptr_conditions_->MinDistanceToStop(), stop_length_crossing);
  if (stop_length <= 0) stop_length = 0.1;
  tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;

  if (fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) < M_PI / 6.0 ||
      fabs(final_trajectory_.trajectory_cost().closest_obj_theta_abs) >= 0.75 * M_PI) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
}

// TODO汇入待更新策略
void BehaviourSelector::InwardStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::INWARD;
  // 汇入结束状态
  if (!ptr_conditions_->is_inward_) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }
  // 低速保持状态(creep) distance 10-200
  if (ptr_conditions_->inwardstatus_ == InwardStatus::Inward_Creep) {
    // AWARN << " Inward_Creep.";
    // 自车目标车速 tar_spped = 15 km/h
    double relative_length = current_behaviour_.relative_distance;
    // double final_length = selector_params_.acc_min_length() +
    //                       current_velocity_ * selector_params_.acc_collision_time();
    double final_length =
        selector_params_.acc_min_length() +
        current_velocity_ * selector_params_.acc_collision_time() -
        ((current_velocity_ * (current_behaviour_.follow_velocity - current_velocity_)) /
         (2 * std::sqrt(5 * 2)));
    // 如果小于期望距离，则进入acc模式，否则按目标速度去行驶
    if (relative_length < final_length) {
      double tar_speed, tar_length;
      // double acc_min_length = current_behaviour_.follow_velocity * 3.6;
      bool b_acc = AdaptiveCruiseControl(
          final_length, relative_length, current_velocity_, current_behaviour_.follow_velocity,
          selector_params_.acc_min_length(), selector_params_.acc_collision_time(),
          selector_params_.acc_delta_length_gain(), selector_params_.acc_delta_speed_gain(),
          selector_params_.acc_prediction_time(), tar_speed, tar_length, tar_accel_);
      if (b_acc != true) {
        // AERROR << "<ACC State>: Accelation Computed Error !";
      }
    } else {
      if (selector_params().limit_speed_outwardcreep() > current_velocity_) {
        tar_accel_ = (selector_params().limit_speed_outwardcreep() - current_velocity_) /
                     selector_params().norm_acc_time();
        if (tar_accel_ > planning_params_->norm_acceleration())
          tar_accel_ = planning_params_->norm_acceleration();
      } else {
        tar_accel_ = (selector_params().limit_speed_outwardcreep() - current_velocity_) /
                     selector_params().norm_dec_time();
        // if (tar_accel_ < planning_params_->norm_deceleration())
        //   tar_accel_ = planning_params_->norm_deceleration();
        tar_accel_ = std::min(tar_accel_, -0.5 * current_velocity_ * current_velocity_ /
                                              remain_dist_to_change_right_);
      }
    }
    return;
  }

  // 汇出未完成停车状态 distance<10
  if (ptr_conditions_->inwardstatus_ == InwardStatus::Inward_Stop) {
    AWARN << " Inward_Stop ";
    double stop_length =
        std::min(ptr_conditions_->MinDistanceToStop(), remain_dist_to_change_left_ - 15);
    if (stop_length <= 0) stop_length = 0.1;
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
    // 停车状态减速度保护
    if (current_velocity_ == 0.0) tar_accel_ = vehicle_param_.standstill_acceleration();
    return;
  }
  return;
}

// 汇出
void BehaviourSelector::OutwardStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::OUTWARD;

  // 汇出结束状态
  if (!ptr_conditions_->is_outward_) {
    behaviour_lon_state_sm_->NextState("Normal");
    return;
  }

  // 低速保持状态(creep) distance 10-200
  if (ptr_conditions_->outwardstatus_ == OutwardStatus::Outward_Creep) {
    AWARN << " Outward_Creep.";
    // 自车目标车速 tar_spped = 15 km/h
    double relative_length = current_behaviour_.relative_distance;
    // double final_length = selector_params_.acc_min_length() +
    //                       current_velocity_ * selector_params_.acc_collision_time();
    double final_length =
        selector_params_.acc_min_length() +
        current_velocity_ * selector_params_.acc_collision_time() -
        ((current_velocity_ * (current_behaviour_.follow_velocity - current_velocity_)) /
         (2 * std::sqrt(5 * 2)));
    // 如果小于期望距离则进入acc模式，否则按目标速度去行驶
    if (relative_length < final_length) {
      double tar_speed, tar_length;
      // double acc_min_length = current_behaviour_.follow_velocity * 3.6;
      bool b_acc = AdaptiveCruiseControl(
          final_length, relative_length, current_velocity_, current_behaviour_.follow_velocity,
          selector_params_.acc_min_length(), selector_params_.acc_collision_time(),
          selector_params_.acc_delta_length_gain(), selector_params_.acc_delta_speed_gain(),
          selector_params_.acc_prediction_time(), tar_speed, tar_length, tar_accel_);
      if (b_acc != true) {
        // AERROR << "<ACC State>: Accelation Computed Error !";
      }
    } else {
      if (selector_params().limit_speed_outwardcreep() > current_velocity_) {
        tar_accel_ = (selector_params().limit_speed_outwardcreep() - current_velocity_) /
                     selector_params().norm_acc_time();
        if (tar_accel_ > planning_params_->norm_acceleration())
          tar_accel_ = planning_params_->norm_acceleration();
      } else {
        tar_accel_ = (selector_params().limit_speed_outwardcreep() - current_velocity_) /
                     selector_params().norm_dec_time();
        // if (tar_accel_ < planning_params_->norm_deceleration())
        //   tar_accel_ = planning_params_->norm_deceleration();
        tar_accel_ = std::min(tar_accel_, -0.5 * current_velocity_ * current_velocity_ /
                                              remain_dist_to_change_right_);
      }
    }
    return;
  }

  // 汇出未完成停车状态 distance<10
  if (ptr_conditions_->outwardstatus_ == OutwardStatus::Outward_Stop) {
    AWARN << " Outward_Stop ";
    double stop_length =
        std::min(ptr_conditions_->MinDistanceToStop(), remain_dist_to_change_right_ - 15);
    if (stop_length <= 0) stop_length = 0.1;
    tar_accel_ = -0.5 * current_velocity_ * current_velocity_ / stop_length;
    // 停车状态减速度保护
    if (current_velocity_ == 0.0) tar_accel_ = vehicle_param_.standstill_acceleration();
    return;
  }
  return;
}

}  // namespace planning
}  // namespace legionclaw
