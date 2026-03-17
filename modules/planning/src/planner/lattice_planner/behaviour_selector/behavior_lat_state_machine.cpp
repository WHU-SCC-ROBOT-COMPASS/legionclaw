/// \file behavior_lat_state_machine_planning.cpp
/// \author Hatem Darweesh
/// \brief OpenPlanner's state machine implementation for different driving
/// behaviors \date Jun 19, 2016

#include <iostream>

#include "modules/common/time/time_tool.h"
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"

namespace legionclaw {
namespace planning {

using namespace legionclaw::common;

void BehaviourSelector::LatNotActiveStateUpdate(const std::string &state_name, int state)
{
  // somthing to do
}

void BehaviourSelector::ForwardStateUpdate(const std::string &state_name, int state)
{
  if ((lat_state_ == SelectorLatFlag::CHANGE_LANE_LEFT) ||
      (lat_state_ == SelectorLatFlag::CHANGE_LANE_RIGHT)) {
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  if (lat_state_ == SelectorLatFlag::STATION) {
    behaviour_lat_state_sm_->NextState("Station");
    return;
  }
  // 匿名函数
  auto IsAvoidanceActState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "Not allow enable swerving  ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) <= selector_params().threshold_lat_offset()) {
      ADEBUG << "End lat offset <= threshold lat offset ";
      return false;
    }
    // minst cost trajectory and current selector are  not identical
    if (store_trajectory_.has_trajectory_points()) {
      if (!(target_trajectory_.global_index() == store_trajectory_.global_index() && IsSameLane())) {
        return false;
      }
    } else {
      return false;
    }
    return true;
  };

  auto IsAvoidancePreState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "Not allow enable swerving  ";
      return false;
    }
    if (!IsSameLane()) {
      // ADEBUG << "Target lane index - stop sign id != 0 ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) <= selector_params().threshold_lat_offset()) {
      ADEBUG << "End lat offset <= threshold lat offset ";
      return false;
    }
    return true;
  };

  auto IsChangeLaneState = [=]() -> bool {
    if (!common_params()->enable_lane_change()) {
      ADEBUG << "Not allow enable swerving  ";
      return false;
    }
    if (IsSameLane()) {
      ADEBUG << "Target lane index - stop sign id = 0 ";
      return false;
    }
    return true;
  };
  current_behaviour_.lat_state = legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
  lat_state_ = SelectorLatFlag::LAT_INVALID;

  if (StationGoalDistanceLess()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Station");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_STATE;
    return;
  }
  // 触发条件：
  if (IsAvoidanceActState()) {
    lat_state_ = SelectorLatFlag::AVOIDANCE_ACTU;
    behaviour_lat_state_sm_->NextState("Avoidance");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCING_STATE;
    return;
  }
  // 触发条件： 1.使能是否开启；2.是否同一车道；3.横向距离是否满足
  if (IsAvoidancePreState()) {
    lat_state_ = SelectorLatFlag::AVOIDANCE_PRE;
    behaviour_lat_state_sm_->NextState("Avoidance");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_PRE_STATE;
    // final_trajectory_ = store_trajectory_;
    return;
  }
  // 触发条件： 1.使能开启；2.距离不满足条件；
  if (IsChangeLaneState()) {
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) > 0) {
      lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
      current_behaviour_.lat_state =
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_PRE_STATE;
      return;
    } else if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) < 0) {
      lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
      current_behaviour_.lat_state =
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_PRE_STATE;
      return;
    }
    behaviour_lat_state_sm_->NextState("ChangeLane");
  }
}

void BehaviourSelector::StationStateUpdate(const std::string &state_name, int state)
{
  auto IsChangeLaneState = [=]() -> bool {
    if (!common_params()->enable_lane_change()) {
      ADEBUG << "Not allow enable swerving  ";
      return false;
    }
    if (IsSameLane()) {
      ADEBUG << "Target lane index - stop sign id = 0 ";
      return false;
    }
    return true;
  };

  if (lat_state_ == SelectorLatFlag::FORWARD) {
    behaviour_lat_state_sm_->NextState("Forward");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    return;
  }
  if(IsChangeLaneState()) {
    if((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) > 0) {
      lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    } else if((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) < 0) {
      lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    }
    behaviour_lat_state_sm_->NextState("Forward");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    return;
  }
  if (lat_state_ == SelectorLatFlag::STATION) {
    behaviour_lat_state_sm_->NextState("StationStop");
    return;
  }
}

void BehaviourSelector::StationStopStateUpdate(const std::string &state_name, int state)
{
  //
  current_behaviour_.lat_state = legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_STATE;

  lat_state_ = SelectorLatFlag::LAT_INVALID;

  if (ptr_conditions_->goal_distance_ > selector_params().distance_to_goal()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Station");
    return;
  }

  if (ptr_conditions_->is_static_) {
    behaviour_lat_state_sm_->NextState("StationWait");
    return;
  }
  // 触发条件： 1.距离满足；2.距离满足且车辆静止
  if (StationArrived()) {
    ptr_conditions_->current_goal_id_ = -1;
    behaviour_lat_state_sm_->NextState("StationArrive");
    current_behaviour_.lat_state_name = "StationArrived";
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_ARRIVED_STATE;
    return;
  }
}

void BehaviourSelector::StationWaitStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_WAIT_STATE;
  if (!ptr_conditions_->is_static_) {
    behaviour_lat_state_sm_->NextState("StationStop");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_STATE;
    return;
  }

  if (ptr_conditions_->goal_distance_ > 10.0) {
    behaviour_lat_state_sm_->NextState("StationStop");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_STATE;
    return;
  }
  if (TimeTool::GetTimeDiffNow(state_timer_) > selector_params().goal_wait_max_time()) {
    ptr_conditions_->current_goal_id_ = -1;
    behaviour_lat_state_sm_->NextState("StationArrive");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_ARRIVED_STATE;
    return;
  }
  if (ptr_conditions_->goal_distance_trajectory_ > selector_params().distance_to_goal()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Station");
    return;
  }
}

void BehaviourSelector::StationArrivedStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_ARRIVED_STATE;
  if (ptr_conditions_->current_goal_id_ > 0) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Station");
    return;
  }
}

void BehaviourSelector::ChangeLaneStateUpdate(const std::string &state_name, int state)
{
  //
  if (lat_state_ == SelectorLatFlag::FORWARD || lat_state_ == SelectorLatFlag::STATION) {
    behaviour_lat_state_sm_->NextState("Forward");
  }
  if (lat_state_ == SelectorLatFlag::CHANGE_LANE_LEFT) {
    if (TimeTool::GetTimeDiffNow(state_timer_) < selector_params().decision_making_time()) {
      decision_making_time_ = selector_params().decision_making_time() -
                              TimeTool::GetTimeDiffNow(state_timer_) +
                              selector_params().pre_lane_change_time();
    }
    ResetTimer();
    behaviour_lat_state_sm_->NextState("ChangeLaneLeftPre");
    return;
  }
  if (lat_state_ == SelectorLatFlag::CHANGE_LANE_RIGHT) {
    if (TimeTool::GetTimeDiffNow(state_timer_) < selector_params().decision_making_time()) {
      decision_making_time_ = selector_params().decision_making_time() -
                              TimeTool::GetTimeDiffNow(state_timer_) +
                              selector_params().pre_lane_change_time();
    }
    ResetTimer();
    behaviour_lat_state_sm_->NextState("ChangeLaneRightPre");
    return;
  }
}

void BehaviourSelector::ChangeLaneLeftPreStateUpdate(const std::string &state_name, int state)
{
  //
  if (lat_state_ == SelectorLatFlag::FORWARD) {
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  // 匿名函数
  auto ToForwardState = [=]() -> bool {
    // TODO 若由于误识别进入了换道状态，此处若想从预换道状态回到forword，
    // 必须等待2s，是否不合理/时间是否过长？
    if (TimeTool::GetTimeDiffNow(state_timer_) <= 1.0) {
      ADEBUG << "Change lane time is too short";
      return false;
    }
    if (ptr_conditions_->target_lane_index_ != ptr_conditions_->current_lane_index_) {
      return false;
    }
    return true;
  };

  auto ToActState = [=]() -> bool {
    if (!common_params()->enable_lane_change()) {
      ADEBUG << "Not allow enable lane change";
      return false;
    }
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) >= 0) {
      ADEBUG << "Target lane index - current lane index >= 0 ";
      return false;
    }
    if (ptr_conditions_->current_lane_closest_obj_distance_ <=
        selector_params().relative_distance_of_next()) {
      ADEBUG << "Closest obj distance is too small";
      return false;
    }
    if (TimeTool::GetTimeDiffNow(state_timer_) <= decision_making_time_) {
      ADEBUG << "continue pre lane change";
      return false;
    }
    return true;
  };

  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_PRE_STATE;
  origin_lane_id_ = frame_.MapMatchInfo().current_lane_id;

  if (ToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  if (StationGoalDistanceLess()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  if (ptr_conditions_->target_lane_index_ > ptr_conditions_->current_lane_index_) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_PRE_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  // 触发条件： 1.允许避障；2.车道ID满足；3.满足避障触发条件；4.预处避障状态大于2s
  if (ToActState()) {
    behaviour_lat_state_sm_->NextState("ChangeLaneLeftAct");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE;
    return;
  }
}

void BehaviourSelector::ChangeLaneRightPreStateUpdate(const std::string &state_name, int state)
{
  if (lat_state_ == SelectorLatFlag::FORWARD) {
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  // 匿名函数
  auto ToForwardState = [=]() -> bool {
    if (TimeTool::GetTimeDiffNow(state_timer_) <= 1.0) {
      ADEBUG << "Change lane time is too short";
      return false;
    }
    if (ptr_conditions_->target_lane_index_ != ptr_conditions_->current_lane_index_) {
      return false;
    }
    return true;
  };
  auto ToActState = [=]() -> bool {
    if (!common_params()->enable_lane_change()) {
      ADEBUG << "Not allow enable lane change !";
      return false;
    }
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) <= 0) {
      ADEBUG << "Target lane index - stop sign id >= 0 ";
      return false;
    }
    if (ptr_conditions_->current_lane_closest_obj_distance_ <=
        selector_params().relative_distance_of_next()) {
      ADEBUG << "Closest obj distance is too small";
      return false;
    }
    if (TimeTool::GetTimeDiffNow(state_timer_) <= decision_making_time_) {
      ADEBUG << "continue pre lane change";
      return false;
    }
    return true;
  };

  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_PRE_STATE;
  origin_lane_id_ = frame_.MapMatchInfo().current_lane_id;

  if (ToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  // if (StationGoalDistanceLess()) {
  //   lat_state_ = SelectorLatFlag::STATION;
  //   behaviour_lat_state_sm_->NextState("ChangeLane");
  //   return;
  // }

  if (ptr_conditions_->target_lane_index_ < ptr_conditions_->current_lane_index_) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_PRE_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
  // 触发条件： 1.允许避障；2.车道ID满足；3.满足避障触发条件；4.预处避障状态大于2s
  if (ToActState()) {
    behaviour_lat_state_sm_->NextState("ChangeLaneRightAct");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE;
    return;
  }
}

void BehaviourSelector::ChangeLaneLeftActStateUpdate(const std::string &state_name, int state)
{
  //
  auto ToForwardState = [=]() -> bool {
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) < 0) {
      return false;
    }
    return true;
  };
  // if (TimeTool::GetTimeDiffNow(state_timer_) < decision_making_time_) return;
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE;

  double lc_lateral_displacement = frame_.MapMatchInfo().lat_offset;
  if (lc_lateral_displacement > 0.5) {
    lc_displacement_state_ = true;
  }
  if (lc_displacement_state_) {
    if (lc_lateral_displacement < 0) {
      lc_dis_complete_ = true;
      lc_displacement_state_ = false;
    }
  }

  if (ToForwardState()) {
    ResetTimer();
    lat_state_ = SelectorLatFlag::FORWARD;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
}

void BehaviourSelector::ChangeLaneRightActStateUpdate(const std::string &state_name, int state)
{
  //
  auto ToForwardState = [=]() -> bool {
    if ((ptr_conditions_->target_lane_index_ - ptr_conditions_->current_lane_index_) > 0) {
      return false;
    }
    return true;
  };
  // if (TimeTool::GetTimeDiffNow(state_timer_) < decision_making_time_) return;
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE;

  double lc_lateral_displacement = frame_.MapMatchInfo().lat_offset;
  if (lc_lateral_displacement < -0.5) {
    lc_displacement_state_ = true;
  }
  if (lc_displacement_state_) {
    if (lc_lateral_displacement > 0) {
      lc_dis_complete_ = true;
      lc_displacement_state_ = false;
    }
  }

  if (ToForwardState()) {
    ResetTimer();
    lat_state_ = SelectorLatFlag::FORWARD;
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
    behaviour_lat_state_sm_->NextState("ChangeLane");
    return;
  }
}

void BehaviourSelector::AvoidanceStateUpdate(const std::string &state_name, int state)
{
  if (lat_state_ == SelectorLatFlag::FORWARD || lat_state_ == SelectorLatFlag::STATION ||
      lat_state_ == SelectorLatFlag::CHANGE_LANE_LEFT ||
      lat_state_ == SelectorLatFlag::CHANGE_LANE_RIGHT) {
    behaviour_lat_state_sm_->NextState("Forward");
    return;
  }
  if (lat_state_ == SelectorLatFlag::AVOIDANCE_ACTU) {
    ptr_conditions_->swerve_lat_offset_ = ptr_conditions_->end_lat_offset_;
    behaviour_lat_state_sm_->NextState("AvoidanceAct");
    return;
  }
  if (lat_state_ == SelectorLatFlag::AVOIDANCE_PRE) {
    behaviour_lat_state_sm_->NextState("AvoidancePre");
    return;
  }
  if (lat_state_ == SelectorLatFlag::AVOIDANCE_BACK_PRE) {
    behaviour_lat_state_sm_->NextState("AvoidanceBackPre");
    return;
  }
}

void BehaviourSelector::AvoidancePreStateUpdate(const std::string &state_name, int state)
{
  //
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_PRE_STATE;
  // 匿名函数
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  auto AvoidancePreToForwardState = [=]() -> bool {
    if (TimeTool::GetTimeDiffNow(state_timer_) <= selector_params().pre_avoidance_min_time()) {
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (!AvoidanceToForwardState()) {
      lat_state_ = SelectorLatFlag::FORWARD;
      behaviour_lat_state_sm_->NextState("Avoidance");
      return false;
    }
    return true;
  };

  auto AvoidancePreToAvoidanceActState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "swerving disenable";
      return false;
    }
    if (!IsSameLane()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) <= selector_params().threshold_lat_offset()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (ptr_conditions_->current_lane_closest_obj_distance_ <=
        selector_params().relative_distance_of_next()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (TimeTool::GetTimeDiffNow(state_timer_) <= selector_params().pre_avoidance_max_time()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (ptr_conditions_->current_lane_closest_obj_distance_ >=
        ptr_conditions_->avoidance_distance_) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    return true;
  };

  //  触发条件： 1.避障时间满足；2.车道ID满足；3.结尾偏差满足；4.开始处偏差满足
  if (AvoidancePreToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (StationGoalDistanceLess()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidancePreToAvoidanceActState()) {
    ptr_conditions_->swerve_lat_offset_ = ptr_conditions_->end_lat_offset_;
    behaviour_lat_state_sm_->NextState("AvoidanceAct");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCING_STATE;
    return;
  }
  if (AvoidanceToChangeLaneLeftState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneRightState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
}

void BehaviourSelector::AvoidanceActStateUpdate(const std::string &state_name, int state)
{
  // 匿名函数
  auto AvoidanceToKeepState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "enable swerving";
      return false;
    }
    if (!IsSameLane()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->swerve_lat_offset_ - ptr_conditions_->start_lat_offset_) >=
        selector_params().threshold_start_end_lat_diff()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    return true;
  };
  auto AvoidanceToBackPreState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (!IsSameLane()) {
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) >= 1.0 * 0.8) {  // TODO:配置
      ADEBUG << "Vehicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->start_lat_offset_) >= 0.5) {  // TODO:配置
      ADEBUG << "Vehicle  ";
      return false;
    }
    return true;
  };

  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCING_STATE;
  if (AvoidanceToChangeLaneLeftState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    // current_behaviour_.lat_state =
    //   legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneRightState()) {
    // current_behaviour_.lat_state =
    //   legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE;
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (TimeTool::GetTimeDiffNow(state_timer_) < selector_params().decision_making_time()) {
    return;
  }

  target_velocity_ *= selector_params().avoidance_speed_ratio();

  if (AvoidanceToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }

  if (ChangeToGoal()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToKeepState()) {
    state_timer_ = TimeTool::NowToSeconds();
    ptr_conditions_->swerve_lat_offset_ = 0;
    behaviour_lat_state_sm_->NextState("AvoidanceKeep");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_KEEP_STATE;
    return;
  }
  if (AvoidanceToBackPreState()) {
    behaviour_lat_state_sm_->NextState("AvoidancePre");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_PRE_STATE;
    return;
  }
}

void BehaviourSelector::AvoidanceKeepStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_KEEP_STATE;
  // 匿名函数
  auto AvoidanceKeepToBackPreState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (!IsSameLane()) {
      ADEBUG << "Vihicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) >= selector_params().threshold_lat_offset()) {
      ADEBUG << "Vihicle  ";
      return false;
    }
    if (TimeTool::GetTimeDiffNow(state_timer_) <= selector_params().keep_avoidance_max_time()) {
      ADEBUG << "Vihicle  ";
      return false;
    }
    return true;
  };

  //  触发条件： 1.避障时间满足；2.车道ID满足；3.结尾偏差满足；4.开始处偏差满足
  if (AvoidanceToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }

  if (ChangeToGoal()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceKeepToBackPreState()) {
    state_timer_ = TimeTool::NowToSeconds();
    behaviour_lat_state_sm_->NextState("AvoidanceBackPre");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_PRE_STATE;
    return;
  }
  if (AvoidanceToChangeLaneLeftState()) {
    // current_behaviour_.lat_state =
    //   legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE;
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneRightState()) {
    // current_behaviour_.lat_state =
    //   legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE;
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
}

void BehaviourSelector::AvoidanceBackPreStateUpdate(const std::string &state_name, int state)
{
  auto AvoidanceToBackActState = [=]() -> bool {
    if (!common_params()->enable_swerving()) {
      ADEBUG << "Avoidance Act timer is too short";
      return false;
    }
    if (!IsSameLane()) {
      ADEBUG << "Vihicle  ";
      return false;
    }
    if (fabs(ptr_conditions_->end_lat_offset_) >= 0.8) {  // TODO:配置
      ADEBUG << "Vihicle  ";
      return false;
    }
    if (ptr_conditions_->target_lane_closest_obj_distance_ <= 2.0) {  // TODO:配置
      ADEBUG << "Vihicle  ";
      return false;
    }
    if (TimeTool::GetTimeDiffNow(state_timer_) <= selector_params().pre_back_max_time()) {
      ADEBUG << "Vihicle  ";
      return false;
    }
    return true;
  };
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_PRE_STATE;
  if (AvoidanceToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (ChangeToGoal()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToBackActState()) {
    state_timer_ = TimeTool::NowToSeconds();
    behaviour_lat_state_sm_->NextState("AvoidanceBackAct");
    current_behaviour_.lat_state =
        legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_STATE;
    return;
  }
  if (AvoidanceToChangeLaneLeftState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneRightState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
}

void BehaviourSelector::AvoidanceBackActStateUpdate(const std::string &state_name, int state)
{
  current_behaviour_.lat_state =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_STATE;
  if (TimeTool::GetTimeDiffNow(state_timer_) < selector_params().decision_making_time()) return;
  if (ChangeToGoal()) {
    lat_state_ = SelectorLatFlag::STATION;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToForwardState()) {
    lat_state_ = SelectorLatFlag::FORWARD;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneLeftState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_LEFT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
  if (AvoidanceToChangeLaneRightState()) {
    lat_state_ = SelectorLatFlag::CHANGE_LANE_RIGHT;
    behaviour_lat_state_sm_->NextState("Avoidance");
    return;
  }
}

}  // namespace planning
}  // namespace legionclaw
