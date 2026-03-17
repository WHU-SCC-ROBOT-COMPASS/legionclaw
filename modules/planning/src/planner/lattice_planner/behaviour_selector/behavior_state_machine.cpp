/// \file behavior_state_machine_planning.cpp
/// \author Hatem Darweesh
/// \brief OpenPlanner's state machine implementation for different driving
/// behaviors \date Jun 19, 2016

#include <iostream>

#include "modules/common/time/time_tool.h"
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"

namespace legionclaw {
namespace planning {

using namespace legionclaw::common;
void BehaviourSelector::BehaviorStateMachineInit()
{
  behaviour_lat_state_sm_.reset(
      new state_machine::StateContext(FLAGS_behavior_lat_state_machine_file, true,
                                      FLAGS_state_machine_log_dir, "behavior_lat_state_machine"));
  behaviour_lon_state_sm_.reset(
      new state_machine::StateContext(FLAGS_behavior_lon_state_machine_file, true,
                                      FLAGS_state_machine_log_dir, "behavior_lon_state_machine"));

  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "LatNotActive",
      std::bind(&BehaviourSelector::LatNotActiveStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ForwardState",
      std::bind(&BehaviourSelector::ForwardStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "StationState",
      std::bind(&BehaviourSelector::StationStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "StationStopState",
      std::bind(&BehaviourSelector::StationStopStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "StationWaitState",
      std::bind(&BehaviourSelector::StationWaitStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "StationArrivedState",
      std::bind(&BehaviourSelector::StationArrivedStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ChangeLaneState",
      std::bind(&BehaviourSelector::ChangeLaneStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ChangeLaneLeftPreState",
      std::bind(&BehaviourSelector::ChangeLaneLeftPreStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ChangeLaneRightPreState",
      std::bind(&BehaviourSelector::ChangeLaneRightPreStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ChangeLaneLeftActState",
      std::bind(&BehaviourSelector::ChangeLaneLeftActStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ChangeLaneRightActState",
      std::bind(&BehaviourSelector::ChangeLaneRightActStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidanceState",
      std::bind(&BehaviourSelector::AvoidanceStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidancePreState",
      std::bind(&BehaviourSelector::AvoidancePreStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidanceKeepState",
      std::bind(&BehaviourSelector::AvoidanceKeepStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidanceBackPreState",
      std::bind(&BehaviourSelector::AvoidanceBackPreStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidanceActState",
      std::bind(&BehaviourSelector::AvoidanceActStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lat_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "AvoidanceBackActState",
      std::bind(&BehaviourSelector::AvoidanceBackActStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "LonNotActive",
      std::bind(&BehaviourSelector::LonNotActiveStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "NormalState",
      std::bind(&BehaviourSelector::NormalStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "PreciseStopState",
      std::bind(&BehaviourSelector::PreciseStopStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "StopState",
      std::bind(&BehaviourSelector::StopStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "TrafficSignStopState",
      std::bind(&BehaviourSelector::TrafficSignStopStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "TrafficSignWaitState",
      std::bind(&BehaviourSelector::TrafficSignWaitStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "TrafficLightStopState",
      std::bind(&BehaviourSelector::TrafficLightStopStateUpdate, this, std::placeholders::_1, 0));
  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "TrafficLightWaitState",
      std::bind(&BehaviourSelector::TrafficLightWaitStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "FollowState",
      std::bind(&BehaviourSelector::FollowStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ACCState",
      std::bind(&BehaviourSelector::ACCStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "TJPState",
      std::bind(&BehaviourSelector::TJPStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "Cut_InState",
      std::bind(&BehaviourSelector::CutInStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "OnComingState",
      std::bind(&BehaviourSelector::OnComingStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "CrossingState",
      std::bind(&BehaviourSelector::CrossingStateUpdate, this, std::placeholders::_1, 0));

  // TODO纵向增加汇入汇出状态
  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "InwardState",
      std::bind(&BehaviourSelector::InwardStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lon_state_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "OutwardState",
      std::bind(&BehaviourSelector::OutwardStateUpdate, this, std::placeholders::_1, 0));

  behaviour_lat_state_sm_->NextState("Forward");
  behaviour_lon_state_sm_->NextState("Normal");

  return;
}

void BehaviourSelector::StateMachineSpin(void *param)
{
  if (behaviour_lat_state_sm_) behaviour_lat_state_sm_->OnUpdate();
  if (behaviour_lon_state_sm_) behaviour_lon_state_sm_->OnUpdate();
}

bool BehaviourSelector::StateMachineReset()
{
  behaviour_lat_state_sm_->Reset();
  behaviour_lon_state_sm_->Reset();
  current_behaviour_.lon_state = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
  lon_state_ = SelectorLonFlag::LON_INVALID;
  current_behaviour_.lat_state = legionclaw::interface::ADCTrajectory::BehaviourLatState::FORWARD_STATE;
  lat_state_ = SelectorLatFlag::LAT_INVALID;
  behaviour_lat_state_sm_->NextState("Forward");
  behaviour_lon_state_sm_->NextState("Normal");
  //   behaviour_lat_state_sm_->NextState("NotActive");
  //   behaviour_lon_state_sm_->NextState("NotActive");
  // BehaviorStateMachineInit();
  return true;
}

}  // namespace planning
}  // namespace legionclaw
