/**
 * @file              parking_state_machine.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-19 03:28:42
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#include "modules/planning/src/planner/parking_planner/geometry/parking_manager.h"

namespace legionclaw {
namespace planning {
bool ParkingManager::ParkingStateMachineInit()
{
  parking_sm_.reset(new state_machine::StateContext(
      FLAGS_parking_state_machine_file, FLAGS_parking_state_machine_log_enable,
      FLAGS_state_machine_log_dir, FLAGS_parking_state_machine_name));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ParkingInitialState",
      std::bind(&ParkingManager::ParkingInitialStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ParkingFinishState",
      std::bind(&ParkingManager::ParkingFinishedStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ParallelParkingBeginState",
      std::bind(&ParkingManager::ParallelParkingBeginStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ParallelParkingForwardBeginState",
                           std::bind(&ParkingManager::ParallelParkingForwardBeginStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ParallelParkingForwardState",
                           std::bind(&ParkingManager::ParallelParkingForwardStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ParallelParkingForwardFinishState",
                           std::bind(&ParkingManager::ParallelParkingForwardFinishStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ParallelParkingBackwardBeginState",
                           std::bind(&ParkingManager::ParallelParkingBackwardBeginStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ParallelParkingBackwardState",
                           std::bind(&ParkingManager::ParallelParkingBackwardStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                           "ParallelParkingBackwardFinishState",
                           std::bind(&ParkingManager::ParallelParkingBackwardFinishStateUpdate,
                                     this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "ParallelParkingFinishState",
      std::bind(&ParkingManager::ParallelParkingFinishStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "VerticalParkingBeginState",
      std::bind(&ParkingManager::VerticalParkingBeginStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "VerticalParkingForwardBeginState",
                           std::bind(&ParkingManager::VerticalParkingForwardBeginStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "VerticalParkingForwardState",
                           std::bind(&ParkingManager::VerticalParkingForwardStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "VerticalParkingForwardFinishState",
                           std::bind(&ParkingManager::VerticalParkingForwardFinishStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "VerticalParkingBackwardBeginState",
                           std::bind(&ParkingManager::VerticalParkingBackwardBeginStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE, "VerticalParkingBackwardState",
                           std::bind(&ParkingManager::VerticalParkingBackwardStateUpdate, this,
                                     std::placeholders::_1, 0));

  parking_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                           "VerticalParkingBackwardFinishState",
                           std::bind(&ParkingManager::VerticalParkingBackwardFinishStateUpdate,
                                     this, std::placeholders::_1, 0));

  parking_sm_->SetCallback(
      state_machine::CallbackType::UPDATE, "VerticalParkingFinishState",
      std::bind(&ParkingManager::VerticalParkingFinishStateUpdate, this, std::placeholders::_1, 0));

  parking_sm_->NextState("ParkingInitial");

  return true;
}

void ParkingManager::ParkingInitialStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ = legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_INITIAL_STATE;
  max_re_planning_count_ = 0;
  // std::cout << "+++++++++++++++++++++++++++++" << "\n";
  if (park_in_) {
    UpdateContext();
  } else {
    UpdateParkingOutStartPose();
  }
  // 车辆到达泊车点，并且静止
  if (IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    switch (parking_type_) {
      case legionclaw::common::ParkingType::PARALLEL_PARKING: {
        // parallel_parking_path_generator_.SetParkingSpot(parking_info_);
        parking_sm_->NextState("ParallelParkingBegin");
      } break;
      case legionclaw::common::ParkingType::VERTICAL_PARKING: {
        // vertical_parking_path_generator_.SetParkingSpot(parking_info_);
        // 计算倒车过程中，方向盘开始回正时进入车库的深度
        parking_sm_->NextState("VerticalParkingBegin");
      } break;
      default:
        AERROR << "unknown parking type.";
        parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "unknown parking type.");
        break;
    }
  }
}

void ParkingManager::ParkingFinishedStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ = legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE;

  if (TimeTool::GetTimeDiffNow(start_time_) > 0.5) {
    start_time_ = TimeTool::NowToSeconds();
    parking_stage_ = ParkingStage::STAGE1;
    parking_sm_->NextState("ParkingInitial");
  }
}
}  // namespace planning
}  // namespace legionclaw
