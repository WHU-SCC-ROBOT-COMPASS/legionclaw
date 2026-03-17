/**
 * @file              parallel_parking_state_machine.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-19 02:51:31
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#include "modules/planning/src/planner/parking_planner/geometry/parking_manager.h"

namespace legionclaw {
namespace planning {

void ParkingManager::ParallelParkingBeginStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BEGIN_STATE;
  // 延迟0.5s
  if (TimeTool::GetTimeDiffNow(start_time_) > 0.5) {
    parking_stage_ = ParkingStage::STAGE1;
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingForwardBegin");
  }
}

void ParkingManager::ParallelParkingForwardBeginStateUpdate(const std::string &state_name,
                                                            int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_BEGIN_STATE;
  // 清空轨迹点
  parking_trajectory_.clear_trajectory_points();

  parking_status_ =
      parallel_parking_path_generator_.ComputeForwardPathInStage1(local_vehicle_current_pose_);

  if (parking_status_ == Status::Ok()) {
    parking_path_ = parallel_parking_path_generator_.GetForwardPath();
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_DRIVE;
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingForward");
  } else {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
  }
}

void ParkingManager::ParallelParkingForwardStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_STATE;
  double match_dis = DBL_MAX;
  int index = -1;
  if (parking_path_.size() > 0) {
    index = MapMatcher::QueryNearestPointWithBuffer(
        parking_path_, {local_vehicle_current_pose_.x(), local_vehicle_current_pose_.y()},
        local_vehicle_current_pose_.theta(), 1.0e-6, match_dis);
    if (index > 0) parking_path_.erase(parking_path_.begin(), parking_path_.begin() + index - 1);
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_DRIVE;
    remain_length_ = (parking_path_.size() - 1) * parallel_parking_conf_.parallel_path_interval();
    parking_status_ = Status::Ok();
  } else {
    parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
    return;
  }

  if (IsNearDestination() == true && IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingForwardFinish");
  }
}

void ParkingManager::ParallelParkingForwardFinishStateUpdate(const std::string &state_name,
                                                             int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_FINISH_STATE;
  // 延迟１s
  if (TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingBackwardBegin");
  }
}

void ParkingManager::ParallelParkingBackwardBeginStateUpdate(const std::string &state_name,
                                                             int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_BEGIN_STATE;
  if (parking_path_.size() <= 0) {
    parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
    return;
  }

  legionclaw::interface::PathPoint start_pose;
  // 计算倒车路径
  if (parking_manager_conf_.start_pose_mode() == 1 && parking_path_.size() > 0) {
    start_pose = parking_path_.back();
  } else {
    start_pose = local_vehicle_current_pose_;
  }

  parking_path_.clear();

  parking_status_ = parallel_parking_path_generator_.ComputeBackwardPathInStage1(start_pose);
  if (parking_status_ == Status::Ok()) {
    parking_path_ = parallel_parking_path_generator_.GetBackwardPath();
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_REVERSE;
    parking_sm_->NextState("ParallelParkingBackward");
  } else {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
    return;
  }
}

void ParkingManager::ParallelParkingBackwardStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_STATE;
  double match_dis = DBL_MAX;
  int index = -1;
  // 基于倒车路径和当前位置更新路径
  if (parking_path_.size() > 0) {
    index = MapMatcher::QueryNearestPointWithBuffer(
        parking_path_, {local_vehicle_current_pose_.x(), local_vehicle_current_pose_.y()},
        local_vehicle_current_pose_.theta(), 1.0e-6, match_dis);
    if (index > 0) parking_path_.erase(parking_path_.begin(), parking_path_.begin() + index - 1);
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_REVERSE;
    remain_length_ = (parking_path_.size() - 1) * parallel_parking_conf_.parallel_path_interval();
    parking_status_ == Status::Ok();
  } else {
    parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
    return;
  }

  if (IsNearDestination() == true && IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingBackwardFinish");
  }
}

void ParkingManager::ParallelParkingBackwardFinishStateUpdate(const std::string &state_name,
                                                              int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_FINISH_STATE;
  if (IsParkingPrecisely() == true && IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingFinish");
  } else if (IsParkingPrecisely() == false && IsTheVehicleStopped() == true &&
             TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("ParallelParkingForwardBegin");
  }
}

void ParkingManager::ParallelParkingFinishStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FINISH_STATE;

  if (parking_status_ != Status::Ok()) {
    AERROR << parking_status_.error_message();
  }

  start_time_ = TimeTool::NowToSeconds();
  parking_sm_->NextState("ParkingFinish");
}
}  // namespace planning
}  // namespace legionclaw