/**
 * @file              vertical_parking_state_machine.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-19 02:51:43
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#include "modules/planning/src/planner/parking_planner/geometry/parking_manager.h"

namespace legionclaw {
namespace planning {

void ParkingManager::VerticalParkingBeginStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BEGIN_STATE;
  if (park_in_) {
    // parking_scheme_ == 1  旧三次泊车（直线前迁）
    // parking_scheme_ == 2  新三次泊车（弧线前迁）
    // parking_scheme_ == 3  小空间多次泊车
    // parking_scheme_ == 4  二次泊车
    // parking_scheme_ == 5  新三次泊车(move3直线改反打圆弧)
    double t_R = vertical_parking_conf_.caculate_turning_radius();
    double t_R1 = vertical_parking_conf_.r1();
    double upward_distance = vertical_parking_conf_.upward_distance();
    double t_d1;
    if (parking_scheme_ == 1) {
      t_d1 = vertical_parking_conf_.d1();
    }
    if (parking_scheme_ == 2 || parking_scheme_ == 3 || parking_scheme_ == 5) {
      t_d1 = 0.3;
    }
    if (parking_scheme_ == 4) {
      t_d1 = 1.286755563;
    }
    double theta_1 = acos((t_R - t_d1) / t_R);
    double parking_length = parking_info_.length();
    double h = t_R * sin(M_PI_2 - theta_1) - (t_R1 - t_R1 * cos(theta_1)) -
               (local_vehicle_current_pose_.y() -
                (parking_length - vertical_parking_conf_.vertical_lon_safe_distance() -
                 vehicle_param_.back_edge_to_center() + upward_distance));
    double theta_t = asin(h / t_R);

    double l2 = t_R1 * sin(theta_1) - (t_R * cos(theta_t) - t_R * cos(M_PI_2 - theta_1));

    double d4 = local_vehicle_current_pose_.y() -
                (parking_length - vertical_parking_conf_.vertical_lon_safe_distance() -
                 vehicle_param_.back_edge_to_center() + upward_distance);
    double theta_2 = acos((t_R - d4) / t_R);
    double l1 = t_R * sin(theta_2);
    if (parking_scheme_ == 4) {
      theta_t = 0;
    }
    double l3 = t_R * sin(theta_1) - (t_R * cos(theta_t) - t_R * cos(M_PI_2 - theta_1));
    double judge_distance;

    if (parking_scheme_ == 1) {
      judge_distance = fabs(l1);
    }
    if (parking_scheme_ == 2 || parking_scheme_ == 3 || parking_scheme_ == 5) {
      judge_distance = fabs(l2);
    }
    if (parking_scheme_ == 4) {
      judge_distance = fabs(l3);
    }

    if (IsTheVehicleStopped() == false) {
      start_time_ = TimeTool::NowToSeconds();
    }
    double parking_position = 0;
    if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
      parking_position = -1;
    else
      parking_position = 1;
    if (!(judge_distance > 0)) {
      AERROR << "parking_space is wrong";
      parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_space is wrong");
      start_time_ = TimeTool::NowToSeconds();
      // parking_sm_->NextState("VerticalParkingFinish");
      return;
    }
    // 如果目标车位太靠后，先进入move0调整，后迁一段距离再开始泊车。

    if (parking_position * local_vehicle_current_pose_.x() > judge_distance) {
      stage1_move_ = Stage1Move::MOVE0;
    } else {
      stage1_move_ = Stage1Move::MOVE1;
    }

    // 延迟1s
    if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 && stage1_move_ == Stage1Move::MOVE1) {
      start_time_ = TimeTool::NowToSeconds();
      parking_stage_ = ParkingStage::STAGE1;
      parking_sm_->NextState("VerticalParkingForwardBegin");
    }

    if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 && stage1_move_ == Stage1Move::MOVE0) {
      start_time_ = TimeTool::NowToSeconds();
      parking_stage_ = ParkingStage::STAGE1;
      parking_sm_->NextState("VerticalParkingBackwardBegin");
    }
  } else {
    // 延迟1s
    // 方向决定move与direction
    switch (out_direction_) {
      case OutDirection::UP:
        stage3_move_ = Stage3Move::OUT_MOVE1;
        break;
      case OutDirection::Down:
        stage3_move_ = Stage3Move::OUT_MOVE2;
        break;
      case OutDirection::Left:
        stage3_move_ = Stage3Move::OUT_MOVE3;
        break;
      case OutDirection::Right:
        stage3_move_ = Stage3Move::OUT_MOVE3;
        break;
      default:
        break;
    }

    if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 &&
        (stage3_move_ == Stage3Move::OUT_MOVE1 || stage3_move_ == Stage3Move::OUT_MOVE3)) {
      start_time_ = TimeTool::NowToSeconds();
      parking_stage_ = ParkingStage::STAGE3;
      parking_sm_->NextState("VerticalParkingForwardBegin");
    }

    if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 && stage3_move_ == Stage3Move::OUT_MOVE2) {
      start_time_ = TimeTool::NowToSeconds();
      parking_stage_ = ParkingStage::STAGE3;
      parking_sm_->NextState("VerticalParkingBackwardBegin");
    }
  }

  parking_status_ = Status::Ok();
}

void ParkingManager::VerticalParkingForwardBeginStateUpdate(const std::string &state_name,
                                                            int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_BEGIN_STATE;
  if (keep_steer) {
    steer_first_ = true;
    keep_steer = false;
  }
  int director_pose;
  // 清空轨迹点
  parking_trajectory_.clear_trajectory_points();
  switch (parking_stage_) {
    case ParkingStage::STAGE1: {
      if (parking_scheme_ == 1) {
        switch (stage1_move_) {
          case Stage1Move::MOVE1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move1_1(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;
            // TODO 测试
          case Stage1Move::MOVE3:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3_5(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 2) {
        switch (stage1_move_) {
          case Stage1Move::MOVE1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move1_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;

          case Stage1Move::MOVE3:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 3) {
        switch (stage1_move_) {
          case Stage1Move::MOVE1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move1_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;

          case Stage1Move::MOVE3:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4_3_1;
            break;

          case Stage1Move::MOVE3_3_1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3_3(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4;
            break;

          case Stage1Move::MOVE3_3_2:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3_3(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 4) {
        switch (stage1_move_) {
          case Stage1Move::MOVE1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move1_4(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 5) {
        switch (stage1_move_) {
          case Stage1Move::MOVE1:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move1_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;

          case Stage1Move::MOVE3:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move3_5(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE4;
            break;

          default:
            break;
        }
      }
    } break;
    case ParkingStage::STAGE2: {
      parking_status_ =
          vertical_parking_path_generator_.ComputeForwardPathInStage2(local_vehicle_current_pose_);
    } break;
    case ParkingStage::STAGE3: {
      switch (out_direction_) {
        case OutDirection::UP:
          parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move1(
              local_vehicle_current_pose_);
          stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
          break;
        // case OutDirection::Down:
        //   parking_status_ =
        //           vertical_parking_path_generator_.ComputePathInStage3Move2(
        //               local_vehicle_current_pose_);
        //   stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
        //   break;
        case OutDirection::Left:
          director_pose = -1;
          switch (stage3_move_) {
            case Stage3Move::OUT_MOVE3:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move3(
                  local_vehicle_current_pose_, director_pose);
              stage3_move_ = Stage3Move::OUT_MOVE4;
              break;
            case Stage3Move::OUT_MOVE5:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move5(
                  local_vehicle_current_pose_);
              stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
              break;
            default:
              break;
          }

          break;
        case OutDirection::Right:
          director_pose = 1;
          switch (stage3_move_) {
            case Stage3Move::OUT_MOVE3:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move3(
                  local_vehicle_current_pose_, director_pose);
              stage3_move_ = Stage3Move::OUT_MOVE4;
              break;
            case Stage3Move::OUT_MOVE5:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move5(
                  local_vehicle_current_pose_);
              stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    } break;
    default:
      break;
  }

  if (parking_status_ == Status::Ok()) {
    parking_path_ = vertical_parking_path_generator_.GetForwardPath();
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_DRIVE;
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingForward");
  } else {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingFinish");
  }
}

void ParkingManager::VerticalParkingForwardStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_STATE;

  if (parking_path_.size() > 0) {
    ParkingPathEraseUpdate();
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_DRIVE;
    parking_status_ = Status::Ok();
  } else {
    parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
    start_time_ = TimeTool::NowToSeconds();
    // parking_sm_->NextState("VerticalParkingFinish");
    return;
  }
  if (IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    if (IsNearDestination() == true) {
      start_time_ = TimeTool::NowToSeconds();
      parking_sm_->NextState("VerticalParkingForwardFinish");
    }
    if (TimeTool::GetTimeDiffNow(start_time_) > 6.0) {
      AERROR << "the vehicle has not reached the expected destination";
      parking_status_ =
          Status(Status::ErrorCode::PLANNING_ERROR, "not reached the expected destination");
      // parking_sm_->NextState("VerticalParkingFinish");
      return;
    }
  }
}

void ParkingManager::VerticalParkingForwardFinishStateUpdate(const std::string &state_name,
                                                             int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_FINISH_STATE;
  keep_steer = true;
  if (stage1_move_ == Stage1Move::MOVE1 && TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_status_ = Status::Ok();
    parking_sm_->NextState("VerticalParkingForwardBegin");
  }
  // 直线前进move3完成之后，更新一下车位
  if (stage1_move_ == Stage1Move::MOVE4) {
    UpdateContext();
  }

  if (IsTheVehicleStopped() == false) {
    start_time_ = TimeTool::NowToSeconds();
  }

  // 延迟1s
  if (stage1_move_ == Stage1Move::MOVE_FINISH && parking_stage_ != ParkingStage::STAGE3) {
    if (IsParkingPrecisely() == true && TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
      start_time_ = TimeTool::NowToSeconds();
      parking_sm_->NextState("VerticalParkingFinish");
    }
  } else if (stage3_move_ == Stage3Move::OUT_MOVE_FINISH &&
             TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingFinish");
  } else if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 &&
             (stage3_move_ != Stage3Move::OUT_MOVE_FINISH &&
              stage1_move_ != Stage1Move::MOVE_FINISH)) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingBackwardBegin");
  }
}

void ParkingManager::VerticalParkingBackwardBeginStateUpdate(const std::string &state_name,
                                                             int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_BEGIN_STATE;
  if (keep_steer) {
    steer_first_ = true;
    keep_steer = false;
  }

  legionclaw::interface::PathPoint start_pose;
  // 计算倒车路径
  if (parking_manager_conf_.start_pose_mode() == 1 && parking_path_.size() > 0) {
    start_pose = parking_path_.back();
  } else {
    start_pose = local_vehicle_current_pose_;
  }

  parking_path_.clear();
  switch (parking_stage_) {
    case ParkingStage::STAGE1: {
      if (parking_scheme_ == 1) {
        switch (stage1_move_) {
          case Stage1Move::MOVE0:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move0_1(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE2;
            break;

          case Stage1Move::MOVE2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move2_1(start_pose);
            stage1_move_ = Stage1Move::MOVE3;
            break;

          case Stage1Move::MOVE4:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move4(start_pose);
            stage1_move_ = Stage1Move::MOVE_FINISH;

            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 2) {
        switch (stage1_move_) {
          case Stage1Move::MOVE0:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move0_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE1;
            break;

          case Stage1Move::MOVE2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move2_2(start_pose);
            stage1_move_ = Stage1Move::MOVE3;
            break;

          case Stage1Move::MOVE4:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move4(start_pose);
            stage1_move_ = Stage1Move::MOVE_FINISH;

            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 3) {
        switch (stage1_move_) {
          case Stage1Move::MOVE0:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move0_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE1;
            break;

          case Stage1Move::MOVE2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move2_2(start_pose);
            stage1_move_ = Stage1Move::MOVE3;
            break;

          case Stage1Move::MOVE4_3_1:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move4_3(start_pose);
            stage1_move_ = Stage1Move::MOVE3_3_1;
            break;

          case Stage1Move::MOVE4_3_2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move4_3(start_pose);
            stage1_move_ = Stage1Move::MOVE3_3_2;
            break;

          case Stage1Move::MOVE4:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move4(start_pose);
            stage1_move_ = Stage1Move::MOVE_FINISH;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 4) {
        switch (stage1_move_) {
          case Stage1Move::MOVE0:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move0_4(start_pose);
            stage1_move_ = Stage1Move::MOVE1;
            break;

          case Stage1Move::MOVE2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move2_4(start_pose);
            stage1_move_ = Stage1Move::MOVE_FINISH;
            break;

          default:
            break;
        }
      }
      if (parking_scheme_ == 5) {
        switch (stage1_move_) {
          case Stage1Move::MOVE0:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move0_2(
                local_vehicle_current_pose_);
            stage1_move_ = Stage1Move::MOVE1;
            break;

          case Stage1Move::MOVE2:

            parking_status_ =
                vertical_parking_path_generator_.ComputePathInStage1Move2_2(start_pose);
            stage1_move_ = Stage1Move::MOVE3;
            break;

          case Stage1Move::MOVE4:

            parking_status_ = vertical_parking_path_generator_.ComputePathInStage1Move4(start_pose);
            stage1_move_ = Stage1Move::MOVE_FINISH;

            break;

          default:
            break;
        }
      }
    } break;
    case ParkingStage::STAGE2: {
      parking_status_ = vertical_parking_path_generator_.ComputeBackwardPathInStage2(start_pose);
    } break;
    case ParkingStage::STAGE3: {
      switch (out_direction_) {
        // case OutDirection::UP:
        //   parking_status_ =
        //       vertical_parking_path_generator_.ComputePathInStage3Move1(
        //           local_vehicle_current_pose_);
        //   stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
        //   break;
        case OutDirection::Down:
          parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move2(
              local_vehicle_current_pose_);
          stage3_move_ = Stage3Move::OUT_MOVE_FINISH;
          break;
        case OutDirection::Left:
          switch (stage3_move_) {
            case Stage3Move::OUT_MOVE4:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move4(
                  local_vehicle_current_pose_);
              stage3_move_ = Stage3Move::OUT_MOVE5;
              break;

            default:
              break;
          }
          break;
        case OutDirection::Right:
          switch (stage3_move_) {
            case Stage3Move::OUT_MOVE4:
              parking_status_ = vertical_parking_path_generator_.ComputePathInStage3Move4(
                  local_vehicle_current_pose_);
              stage3_move_ = Stage3Move::OUT_MOVE5;
              break;

            default:
              break;
          }
          break;

        default:
          break;
      }
    } break;
    default:
      break;
  }

  if (parking_status_ == Status::Ok()) {
    parking_path_ = vertical_parking_path_generator_.GetBackwardPath();
    if (parking_path_.size() <= 0) {
      parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
      start_time_ = TimeTool::NowToSeconds();
      // parking_sm_->NextState("VerticalParkingFinish");
      return;
    }
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_REVERSE;
    parking_sm_->NextState("VerticalParkingBackward");
  } else {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingFinish");
    return;
  }
}

void ParkingManager::VerticalParkingBackwardStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_STATE;
  // 基于倒车路径和当前位置更新路径
  if (parking_path_.size() > 0) {
    ParkingPathEraseUpdate();
    target_gear_position_ = legionclaw::common::GearPosition::GEAR_REVERSE;
    parking_status_ == Status::Ok();
  } else {
    parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
    start_time_ = TimeTool::NowToSeconds();
    // parking_sm_->NextState("VerticalParkingFinish");
    return;
  }

  if (IsTheVehicleStopped() == true) {
    start_time_ = TimeTool::NowToSeconds();
    if (IsNearDestination() == true) {
      start_time_ = TimeTool::NowToSeconds();
      parking_sm_->NextState("VerticalParkingBackwardFinish");
    }
    if (TimeTool::GetTimeDiffNow(start_time_) > 6.0) {
      AERROR << "the vehicle has not reached the expected destination";
      parking_status_ =
          Status(Status::ErrorCode::PLANNING_ERROR, "not reached the expected destination");
      // parking_sm_->NextState("VerticalParkingFinish");
      return;
    }
  }
}

void ParkingManager::VerticalParkingBackwardFinishStateUpdate(const std::string &state_name,
                                                              int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_FINISH_STATE;
  keep_steer = true;
  if (stage1_move_ == Stage1Move::MOVE1 && TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_status_ = Status::Ok();
    parking_sm_->NextState("VerticalParkingForwardBegin");
  }

  if (stage1_move_ == Stage1Move::MOVE2 && TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_status_ = Status::Ok();
    parking_sm_->NextState("VerticalParkingBackwardBegin");
  }

  // 继续发送规划轨迹
  //  if (parking_path_.size() > 0) {
  //    ParkingPathEraseUpdate();
  //    target_gear_position_ = legionclaw::common::GearPosition::GEAR_DRIVE;
  //    parking_status_ = Status::Ok();
  //  } else {
  //    parking_status_ =
  //        Status(Status::ErrorCode::PLANNING_ERROR, "parking_path_ is null");
  //    start_time_ = TimeTool::NowToSeconds();
  //    parking_sm_->NextState("VerticalParkingFinish");
  //  }

  if (IsTheVehicleStopped() == false) {
    start_time_ = TimeTool::NowToSeconds();
  }

  if (stage1_move_ == Stage1Move::MOVE_FINISH && parking_stage_ != ParkingStage::STAGE3) {
    if (IsParkingPrecisely() == true && TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
      start_time_ = TimeTool::NowToSeconds();
      parking_sm_->NextState("VerticalParkingFinish");
    } else if (IsParkingPrecisely() == false && TimeTool::GetTimeDiffNow(start_time_) > 1.0 &&
               max_re_planning_count_ < parking_manager_conf_.max_re_planning_count()) {
      parking_stage_ = ParkingStage::STAGE2;
      start_time_ = TimeTool::NowToSeconds();
      max_re_planning_count_++;
      parking_sm_->NextState("VerticalParkingForwardBegin");
    } else if (IsParkingPrecisely() == false && TimeTool::GetTimeDiffNow(start_time_) > 1.0 &&
               max_re_planning_count_ >= parking_manager_conf_.max_re_planning_count()) {
      start_time_ = TimeTool::NowToSeconds();
      parking_sm_->NextState("VerticalParkingFinish");
    }
  } else if (stage3_move_ == Stage3Move::OUT_MOVE_FINISH &&
             TimeTool::GetTimeDiffNow(start_time_) > 1.0) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingFinish");
  } else if (TimeTool::GetTimeDiffNow(start_time_) > 1.0 &&
             (stage3_move_ != Stage3Move::OUT_MOVE_FINISH &&
              stage1_move_ != Stage1Move::MOVE_FINISH)) {
    start_time_ = TimeTool::NowToSeconds();
    parking_sm_->NextState("VerticalParkingForwardBegin");
  }
}

void ParkingManager::VerticalParkingFinishStateUpdate(const std::string &state_name, int state)
{
  behaviour_lat_state_ =
      legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FINISH_STATE;
  keep_steer = true;
  if (parking_status_ != Status::Ok()) {
    AERROR << parking_status_.error_message();
  }

  // 判断方向盘是否打正
  start_time_ = TimeTool::NowToSeconds();
  parking_stage_ = ParkingStage::STAGE_NONE;
  out_direction_ = OutDirection::DirectionNone;
  stage1_move_ = ParkingManager::Stage1Move::MOVE_NONE;
  stage3_move_ = ParkingManager::Stage3Move::OUT_MOVE_NONE;
  first_hit_ = false;
  parking_sm_->NextState("ParkingFinish");
}
}  // namespace planning
}  // namespace legionclaw
