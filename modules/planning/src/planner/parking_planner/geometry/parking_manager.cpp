
#include "modules/planning/src/planner/parking_planner/geometry/parking_manager.h"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/planning/src/common/map_matcher/map_matcher.h"
#include "modules/planning/src/common/math/include/mat3.hpp"

using namespace std;

/***********************************************************
* @class ParkingManager
* @brief 生成泊车轨迹信息

class ParkingManager
***********************************************************/
namespace legionclaw {
namespace planning {
using namespace legionclaw::interface;
void ParkingManager::Init(const PlanningConf *planning_conf)
{
  is_init_ = false;
  vehicle_param_ = planning_conf->vehicle_param();
  parallel_parking_conf_ = planning_conf->parking_conf().parallel_parking_conf();
  vertical_parking_conf_ = planning_conf->parking_conf().vertical_parking_conf();
  spiral_curve_conf_ = planning_conf->parking_conf().spiral_curve_conf();
  parking_manager_conf_ = planning_conf->parking_conf().parking_manager_conf();
  safe_width = planning_conf->safe_width_lower_limit();
  safe_length = planning_conf->safe_length_lower_limit();
  // 变量初始化
  start_time_ = 0.0;
  lat_error_ = 0;
  lon_error_ = 0;
  yaw_error_ = 0;
  parking_scheme_ = vertical_parking_conf_.scheme();
  max_re_planning_count_ = 0;
  first_hit_ = false;
  need_max_turn = false;
  vehicle_params_ = planning_conf->vehicle_param();
  legionclaw::interface::VehicleConfig vehicle_params;
  vehicle_params.set_vehicle_param(vehicle_params_);
  VehicleConfigHelper::Init(vehicle_params);
  dtc = parking_manager_conf_.dtc_window_size();
  behaviour_lat_state_ = legionclaw::interface::ADCTrajectory::BehaviourLatState::LAT_NOT_ACTIVE_STATE;
  behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
  parking_stage_ = ParkingStage::STAGE_NONE;
  stage1_move_ = Stage1Move::MOVE_NONE;
  target_gear_position_ = legionclaw::common::GearPosition::GEAR_PARKING;
  parking_status_ = Status(Status::ErrorCode::PLANNING_ERROR_NOT_READY);
  parallel_parking_path_generator_ = ParallelParkingPathGenerator(planning_conf);
  vertical_parking_path_generator_ = VerticalParkingPathGenerator(planning_conf);

  parking_sm_ = nullptr;
  park_in_ = true;
  out_direction_ = OutDirection::DirectionNone;
  ParkingStateMachineInit();
  is_init_ = true;
}

bool ParkingManager::JudgeNeedMaxTurn(const legionclaw::interface::PathPoint p1,
                                      const legionclaw::interface::PathPoint p2)
{
  double dx = p1.x() - p2.x();
  double dy = p1.y() - p2.y();
  double distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.2) {
    return true;
  }
  return false;
}

legionclaw::common::Status ParkingManager::UpdateParkingInfo(legionclaw::common::ParkingType parking_type)
{
  switch (parking_type) {
    case legionclaw::common::ParkingType::PARALLEL_PARKING: {
      double back_distance =
          0.5 * parking_info_.width() - parallel_parking_conf_.parallel_lat_safe_distance();
      UpdateTargetParkingPose(parking_info_, -back_distance);
    } break;
    case legionclaw::common::ParkingType::VERTICAL_PARKING: {
      double back_distance =
          0.5 * parking_info_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
          vehicle_param_.back_edge_to_center();  // 车库中心离泊车坐标系原点的距离
      UpdateTargetParkingPose(parking_info_, -back_distance);
      // 计算倒车过程中，方向盘开始回正时进入车库的深度
    } break;
    default:
      AERROR << "unknown parking type.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "unknown parking type.");
      break;
  }
  return legionclaw::common::Status::Ok();
}

legionclaw::common::Status ParkingManager::UpdateContext()
{
  origin_to_up = parking_info_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
                 vehicle_param_.back_edge_to_center();
  switch (parking_type_) {
    case legionclaw::common::ParkingType::PARALLEL_PARKING: {
      double back_distance = 0.5 * parking_info_.length() -
                             parallel_parking_conf_.parallel_lon_safe_distance() -
                             vehicle_param_.back_edge_to_center();
      UpdateTargetParkingPose(parking_info_, -back_distance);
      parallel_parking_path_generator_.SetParkingSpot(parking_info_);
    } break;
    case legionclaw::common::ParkingType::VERTICAL_PARKING: {
      double back_distance =
          0.5 * parking_info_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
          vehicle_param_.back_edge_to_center();  // 车库中心离泊车坐标系原点的距离
      UpdateTargetParkingPose(parking_info_, -back_distance);
      // std::cout << "=============================================" <<
      // "\n"; std::cout << "x: " <<
      // parking_info_.center_point_of_parking().x()
      //           << "\n";
      // std::cout << "y: " << parking_info_.center_point_of_parking().y()
      //           << "\n";
      // std::cout << "theta: " << parking_info_.theta() << "\n";
      // std::cout << "width: " << parking_info_.width() << "\n";
      // std::cout << "length: " << parking_info_.length() << "\n";
      vertical_parking_path_generator_.SetParkingSpot(parking_info_);
      // std::cout << "=============================================" <<
      // "\n";
      // 计算倒车过程中，方向盘开始回正时进入车库的深度
    } break;
    default:
      AERROR << "unknown parking type.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "unknown parking type.");
      break;
  }
  // 参数3为车库方向东方向的夹角
  transfer_pg_.SetOrigin(target_parking_pose_.x(), target_parking_pose_.y(),
                         target_parking_pose_.theta() - M_PI_2);
  transfer_pg_.TransferG2L(vehicle_current_pose_,
                           local_vehicle_current_pose_);  // 始点坐标转换
  return Status().Ok();
}

void ParkingManager::UpdateParkingOutStartPose()
{
  transfer_pg_.SetOrigin(vehicle_current_pose_.x(), vehicle_current_pose_.y(),
                         vehicle_current_pose_.theta() - M_PI_2);
  transfer_pg_.TransferG2L(vehicle_current_pose_,
                           local_vehicle_current_pose_);  // 始点坐标转换
}

void ParkingManager::SetOutDirection(OutDirection out_direction) { out_direction_ = out_direction; }
legionclaw::common::Status ParkingManager::IsGeometryOK(Frame *frame)
{
  legionclaw::interface::ParkingInfo parking_info = frame->GetParkingInfo();
  legionclaw::interface::PathPoint cur_pose = frame->VehicleState().pose;
  Transfer transfer_pg;
  legionclaw::interface::PathPoint local_cur_pose;
  double out_d = vertical_parking_conf_.out_d();
  double parking_position = 0;
  double pre_s = 0.5;
  double origin_to_up_d =
      (vehicle_param_.front_edge_to_center() + vehicle_param_.back_edge_to_center()) / 2;
  double R = vertical_parking_conf_.caculate_turning_radius();

  bool is_collision = false;
  double lon_protect = 2;
  double lat_protect = 2;
  double lat_error = 0.2;

  Box2d safe_box =
      VehicleConfigHelper::GetDirSafeBoundingBox(cur_pose, lon_protect, lat_protect, lat_error);
  Polygon2d safe_ply(safe_box);
  for (auto obstacle : frame->ObstacleList()) {
    if (obstacle.perception_obstacle().polygon().HasOverlap(safe_ply)) {
      is_collision = true;
    }
  }
  if (is_collision) {
    AERROR << "park_info error.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "park_info error");
  }

  double back_distance = 0.5 * parking_info.length() -
                         vertical_parking_conf_.vertical_lon_safe_distance() -
                         vehicle_param_.back_edge_to_center();  // 车库中心离泊车坐标系原点的距离
  UpdateTargetParkingPose(parking_info, -back_distance);
  transfer_pg.SetOrigin(parking_info.center_point_of_parking().x(),
                        parking_info.center_point_of_parking().y(), parking_info.theta() - M_PI_2);
  transfer_pg.TransferG2L(cur_pose, local_cur_pose);

  if (fabs(local_cur_pose.theta()) > M_PI_2) {
    parking_position = -1;
  } else {
    parking_position = 1;
  }
  if (local_cur_pose.y() - vehicle_param_.front_edge_to_center() < out_d) {
    AERROR << "park_info error.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "park_info error");
  } else {
    double theta = acos((R - out_d) / R);
    ;
    // 2*(R-R*cos(theta))<=out_d
    //  double theta=acos((R-out_d/2)/R);
    legionclaw::interface::PathPoint Aim_pathpoint;
    Aim_pathpoint.set_x(parking_position * R * sin(theta));
    Aim_pathpoint.set_y(origin_to_up_d + back_distance + out_d);
    Aim_pathpoint.set_theta(M_PI_2 + parking_position * M_PI_2);
    local_cur_pose.set_x(local_cur_pose.x() + pre_s);

    double lat = abs(Aim_pathpoint.x() - local_cur_pose.x());
    double lon = abs(Aim_pathpoint.y() - local_cur_pose.y());
    double yaw =
        legionclaw::common::math::NormalizeAngle(Aim_pathpoint.theta() - local_cur_pose.theta());

    if ((parking_position == -1 && (local_cur_pose.x() < Aim_pathpoint.x())) ||
        (parking_position == 1 && (local_cur_pose.x() > Aim_pathpoint.x())) || lat > 10) {
      AERROR << "park_info error.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "park_info error");
    } else {
      if (Aim_pathpoint.y() - local_cur_pose.y() <= 0) {
        // 车头偏下
        if (local_cur_pose.theta() < 0 || local_cur_pose.theta() > 3 * M_PI_2) {
          if (lat >= R * sin(yaw) && lon <= R * (1 - cos(yaw)) && yaw <= M_PI_2 / 2) {
            return Status::Ok();
            ;
            ;
          }
        } else {
          if (lat >= R * sin(yaw) + 2 * R * sin(theta) && lon <= out_d && yaw <= M_PI_2 / 2) {
            return Status::Ok();
            ;
            ;
          }
        }
      }
      // 目标点位于当前位置上方
      else {
        // 车头偏下
        if (local_cur_pose.theta() < 0 || local_cur_pose.theta() > 3 * M_PI_2) {
          if (lat >= R * sin(yaw) + 2 * R * sin(theta) && lon <= out_d && yaw <= M_PI_2 / 2) {
            return Status::Ok();
          }
        } else {
          if (lat >= R * sin(yaw) && lon <= R * (1 - cos(yaw)) && yaw <= M_PI_2 / 2) {
            return Status::Ok();
          }
        }
      }
    }
  }
  AERROR << "park_info error.";
  return Status(Status::ErrorCode::PLANNING_ERROR, "park_info error");
}

legionclaw::common::Status ParkingManager::Process(Frame *frame,
                                              legionclaw::interface::ADCTrajectory *ptr_adc_trajectory,
                                              legionclaw::interface::PlanningAnalysis *ptr_analysis)
{
  if (out_direction_ != OutDirection::DirectionNone) {
    park_in_ = false;
  } else {
    park_in_ = true;
  }

  if (frame == nullptr) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "frame is nullptr.");
  }

  if (park_in_) {
    // TODO 增加CheckIn(),校验停车点和泊车位的时效性
    parking_info_ = frame->GetParkingInfo();
    if (parking_info_.parking_status() == ParkingStatus::PARKING_DISENABLE) {
      return Status(Status::ErrorCode::PLANNING_ERROR,
                    "parking_info is_parking_enable() is false.");
    }

    if (parking_info_.parking_type() == legionclaw::common::ParkingType::INVALID_PARKING) {
      return Status(Status::ErrorCode::PLANNING_ERROR,
                    "parking_info parking_type() is "
                    "legionclaw::common::ParkingType::INVALID_PARKING.");
    }
    vehicle_state_ = frame->VehicleState();
    parking_type_ = frame->GetParkingInfo().parking_type();
    obstacle_list_ = frame->ObstacleList();

    UpdateVehicleCurrentPose(vehicle_state_.pose);

    if (first_hit_ == false) {
      UpdateContext();
      first_hit_ = true;
    }
  } else {
    vehicle_state_ = frame->VehicleState();
    parking_type_ = frame->GetParkingInfo().parking_type();
    obstacle_list_ = frame->ObstacleList();

    UpdateVehicleCurrentPose(vehicle_state_.pose);

    if (first_hit_ == false) {
      // TODO 添加水平泊出方案
      parking_type_ = legionclaw::common::ParkingType::VERTICAL_PARKING;
      UpdateContext();
      // UpdateParkingOutStartPose();
      first_hit_ = true;
    }
  }
  transfer_pg_.TransferG2L(vehicle_current_pose_,
                           local_vehicle_current_pose_);  // 始点坐标转换
  parking_status_ = legionclaw::common::Status::Ok();
  // 状态机更新
  parking_sm_->OnUpdate();

  if (parking_status_ != legionclaw::common::Status::Ok()) {
    return parking_status_;
  }

  // if(parking_sm_->GetCurrentStateName()=="ParkingInitialState"){
  //    return legionclaw::common::Status::Ok();
  // }
  if (parking_sm_->GetCurrentStateName() == "ParkingInitialState" ||
      parking_sm_->GetCurrentStateName() == "ParallelParkingBeginState" ||
      parking_sm_->GetCurrentStateName() == "ParallelParkingForwardBeginState" ||
      parking_sm_->GetCurrentStateName() == "VerticalParkingBeginState" ||
      parking_sm_->GetCurrentStateName() == "VerticalParkingForwardBeginState" ||
      parking_sm_->GetCurrentStateName() == "VerticalParkingBackwardBeginState")
    return parking_status_;

  if (parking_sm_->GetCurrentStateName() == "ParkingInitialState") {
    return legionclaw::common::Status::Ok();
  }

  std::vector<legionclaw::interface::PathPoint> parking_path_global;

  // 坐标系转换（泊车坐标系转全局坐标系）
  if (TransferPark2Global(parking_path_, parking_path_global) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "TransferPark2Global() failed.");
  }

  behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;

  bool in_collision =
      InCollision(parking_manager_conf_.ttc_window_size(), ptr_analysis, parking_trajectory_);

  if (in_collision) {
    behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::STOP;
  } else if (parking_sm_->GetCurrentStateName() == "ParkingFinishState") {
    behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::PRECISE_STOP;
  } else {
    behaviour_lon_state_ = legionclaw::interface::ADCTrajectory::BehaviourLonState::NORMAL;
  }

  if (ComputeLonInfo(parking_path_global,  // cur_speed, cur_acc,
                     vertical_parking_conf_.vertical_path_interval(), target_gear_position_,
                     in_collision, &parking_trajectory_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,

                  "ComputeLonInfo() failed.");
  }

  // if (need_max_turn) {
  //   for (auto &i : *(parking_trajectory_.mutable_trajectory_points())) {
  //     i.set_is_steer_valid(true);
  //     int a = vertical_parking_path_generator_.get_parking_position();
  //     i.set_steer(a * vertical_parking_conf_.wheel_theta());
  //   }
  //   need_max_turn = false;
  // }

  if ((steer_first_ && !IsVehicleSteerNearCmd() &&
       parking_trajectory_.mutable_trajectory_points()->size() != 0) &&
      (stage1_move_ != Stage1Move::MOVE_FINISH || stage3_move_ != Stage3Move::OUT_MOVE_FINISH)) {
    for (auto &i : *(parking_trajectory_.mutable_trajectory_points())) {
      i.set_v(0);
    }
  }
  // 轨迹生成
  UpdateADCTrajectory(&parking_trajectory_, ptr_adc_trajectory);

  return parking_status_;
}

void ParkingManager::ParkingPathEraseUpdate()
{
  int index = -1;
  double match_dis = DBL_MAX;
  index = MapMatcher::QueryNearestPointWithBuffer(
      parking_path_, {local_vehicle_current_pose_.x(), local_vehicle_current_pose_.y()},
      local_vehicle_current_pose_.theta(), 1.0e-6, match_dis);

  if (index > 0) {
    uint32_t len = index - 1;
    if (len > parking_path_.size()) {
      // 保留一个点
      len = parking_path_.size() - 1;
    }
    parking_path_.erase(parking_path_.begin(), parking_path_.begin() + len);
  }

  remain_length_ = (parking_path_.size() - 1) * vertical_parking_conf_.vertical_path_interval();
}

bool ParkingManager::IsTheVehicleStopped()
{
  if (vehicle_state_.moving_status == legionclaw::common::MovingStatus::STATIONARY) {
    return true;
  }
  return false;
}

bool ParkingManager::IsHalfDestination()
{
  if (parking_path_.size() < size_t(num_size / 2)) {
    return true;
  }
  return false;
}

bool ParkingManager::IsNearDestination()
{
  if ((stage1_move_ == Stage1Move::MOVE3) &&
      (remain_length_ <= parking_manager_conf_.threshold_remian_length())) {
    return true;
  }

  if (remain_length_ <= parking_manager_conf_.threshold_remian_length()) {
    return true;
  }

  return false;
}

bool ParkingManager::IsParkingPrecisely()
{
  double dx = vehicle_current_pose_.x() - target_parking_pose_.x();
  double dy = vehicle_current_pose_.y() - target_parking_pose_.y();
  double theta = target_parking_pose_.theta();
  if (parking_type_ == legionclaw::common::ParkingType::PARALLEL_PARKING) {
    theta = theta + M_PI_2;
  }
  double yaw_diff = math::AngleDiff(theta,
                                    vehicle_current_pose_.theta());  // 角度误差
  double lat_error = dx * std::sin(theta) - dy * std::cos(theta);    // 横向位置误差
  double lon_error = dx * std::cos(theta) + dy * std::sin(theta);    // 纵向位置误差

  double temp = R2D(std::abs(yaw_diff));

  lon_error_ = lon_error;
  lat_error_ = lat_error;
  yaw_error_ = temp;

  if (lon_error <= parking_manager_conf_.threshold_lon_error() &&
      fabs(lat_error) <= parking_manager_conf_.threshold_lat_error() &&
      temp <= parking_manager_conf_.threshold_yaw_error()) {
    return true;
  }

  return false;
}

void ParkingManager::UpdateADCTrajectory(legionclaw::interface::Trajectory *trajectory,
                                         legionclaw::interface::ADCTrajectory *ptr_adc_trajectory)
{
  ptr_adc_trajectory->set_behaviour_lat_state(behaviour_lat_state_);
  ptr_adc_trajectory->set_behaviour_lon_state(behaviour_lon_state_);
  ptr_adc_trajectory->set_driving_mode(legionclaw::common::DrivingMode::COMPLETE_AUTO_DRIVE);
  ptr_adc_trajectory->set_trajectory_points(trajectory->mutable_trajectory_points());
}

bool ParkingManager::TransferPark2Global(const std::vector<legionclaw::interface::PathPoint> &path_p,
                                         std::vector<legionclaw::interface::PathPoint> &path_g)
{
  int num = path_p.size();
  if (num <= 0) {
    AERROR << "!!!path_p is empty" << "\n";
    return false;
  }

  path_g.resize(num);
  for (int i = 0; i < num; i++) {
    transfer_pg_.TransferL2G(path_p.at(i), path_g.at(i));
  }
  return true;
}

void ParkingManager::GetSafetyBorderPolygon(const legionclaw::interface::PathPoint &curr_state,
                                            const legionclaw::interface::VehicleParam &vehicle_param,
                                            const double &safe_width, const double &safe_length,
                                            math::Polygon2d &polygon)
{
  double c_lateral_d = 0.5 * vehicle_param.width() + safe_width;
  double c_long_front_d = 0.5 * vehicle_param.length() + 0.5 * vehicle_param.length() + safe_length;
  double c_long_back_d = 0.5 * vehicle_param.length() + safe_length - 0.5 * vehicle_param.length();

  // 旋转矩阵
  math::Mat3 inv_rotation_mat(curr_state.theta() - M_PI_2);
  // 平移矩阵
  math::Mat3 inv_translation_mat(curr_state.x(), curr_state.y());

  legionclaw::interface::PointBasic bottom_left;
  bottom_left.set_x(-c_lateral_d);
  bottom_left.set_y(-c_long_back_d);
  bottom_left.set_z(0.0);
  bottom_left.set_theta(0.0);

  legionclaw::interface::PointBasic bottom_right;
  bottom_right.set_x(c_lateral_d);
  bottom_right.set_y(-c_long_back_d);
  bottom_right.set_z(0.0);
  bottom_right.set_theta(0.0);

  legionclaw::interface::PointBasic top_right;
  top_right.set_x(c_lateral_d);
  top_right.set_y(c_long_front_d);
  top_right.set_z(0.0);
  top_right.set_theta(0.0);

  legionclaw::interface::PointBasic top_left;
  top_left.set_x(-c_lateral_d);
  top_left.set_y(c_long_front_d);
  top_left.set_z(0.0);
  top_left.set_theta(0.0);

  bottom_left = inv_rotation_mat * bottom_left;
  bottom_left = inv_translation_mat * bottom_left;

  top_right = inv_rotation_mat * top_right;
  top_right = inv_translation_mat * top_right;

  bottom_right = inv_rotation_mat * bottom_right;
  bottom_right = inv_translation_mat * bottom_right;

  top_left = inv_rotation_mat * top_left;
  top_left = inv_translation_mat * top_left;

  vector<math::Vec2d> poly_vertices;
  polygon.ClearPoints();
  poly_vertices.push_back(math::Vec2d(bottom_left.x(), bottom_left.y()));
  poly_vertices.push_back(math::Vec2d(bottom_right.x(), bottom_right.y()));
  poly_vertices.push_back(math::Vec2d(top_right.x(), top_right.y()));
  poly_vertices.push_back(math::Vec2d(top_left.x(), top_left.y()));
  polygon = math::Polygon2d(poly_vertices);
}

bool ParkingManager::ComputeLonInfoLinear(
    const std::vector<legionclaw::interface::PathPoint> &parking_path, const double &cur_speed,
    const double &cur_acc, const double &path_interval, const GearPosition &gear_flag,
    bool in_collision, Trajectory *ptr_trajectory)
{
  if (ptr_trajectory == nullptr) {
    AERROR << "ptr_trajectory is null";
    return false;
  }
  if (path_interval < 1e-9) {
    AERROR << "path_interval is too mall";
    return false;
  }

  ptr_trajectory->clear_trajectory_points();

  if (parking_path.size() == 0) return false;

  double target_speed = 0.0;
  GearPosition target_gear = GEAR_PARKING;
  if (!in_collision) {
    if (gear_flag == GEAR_DRIVE)  // 无碰撞且前进部分
    {
      target_speed = parking_manager_conf_.suggest_speed_forward() / 3.6;
      target_gear = GEAR_DRIVE;
    } else if (gear_flag == GEAR_REVERSE)  // 无碰撞且倒车部分
    {
      target_speed = parking_manager_conf_.suggest_speed_backward() / 3.6;
      target_gear = GEAR_REVERSE;
    }
  }

  // 计算纵向信息
  //  double mileage = 0.0;
  double kappa_speed = 0.0;
  double max_lat_acc = 0.009;       // write to config file
  double min_speed = 1.0;           //(km/h) //write to config file
  double norm_acceleration = 0.3;   // write to config file
  double norm_deceleration = -0.5;  // write to config file
  double acc = 0.0;
  double up_v = cur_speed;
  double down_v = 0.0;
  double tmp_v = 0.0;
  double last_t = 0.0;
  double last_v = cur_speed;
  for (unsigned int i = 1; i < parking_path.size(); ++i) {
    legionclaw::interface::PathPoint path_point = parking_path[i];
    legionclaw::interface::TrajectoryPoint trajectory_point;

    trajectory_point.set_gear(target_gear);
    if (i == 0) {
      path_point.set_s(0.0);
      trajectory_point.set_relative_time(0.0);
      trajectory_point.set_v(cur_speed);
      trajectory_point.set_a(cur_acc);
    } else {
      path_point.set_s(path_interval * i);

      if (target_speed > cur_speed)
        acc = norm_acceleration;
      else if (target_speed < cur_speed)
        acc = norm_deceleration;

      up_v = cur_speed * cur_speed + 2.0 * acc * path_interval * i;
      if (up_v > 0.0)
        up_v = sqrt(up_v);
      else
        up_v = 0.0;
      if (up_v > target_speed) up_v = target_speed;

      down_v = -2.0 * norm_deceleration * path_interval * (parking_path.size() - 1 - i);
      if (down_v > 0.0)
        down_v = sqrt(down_v);
      else
        down_v = 0.0;
      if (down_v > target_speed) down_v = target_speed;

      tmp_v = up_v > down_v ? down_v : up_v;
      // trajectory_point.set_path_point(it);
      if (fabs(path_point.kappa()) < 1e-9)
        kappa_speed = tmp_v;
      else
        kappa_speed = sqrt(fabs(max_lat_acc / path_point.kappa()));
      if (kappa_speed < min_speed / 3.6)  // 防止速度过小
        kappa_speed = min_speed / 3.6;
      if (kappa_speed > tmp_v) kappa_speed = tmp_v;
      trajectory_point.set_v(kappa_speed);

      double dt = 0.0;
      double a = 0.0;
      constexpr double epsilon = 1e-9;
      if (std::fabs(last_v + kappa_speed) > epsilon) {
        dt = 2.0 * path_interval / (last_v + kappa_speed);
        a = (kappa_speed - last_v) / dt;
      }
      trajectory_point.set_relative_time(last_t + dt);
      trajectory_point.set_a(a);

      last_v = kappa_speed;
      last_t += dt;
    }
    trajectory_point.set_path_point(path_point);
    ptr_trajectory->add_trajectory_points(trajectory_point);
  }

  // 延长一段轨迹
  Trajectory extend_trajectory;

  if (ExtendTrajectory(ptr_trajectory->trajectory_points().back().path_point(),
                       parking_manager_conf_.extend_length(), path_interval, gear_flag,
                       extend_trajectory)) {
    ptr_trajectory->mutable_trajectory_points()->insert(
        ptr_trajectory->trajectory_points().end(), extend_trajectory.trajectory_points().begin(),
        extend_trajectory.trajectory_points().end());
  }

  return true;
}
bool ParkingManager::ComputeLonInfo(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                                    const double &path_interval, const GearPosition &gear_flag,
                                    bool in_collision, Trajectory *ptr_trajectory)
{
  if (ptr_trajectory == nullptr) {
    AERROR << "ptr_trajectory is null";
    return false;
  }

  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  ptr_trajectory->clear_trajectory_points();

  if (parking_path.size() == 0) return false;

  double target_speed = 0.0;
  GearPosition target_gear = GEAR_PARKING;
  if (!in_collision) {
    if (gear_flag == GEAR_DRIVE)  // 无碰撞且前进部分
    {
      target_speed = parking_manager_conf_.suggest_speed_forward() / 3.6;
      target_gear = GEAR_DRIVE;
    } else if (gear_flag == GEAR_REVERSE)  // 无碰撞且倒车部分
    {
      target_speed = parking_manager_conf_.suggest_speed_backward() / 3.6;
      target_gear = GEAR_REVERSE;
    }
  }

  // 计算纵向信息
  double s = 0.0;
  bool empty_path = true;
  double t = 0.0;
  // double v = 0.0;

  for (unsigned int i = 0; i < parking_path.size(); ++i) {
    legionclaw::interface::PathPoint path_point = parking_path[i];
    legionclaw::interface::TrajectoryPoint trajectory_point;
    // trajectory_point.set_path_point(it);
    trajectory_point.set_v(target_speed);

    trajectory_point.set_a(0.0);
    trajectory_point.set_gear(target_gear);
    if (empty_path) {
      path_point.set_s(s);
      trajectory_point.set_relative_time(0.0);
      empty_path = false;
    } else {
      double dx = parking_path[i].x() - parking_path[i - 1].x();
      double dy = parking_path[i].y() - parking_path[i - 1].y();
      double ds = std::hypot(dx, dy);
      s += ds;
      path_point.set_s(s);

      double average_v = target_speed;

      // constexpr double epsilon = 1e-9;
      double dt = 0.0;
      if (average_v > 0.01) {
        dt = ds / average_v;
      } else {
        dt = 0.0;
      }

      t += dt;
      trajectory_point.set_relative_time(t);
    }

    trajectory_point.set_path_point(path_point);
    trajectory_point.set_is_steer_valid(true);
    ptr_trajectory->add_trajectory_points(trajectory_point);
  }

  // 延长一段轨迹
  Trajectory extend_trajectory;

  if (ExtendTrajectory(ptr_trajectory->trajectory_points().back().path_point(),
                       parking_manager_conf_.extend_length(), path_interval, gear_flag,
                       extend_trajectory)) {
    ptr_trajectory->mutable_trajectory_points()->insert(
        ptr_trajectory->trajectory_points().end(), extend_trajectory.trajectory_points().begin(),
        extend_trajectory.trajectory_points().end());
  }

  return true;
}

bool ParkingManager::IsVehicleSteerNearCmd()
{
  double steer_cmd;
  steer_cmd = vertical_parking_path_generator_.steer_value_;

  double steer_diff = std::abs(vehicle_state_.steer - steer_cmd);

  if (steer_diff < parking_manager_conf_.is_near_theta()) {
    steer_first_ = false;
    return true;
  }
  std::cout << steer_diff << "Wheel steering did not meet expectations." << "\n";
  if ((abs(vehicle_state_.steer) > abs(steer_cmd) && (vehicle_state_.steer * steer_cmd > 0)) ||
      steer_cmd == 0) {
    steer_first_ = false;
    return true;
  }
  return false;
}

bool ParkingManager::ComputeLonInfo(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                                    const double &path_interval, const GearPosition &gear_flag,
                                    const double &collision_distance, Trajectory *ptr_trajectory)
{
  if (ptr_trajectory == nullptr) {
    AERROR << "ptr_trajectory is null";
    return false;
  }

  std::vector<legionclaw::interface::TrajectoryPoint> *ptr_trajectory_points =
      ptr_trajectory->mutable_trajectory_points();

  ptr_trajectory->clear_trajectory_points();

  if (parking_path.size() == 0) {
    return false;
  }

  double target_speed = 0.0;
  common::GearPosition target_gear = GEAR_PARKING;
  if (collision_distance < 0.0 && gear_flag == GEAR_DRIVE)  // 无碰撞且前进部分
  {
    target_speed = parking_manager_conf_.suggest_speed_forward() / 3.6;
    target_gear = GEAR_DRIVE;
  } else if (collision_distance < 0.0 && gear_flag == GEAR_REVERSE)  // 无碰撞且倒车部分
  {
    target_speed = parking_manager_conf_.suggest_speed_backward() / 3.6;
    target_gear = GEAR_REVERSE;
  }

  // 计算纵向信息
  double mileage = 0.0;

  for (unsigned int i = 0; i < parking_path.size(); ++i) {
    legionclaw::interface::PathPoint path_point = parking_path[i];
    legionclaw::interface::TrajectoryPoint trajectory_point;
    // trajectory_point.set_path_point(it);
    trajectory_point.set_v(target_speed);
    trajectory_point.set_a(0.0);
    trajectory_point.set_gear(target_gear);
    if (i == 0) {
      path_point.set_s(mileage);
      trajectory_point.set_relative_time(0.0);
    } else {
      mileage += math::Norm(parking_path[i].x() - parking_path[i - 1].x(),
                            parking_path[i].y() - parking_path[i - 1].y());
      path_point.set_s(mileage);
      if (trajectory_point.v() + trajectory_point.v() == 0.0) {
        trajectory_point.set_relative_time((*ptr_trajectory_points)[i - 1].relative_time());
      } else {
        double s1 = (*ptr_trajectory_points)[i].path_point().s();
        double s2 = (*ptr_trajectory_points)[i - 1].path_point().s();
        double v1 = (*ptr_trajectory_points)[i].v();
        double v2 = (*ptr_trajectory_points)[i - 1].v();
        // 取平均速度
        trajectory_point.set_relative_time((*ptr_trajectory_points)[i - 1].relative_time() +
                                           2.0 * -(s1 - s2) / (v1 + v2));
      }
    }
    trajectory_point.set_path_point(path_point);
    ptr_trajectory->add_trajectory_points(trajectory_point);
  }

  // 延长一段轨迹
  Trajectory extend_trajectory;

  if (ExtendTrajectory(ptr_trajectory->trajectory_points().back().path_point(),
                       parking_manager_conf_.extend_length(), path_interval, gear_flag,
                       extend_trajectory)) {
    ptr_trajectory->mutable_trajectory_points()->insert(
        ptr_trajectory->trajectory_points().end(), extend_trajectory.trajectory_points().begin(),
        extend_trajectory.trajectory_points().end());
  }

  return true;
}

bool ParkingManager::ExtendTrajectory(const legionclaw::interface::PathPoint &start_pos,
                                      const double &extend_length, const double &interval,
                                      const GearPosition &gear_flag, Trajectory &extend_trajectory)
{
  double interval_tmp = interval;
  legionclaw::interface::PathPoint path_point;
  legionclaw::interface::TrajectoryPoint trajectory_point;
  path_point = start_pos;

  if (gear_flag == legionclaw::common::GearPosition::GEAR_REVERSE) interval_tmp *= -1.0;

  double s = 0.0;
  extend_trajectory.clear_trajectory_points();
  while (s <= extend_length) {
    s += fabs(interval_tmp);
    double x = path_point.x();
    double y = path_point.y();
    x += interval_tmp * cos(path_point.theta());
    y += interval_tmp * sin(path_point.theta());
    path_point.set_x(x);
    path_point.set_y(y);
    trajectory_point.set_a(0.0);
    trajectory_point.set_gear(legionclaw::common::GearPosition::GEAR_PARKING);
    trajectory_point.set_v(0.0);
    path_point.set_s(start_pos.s() + s);
    path_point.set_kappa(0.0);
    trajectory_point.set_path_point(path_point);
    trajectory_point.set_is_steer_valid(true);
    extend_trajectory.add_trajectory_points(trajectory_point);
  }

  return true;
}
bool ParkingManager::InCollision(const double &ttc_window_size,
                                 legionclaw::interface::PlanningAnalysis *ptr_analysis,
                                 legionclaw::interface::Trajectory trajectory)
{
  // 计算转弯半径
  double s = 0.02;
  double temp_steer = std::abs(vehicle_state_.steer);
  double steer = (temp_steer < 0.1) ? 0.1 : temp_steer;

  double r = vehicle_param_.wheelbase() / steer;
  // 线速度转换为角速度
  double omega = std::abs(vehicle_state_.speed) / r;
  double l = omega * ttc_window_size * r;
  double t_dtc = max(l, dtc);
  int32_t preview_window_size = t_dtc / s + 1 + 0.5;

  legionclaw::planning::PlanningParkingDebug planningdeug;
  planningdeug.set_lat_error(lat_error_);
  planningdeug.set_lon_error(lon_error_);
  planningdeug.set_yaw_error(yaw_error_);
  ptr_analysis->set_planning_parking_debug(planningdeug);
  std::vector<legionclaw::interface::PathPoint> tra_points;
  legionclaw::interface::PathPoint tra_point;
  for (auto i : trajectory.trajectory_points()) {
    tra_point = i.path_point();
    tra_points.emplace_back(tra_point);
  }

  if ((tra_points.size() > 0) == true) {
    double distance = sqrt((tra_points.at(0).x() - vehicle_current_pose_.x()) *
                               (tra_points.at(0).x() - vehicle_current_pose_.x()) +
                           (tra_points.at(0).y() - vehicle_current_pose_.y()) *
                               (tra_points.at(0).y() - vehicle_current_pose_.y()));
    int point_i = 0;
    for (int i = 0; i < int(tra_points.size() - 1); i++) {
      double dx1 = tra_points.at(i + 1).x() - vehicle_current_pose_.x();
      double dy1 = tra_points.at(i + 1).y() - vehicle_current_pose_.y();
      double caculate_distance = sqrt(dx1 * dx1 + dy1 * dy1);
      if (caculate_distance < distance) {
        distance = caculate_distance;
        point_i = i + 1;
      }
    }

    double dx = vehicle_current_pose_.x() - tra_points.at(point_i).x();
    double dy = vehicle_current_pose_.y() - tra_points.at(point_i).y();
    double lat_error = dx * std::sin(target_parking_pose_.theta()) -
                       dy * std::cos(target_parking_pose_.theta());  // 横向位置误差
    double lon_protect = vertical_parking_conf_.vehicle_lon_buffer();
    double lat_protect = vertical_parking_conf_.vehicle_lat_buffer() + 2 * fabs(lat_error);
    for (int32_t i = 0; i < preview_window_size; i++) {
      tra_point = tra_points.at(i);

      if (int32_t(tra_points.size() - 1) == i) {
        i = preview_window_size;
      }
      Box2d t_safeboundingbox =
          VehicleConfigHelper::GetSafeBoundingBox(tra_point, lon_protect, lat_protect);

      legionclaw::common::math::Polygon2d safeboundingbox(t_safeboundingbox);

      for (auto obstacle : obstacle_list_) {
        auto polygon_points = obstacle.perception_obstacle().polygon().points();
        // std::vector<Vec2d> local_points;
        // std::for_each(polygon_points.begin(), polygon_points.end(),
        //               [=, &local_points](Vec2d& vec_2d) {
        //                 Vec2d out;
        //                 transfer_pg_.PathPointNormalizing(vec_2d, out);
        //                 local_points.emplace_back(out);
        //               });
        // Polygon2d local_polygon(local_points);
        // if (local_polygon.HasOverlap(safeboundingbox)) {
        //   dtc=t_dtc;
        //   return true;
        if (obstacle.perception_obstacle().polygon().HasOverlap(safeboundingbox)) {
          dtc = t_dtc;
          return true;
        }
        //
      }
    }
  }
  dtc = parking_manager_conf_.dtc_window_size();
  return false;
}

legionclaw::interface::Polygon2D ParkingManager::Polygon2d_to_Polygon2D(
    const legionclaw::common::math::Polygon2d p1)
{
  Polygon2D p2;
  for (unsigned int i = 0; i < p1.points().size(); i++) {
    Vec2d temp = p1.points().at(i);
    Point2D point;
    point.set_x(temp.x());
    point.set_y(temp.y());
    p2.add_points(point);
  }
  return p2;
}

bool ParkingManager::CollisionCheck(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                                    const double &safe_width, const double &safe_length,
                                    const double &safe_distance, double &collision_distance)
{
  if (parking_path.size() == 0) return false;

  double delta_s = 0.0;
  double total_s = 0.0;
  double step_s = vehicle_param_.width();

  double x = parking_path.front().x();
  double y = parking_path.front().y();
  for (unsigned int i = 1; i < parking_path.size(); ++i) {
    delta_s += math::Norm(parking_path.at(i).x() - x, parking_path.at(i).y() - y);
    total_s += math::Norm(parking_path.at(i).x() - x, parking_path.at(i).y() - y);
    x = parking_path.at(i).x();
    y = parking_path.at(i).y();

    if (delta_s >= step_s || total_s >= safe_distance - safe_length) {
      // TODO构建碰撞检测区域（多边形）
      math::Polygon2d car_safety_border(
          vehicle_state_.pose.x(), vehicle_state_.pose.y(), vehicle_state_.pose.theta(),
          vehicle_param_.width() + safe_width, vehicle_param_.length() + safe_length);

      // TODO 碰撞检测 需要测试
      for (auto obstacle : obstacle_list_) {
        math::Polygon2d obstacle_polygon2d(obstacle.mutable_perception_obstacle()->position().x(),
                                           obstacle.mutable_perception_obstacle()->position().y(),
                                           obstacle.mutable_perception_obstacle()->theta(),
                                           obstacle.mutable_perception_obstacle()->width(),
                                           obstacle.mutable_perception_obstacle()->length());
        if (obstacle_polygon2d.HasOverlap(car_safety_border)) {
          collision_distance = total_s;
        }
      }

      delta_s = 0.0;
    }

    if (total_s >= safe_distance - safe_length) break;
  }

  return true;
}
legionclaw::interface::TrajectoryPoint ParkingManager::PathPoint2WayPoint(legionclaw::interface::PathPoint in)
{
  legionclaw::interface::TrajectoryPoint out;
  out.set_path_point(in);

  return out;
}

void ParkingManager::UpdateVehicleCurrentPose(
    const legionclaw::interface::PathPoint vehicle_current_pose)
{
  vehicle_current_pose_ = vehicle_current_pose;
}

void ParkingManager::UpdateTargetParkingPose(const legionclaw::interface::PathPoint target_parking_pos)
{
  target_parking_pose_ = target_parking_pos;
}
void ParkingManager::UpdateTargetParkingPose(const double &x, const double &y, const double &yaw)
{
  target_parking_pose_.set_x(x);
  target_parking_pose_.set_y(y);
  target_parking_pose_.set_theta(yaw);
}
void ParkingManager::UpdateTargetParkingPose(legionclaw::interface::ParkingInfo parking_info)
{
  target_parking_pose_.set_x(parking_info.center_point_of_parking().x());
  target_parking_pose_.set_y(parking_info.center_point_of_parking().y());
  target_parking_pose_.set_theta(parking_info.theta());
}
void ParkingManager::UpdateTargetParkingPose(legionclaw::interface::ParkingInfo parking_info,
                                             double length)
{
  // 由车库几何中心转到车辆后轴中心
  double theta = parking_info.theta();
  if (parking_type_ == legionclaw::common::ParkingType::PARALLEL_PARKING) {
    theta = theta + M_PI_2;
  }
  target_parking_pose_.set_x(parking_info.center_point_of_parking().x() + length * cos(theta));
  target_parking_pose_.set_y(parking_info.center_point_of_parking().y() + length * sin(theta));
  target_parking_pose_.set_theta(parking_info.theta());
}

bool ParkingManager::Reset()
{
  if (is_init_ == false) {
    return false;
  }

  parking_status_ = Status(Status::ErrorCode::OK);
  parking_sm_->Reset();
  parking_sm_->NextState("ParkingInitial");
  parking_stage_ = ParkingStage::STAGE_NONE;
  out_direction_ = OutDirection::DirectionNone;
  stage1_move_ = ParkingManager::Stage1Move::MOVE_NONE;
  stage3_move_ = ParkingManager::Stage3Move::OUT_MOVE_NONE;
  first_hit_ = false;
  return true;
}
}  // namespace planning
}  // namespace legionclaw
