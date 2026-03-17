

#include "modules/planning/src/planner/parking_planner/geometry/parallel_parking_path_generator.h"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>

#include "modules/common/math/math_utils.h"
#include "modules/common/logging/logging.h"

using namespace std;
/***********************************************************
* @class ParallelParkingPathGenerator
* @brief 生成泊车轨迹信息

class ParallelParkingPathGenerator
***********************************************************/
namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
/**
 * @brief 构造函数
 */
ParallelParkingPathGenerator::ParallelParkingPathGenerator(const PlanningConf *planning_conf)
{
  vehicle_param_ = planning_conf->vehicle_param();
  SetVehicleParam(vehicle_param_);
  parallel_parking_conf_ = planning_conf->parking_conf().parallel_parking_conf();
  UpdateSpiralCurveConfig(planning_conf->parking_conf().spiral_curve_conf());  // 构建可行驶区域
  ConstructFreeSpace();
}

void ParallelParkingPathGenerator::SetParkingSpot(const legionclaw::interface::ParkingInfo &parking_spot)
{
  parking_spot_ = parking_spot;
}

bool ParallelParkingPathGenerator::IsParkingSpaceValid()
{
  H_ = 0.5 * (vehicle_param_.width() + parking_spot_.width()) + 1.5;  // TODO 此处1.5可灵活配置
  H_ = max(H_, fabs(local_vehicle_current_pose_.y()));
  if (4.0 * vehicle_param_.min_turning_radius() >= H_) {
    S_ = sqrt(4.0 * H_ * vehicle_param_.min_turning_radius() - H_ * H_);  // TODO 此处+1可去掉
  } else
    return false;
  return true;

  // double temp =
  //     pow((vehicle_param_.min_turning_radius() + 0.5 * vehicle_param_.width()),
  //         2) +
  //     pow((vehicle_param_.length() - vehicle_param_.back_edge_to_center()),
  //         2) -
  //     pow((parking_spot_.length() - vehicle_param_.back_edge_to_center() -
  //          parallel_parking_conf_.parallel_lon_safe_distance()),
  //         2);
  // return parking_spot_.width() <=
  //        2.0 * (vehicle_param_.min_turning_radius() - sqrt(temp));
}

Status ParallelParkingPathGenerator::ComputeForwardPathInStage1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2) {
    parking_position_ = -1;
  } else {
    parking_position_ = 1;
  }

  if (!IsParkingSpaceValid()) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "The parking space is invalid.");
  }
  UpdateStage1ForwardControlPoints(parking_position_);
  // 判断标准情况下，泊车空间是否足够
  if (IsEnoughFreeSpace(stage1_f_a1_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "there is no enough free space to parking.");
  }

  forward_path_.clear();
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  spiral::CubicSpiralCurve csc(cur_pose, stage1_f_a0_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = cur_pose.x() - stage1_f_a0_.x();
  double dy = cur_pose.y() - stage1_f_a0_.y();
  int num = (int)(math::Norm(dx, dy) / parallel_parking_conf_.parallel_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }
  forward_path_.insert(forward_path_.end(), forward_path1.begin(), forward_path1.end());
  // 往后延长一段forward_path2，便于控制将车停正（option）
  if (parallel_parking_conf_.parallel_forward_extend_distance_keep() > 0.0) {
    double extend_length = parallel_parking_conf_.parallel_forward_extend_distance_keep();
    ExtendPath(stage1_f_a0_, extend_length, parallel_parking_conf_.parallel_path_interval(),
               legionclaw::common::GearPosition::GEAR_DRIVE, forward_path2);
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  return Status::Ok();
}

Status ParallelParkingPathGenerator::ComputeBackwardPathInStage1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  if (parallel_parking_conf_.parallel_parking_order() != 3 &&
      parallel_parking_conf_.parallel_parking_order() != 5) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "spiral order error");
  }
  local_vehicle_current_pose_ = cur_pose;
  UpdateStage1BackwardControlPoints(parking_position_);
  // y=ax+b
  double R = vehicle_param_.min_turning_radius();
  double a = (S_ * (H_ - 2 * R) + sqrt(4 * R * R * (S_ * S_ + H_ * H_) - 16 * R * R * R * H_)) /
             (S_ * S_ - 4 * R * R);
  double theta = atan(a);
  // if (parking_position_ < 0) theta = M_PI - theta;

  // A1----->A2（圆弧）
  std::vector<legionclaw::interface::PathPoint> backward_path2;
  double ox1 = parking_position_ * S_;
  double oy1 = H_ - R;
  // TODO 右边为(M_PI_2，M_PI_2+theta) 左边为(M_PI_2，M_PI_2-theta)
  CalculateCirclePath(
      ox1, oy1,
      M_PI_2 +
          parking_position_ * parallel_parking_conf_.parallel_parking_start_rad_offset() * M_PI,
      M_PI_2 + parking_position_ *
                   (theta - parallel_parking_conf_.parallel_parking_end_rad_offset() * M_PI),
      parallel_parking_conf_.parallel_path_interval(), R, backward_path2);

  // A3----->A4（圆弧）
  std::vector<legionclaw::interface::PathPoint> backward_path4;
  double ox2 = 0.0;
  double oy2 = R;
  CalculateCirclePath(ox2, oy2, -M_PI_2 + parking_position_ * theta, -M_PI_2,
                      parallel_parking_conf_.parallel_path_interval(), R, backward_path4);

  // A0----->A1（曲线拟合）
  int num = (int)(math::Norm(stage1_b_a0_.x() - backward_path2.front().x(),
                             stage1_b_a0_.y() - backward_path2.front().y()) /
                  parallel_parking_conf_.parallel_path_interval());
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  if (parallel_parking_conf_.parallel_parking_order() == 3) {
    spiral::CubicSpiralCurve csc_01(stage1_b_a0_, backward_path2.front());
    csc_01.SetSpiralConfig(spiral_curve_config_);
    if (csc_01.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      // return Status(Status::ErrorCode::PLANNING_ERROR,
      //               "CalculatePath() failed.");
    }
    if (csc_01.GetPathVec(num, &backward_path1) == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
  } else {
    spiral::QuinticSpiralCurve csc_01(stage1_b_a0_, backward_path2.front());
    csc_01.SetSpiralConfig(spiral_curve_config_);
    if (csc_01.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_01.GetPathVec(num, &backward_path1) == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
  }

  // A2----->A3（曲线拟合）
  num = (int)(math::Norm(backward_path2.back().x() - backward_path4.front().x(),
                         backward_path2.back().y() - backward_path4.front().y()) /
              parallel_parking_conf_.parallel_path_interval());
  std::vector<legionclaw::interface::PathPoint> backward_path3;
  if (num > 1) {
    if (parallel_parking_conf_.parallel_parking_order() == 3) {
      spiral::CubicSpiralCurve csc_23(backward_path4.front(), backward_path2.back());
      csc_23.SetSpiralConfig(spiral_curve_config_);
      if (csc_23.CalculatePath() == false) {
        AERROR << "CalculatePath() failed.";
        return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
      }
      if (csc_23.GetPathVec(num, &backward_path3) == false) {
        AERROR << "CalculatePath() failed.";
        return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
      }
    } else {
      spiral::QuinticSpiralCurve csc_23(backward_path4.front(), backward_path2.back());
      csc_23.SetSpiralConfig(spiral_curve_config_);
      if (csc_23.CalculatePath() == false) {
        AERROR << "CalculatePath() failed.";
        return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
      }
      if (csc_23.GetPathVec(num, &backward_path3) == false) {
        AERROR << "CalculatePath() failed.";
        return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
      }
    }
    std::reverse(backward_path3.begin(), backward_path3.end());
  }

  // 拼接
  backward_path_.clear();
  if (backward_path1.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path1.begin(), backward_path1.end());
  if (backward_path2.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path2.begin() + 1, backward_path2.end());
  if (backward_path3.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path3.begin() + 1, backward_path3.end());
  if (backward_path4.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path4.begin() + 1, backward_path4.end());

  return Status::Ok();
}

bool ParallelParkingPathGenerator::ConstructFreeSpace()
{
  // 设置可行驶区域
  double back_distance = 0.5 * parking_spot_.length() -
                         parallel_parking_conf_.parallel_lat_safe_distance() -
                         vehicle_param_.back_edge_to_center();  // 车库中心离泊车坐标系原点的距离

  math::Polygon2d polygon_left(back_distance - parking_spot_.length(), 0, M_PI_2,
                               parking_spot_.length(), parking_spot_.width());
  math::Polygon2d polygon_right(back_distance + parking_spot_.width(), 0, M_PI_2,
                                parking_spot_.length(), parking_spot_.width());
  math::Polygon2d polygon_forward(
      0.0, 0.5 * parking_spot_.width() + parallel_parking_conf_.parallel_park_lane_width() + 1,
      M_PI_2, 40.0, 2);

  road_polygon_.clear();
  road_polygon_.push_back(polygon_left);
  road_polygon_.push_back(polygon_right);
  road_polygon_.push_back(polygon_forward);

  return true;
}

// TODO
Status ParallelParkingPathGenerator::ComputeForwardPathInStage2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;

  UpdateStage2ForwardControlPoints(parking_position_);

  // 判断标准情况下，泊车空间是否足够
  if (IsEnoughFreeSpace(stage2_f_a1_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "there is no enough free space to parking.");
  }

  // 判断标准情况下，泊车空间是否足够
  if (IsEnoughFreeSpace(stage2_f_a1_) == false) {
    AERROR << "there is no enough free space to parking.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "there is no enough free space to parking.");
  }

  forward_path_.clear();
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(cur_pose, stage2_f_a1_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = cur_pose.x() - stage2_f_a1_.x();
  double dy = cur_pose.y() - stage2_f_a1_.y();
  int num = (int)(math::Norm(dx, dy) / parallel_parking_conf_.parallel_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }
  forward_path_.insert(forward_path_.end(), forward_path1.begin(), forward_path1.end());
  // 往后延长一段forward_path2，便于控制将车停正（option）
  if (parallel_parking_conf_.parallel_forward_extend_distance_keep() > 0.0) {
    double extend_length = parallel_parking_conf_.parallel_forward_extend_distance_keep();
    ExtendPath(stage2_f_a1_, extend_length, parallel_parking_conf_.parallel_path_interval(),
               legionclaw::common::GearPosition::GEAR_DRIVE, forward_path2);
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  return Status::Ok();
}

// TODO
Status ParallelParkingPathGenerator::ComputeBackwardPathInStage2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  if (parallel_parking_conf_.parallel_parking_order() != 3 &&
      parallel_parking_conf_.parallel_parking_order() != 5) {
    AERROR << "spiral order error";
    return Status(Status::ErrorCode::PLANNING_ERROR, "spiral order error");
  }
  local_vehicle_current_pose_ = cur_pose;
  UpdateStage2BackwardControlPoints(parking_position_);

  // A0----->A1（曲线拟合）
  std::vector<legionclaw::interface::PathPoint> backward_path1;

  double dx = cur_pose.x() - stage2_b_a1_.x();
  double dy = cur_pose.y() - stage2_b_a1_.y();
  int num = (int)(math::Norm(dx, dy) / parallel_parking_conf_.parallel_path_interval());
  if (parallel_parking_conf_.parallel_parking_order() == 3) {
    spiral::CubicSpiralCurve csc_23(stage2_b_a1_, cur_pose);
    csc_23.SetSpiralConfig(spiral_curve_config_);
    if (csc_23.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_23.GetPathVec(num, &backward_path1) == false) {
      AERROR << "GetPathVec() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
    }
  } else {
    spiral::QuinticSpiralCurve csc_01(stage2_b_a1_, cur_pose);
    csc_01.SetSpiralConfig(spiral_curve_config_);
    if (csc_01.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_01.GetPathVec(num, &backward_path1) == false) {
      AERROR << "GetPathVec() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
    }
  }

  std::reverse(backward_path1.begin(), backward_path1.end());

  // 拼接
  backward_path_.clear();
  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin(), backward_path1.end());
  }

  return Status::Ok();
}

void ParallelParkingPathGenerator::UpdateStage1ForwardControlPoints(int parking_position)
{
  stage1_f_a0_.set_x(S_ + parallel_parking_conf_.parallel_min_forward_extend_distance());
  stage1_f_a0_.set_y(H_);
  stage1_f_a0_.set_theta(0.0);
  stage1_f_a0_.set_kappa(0.0);

  stage1_f_a1_ = stage1_f_a0_;
  stage1_f_a1_.set_x(stage1_f_a0_.x() +
                     parallel_parking_conf_.parallel_forward_extend_distance_keep());
  if (parking_position == -1) {
    stage1_f_a0_.set_x(-stage1_f_a0_.x());
    stage1_f_a0_.set_theta(M_PI);
    stage1_f_a1_.set_x(-stage1_f_a1_.x());
    stage1_f_a1_.set_theta(M_PI);
  }
}

void ParallelParkingPathGenerator::UpdateStage1BackwardControlPoints(int parking_position)
{
  stage1_b_a0_ = local_vehicle_current_pose_;

  // A4：最终停车点（车辆后轴中心点）
  stage1_b_a4_.set_x(0.0);
  stage1_b_a4_.set_y(0.0);
  stage1_b_a4_.set_theta(0.0);
  stage1_b_a4_.set_kappa(parking_position / vehicle_param_.min_turning_radius());  // 左+右-
  stage1_b_a4_.set_dkappa(0.0);
  stage1_b_a4_.set_ddkappa(0.0);
  if (parking_position == -1) {
    stage1_b_a4_.set_theta(M_PI);
  }
}

void ParallelParkingPathGenerator::UpdateStage2ForwardControlPoints(int parking_position)
{
  stage2_f_a0_ = local_vehicle_current_pose_;

  double x = 0.5 * parking_spot_.length() - parallel_parking_conf_.parallel_lon_safe_distance();
  double l = 0.5 * vehicle_param_.length() - vehicle_param_.back_edge_to_center();
  x = x - 0.5 * vehicle_param_.length() - l;
  stage2_f_a1_.set_x(x);
  stage2_f_a1_.set_y(0.0);
  stage2_f_a1_.set_theta(0.0);
  stage2_f_a1_.set_kappa(0.0);  // 左+右-
  stage2_f_a1_.set_dkappa(0.0);
  stage2_f_a1_.set_ddkappa(0.0);
}

void ParallelParkingPathGenerator::UpdateStage2BackwardControlPoints(int parking_position)
{
  stage2_b_a0_ = local_vehicle_current_pose_;

  double x = 0.5 * parking_spot_.length() - parallel_parking_conf_.parallel_lon_safe_distance() -
             vehicle_param_.back_edge_to_center();
  ;
  x *= (-1.0);
  stage2_b_a1_.set_x(x);
  stage2_b_a1_.set_y(0.0);
  stage2_b_a1_.set_theta(0.0);
  stage2_b_a1_.set_kappa(0.0);  // 左+右-
  stage2_b_a1_.set_dkappa(0.0);
  stage2_b_a1_.set_ddkappa(0.0);
}
}  // namespace planning
}  // namespace legionclaw