
#include "modules/planning/src/planner/parking_planner/geometry/vertical_parking_path_generator.h"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>

#include "modules/common/math/math_tools.h"
#include "modules/common/logging/logging.h"

using namespace std;
namespace legionclaw {
namespace planning {
/***********************************************************
* @class VerticalParkingPathGenerator
* @brief 生成泊车轨迹信息

class VerticalParkingPathGenerator
***********************************************************/

/**
 * @brief 构造函数
 */
VerticalParkingPathGenerator::VerticalParkingPathGenerator(const PlanningConf *planning_conf)
{
  vehicle_param_ = planning_conf->vehicle_param();
  vertical_parking_conf_ = planning_conf->parking_conf().vertical_parking_conf();
  UpdateSpiralCurveConfig(planning_conf->parking_conf().spiral_curve_conf());
  // 构建可行驶区域
  ConstructFreeSpace();
  forward_distance = vertical_parking_conf_.forward_distance();
  upward_distance = vertical_parking_conf_.upward_distance();
  R = vertical_parking_conf_.caculate_turning_radius();
  d1 = vertical_parking_conf_.d1();
  d = vertical_parking_conf_.road_width();
  R1 = vertical_parking_conf_.r1();
  theta_4 = vertical_parking_conf_.theta_4();
  origin_to_up_d = vehicle_param_.front_edge_to_center();
  out_d = vertical_parking_conf_.out_d();
  backword_distance = vertical_parking_conf_.backword_distance();
}

void VerticalParkingPathGenerator::SetParkingSpot(const legionclaw::interface::ParkingInfo &parking_spot)
{
  parking_spot_ = parking_spot;
  ComputeInDistance(vehicle_param_.min_turning_radius(),
                    vertical_parking_conf_.vertical_lat_safe_distance(), vehicle_param_.width(),
                    parking_spot_.width());
}

void VerticalParkingPathGenerator::ComputeInDistance(const double &turning_radius,
                                                     const double &safe_distance,
                                                     const double &vehicle_width,
                                                     const double &parking_width)
{
  in_distance_ = pow((turning_radius - 0.5 * vehicle_width - safe_distance), 2) -
                 pow((turning_radius - 0.5 * parking_width), 2);
  if (in_distance_ >= 0.0) {
    in_distance_ = sqrt(in_distance_);
  } else {
    in_distance_ = 0.0;
  }
}

Status VerticalParkingPathGenerator::ComputeForwardPathInStage1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  UpdateStage1ForwardControlPoints(parking_position_);

  // 判断标准情况下，泊车空间是否足够
  //  if (IsEnoughFreeSpace(stage1_f_a1_) == false) {
  //    return Status(Status::ErrorCode::PLANNING_ERROR,
  //                  "there is no enough free space to parking.");
  //  }

  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(cur_pose, stage1_f_a0_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = cur_pose.x() - stage1_f_a0_.x();
  double dy = cur_pose.y() - stage1_f_a0_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }
  forward_path_.insert(forward_path_.end(), forward_path1.begin(), forward_path1.end());
  // 往后延长一段forward_path2，便于控制将车停正（option）
  if (vertical_parking_conf_.vertical_forward_extend_distance_keep() > 0.0) {
    double extend_length = vertical_parking_conf_.vertical_forward_extend_distance_keep();
    ExtendPath(stage1_f_a0_, extend_length, vertical_parking_conf_.vertical_path_interval(),
               legionclaw::common::GearPosition::GEAR_DRIVE, forward_path2);
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputeBackwardPathInStage1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  if (vertical_parking_conf_.vertical_parking_order() != 3 &&
      vertical_parking_conf_.vertical_parking_order() != 5) {
    return Status(Status::ErrorCode::PLANNING_ERROR, "spiral order error");
  }

  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  UpdateStage1BackwardControlPoints(parking_position_);

  // 圆弧
  std::vector<legionclaw::interface::PathPoint> backward_path2;
  double ox = stage1_b_a1_.x();
  double oy = stage1_b_a2_.y();
  if (cur_pose.theta() > 0.0) {
    angle_offset_ = cur_pose.theta();
  }

  CalculateCirclePath(
      ox, oy,
      M_PI_2 +
          parking_position_ * vertical_parking_conf_.vertical_parking_start_rad_offset() * M_PI,
      M_PI_2 + parking_position_ * M_PI_2 -
          parking_position_ * vertical_parking_conf_.vertical_parking_end_rad_offset() * M_PI,
      vertical_parking_conf_.vertical_path_interval(), vehicle_param_.min_turning_radius(),
      backward_path2);

  // // A0----->A1（曲线拟合）
  int num = (int)(math::Norm(stage1_b_a0_.x() - backward_path2.front().x(),
                             stage1_b_a0_.y() - backward_path2.front().y()) /
                  vertical_parking_conf_.vertical_path_interval());
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  if (vertical_parking_conf_.vertical_parking_order() == 3) {
    spiral::CubicSpiralCurve csc_01(backward_path2.front(), stage1_b_a0_);
    csc_01.SetSpiralConfig(spiral_curve_config_);
    if (csc_01.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_01.GetPathVec(num, &backward_path1) == false) {
      AERROR << "GetPathVec() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
    }
  } else {
    spiral::QuinticSpiralCurve csc_01(backward_path2.front(), stage1_b_a0_);
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

  // // A2----->A3（曲线拟合）
  num = (int)(math::Norm(backward_path2.back().x() - stage1_b_a3_.x(),
                         backward_path2.back().y() - stage1_b_a3_.y()) /
              vertical_parking_conf_.vertical_path_interval());
  std::vector<legionclaw::interface::PathPoint> backward_path3;
  if (vertical_parking_conf_.vertical_parking_order() == 3) {
    spiral::CubicSpiralCurve csc_23(stage1_b_a3_, backward_path2.back());
    csc_23.SetSpiralConfig(spiral_curve_config_);
    if (csc_23.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_23.GetPathVec(num, &backward_path3) == false) {
      AERROR << "GetPathVec() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
    }
  } else {
    spiral::QuinticSpiralCurve csc_23(stage1_b_a3_, backward_path2.back());
    csc_23.SetSpiralConfig(spiral_curve_config_);
    if (csc_23.CalculatePath() == false) {
      AERROR << "CalculatePath() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
    }
    if (csc_23.GetPathVec(num, &backward_path3) == false) {
      AERROR << "GetPathVec() failed.";
      return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
    }
  }

  std::reverse(backward_path3.begin(), backward_path3.end());

  // 拼接

  if (backward_path1.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path1.begin(), backward_path1.end());
  if (backward_path2.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path2.begin() + 1, backward_path2.end());
  if (backward_path3.size() > 0)
    backward_path_.insert(backward_path_.end(), backward_path3.begin() + 1, backward_path3.end());

  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

bool VerticalParkingPathGenerator::ConstructFreeSpace()
{
  // 设置可行驶区域
  double back_distance = 0.5 * parking_spot_.length() -
                         vertical_parking_conf_.vertical_lon_safe_distance() -
                         vehicle_param_.back_edge_to_center();  // 车库中心离泊车坐标系原点的距离

  math::Polygon2d polygon_left(-parking_spot_.width(), back_distance, M_PI_2, parking_spot_.width(),
                               parking_spot_.length());
  math::Polygon2d polygon_right(parking_spot_.width(), back_distance, M_PI_2, parking_spot_.width(),
                                parking_spot_.length());
  math::Polygon2d polygon_forward(0.0,
                                  back_distance + 0.5 * parking_spot_.length() +
                                      vertical_parking_conf_.vertical_park_lane_width() + 1,
                                  M_PI_2, 40.0, 2);

  road_polygon_.clear();
  road_polygon_.push_back(polygon_left);
  road_polygon_.push_back(polygon_right);
  road_polygon_.push_back(polygon_forward);

  return true;
}
int VerticalParkingPathGenerator::get_parking_position() { return parking_position_; }

Status VerticalParkingPathGenerator::ComputePathInStage1Move0_1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;
  d4 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center() + upward_distance);
  if (fabs(R - d4) > 1) {
    AERROR << "Calculate theta_2 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_2  failed.");
  }
  theta_2 = acos((R - d4) / R);

  double t_theta1 = theta_2 / 2;
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center()) -
       (R - R * cos(t_theta1));
  // if ((d - d3) < d2) {
  //   AERROR << "CalculatePath() failed.";
  //   return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath()
  //   failed.");
  // }
  UpdateStage1Move0_1ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(stage1_m0_a1_, local_vehicle_current_pose_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m0_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m0_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &backward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  std::reverse(backward_path1.begin(), backward_path1.end());
  backward_path_.assign(backward_path1.begin(), backward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move0_2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  // d1 = 0.2;
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate t_theta1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate t_theta1  failed.");
  }
  double t_theta1 = acos((R - d1) / R);
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center());

  // R1=2*R;
  // move2结束 y方向 据原点距离
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate theta_1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_1  failed.");
  }
  theta_1 = acos((R - d1) / R);
  double h = R * sin(M_PI_2 - theta_1) - (R1 - R1 * cos(theta_1)) -
             (local_vehicle_current_pose_.y() -
              (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
               vehicle_param_.back_edge_to_center() + upward_distance));
  if (fabs(h / R) > 1) {
    AERROR << "Calculate theta_t failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_t failed.");
  }
  theta_t = asin(h / R);

  l = R1 * sin(theta_1) - (R * cos(theta_t) - R * cos(M_PI_2 - theta_1));
  UpdateStage1Move0_2ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(stage1_m0_a1_, local_vehicle_current_pose_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m0_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m0_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &backward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  std::reverse(backward_path1.begin(), backward_path1.end());
  backward_path_.assign(backward_path1.begin(), backward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move0_4(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  d1 = 1.286755563;
  R1 = 5.5;
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate theta_1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_1  failed.");
  }
  double t_theta1 = acos((R - d1) / R);
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center());

  // R1=2*R;
  // move2结束 y方向 据原点距离
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate theta_1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_1  failed.");
  }
  theta_1 = acos((R - d1) / R);
  double h = R * sin(M_PI_2 - theta_1) - (R1 - R1 * cos(theta_1)) -
             (local_vehicle_current_pose_.y() -
              (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
               vehicle_param_.back_edge_to_center() + upward_distance));
  if (fabs(h / R) > 1) {
    AERROR << "Calculate theta_t failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_t failed.");
  }
  theta_t = asin(h / R);

  l = R1 * sin(theta_1) - (R * cos(theta_t) - R * cos(M_PI_2 - theta_1));
  UpdateStage1Move0_4ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(stage1_m0_a1_, local_vehicle_current_pose_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m0_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m0_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &backward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  std::reverse(backward_path1.begin(), backward_path1.end());
  backward_path_.assign(backward_path1.begin(), backward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move1_1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  d4 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center() + upward_distance);
  if (fabs((R - d4) / R) > 1) {
    AERROR << "Calculate theta_2 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_2  failed.");
  }
  theta_2 = acos((R - d4) / R);

  double t_theta1 = theta_2 / 2;
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);

  //(R-R*cos(t_theta1) 方法2 转动起始点y方向距离
  // 理论需要空间
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center()) -
       (R - R * cos(t_theta1));
  // move2 圆弧 y方向距离
  std::cout << "d3+(R-R*cos(t_theta1)) :" << d3 + (R - R * cos(t_theta1)) << "\n";
  // if ((d - d3) < d2) {
  //   AERROR << "CalculatePath() failed.";
  //   return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath()
  //   failed.");
  // }

  UpdateStage1Move1_1ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(local_vehicle_current_pose_, stage1_m1_a1_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m1_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m1_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  forward_path_.assign(forward_path1.begin(), forward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move1_2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();

  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  // d1 = 0.2;
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate t_theta1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate t_theta1  failed.");
  }
  double t_theta1 = acos((R - d1) / R);
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center());

  // R1=2*R;
  // move2结束 y方向 据原点距离
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate t_theta1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate t_theta1  failed.");
  }
  theta_1 = acos((R - d1) / R);
  double h = R * sin(M_PI_2 - theta_1) - (R1 - R1 * cos(theta_1)) -
             (local_vehicle_current_pose_.y() -
              (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
               vehicle_param_.back_edge_to_center() + upward_distance));
  if (fabs(h / R) > 1) {
    AERROR << "Calculate theta_t failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_t failed.");
  }
  theta_t = asin(h / R);

  std::cout << "d1: " << (R1 - R1 * cos(theta_1)) << endl;
  std::cout << "d2: " << d2 << endl;
  std::cout << "d3: " << d3 << endl;
  std::cout << "theta_1: " << legionclaw::common::math::R2D(theta_1) << endl;
  std::cout << "theta_t1: " << legionclaw::common::math::R2D(theta_t) << endl;

  l = R1 * sin(theta_1) - (R * cos(theta_t) - R * cos(M_PI_2 - theta_1));
  UpdateStage1Move1_2ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(local_vehicle_current_pose_, stage1_m1_a1_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m1_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m1_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  double ox = stage1_m1_a1_.x();
  double oy = local_vehicle_current_pose_.y() + R1;
  double start_theta = -M_PI_2;
  double end_theta = -M_PI_2 + parking_position_ * theta_1;

  CalculateCirclePath(ox, oy, end_theta, start_theta,
                      vertical_parking_conf_.vertical_path_interval(), R1, forward_path2);

  std::reverse(forward_path2.begin(), forward_path2.end());
  if (forward_path1.size() > 0) {
    forward_path_.assign(forward_path1.begin(), forward_path1.end());
    if (forward_path2.size() > 0)
      forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move1_4(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  if (fabs(local_vehicle_current_pose_.theta()) > M_PI_2)
    parking_position_ = -1;
  else
    parking_position_ = 1;

  d1 = 1.286755563;
  R1 = 5.5;
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate t_theta1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate t_theta1  failed.");
  }
  double t_theta1 = acos((R - d1) / R);
  double t_theta2 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  double t_theta3 = M_PI_2 - t_theta1 - t_theta2;
  L = sqrt((vehicle_param_.width() / 2) * (vehicle_param_.width() / 2) +
           vehicle_param_.front_edge_to_center() * vehicle_param_.front_edge_to_center()) +
      0.2;

  d2 = L * cos(t_theta3);
  d3 = local_vehicle_current_pose_.y() -
       (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
        vehicle_param_.back_edge_to_center());

  // R1=2*R;
  // move2结束 y方向 据原点距离
  if (fabs((R - d1) / R) > 1) {
    AERROR << "Calculate t_theta1 failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate t_theta1  failed.");
  }
  theta_1 = acos((R - d1) / R);
  double h = R * sin(M_PI_2 - theta_1) - (R1 - R1 * cos(theta_1)) -
             (local_vehicle_current_pose_.y() -
              (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
               vehicle_param_.back_edge_to_center() + upward_distance));
  if (fabs(h / R) > 1) {
    AERROR << "Calculate theta_t failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "Calculate theta_t failed.");
  }
  theta_t = asin(h / R);

  std::cout << "d1: " << (R1 - R1 * cos(theta_1)) << endl;
  std::cout << "d2: " << d2 << endl;
  std::cout << "d3: " << d3 << endl;
  std::cout << "theta_1: " << legionclaw::common::math::R2D(theta_1) << endl;
  std::cout << "theta_t1: " << legionclaw::common::math::R2D(theta_t) << endl;

  l = (R1 * sin(theta_1) - (R * cos(theta_t) - R * cos(M_PI_2 - theta_1)));
  UpdateStage1Move1_4ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(local_vehicle_current_pose_, stage1_m1_a1_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = local_vehicle_current_pose_.x() - stage1_m1_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m1_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  double ox = stage1_m1_a1_.x();
  double oy = local_vehicle_current_pose_.y() + R1;
  double start_theta = -M_PI_2;
  double end_theta = -M_PI_2 + parking_position_ * theta_1;

  CalculateCirclePath(ox, oy, end_theta, start_theta,
                      vertical_parking_conf_.vertical_path_interval(), R1, forward_path2);

  std::reverse(forward_path2.begin(), forward_path2.end());
  if (forward_path1.size() > 0) {
    forward_path_.assign(forward_path1.begin(), forward_path1.end());
    if (forward_path2.size() > 0)
      forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move2_1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  UpdateStage1Move2_1ControlPoints(parking_position_);
  std::cout << "theta_2: " << legionclaw::common::math::R2D(theta_2) << endl;
  // 圆弧

  std::vector<legionclaw::interface::PathPoint> backward_path1;
  double ox = local_vehicle_current_pose_.x();
  double oy = local_vehicle_current_pose_.y() - R;

  theta_2 = legionclaw::common::math::NormalizeAngle(theta_2);
  double start_theta = M_PI_2;
  double end_theta = start_theta + parking_position_ * theta_2;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, backward_path1);

  backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move2_2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  UpdateStage1Move2_2ControlPoints(parking_position_);
  // 圆弧

  std::vector<legionclaw::interface::PathPoint> backward_path1;
  // double ox = parking_position_ * (l_r1-fabs(stage1_m1_a1_.x()));
  double ox = local_vehicle_current_pose_.x() + parking_position_ * R * cos(theta_2 + theta_t);
  double oy = local_vehicle_current_pose_.y() - R * sin(theta_2 + theta_t);

  theta_2 = legionclaw::common::math::NormalizeAngle(theta_2);
  std::cout << "theta_2: " << legionclaw::common::math::R2D(theta_2) << endl;
  double start_theta =
      M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * (theta_2 + theta_t);
  double end_theta = start_theta + parking_position_ * theta_2;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, backward_path1);

  backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move2_4(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  UpdateStage1Move2_4ControlPoints(parking_position_);
  // 圆弧

  std::vector<legionclaw::interface::PathPoint> backward_path1, backward_path2, backward_path3;
  // double ox = parking_position_ * (l_r1-fabs(stage1_m1_a1_.x()));
  double ox = local_vehicle_current_pose_.x() + parking_position_ * R * cos(theta_2 + theta_t);
  double oy = local_vehicle_current_pose_.y() - R * sin(theta_2 + theta_t);

  theta_2 = legionclaw::common::math::NormalizeAngle(theta_2);
  std::cout << "theta_2: " << legionclaw::common::math::R2D(theta_2) << endl;
  double start_theta =
      M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * (theta_2 + theta_t);
  double end_theta = start_theta + parking_position_ * (6 * (theta_2 + theta_t)) / 12;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, backward_path1);
  spiral::CubicSpiralCurve csc(stage1_m2_a2_, backward_path1.back());
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = backward_path1.back().x() - stage1_m2_a2_.x();
  double dy = backward_path1.back().y() - stage1_m2_a2_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &backward_path2) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  std::reverse(backward_path2.begin(), backward_path2.end());

  double extend_length = backward_path2.back().y();

  ExtendPath(stage1_m2_a2_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_REVERSE, backward_path3);

  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  }
  if (backward_path2.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path2.begin() + 1, backward_path2.end());
  }
  if (backward_path3.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path3.begin() + 1, backward_path3.end());
  }

  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move3(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  upward_distance = local_vehicle_current_pose_.y() -
                    (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
                     vehicle_param_.back_edge_to_center());
  std::cout << "upward_distance: " << upward_distance << endl;
  UpdateStage1Move3ControlPoints(parking_position_);

  std::vector<legionclaw::interface::PathPoint> forward_path1;
  double dx = local_vehicle_current_pose_.x() - stage1_m3_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m3_a1_.y();
  double extend_length = math::Norm(dx, dy);

  ExtendPath(local_vehicle_current_pose_, extend_length,
             vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_DRIVE, forward_path1);

  forward_path_.assign(forward_path1.begin(), forward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move4_3(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  theta_4_1 =
      (1 * (parking_position_ * M_PI_2 - parking_position_ * local_vehicle_current_pose_.theta())) /
      4;
  double theta_3_1 = parking_position_ * M_PI_2 -
                     parking_position_ * local_vehicle_current_pose_.theta() - theta_4_1;
  std::cout << "theta_3_1: " << theta_3_1 << endl;
  UpdateStage1Move4_3ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;
  double t_dx = local_vehicle_current_pose_.x() - 0;
  double t_dy = fabs(local_vehicle_current_pose_.x()) * cos(theta_3_1 + theta_4_1);
  double temp_l = sqrt(t_dx * t_dx + t_dy * t_dy);
  double temp_R = temp_l / tan((theta_4_1 + theta_3_1) / 2);
  if (temp_R < vehicle_param_.min_turning_radius()) {
    temp_R = vehicle_param_.min_turning_radius();
  }

  double ox =
      local_vehicle_current_pose_.x() + parking_position_ * temp_R * cos(theta_3_1 + theta_4_1);
  double oy = local_vehicle_current_pose_.y() - temp_R * sin(theta_3_1 + theta_4_1);

  double start_theta =
      M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * (theta_3_1 + theta_4_1);
  double end_theta = M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * theta_4_1;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), temp_R, backward_path1);

  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  }
  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move3_3(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  upward_distance = local_vehicle_current_pose_.y() -
                    (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
                     vehicle_param_.back_edge_to_center());
  std::cout << "upward_distance: " << upward_distance << endl;
  UpdateStage1Move3_3ControlPoints(parking_position_);

  std::vector<legionclaw::interface::PathPoint> forward_path1;
  double dx = local_vehicle_current_pose_.x() - stage1_m3_3_a1_.x();
  double dy = local_vehicle_current_pose_.y() - stage1_m3_3_a1_.y();
  double extend_length = math::Norm(dx, dy);

  ExtendPath(local_vehicle_current_pose_, extend_length,
             vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_DRIVE, forward_path1);

  forward_path_.assign(forward_path1.begin(), forward_path1.end());

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move3_5(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  UpdateStage1Move3_5ControlPoints(parking_position_);
  double t_theta = legionclaw::common::math::NormalizeAngle(local_vehicle_current_pose_.theta());
  t_theta = M_PI_2 - parking_position_ * M_PI_2 + parking_position_ * t_theta;
  // 圆弧
  std::vector<legionclaw::interface::PathPoint> forward_path1;
  double ox = local_vehicle_current_pose_.x() - parking_position_ * R3 * cos(theta_3 + theta_t1);
  double oy = local_vehicle_current_pose_.y() + R3 * sin(theta_3 + theta_t1);

  double start_theta = -M_PI_2 + parking_position_ * (t_theta);
  double end_theta = start_theta + parking_position_ * theta_t1;

  CalculateCirclePath(ox, oy, end_theta, start_theta,
                      vertical_parking_conf_.vertical_path_interval(), R3, forward_path1);
  std::reverse(forward_path1.begin(), forward_path1.end());
  forward_path_.insert(forward_path_.end(), forward_path1.begin() + 1, forward_path1.end());
  steer_value_ = -parking_position_ * atan(vehicle_param_.wheelbase() / R3);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage1Move4(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  double temptheta = legionclaw::common::math::NormalizeAngle(local_vehicle_current_pose_.theta());
  theta_4 = (1 * (parking_position_ * M_PI_2 - parking_position_ * temptheta)) / 2;
  theta_3 = parking_position_ * M_PI_2 - parking_position_ * temptheta - theta_4;
  UpdateStage1Move4ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1, backward_path2, backward_path3;
  double t_dx = local_vehicle_current_pose_.x() - 0;
  double t_dy = fabs(local_vehicle_current_pose_.x()) * cos(temptheta);
  double temp_l = sqrt(t_dx * t_dx + t_dy * t_dy);
  double temp_R = temp_l / tan((theta_4 + theta_3) / 2);
  std::cout << "R: " << temp_R << endl;
  std::cout << "theta3: " << legionclaw::common::math::R2D(theta_3) << endl;
  std::cout << "theta4: " << legionclaw::common::math::R2D(theta_4) << endl;
  if (temp_R < vehicle_param_.min_turning_radius()) {
    stage1_m4_a2_.set_y(
        max((stage1_m4_a2_.y() - vehicle_param_.min_turning_radius() + temp_R), 0.0));
    temp_R = vehicle_param_.min_turning_radius();
  }
  double ox = local_vehicle_current_pose_.x() + parking_position_ * temp_R * cos(theta_3 + theta_4);
  double oy = local_vehicle_current_pose_.y() - temp_R * sin(theta_3 + theta_4);

  double start_theta =
      M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * (theta_3 + theta_4);
  double end_theta = M_PI_2 + parking_position_ * M_PI_2 - parking_position_ * theta_4;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), temp_R, backward_path1);
  legionclaw::interface::PathPoint t_stage1_m4_a2;

  if (vertical_parking_conf_.scheme() == 5 || vertical_parking_conf_.scheme() == 1) {
    t_stage1_m4_a2 = local_vehicle_current_pose_;
  } else {
    t_stage1_m4_a2 = backward_path1.back();
  }
  spiral::CubicSpiralCurve csc(stage1_m4_a2_, t_stage1_m4_a2);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = backward_path1.back().x() - stage1_m4_a2_.x();
  double dy = backward_path1.back().y() - stage1_m4_a2_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &backward_path2) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }

  std::reverse(backward_path2.begin(), backward_path2.end());

  double extend_length = max(backward_path2.back().y(), 0.0);

  ExtendPath(stage1_m4_a2_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_REVERSE, backward_path3);

  // backward_path_.assign(backward_path1.begin(), backward_path1.end());

  if (backward_path1.size() > 0 && vertical_parking_conf_.scheme() != 5 &&
      vertical_parking_conf_.scheme() != 1) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  }
  if (backward_path2.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path2.begin() + 1, backward_path2.end());
  }
  if (backward_path3.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path3.begin() + 1, backward_path3.end());
  }

  if (vertical_parking_conf_.scheme() == 5 || vertical_parking_conf_.scheme() == 1) {
    steer_value_ = 0;
  } else {
    steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  }

  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputeForwardPathInStage2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  UpdateStage2ForwardControlPoints(parking_position_);

  // 判断标准情况下，泊车空间是否足够
  //  if (IsEnoughFreeSpace(stage2_f_a1_) == false) {
  //    AERROR << "there is no enough free space to parking.";
  //    return Status(Status::ErrorCode::PLANNING_ERROR,
  //                  "there is no enough free space to parking.");
  //  }

  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;
  // 使用3次螺旋曲线插值生成最终曲线
  spiral::CubicSpiralCurve csc(stage2_f_a0_, stage2_f_a1_);
  csc.SetSpiralConfig(spiral_curve_config_);
  if (csc.CalculatePath() == false) {
    AERROR << "CalculatePath() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "CalculatePath() failed.");
  }
  double dx = stage2_f_a0_.x() - stage2_f_a1_.x();
  double dy = stage2_f_a0_.y() - stage2_f_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (csc.GetPathVec(num, &forward_path1) == false) {
    AERROR << "GetPathVec() failed.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "GetPathVec() failed.");
  }
  forward_path_.insert(forward_path_.end(), forward_path1.begin(), forward_path1.end());
  // 往后延长一段forward_path2，便于控制将车停正（option）
  if (vertical_parking_conf_.vertical_forward_extend_distance_keep() > 0.0) {
    double extend_length = vertical_parking_conf_.vertical_forward_extend_distance_keep();
    ExtendPath(stage2_f_a1_, extend_length, vertical_parking_conf_.vertical_path_interval(),
               legionclaw::common::GearPosition::GEAR_DRIVE, forward_path2);
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputeBackwardPathInStage2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  backward_path_.clear();
  if (vertical_parking_conf_.vertical_parking_order() != 3 &&
      vertical_parking_conf_.vertical_parking_order() != 5) {
    AERROR << "spiral order error";
    return Status(Status::ErrorCode::PLANNING_ERROR, "spiral order error");
  }

  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  UpdateStage2BackwardControlPoints(parking_position_);

  // A0----->A1（曲线拟合）
  std::vector<legionclaw::interface::PathPoint> backward_path1;

  double dx = cur_pose.x() - stage2_b_a1_.x();
  double dy = cur_pose.y() - stage2_b_a1_.y();
  int num = (int)(math::Norm(dx, dy) / vertical_parking_conf_.vertical_path_interval());
  if (vertical_parking_conf_.vertical_parking_order() == 3) {
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

  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin(), backward_path1.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

// 几何泊出（临时方案）
Status VerticalParkingPathGenerator::ComputePathInStage3Move1(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  forward_path_.clear();
  parking_position_ = 1;
  UpdateStage3Move1ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1;

  double extend_length = abs(stage3_m1_a1_.y());

  ExtendPath(stage3_m1_a0_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_DRIVE, forward_path1);

  if (forward_path1.size() > 0) {
    forward_path_.insert(forward_path_.end(), forward_path1.begin() + 1, forward_path1.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage3Move2(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  backward_path_.clear();
  parking_position_ = -1;
  UpdateStage3Move2ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;

  double extend_length = abs(stage3_m2_a1_.y());

  ExtendPath(stage3_m2_a0_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_REVERSE, backward_path1);

  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage3Move3(
    const legionclaw::interface::PathPoint &cur_pose, int director_pose)
{
  local_vehicle_current_pose_ = cur_pose;
  parking_position_ = director_pose;
  forward_path_.clear();

  UpdateStage3Move3ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;

  double ox = local_vehicle_current_pose_.x() + parking_position_ * R;
  double oy = local_vehicle_current_pose_.y() + stage3_m3_a1_.y();

  double start_theta = M_PI_2 + parking_position_ * M_PI_2;
  double end_theta = start_theta - parking_position_ * out_theta3;

  CalculateCirclePath(ox, oy, end_theta, start_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, forward_path2);
  std::reverse(forward_path2.begin(), forward_path2.end());
  double extend_length = stage3_m3_a1_.y();

  ExtendPath(stage3_m3_a0_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_DRIVE, forward_path1);

  if (forward_path1.size() > 0) {
    forward_path_.insert(forward_path_.end(), forward_path1.begin() + 1, forward_path1.end());
  }
  if (forward_path2.size() > 0) {
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }

  steer_value_ = 0;
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage3Move4(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;

  backward_path_.clear();
  t_theta =
      legionclaw::common::math::NormalizeAngle(M_PI_2 - parking_position_ * M_PI_2 +
                                          parking_position_ * local_vehicle_current_pose_.theta());
  UpdateStage3Move4ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> backward_path1;

  double ox = local_vehicle_current_pose_.x() - parking_position_ * R * sin(t_theta);
  double oy = local_vehicle_current_pose_.y() + R * cos(t_theta);

  double start_theta = 3 * M_PI_2 + parking_position_ * (t_theta);
  double end_theta = 3 * M_PI_2 + parking_position_ * out_theta1;

  CalculateCirclePath(ox, oy, start_theta, end_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, backward_path1);

  if (backward_path1.size() > 0) {
    backward_path_.insert(backward_path_.end(), backward_path1.begin() + 1, backward_path1.end());
  }

  steer_value_ = -parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

Status VerticalParkingPathGenerator::ComputePathInStage3Move5(
    const legionclaw::interface::PathPoint &cur_pose)
{
  local_vehicle_current_pose_ = cur_pose;

  forward_path_.clear();

  UpdateStage3Move5ControlPoints(parking_position_);
  std::vector<legionclaw::interface::PathPoint> forward_path1, forward_path2;

  double ox = local_vehicle_current_pose_.x() + parking_position_ * R * sin(out_theta1);
  double oy = local_vehicle_current_pose_.y() - R * cos(out_theta1);

  double start_theta = M_PI_2 + parking_position_ * out_theta1;
  double end_theta = M_PI_2;

  CalculateCirclePath(ox, oy, end_theta, start_theta,
                      vertical_parking_conf_.vertical_path_interval(), R, forward_path1);
  std::reverse(forward_path1.begin(), forward_path1.end());
  double extend_length = 1.0;
  stage3_m5_a1_.set_x(forward_path1.back().x());
  stage3_m5_a1_.set_y(forward_path1.back().y());
  ExtendPath(stage3_m5_a1_, extend_length, vertical_parking_conf_.vertical_path_interval(),
             legionclaw::common::GearPosition::GEAR_DRIVE, forward_path2);
  if (forward_path1.size() > 0) {
    forward_path_.insert(forward_path_.end(), forward_path1.begin() + 1, forward_path1.end());
  }
  if (forward_path2.size() > 0) {
    forward_path_.insert(forward_path_.end(), forward_path2.begin() + 1, forward_path2.end());
  }
  steer_value_ = parking_position_ * atan(vehicle_param_.wheelbase() / R);
  return Status::Ok();
}

// 如果目标车位太靠后，先进入move0调整，后迁一段距离再开始泊车。
void VerticalParkingPathGenerator::UpdateStage1Move0_1ControlPoints(int parking_position_)
{
  stage1_m0_a0_ = local_vehicle_current_pose_;
  stage1_m0_a1_ = stage1_m0_a0_;
  stage1_m0_a1_.set_x(parking_position_ * R * sin(theta_2));
  stage1_m0_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
}

void VerticalParkingPathGenerator::UpdateStage1Move0_2ControlPoints(int parking_position_)
{
  stage1_m0_a0_ = local_vehicle_current_pose_;
  stage1_m0_a1_ = stage1_m0_a0_;
  stage1_m0_a1_.set_x(-parking_position_ * (l + 1.0));
  stage1_m0_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
}

void VerticalParkingPathGenerator::UpdateStage1Move0_4ControlPoints(int parking_position_)
{
  stage1_m0_a0_ = local_vehicle_current_pose_;
  stage1_m0_a1_ = stage1_m0_a0_;
  stage1_m0_a1_.set_x(-parking_position_ * (l + 1.0));
  stage1_m0_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
}

void VerticalParkingPathGenerator::UpdateStage1Move1_1ControlPoints(int parking_position_)
{
  stage1_m1_a0_ = local_vehicle_current_pose_;
  stage1_m1_a1_ = stage1_m1_a0_;
  stage1_m1_a1_.set_x(parking_position_ * R * sin(theta_2));
  stage1_m1_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
}

void VerticalParkingPathGenerator::UpdateStage1Move1_2ControlPoints(int parking_position_)
{
  stage1_m1_a0_ = local_vehicle_current_pose_;
  stage1_m1_a1_ = stage1_m1_a0_;
  stage1_m1_a1_.set_x(-parking_position_ * l);
  // stage1_m1_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2 +
  //                         parking_position_ * M_PI / 72);
  stage1_m1_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
  stage1_m1_a2_ = stage1_m1_a1_;
  stage1_m1_a2_.set_x(stage1_m1_a1_.x() + parking_position_ * R1 * sin(theta_1));
  stage1_m1_a2_.set_y(stage1_m1_a1_.y() + d1);
  stage1_m1_a2_.set_theta(M_PI_2 - parking_position_ * M_PI_2 + parking_position_ * theta_1);
}

void VerticalParkingPathGenerator::UpdateStage1Move1_4ControlPoints(int parking_position_)
{
  stage1_m1_a0_ = local_vehicle_current_pose_;
  stage1_m1_a1_ = stage1_m1_a0_;
  stage1_m1_a1_.set_x(-parking_position_ * l);
  // stage1_m1_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2 +
  //                         parking_position_ * M_PI / 72);
  stage1_m1_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
  stage1_m1_a2_ = stage1_m1_a1_;
  stage1_m1_a2_.set_x(stage1_m1_a1_.x() + parking_position_ * R1 * sin(theta_1));
  stage1_m1_a2_.set_y(stage1_m1_a1_.y() + d1);
  stage1_m1_a2_.set_theta(M_PI_2 - parking_position_ * M_PI_2 + parking_position_ * theta_1);
}

void VerticalParkingPathGenerator::UpdateStage1Move2_1ControlPoints(int parking_position_)
{
  stage1_m2_a0_ = local_vehicle_current_pose_;
  // d4 = local_vehicle_current_pose_.y() -
  //      (parking_spot_.length() -
  //       vertical_parking_conf_.vertical_lon_safe_distance() -
  //       vehicle_param_.back_edge_to_center() + upward_distance);
  // theta_2 = acos((R - d4) / R);
  if (fabs(local_vehicle_current_pose_.x()) / R > 1) {
    AERROR << "Calculate theta_2 failed.";
  }
  theta_2 = asin(fabs(local_vehicle_current_pose_.x()) / R);
  stage1_m2_a1_.set_x(stage1_m2_a0_.x() - parking_position_ * R * sin(theta_2));
  stage1_m2_a1_.set_y(stage1_m2_a0_.y() - (R - R * cos(theta_2)));
  stage1_m2_a1_.set_theta(M_PI + parking_position_ * theta_2);
  stage1_m2_a1_.set_kappa(0.0);
  stage1_m2_a1_.set_dkappa(0.0);
  stage1_m2_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move2_2ControlPoints(int parking_position_)
{
  stage1_m2_a0_ = local_vehicle_current_pose_;
  double temp_theta_1 =
      parking_position_ * M_PI_2 - parking_position_ * local_vehicle_current_pose_.theta();
  if (fabs((fabs(local_vehicle_current_pose_.x()) + R * cos(temp_theta_1)) / R) > 1) {
    AERROR << "Calculate theta_t failed.";
  }
  theta_t = acos((fabs(local_vehicle_current_pose_.x()) + R * cos(temp_theta_1)) / R);
  theta_2 = temp_theta_1 - theta_t;
  std::cout << "theta_t2: " << legionclaw::common::math::R2D(theta_t) << endl;

  stage1_m2_a1_.set_x(0);
  stage1_m2_a1_.set_y(local_vehicle_current_pose_.y() - R * sin(theta_t));
  stage1_m2_a1_.set_theta(local_vehicle_current_pose_.theta() + parking_position_ * theta_2);
  stage1_m2_a1_.set_kappa(0.0);
  stage1_m2_a1_.set_dkappa(0.0);
  stage1_m2_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move2_4ControlPoints(int parking_position_)
{
  stage1_m2_a0_ = local_vehicle_current_pose_;
  double temp_theta_1 =
      parking_position_ * M_PI_2 - parking_position_ * local_vehicle_current_pose_.theta();
  // O_2 x方向与原点距离
  double t_l = ((R + R1) / R1) * (fabs(local_vehicle_current_pose_.x() - stage1_m1_a1_.x()));
  // 使用x重计算 theta_t 使move2 结束位置x=0
  double t_l1;
  if (local_vehicle_current_pose_.x() * stage1_m1_a1_.x() < 0) {
    t_l1 = -fabs(stage1_m1_a1_.x());
  } else {
    t_l1 = fabs(stage1_m1_a1_.x());
  }
  if (fabs((t_l + t_l1) / R) > 1) {
    AERROR << "Calculate theta_t failed.";
  }
  theta_t = acos((t_l + t_l1) / R);
  std::cout << "theta_t2: " << legionclaw::common::math::R2D(theta_t) << endl;
  theta_2 = temp_theta_1 - theta_t;

  stage1_m2_a1_.set_x(0);
  stage1_m2_a1_.set_y(local_vehicle_current_pose_.y() - R * sin(theta_t));
  stage1_m2_a1_.set_theta(local_vehicle_current_pose_.theta() + parking_position_ * theta_2);
  stage1_m2_a1_.set_kappa(0.0);
  stage1_m2_a1_.set_dkappa(0.0);
  stage1_m2_a1_.set_ddkappa(0.0);

  stage1_m2_a2_.set_x(0.0);
  stage1_m2_a2_.set_y(0.5);
  stage1_m2_a2_.set_theta(M_PI_2);
  stage1_m2_a2_.set_kappa(0.0);
  stage1_m2_a2_.set_dkappa(0.0);
  stage1_m2_a2_.set_ddkappa(0.0);

  stage1_m2_a3_.set_x(0.0);
  stage1_m2_a3_.set_y(0.0);
  stage1_m2_a3_.set_theta(M_PI_2);
  stage1_m2_a3_.set_kappa(0.0);
  stage1_m2_a3_.set_dkappa(0.0);
  stage1_m2_a3_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move3ControlPoints(int parking_position_)
{
  double temp_theta_2 =
      parking_position_ * M_PI_2 - parking_position_ * local_vehicle_current_pose_.theta();
  double temp_theta_3 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  temp_theta_2 = legionclaw::common::math::NormalizeAngle(temp_theta_2);
  d2 = L * cos(temp_theta_2 - temp_theta_3);
  double t_d3 = local_vehicle_current_pose_.y() -
                (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
                 vehicle_param_.back_edge_to_center());
  // forward_distance=(d-d2-upward_distance)/cos(temp_theta_2);
  std::cout << "forward_distance1: " << forward_distance << endl;
  std::cout << "t_d2 : " << d2 << endl;
  std::cout << "t_d3: " << t_d3 << endl;
  std::cout << "fd_y: " << forward_distance * sin(temp_theta_2) << endl;
  double t_R = forward_distance / atan(temp_theta_2 / 2);
  std::cout << "t_R: " << t_R << endl;
  stage1_m3_a0_ = local_vehicle_current_pose_;
  stage1_m3_a1_.set_x(local_vehicle_current_pose_.x() +
                      parking_position_ * (t_R - t_R * cos(temp_theta_2)));
  // stage1_m3_a1_.set_y(local_vehicle_current_pose_.y()+(d-d2-upward_distance));
  stage1_m3_a1_.set_y(local_vehicle_current_pose_.y() +
                      (t_R * sin(temp_theta_2) - t_R * sin(temp_theta_2 / 2)));

  stage1_m3_a1_.set_theta(stage1_m3_a0_.theta());
  stage1_m3_a1_.set_kappa(0.0);
  stage1_m3_a1_.set_dkappa(0.0);
  stage1_m3_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move4_3ControlPoints(int parking_position_)
{
  // forward_distance=1.5;
  stage1_m4_3_a0_ = local_vehicle_current_pose_;

  stage1_m4_3_a1_ = local_vehicle_current_pose_;
  stage1_m4_3_a1_.set_x(parking_position_ * R - parking_position_ * R * cos(theta_4_1));
  stage1_m4_3_a1_.set_y(
      parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
      vehicle_param_.back_edge_to_center() + upward_distance - forward_distance + R * sin(theta_4));
  stage1_m4_3_a1_.set_theta(M_PI_2 - parking_position_ * theta_4_1);
}

void VerticalParkingPathGenerator::UpdateStage1Move3_3ControlPoints(int parking_position_)
{
  double temp_theta_2 =
      parking_position_ * M_PI_2 - parking_position_ * local_vehicle_current_pose_.theta();
  double temp_theta_3 = atan((vehicle_param_.width() / 2) / vehicle_param_.front_edge_to_center());
  temp_theta_2 = legionclaw::common::math::NormalizeAngle(temp_theta_2);
  d2 = L * cos(temp_theta_2 - temp_theta_3);
  double t_d3 = local_vehicle_current_pose_.y() -
                (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
                 vehicle_param_.back_edge_to_center());
  forward_distance = (d - d2 - upward_distance) / cos(temp_theta_2);
  std::cout << "forward_distance2: " << forward_distance << endl;
  std::cout << "t_d2 : " << d2 << endl;
  std::cout << "t_d3: " << t_d3 << endl;
  std::cout << "fd_y: " << forward_distance * sin(temp_theta_2) << endl;
  double t_R = forward_distance / atan(temp_theta_2 / 2);

  stage1_m3_3_a0_ = local_vehicle_current_pose_;
  stage1_m3_3_a1_.set_x(local_vehicle_current_pose_.x() +
                        parking_position_ * (t_R - t_R * cos(temp_theta_2)));
  // stage1_m3_a1_.set_y(local_vehicle_current_pose_.y()+(d-d2-upward_distance));
  stage1_m3_3_a1_.set_y(local_vehicle_current_pose_.y() +
                        (t_R * sin(temp_theta_2) - t_R * sin(temp_theta_2 / 2)));

  stage1_m3_3_a1_.set_theta(stage1_m3_a0_.theta());
  stage1_m3_3_a1_.set_kappa(0.0);
  stage1_m3_3_a1_.set_dkappa(0.0);
  stage1_m3_3_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move3_5ControlPoints(int parking_position_)
{
  double temp_theta_3 = legionclaw::common::math::NormalizeAngle(local_vehicle_current_pose_.theta());
  temp_theta_3 = parking_position_ * M_PI_2 - parking_position_ * temp_theta_3;
  // temp_theta_3 = legionclaw::common::math::NormalizeAngle(temp_theta_3);
  double arc_length = forward_distance;
  theta_t1 = M_PI_2 / 9;
  R3 = arc_length / theta_t1;
  theta_3 = temp_theta_3 - theta_t1;
  stage1_m3_3_a0_ = local_vehicle_current_pose_;
  stage1_m3_3_a1_.set_x(local_vehicle_current_pose_.x() +
                        parking_position_ *
                            (R * cos(temp_theta_3 - theta_t1) - R * cos(temp_theta_3)));

  stage1_m3_3_a1_.set_y(local_vehicle_current_pose_.y() +
                        (R * sin(temp_theta_3) - R * sin(temp_theta_3 - theta_t1)));

  stage1_m3_3_a1_.set_theta(local_vehicle_current_pose_.theta() + parking_position_ * theta_t1);
  stage1_m3_3_a1_.set_kappa(0.0);
  stage1_m3_3_a1_.set_dkappa(0.0);
  stage1_m3_3_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1Move4ControlPoints(int parking_position_)
{
  // forward_distance=1.5;
  stage1_m4_a0_ = local_vehicle_current_pose_;

  stage1_m4_a1_ = local_vehicle_current_pose_;
  stage1_m4_a1_.set_x(local_vehicle_current_pose_.x() -
                      parking_position_ * R * (cos(theta_4) - cos(theta_4 + theta_3)));
  stage1_m4_a1_.set_y(local_vehicle_current_pose_.y() -
                      fabs(local_vehicle_current_pose_.x() * sin(theta_4)));
  stage1_m4_a1_.set_theta(M_PI_2 - parking_position_ * theta_4);

  stage1_m4_a2_.set_x(0.0);
  stage1_m4_a2_.set_y(0.5);
  stage1_m4_a2_.set_theta(M_PI_2);
  stage1_m4_a2_.set_kappa(0.0);
  stage1_m4_a2_.set_dkappa(0.0);
  stage1_m4_a2_.set_ddkappa(0.0);

  stage1_m4_a3_.set_x(0.0);
  stage1_m4_a3_.set_y(0.0);
  stage1_m4_a3_.set_theta(M_PI_2);
  stage1_m4_a3_.set_kappa(0.0);
  stage1_m4_a3_.set_dkappa(0.0);
  stage1_m4_a3_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage1ForwardControlPoints(int parking_position_)
{
  stage1_f_a0_.set_x(parking_position_ *
                     (vehicle_param_.min_turning_radius() +
                      vertical_parking_conf_.vertical_min_forward_extend_distance()));
  stage1_f_a0_.set_y(parking_spot_.length() - in_distance_ -
                     vertical_parking_conf_.vertical_lon_safe_distance() -
                     vehicle_param_.back_edge_to_center() + vehicle_param_.min_turning_radius());
  stage1_f_a0_.set_theta((1 - parking_position_) * M_PI_2);
  stage1_f_a0_.set_kappa(0.0);

  stage1_f_a1_ = stage1_f_a0_;
  stage1_f_a1_.set_x(stage1_f_a0_.x() +
                     parking_position_ *
                         vertical_parking_conf_.vertical_forward_extend_distance_keep());
}

void VerticalParkingPathGenerator::UpdateStage1BackwardControlPoints(int parking_position_)
{
  stage1_b_a0_ = local_vehicle_current_pose_;
  stage1_b_a0_.set_z(0.0);

  stage1_b_a1_.set_x(parking_position_ * vehicle_param_.min_turning_radius());
  stage1_b_a1_.set_y(parking_spot_.length() - in_distance_ -
                     vertical_parking_conf_.vertical_lon_safe_distance() -
                     vehicle_param_.back_edge_to_center() + vehicle_param_.min_turning_radius());
  stage1_b_a1_.set_z(0.0);
  stage1_b_a1_.set_theta((1 - parking_position_) * M_PI_2);
  stage1_b_a1_.set_kappa(-parking_position_ / vehicle_param_.min_turning_radius());  // 左+右-
  stage1_b_a1_.set_dkappa(0.0);
  stage1_b_a1_.set_ddkappa(0.0);

  stage1_b_a2_.set_x(0.0);
  stage1_b_a2_.set_y(parking_spot_.length() - in_distance_ -
                     vertical_parking_conf_.vertical_lon_safe_distance() -
                     vehicle_param_.back_edge_to_center());
  stage1_b_a2_.set_z(0.0);
  stage1_b_a2_.set_theta(M_PI_2);
  stage1_b_a2_.set_kappa(-parking_position_ / vehicle_param_.min_turning_radius());  // 左+右-
  stage1_b_a2_.set_dkappa(0.0);
  stage1_b_a2_.set_ddkappa(0.0);

  stage1_b_a3_.set_x(0.0);
  stage1_b_a3_.set_y(0.0);
  stage1_b_a3_.set_z(0.0);
  stage1_b_a3_.set_theta(M_PI_2);
  stage1_b_a3_.set_kappa(0.0);  // 左+右-
  stage1_b_a3_.set_dkappa(0.0);
  stage1_b_a3_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage2ForwardControlPoints(int parking_position_)
{
  // stage2_b_a0_ = local_vehicle_current_pose_;
  stage2_f_a0_.set_x(0.0);
  stage2_f_a0_.set_y(0.0);
  stage2_f_a0_.set_theta(M_PI_2);
  stage2_f_a0_.set_kappa(0.0);
  stage2_f_a0_.set_dkappa(0.0);
  stage2_f_a0_.set_ddkappa(0.0);

  stage2_f_a1_.set_x(0.0);
  // A2点
  double y = vertical_parking_conf_.road_width() +
             (parking_spot_.length() - vertical_parking_conf_.vertical_lon_safe_distance() -
              vehicle_param_.back_edge_to_center()) -
             (vehicle_param_.length() + 0.2);
  // double y = parking_spot_.length() -
  //            vertical_parking_conf_.vertical_lon_safe_distance() -
  //            vehicle_param_.back_edge_to_center();
  stage2_f_a1_.set_y(y);
  stage2_f_a1_.set_theta(M_PI_2);
  stage2_f_a1_.set_kappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage2BackwardControlPoints(int parking_position_)
{
  stage2_b_a0_ = local_vehicle_current_pose_;

  stage2_b_a1_.set_x(0.0);
  stage2_b_a1_.set_y(0.0);
  stage2_b_a1_.set_theta(M_PI_2);
  stage2_b_a1_.set_kappa(0.0);
  stage2_b_a1_.set_dkappa(0.0);
  stage2_b_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage3Move1ControlPoints(int parking_position_)
{
  stage3_m1_a0_ = local_vehicle_current_pose_;

  stage3_m1_a1_.set_x(0.0);
  stage3_m1_a1_.set_y(origin_to_up_d);
  stage3_m1_a1_.set_theta(M_PI_2);
  stage3_m1_a1_.set_kappa(0.0);
  stage3_m1_a1_.set_dkappa(0.0);
  stage3_m1_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage3Move2ControlPoints(int parking_position_)
{
  stage3_m2_a0_ = local_vehicle_current_pose_;

  stage3_m2_a1_.set_x(0.0);
  stage3_m2_a1_.set_y(-origin_to_up_d);
  stage3_m2_a1_.set_theta(M_PI_2);
  stage3_m2_a1_.set_kappa(0.0);
  stage3_m2_a1_.set_dkappa(0.0);
  stage3_m2_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage3Move3ControlPoints(int parking_position_)
{
  stage3_m3_a0_ = local_vehicle_current_pose_;
  if (((R - out_d) / R) > 1) {
    AERROR << "Calculate theta_t failed.";
  }
  out_theta1 = acos((R - out_d) / R);
  out_theta2 = backword_distance / R;
  out_theta3 = M_PI_2 - out_theta1 - out_theta2;

  stage3_m3_a2_.set_x(R * (sin(out_theta1 + out_theta2) - sin(out_theta1)));
  stage3_m3_a2_.set_y(origin_to_up_d + R * (cos(out_theta1) - cos(out_theta1 + out_theta2)));
  stage3_m3_a2_.set_theta(M_PI_2 - parking_position_ * out_theta3);
  stage3_m3_a2_.set_kappa(0.0);
  stage3_m3_a2_.set_dkappa(0.0);
  stage3_m3_a2_.set_ddkappa(0.0);

  stage3_m3_a1_.set_x(0.0);
  stage3_m3_a1_.set_y(stage3_m3_a2_.y() - R * sin(out_theta3));
  stage3_m3_a1_.set_theta(M_PI_2);
  stage3_m3_a1_.set_kappa(0.0);
  stage3_m3_a1_.set_dkappa(0.0);
  stage3_m3_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage3Move4ControlPoints(int parking_position_)
{
  stage3_m4_a0_ = local_vehicle_current_pose_;

  stage3_m4_a1_.set_x(stage3_m4_a0_.x() -
                      parking_position_ * (R * (sin(t_theta) - sin(out_theta1))));
  stage3_m4_a1_.set_y(stage3_m4_a0_.y() - (R * (cos(out_theta1) - cos(t_theta))));
  stage3_m4_a1_.set_theta(stage3_m4_a0_.theta() - parking_position_ * out_theta2);
  stage3_m4_a1_.set_kappa(0.0);
  stage3_m4_a1_.set_dkappa(0.0);
  stage3_m4_a1_.set_ddkappa(0.0);
}

void VerticalParkingPathGenerator::UpdateStage3Move5ControlPoints(int parking_position_)
{
  stage3_m5_a0_ = local_vehicle_current_pose_;
  out_theta1 =
      legionclaw::common::math::NormalizeAngle(M_PI_2 - parking_position_ * M_PI_2 +
                                          parking_position_ * local_vehicle_current_pose_.theta());
  stage3_m5_a1_.set_x(stage3_m5_a0_.x() + parking_position_ * R * sin(out_theta1));
  stage3_m5_a1_.set_y(stage3_m5_a0_.y() + (R * (1 - cos(out_theta1))));
  stage3_m5_a1_.set_theta(M_PI_2 - parking_position_ * M_PI_2);
  stage3_m5_a1_.set_kappa(0.0);
  stage3_m5_a1_.set_dkappa(0.0);
  stage3_m5_a1_.set_ddkappa(0.0);

  stage3_m5_a2_ = stage3_m5_a1_;
  stage3_m5_a2_.set_x(stage3_m5_a1_.x() + parking_position_ * 1.0);
}
}  // namespace planning
}  // namespace legionclaw
