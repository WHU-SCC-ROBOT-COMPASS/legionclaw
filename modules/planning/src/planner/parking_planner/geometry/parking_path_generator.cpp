
#include "modules/planning/src/planner/parking_planner/geometry/parking_path_generator.h"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>

using namespace std;
namespace legionclaw {
namespace planning {

/***********************************************************
* @class ParkingPathGenerator
* @brief 生成泊车轨迹信息

class ParkingPathGenerator
***********************************************************/
bool ParkingPathGenerator::ExtendPath(const legionclaw::interface::PathPoint &start_pos,
                                      const double &extend_length, const double &interval,
                                      const legionclaw::common::GearPosition &gear_position,
                                      std::vector<legionclaw::interface::PathPoint> &extend_path)
{
  double _interval = interval;
  legionclaw::interface::PathPoint temp_p;
  temp_p = start_pos;

  if (gear_position == legionclaw::common::GearPosition::GEAR_REVERSE) _interval *= -1.0;

  double s = 0.0;
  double temp_x = temp_p.x();
  double temp_y = temp_p.y();
  extend_path.clear();
  while (s <= extend_length) {
    s += interval;
    // 曲率和方向不变
    temp_x += _interval * cos(start_pos.theta());
    temp_y += _interval * sin(start_pos.theta());

    temp_p.set_x(temp_x);
    temp_p.set_y(temp_y);

    extend_path.push_back(temp_p);
  }

  return true;
}

bool ParkingPathGenerator::CalculateCirclePath(const double &ox, const double &oy,
                                               const double &theta0, const double &theta1,
                                               const double &dl, const double &r,
                                               std::vector<legionclaw::interface::PathPoint> &out_path)
{
  int sign = (theta0 < theta1) ? 1 : -1;
  double theta = theta0;
  double dtheta = sign * dl / r;
  double circle_x, circle_y;
  legionclaw::interface::PathPoint temp_pose;
  out_path.clear();
  for (; sign * theta <= sign * theta1; theta += dtheta) {
    circle_x = r * cos(theta);
    circle_y = r * sin(theta);

    temp_pose.set_x(circle_x + ox);
    temp_pose.set_y(circle_y + oy);
    temp_pose.set_theta(NormalizeAngle(theta - sign * M_PI_2));
    temp_pose.set_kappa(-sign / r);
    out_path.push_back(temp_pose);
  }

  return true;
}

bool ParkingPathGenerator::IsEnoughFreeSpace(const legionclaw::interface::PathPoint &pose)
{
  double heading_d = pose.theta();  // math::to_radians(pose.theta());
  // 定位为车辆后轴中心
  double x = pose.x() + vehicle_param_.back_edge_to_center() * cos(heading_d);
  double y = pose.y() + vehicle_param_.back_edge_to_center() * sin(heading_d);
  math::Polygon2d self_car(x, y, pose.theta(), vehicle_param_.width(), vehicle_param_.length());

  for (auto polygon : road_polygon_) {
    if (self_car.HasOverlap(polygon) == true)  // 如果有碰撞，表示不可通行
      return false;
  }

  return true;
}

void ParkingPathGenerator::SetVehicleParam(const legionclaw::interface::VehicleParam &vehicle_param)
{
  vehicle_param_ = vehicle_param;
}

void ParkingPathGenerator::UpdateSpiralCurveConfig(SpiralCurveConf spiral_curve_conf)
{
  spiral_curve_config_.simpson_size = spiral_curve_conf.simpson_size();
  spiral_curve_config_.newton_raphson_tol = spiral_curve_conf.newton_raphson_tol();
  spiral_curve_config_.newton_raphson_max_iter = spiral_curve_conf.newton_raphson_max_iter();
}
}  // namespace planning
}  // namespace legionclaw