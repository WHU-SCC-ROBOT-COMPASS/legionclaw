
#include "modules/common/math/math_tools.h"

#include <iostream>

#include "modules/common/logging/logging.h"
#include "modules/common/math/math_utils.h"

using namespace std;
namespace legionclaw {
namespace common {
namespace math {
// 角度 - 转 - 弧度
double D2R(double d) { return d * M_PI / 180.0; }
// 弧度 - 转 - 角度
double R2D(double r) { return r * 180.0 / M_PI; }

// 计算头指向
// 局部坐标系的x轴在全局坐标系下的头指向,单位：°
double GetHeading(double x1, double y1, double x2, double y2) {
  double xxx;
  double xx = (x2 - x1);
  double yy = (y2 - y1);

  double length = sqrt(xx * xx + yy * yy);

  if (length == 0)
    length = 0.000000001;

  double theta = 0.0;
  double theta_d = 0.0;

  xxx = fabs(xx);
  // atan 计算的角度范围为 +-180度， 而且xx和yy有符号问题。需要测试一下。
  if (xx > 0 && yy > 0) // 1
  {
    theta = asin(xxx / length);
    theta_d = R2D(theta);
  }
  if (xx < 0 && yy > 0) // 4
  {
    theta = asin(xxx / length);
    theta_d = 360 - R2D(theta);
  }
  if (xx > 0 && yy < 0) // 2
  {
    theta = asin(xxx / length);
    theta_d = 180 - R2D(theta);
  }
  if (xx < 0 && yy < 0) // 3
  {
    theta = asin(xxx / length);
    theta_d = 180 + R2D(theta);
  }

  if (xx == 0 && yy > 0)
    theta_d = 0;
  if (xx == 0 && yy < 0)
    theta_d = 180;
  if (xx > 0 && yy == 0)
    theta_d = 90;
  if (xx < 0 && yy == 0)
    theta_d = 270;

  return theta_d;
}

// 求取相关点的头指向,返回值为弧度
double GetHeadingRadian(double dx, double dy) {
  double fabs_dy;
  double length = sqrt(dx * dx + dy * dy);

  if (length == 0)
    length = 0.000000001;

  double theta = 0.0;
  fabs_dy = fabs(dy);

  if (dx > 0 && dy > 0) // 1
  {
    theta = asin(fabs_dy / length);
  }
  if (dx < 0 && dy > 0) // 2
  {
    theta = M_PI - asin(fabs_dy / length);
  }
  if (dx > 0 && dy < 0) // 4
  {
    theta = 2 * M_PI - asin(fabs_dy / length);
  }
  if (dx < 0 && dy < 0) // 3
  {
    theta = M_PI + asin(fabs_dy / length);
  }

  if (dx == 0 && dy > 0)
    theta = 0.5 * M_PI;
  if (dx == 0 && dy < 0)
    theta = 1.5 * M_PI;
  if (dx > 0 && dy == 0)
    theta = 0;
  if (dx < 0 && dy == 0)
    theta = M_PI;

  return theta;
}

Transfer::Transfer() {
  m_x0_ = 0.0;
  m_y0_ = 0.0;
  m_angle_ = 0.0;
  translate_origin_.reserve(3);
}

Transfer::~Transfer() {}

void Transfer::SetOrigin(double x0, double y0, double angle) {
  m_x0_ = x0;
  m_y0_ = y0;
  m_angle_ = angle;
  translate_origin_.clear();
  translate_origin_.emplace_back(x0);
  translate_origin_.emplace_back(y0);
  translate_origin_.emplace_back(angle);
}

bool Transfer::TransferL2G(double x_local, double y_local, double& x_global,
                           double& y_global) {
  //旋转
  x_global = cos(m_angle_) * x_local - sin(m_angle_) * y_local;
  y_global = sin(m_angle_) * x_local + cos(m_angle_) * y_local;

  //平移
  x_global += m_x0_;
  y_global += m_y0_;

  return true;
}

//局部坐标系转全局坐标系下
bool Transfer::TransferL2G(double x_local, double y_local, double h_local,
                           double& x_global, double& y_global,
                           double& h_global) {
  TransferL2G(x_local, y_local, x_global, y_global);

  h_global = h_local + m_angle_;

  if (h_global > 2 * M_PI)
    h_global -= 2 * M_PI;

  return true;
}

// 局部坐标系 - 转 - 全局坐标系
bool Transfer::TransferL2G(legionclaw::interface::PathPoint p_local,
                           legionclaw::interface::PathPoint& p_global) {
  double x = 0.0, y = 0.0, h = 0.0;
  TransferL2G(p_local.x(), p_local.y(), p_local.theta(), x, y, h);
  p_global.set_x(x);
  p_global.set_y(y);
  p_global.set_theta(h);
  p_global.set_kappa(p_local.kappa());

  return true;
}

/**
 * @brief 将全局坐标系的导航点坐标,航向角和曲率转换到局部坐标系下。
 * @param point_global 输入量：全局坐标系下起点。
 * @param point_local 输出量：局部坐标系下终点。
 */
bool Transfer::TransferG2L(legionclaw::interface::PathPoint point_global,
                           legionclaw::interface::PathPoint& point_local) {
  // 世界坐标系转全局坐标系，需要先平移后旋转
  double delta_x = point_global.x() - m_x0_;
  double delta_y = point_global.y() - m_y0_;

  double heading = point_global.theta() - m_angle_;
  point_local.set_x(delta_x * cos(-m_angle_) - delta_y * sin(-m_angle_));
  point_local.set_y(delta_x * sin(-m_angle_) + delta_y * cos(-m_angle_));
  point_local.set_kappa(point_global.kappa());

  if (heading > 2 * M_PI) {
    heading -= 2 * M_PI;
  }
  // if (heading < 0.0)
  // {
  // 	heading += 2 * M_PI;
  // }
  point_local.set_theta(NormalizeAngle(heading));

  return true;
}

const std::vector<double>& Transfer::GetOrigin() const {
  return translate_origin_;
}

bool Transfer::PathPointNormalizing(const std::vector<double>& in,
                                    std::vector<double>& out) {
  if (in.size() != 3 || translate_origin_.size() != 3) {
    AERROR << "input size error.";
    return false;
  }

  double delta_x = in[0] - translate_origin_[0];
  double delta_y = in[1] - translate_origin_[1];
  out.clear();
  out.emplace_back(delta_x * std::cos(-translate_origin_[2]) -
                   delta_y * std::sin(-translate_origin_[2]));
  out.emplace_back(delta_x * std::sin(-translate_origin_[2]) +
                   delta_y * std::cos(-translate_origin_[2]));
  out.emplace_back(common::math::NormalizeAngle(in[2] - translate_origin_[2]));

  return true;
}

bool Transfer::PathPointDeNormalizing(const std::vector<double>& in,
                                      std::vector<double>& out) {
  if (in.size() != 3 || translate_origin_.size() != 3) {
    AERROR << "input size error.";
    return false;
  }

  double x = in[0] * std::cos(translate_origin_[2]) -
             in[1] * std::sin(translate_origin_[2]);
  double y = in[0] * std::sin(translate_origin_[2]) +
             in[1] * std::cos(translate_origin_[2]);
  out.clear();
  x += translate_origin_[0];
  out.emplace_back(x);
  y += translate_origin_[1];
  out.emplace_back(y);
  out.emplace_back(common::math::NormalizeAngle(in[2] + translate_origin_[2]));

  return true;
}

bool Transfer::PathPointNormalizing(const Vec2d& in, Vec2d& out) {
  if (translate_origin_.size() != 3) {
    AERROR << "translate_origin_ size error.";
    return false;
  }

  double delta_x = in.x() - translate_origin_[0];
  double delta_y = in.y() - translate_origin_[1];
  out.set_x(delta_x * std::cos(-translate_origin_[2]) -
            delta_y * std::sin(-translate_origin_[2]));
  out.set_y(delta_x * std::sin(-translate_origin_[2]) +
            delta_y * std::cos(-translate_origin_[2]));

  return true;
}

bool Transfer::PathPointDeNormalizing(const Vec2d& in, Vec2d& out) {
  if (translate_origin_.size() != 3) {
    AERROR << "translate_origin_ size error.";
    return false;
  }

  double temp_x = in.x() * std::cos(translate_origin_[2]) -
                  in.y() * std::sin(translate_origin_[2]);
  double temp_y = in.x() * std::sin(translate_origin_[2]) +
                  in.y() * std::cos(translate_origin_[2]);
  out.set_x(temp_x + translate_origin_[0]);
  out.set_y(temp_y + translate_origin_[1]);

  return true;
}
} // namespace math
} // namespace common
} // namespace legionclaw
