/**
 * @file              mat3.hpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-17 01:56:41
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#pragma once

#include <math.h>

#include "modules/common/interface/point_basic.hpp"
#include "modules/common/interface/path_point.hpp"

namespace legionclaw {
namespace planning {
namespace math {
class Mat3 {
  double m[3][3];

 public:
  Mat3() {
    // initialize Identity by default
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) m[i][j] = 0;

    m[0][0] = m[1][1] = m[2][2] = 1;
  }

  Mat3(double trans_x, double trans_y, bool mirror_x, bool mirror_y) {
    m[0][0] = (mirror_x == true) ? -1 : 1;
    m[0][1] = 0;
    m[0][2] = trans_x;
    m[1][0] = 0;
    m[1][1] = (mirror_y == true) ? -1 : 1;
    m[1][2] = trans_y;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double trans_x, double trans_y) {
    m[0][0] = 1;
    m[0][1] = 0;
    m[0][2] = trans_x;
    m[1][0] = 0;
    m[1][1] = 1;
    m[1][2] = trans_y;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double rotation_angle) {
    double c = cos(rotation_angle);
    double s = sin(rotation_angle);
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = 0;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = 0;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(legionclaw::interface::PointBasic rotation_center) {
    double c = cos(rotation_center.theta());
    double s = sin(rotation_center.theta());
    double u = rotation_center.x();
    double v = rotation_center.y();
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = -u * c + v * s + u;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = -u * s - v * c + v;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  legionclaw::interface::PointBasic operator*(legionclaw::interface::PointBasic v) {
    legionclaw::interface::PointBasic point_basic = v;
    point_basic.set_x(m[0][0] * point_basic.x() + m[0][1] * point_basic.y() +
                      m[0][2] * 1);
    point_basic.set_y(m[1][0] * point_basic.x() + m[1][1] * point_basic.y() +
                      m[1][2] * 1);
    return point_basic;
  }

    Mat3(legionclaw::interface::PathPoint rotationCenter) {
    double c = cos(rotationCenter.theta());
    double s = sin(rotationCenter.theta());
    double u = rotationCenter.x();
    double v = rotationCenter.y();
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = -u * c + v * s + u;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = -u * s - v * c + v;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  legionclaw::interface::PathPoint operator*(legionclaw::interface::PathPoint v) {
    legionclaw::interface::PathPoint point_basic = v;
    point_basic.set_x(m[0][0] * point_basic.x() + m[0][1] * point_basic.y() +
                      m[0][2] * 1);
    point_basic.set_y(m[1][0] * point_basic.x() + m[1][1] * point_basic.y() +
                      m[1][2] * 1);
    return point_basic;
  }
};
}  // namespace math
}  // namespace planning
}  // namespace legionclaw