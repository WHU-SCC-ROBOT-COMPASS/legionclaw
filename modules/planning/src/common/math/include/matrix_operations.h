
/// \file matrix_operations.h
/// \brief Simple matrix operations
/// \author Hatem Darweesh
/// \date Jun 19, 2016

#pragma once

#include <math.h>

#include "modules/common/interface/path_point.hpp"

using namespace legionclaw::interface;
namespace legionclaw {
namespace planning {
namespace math {
class Mat3 {
 public:
  Mat3() {
    // initialize Identity by default
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) m[i][j] = 0;

    m[0][0] = m[1][1] = m[2][2] = 1;
  }

  Mat3(double transX, double transY, bool mirrorX, bool mirrorY) {
    m[0][0] = (mirrorX == true) ? -1 : 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = (mirrorY == true) ? -1 : 1;
    m[1][2] = transY;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double transX, double transY) {
    m[0][0] = 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = 1;
    m[1][2] = transY;
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

  Mat3(PathPoint rotationCenter) {
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

  PathPoint operator*(PathPoint v) {
    PathPoint _v = v;
    v.x = m[0][0] * _v.x() + m[0][1] * _v.y() + m[0][2] * 1;
    v.y = m[1][0] * _v.x() + m[1][1] * _v.y() + m[1][2] * 1;
    return v;
  }

 private:
  double m[3][3];
};

}  // namespace math
}  // namespace planning
}  // namespace legionclaw
