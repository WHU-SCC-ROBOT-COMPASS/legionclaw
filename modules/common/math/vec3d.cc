/******************************************************************************
 * Copyright 2017 The LegionClaw Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "vec3d.h"

#include <cmath>
#include <iostream>

using namespace std;

//#include "modules/common/log.h"
//#include "modules/common/util/string_util.h"

namespace legionclaw {
namespace common {
namespace math {

// Vec3d Vec3d::CreateUnitVec3d(const double angle) {
//   return Vec3d(cos(angle), sin(angle));
// }

double Vec3d::Length() const { return std::hypot(x_, y_); }

double Vec3d::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec3d::Angle() const { return std::atan2(y_, x_); }

void Vec3d::Normalize() {
  const double l = Length();
  if (l > MathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec3d::DistanceTo(const Vec3d &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec3d::DistanceSquareTo(const Vec3d &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  const double dz = z_ - other.z_;
  return dx * dx + dy * dy + dz * dz;
}

double Vec3d::CrossProd(const Vec3d &other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec3d::InnerProd(const Vec3d &other) const {
  return x_ * other.x() + y_ * other.y();
}

// Vec3d Vec3d::rotate(const double angle) const {
//   return Vec3d(x_ * cos(angle) - y_ * sin(angle),
//                x_ * sin(angle) + y_ * cos(angle));
// }

// void Vec3d::SelfRotate(const double angle) {
//   double tmp_x = x_;
//   x_ = x_ * cos(angle) - y_ * sin(angle);
//   y_ = tmp_x * sin(angle) + y_ * cos(angle);
// }

Vec3d Vec3d::operator+(const Vec3d &other) const {
  return Vec3d(x_ + other.x(), y_ + other.y(), z_ + other.z());
}

Vec3d Vec3d::operator-(const Vec3d &other) const {
  return Vec3d(x_ - other.x(), y_ - other.y(), z_ - other.z());
}

Vec3d Vec3d::operator*(const double ratio) const {
  return Vec3d(x_ * ratio, y_ * ratio, z_ * ratio);
}

Vec3d Vec3d::operator/(const double ratio) const {
  // CHECK_GT(std::abs(ratio), MathEpsilon);
  if (std::abs(ratio) <= MathEpsilon) {
    std::cout << "operator/(const double ratio) const std::abs(ratio) <=  "
                 "MathEpsilon"
              << endl;
    return Vec3d(0.0, 0.0, 0.0);
  }
  return Vec3d(x_ / ratio, y_ / ratio, z_ / ratio);
}

Vec3d &Vec3d::operator+=(const Vec3d &other) {
  x_ += other.x();
  y_ += other.y();
  z_ += other.z();
  return *this;
}

Vec3d &Vec3d::operator-=(const Vec3d &other) {
  x_ -= other.x();
  y_ -= other.y();
  z_ -= other.z();
  return *this;
}

Vec3d &Vec3d::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  z_ *= ratio;
  return *this;
}

Vec3d &Vec3d::operator/=(const double ratio) {
  // CHECK_GT(std::abs(ratio), MathEpsilon);
  if (std::abs(ratio) <= MathEpsilon) {
    std::cout
        << "operator/=(const double ratio) std::abs(ratio) <= MathEpsilon"
        << endl;
    return *this;
  }
  x_ /= ratio;
  y_ /= ratio;
  z_ /= ratio;
  return *this;
}

bool Vec3d::operator==(const Vec3d &other) const {
  return (std::abs(x_ - other.x()) < MathEpsilon &&
          std::abs(y_ - other.y()) < MathEpsilon &&
          std::abs(z_ - other.z()) < MathEpsilon);
}

Vec3d operator*(const double ratio, const Vec3d &vec) { return vec * ratio; }

// std::string Vec3d::DebugString() const {
//  return util::StrCat("Vec3d ( x = ", x_, "  y = ", y_, " )");
//}

}  // namespace math
}  // namespace common
}  // namespace legionclaw
