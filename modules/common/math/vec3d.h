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

/**
 * @file
 * @brief Defines the Vec3d class.
 */

#ifndef MODULES_COMMON_MATH_VEC3D_H_
#define MODULES_COMMON_MATH_VEC3D_H_

#include <cmath>
#include <string>

/**
 * @namespace legionclaw::common::math
 * @brief legionclaw::common::math
 */
namespace legionclaw {
namespace common {
namespace math {

constexpr double MathEpsilon = 1e-10;

/**
 * @class Vec3d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec3d {
public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec3d(const double x, const double y, const double z) noexcept
      : x_(x), y_(y), z_(z) {}

  //! Constructor returning the zero vector.
  constexpr Vec3d() noexcept : Vec3d(0, 0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  // static Vec3d CreateUnitVec3d(const double angle);

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Getter for z component
  double z() const { return z_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Setter for y component
  void set_z(const double z) { z_ = z; }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec3d& other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec3d& other) const;

  //! Returns the "cross" product between these two Vec3d (non-standard).
  double CrossProd(const Vec3d& other) const;

  //! Returns the inner product between these two Vec3d.
  double InnerProd(const Vec3d& other) const;

  // //! rotate the vector by angle.
  // Vec3d rotate(const double angle) const;

  // //! rotate the vector itself by angle.
  // void SelfRotate(const double angle);
  //! Sums two Vec3d
  Vec3d operator+(const Vec3d& other) const;

  //! Subtracts two Vec3d
  Vec3d operator-(const Vec3d& other) const;

  //! Multiplies Vec3d by a scalar
  Vec3d operator*(const double ratio) const;

  //! Divides Vec3d by a scalar
  Vec3d operator/(const double ratio) const;

  //! Sums another Vec3d to the current one
  Vec3d& operator+=(const Vec3d& other);

  //! Subtracts another Vec3d to the current one
  Vec3d& operator-=(const Vec3d& other);

  //! Multiplies this Vec3d by a scalar
  Vec3d& operator*=(const double ratio);

  //! Divides this Vec3d by a scalar
  Vec3d& operator/=(const double ratio);

  //! Compares two Vec3d
  bool operator==(const Vec3d& other) const;

  //! Returns a human-readable string representing this object
  std::string DebugString() const;

protected:
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
};

//! Multiplies the given Vec3d by a given scalar
Vec3d operator*(const double ratio, const Vec3d& vec);

} // namespace math
} // namespace common
} // namespace legionclaw

#endif /* MODULES_COMMON_MATH_VEC3D_H_ */
