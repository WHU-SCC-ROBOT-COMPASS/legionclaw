/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @brief Some util functions.
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/logging/logging.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/configs/config_gflags.h"

/**
 * @namespace legionclaw::common::util
 * @brief legionclaw::common::util
 */
namespace legionclaw {
namespace common {
namespace util {
using namespace legionclaw::interface;
template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  // return a.GetTypeName() == b.GetTypeName() &&
  //        a.SerializeAsString() == b.SerializeAsString();
  // Test shows that the above method is 5 times faster than the
  // API: google::protobuf::util::MessageDifferencer::Equals(a, b);
  return true;
}

struct PairHash {
  template <typename T, typename U>
  size_t operator()(const std::pair<T, U>& pair) const {
    return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
  }
};

template <typename T> bool WithinBound(T start, T end, T value) {
  return value >= start && value <= end;
}

PointENU operator+(const PointENU enu, const math::Vec2d& xy);

/**
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const T delta = (end - start) / num;
  sliced->resize(num + 1);
  T s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

/**
 * calculate the distance beteween Point u and Point v, which are all have
 * member function x() and y() in XY dimension.
 * @param u one point that has member function x() and y().
 * @param b one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2), i.e., the Euclid distance on XY
 * dimension.
 */
template <typename U, typename V> double DistanceXY(const U& u, const V& v) {
  U temp1 = u;
  V temp2 = v;

  return std::hypot(temp1.x() - temp2.x(), temp1.y() - temp2.y());
}

/**
 * Check if two points u and v are the same point on XY dimension.
 * @param u one point that has member function x() and y().
 * @param v one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2) < epsilon, i.e., the Euclid distance
 * on XY dimension.
 */
template <typename U, typename V> bool SamePointXY(const U& u, const V& v) {
  static constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2);

// Test whether two float or double numbers are equal.
// ulp: units in the last place.
template <typename T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
IsFloatEqual(T x, T y, int ulp = 2) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}
} // namespace util
} // namespace common
} // namespace legionclaw

template <typename T> class FunctionInfo {
public:
  typedef int (T::*Function)();
  Function function_;
  std::string fun_name_;
};

// template <typename T, size_t count>
// bool ExcuteAllFunctions(T* obj, FunctionInfo<T> fun_list[]) {
//   for (size_t i = 0; i < count; i++) {
//     if ((obj->*(fun_list[i].function_))() != legionclaw::cyber::SUCC) {
//       AERROR << fun_list[i].fun_name_ << " failed.";
//       return false;
//     }
//   }
//   return true;
// }

template <typename A, typename B>
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  return os << "first: " << p.first << ", second: " << p.second;
}

#define UNIQUE_LOCK_MULTITHREAD(mutex_type)                                    \
  std::unique_ptr<std::unique_lock<std::mutex>> lock_ptr = nullptr;            \
  if (FLAGS_multithread_run) {                                                 \
    lock_ptr.reset(new std::unique_lock<std::mutex>(mutex_type));              \
  }
