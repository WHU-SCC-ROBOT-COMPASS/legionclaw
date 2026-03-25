/*
 * @Author       : jiangchengjie
 * @Date         : 2021-06-08 17:29:56
 * @Version      :
 * @LastEditors  : jiangchengjie
 * @LastEditTime : 2021-06-08 18:02:08
 * @Description  :
 */
/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GEOMETRY_MSGS_TRANSFORM_STAMPED_H
#define GEOMETRY_MSGS_TRANSFORM_STAMPED_H

#include <iostream>
#include <stdint.h>
#include <vector>

namespace tf2_geometry_msgs {

struct Header {
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
  Header() : seq(0), stamp(0), frame_id("") {}
};

struct Vector3 {
  double x;
  double y;
  double z;
  Vector3() : x(0.0), y(0.0), z(0.0) {}
};

struct Point {
  double x;
  double y;
  double z;
  Point() : x(0.0), y(0.0), z(0.0) {}
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
  Quaternion() : x(0.0), y(0.0), z(0.0), w(0.0) {}
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};

struct QuaternionStamped {
  Header header;
  Quaternion quaternion;
};

struct PoseWithCovariance {
  Pose pose;
  double covariance[36];
};

struct PoseWithCovarianceStamped {
  Header header;
  PoseWithCovariance pose;
};

struct Point32 {
  double x;
  double y;
  double z;
  Point32() : x(0.0), y(0.0), z(0.0) {}
};

struct Polygon {
  std::vector<Point32> points;
};

} // namespace tf2_geometry_msgs

#endif
