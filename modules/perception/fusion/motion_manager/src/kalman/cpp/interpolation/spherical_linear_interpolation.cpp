// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "interpolation/spherical_linear_interpolation.hpp"

namespace interpolation
{
tf2_geometry_msgs::Quaternion slerp(
  const tf2_geometry_msgs::Quaternion & src_quat, const tf2_geometry_msgs::Quaternion & dst_quat,
  const double ratio)
{
  tf2::Quaternion src_tf;
  tf2::Quaternion dst_tf;
  tf2::quaternionMsgToTF(src_quat, src_tf);
  tf2::quaternionMsgToTF(dst_quat, dst_tf);
  const auto interpolated_quat = tf2::slerp(src_tf, dst_tf, ratio);
  tf2_geometry_msgs::Quaternion quaternion;
  quaternion.w = interpolated_quat.getW();
  quaternion.x = interpolated_quat.getX();
  quaternion.y = interpolated_quat.getY();
  quaternion.z = interpolated_quat.getZ();
  return quaternion;
}

std::vector<tf2_geometry_msgs::Quaternion> slerp(
  const std::vector<double> & base_keys,
  const std::vector<tf2_geometry_msgs::Quaternion> & base_values,
  const std::vector<double> & query_keys)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = interpolation_utils::validateKeys(base_keys, query_keys);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  // calculate linear interpolation
  std::vector<tf2_geometry_msgs::Quaternion> query_values;
  size_t key_index = 0;
  for (const auto query_key : validated_query_keys) {
    while (base_keys.at(key_index + 1) < query_key) {
      ++key_index;
    }

    const auto src_quat = base_values.at(key_index);
    const auto dst_quat = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    const auto interpolated_quat = slerp(src_quat, dst_quat, ratio);
    query_values.push_back(interpolated_quat);
  }

  return query_values;
}

tf2_geometry_msgs::Quaternion lerpOrientation(
  const tf2_geometry_msgs::Quaternion & o_from, const tf2_geometry_msgs::Quaternion & o_to,
  const double ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::quaternionMsgToTF(o_from, q_from);
  tf2::quaternionMsgToTF(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);

  tf2_geometry_msgs::Quaternion quaternion;
  quaternion.w = q_interpolated.getW();
  quaternion.x = q_interpolated.getX();
  quaternion.y = q_interpolated.getY();
  quaternion.z = q_interpolated.getZ();
  return quaternion;
}
}  // namespace interpolation
