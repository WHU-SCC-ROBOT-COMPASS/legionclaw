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

#ifndef UTILS__ROS__UUID_HELPER_HPP_
#define UTILS__ROS__UUID_HELPER_HPP_

#include <common/interface/uuid.hpp>

#include <boost/uuid/uuid.hpp>

#include <algorithm>
#include <random>
#include <string>

namespace utils
{
inline motion_manager::interface::UUID generateUUID()
{
  // Generate random number
  motion_manager::interface::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  uuid.mutable_uuid()->resize(16);
  std::generate(uuid.mutable_uuid()->begin(), uuid.mutable_uuid()->end(), bit_eng);
  return uuid;
}
inline std::string toHexString(const motion_manager::interface::UUID & id)
{
  std::stringstream ss;
  // id.uuid().resize(3);
  for (auto i = 0; i < 3; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid()[i];
  }
  return ss.str();
}

inline boost::uuids::uuid toBoostUUID(const motion_manager::interface::UUID & id)
{
  boost::uuids::uuid boost_uuid{};
  std::copy(id.uuid().begin(), id.uuid().end(), boost_uuid.begin());
  return boost_uuid;
}

inline motion_manager::interface::UUID toUUIDMsg(const boost::uuids::uuid & id)
{
  motion_manager::interface::UUID ros_uuid{};
  std::copy(id.begin(), id.end(), ros_uuid.mutable_uuid()->begin());
  return ros_uuid;
}

}  // namespace utils

#endif  // UTILS__ROS__UUID_HELPER_HPP_
