/**
 * @file    message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#include "modules/common/enum/enum.h"

#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/point_cloud.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/obstacle_list.hpp"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T* t) = 0;
  virtual void PublishObstacleList(legionclaw::interface::ObstacleList msg) = 0;
  virtual void PublishFaults(legionclaw::interface::Faults msg) = 0;
  virtual bool Activate() = 0;
  virtual bool DeActivate() = 0;
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
