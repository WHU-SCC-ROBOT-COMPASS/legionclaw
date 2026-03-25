/**
 * @file    message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/point_cloud.hpp"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T *t) = 0;
  virtual void PublishObstacleList(legionclaw::interface::ObstacleList msg) = 0;
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
