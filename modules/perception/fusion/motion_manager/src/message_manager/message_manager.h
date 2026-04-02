/**
 * @file    message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#include "modules/common/enum/enum.h"

#include "modules/common/interface/location.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"

/**
 * @namespace legionclaw::perception::fusion
 * @brief legionclaw::perception::fusion
 */

namespace legionclaw {
namespace perception {
namespace fusion {
using namespace legionclaw::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T *t) = 0;
  virtual void
  PublishObstacleListOutput(legionclaw::interface::ObstacleList msg) = 0;
  virtual bool Activate() { return true; }
  virtual bool DeActivate() { return true; }
};
} // namespace fusion
} // namespace perception
} // namespace legionclaw
