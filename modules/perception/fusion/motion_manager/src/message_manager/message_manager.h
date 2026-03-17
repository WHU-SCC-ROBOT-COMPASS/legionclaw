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
 * @namespace legion::perception::fusion
 * @brief legion::perception::fusion
 */

namespace legion {
namespace perception {
namespace fusion {
using namespace legion::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T *t) = 0;
  virtual void
  PublishObstacleListOutput(legion::interface::ObstacleList msg) = 0;
  virtual bool Activate() = 0;
  virtual bool DeActivate() = 0;
};
} // namespace fusion
} // namespace perception
} // namespace legion
