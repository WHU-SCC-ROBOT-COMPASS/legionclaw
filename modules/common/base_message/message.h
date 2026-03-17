/**
 * @file              message.h
 * @author       duanchengwen (duanchengwen@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-06-25 11:51:09
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#pragma once

#include <iostream>

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {
enum MessageType {
  LCM = 0,
  DDS = 1,
  ROS = 2,
  ADSFI = 3,
  ROS2 = 4,
};

typedef struct {
  MessageType type;
  std::string name;
  std::string url;
} Message;
} // namespace common
} // namespace legionclaw
