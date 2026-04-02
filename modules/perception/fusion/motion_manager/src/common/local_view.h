/**
 * @file    local_view.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/location.hpp"
#include "modules/common/interface/obstacle_list.hpp"

/**
 * @namespace legionclaw::perception::fusion
 * @brief legionclaw::perception::fusion
 */

namespace legionclaw {
namespace perception {
namespace fusion {
struct LocalView {
  legionclaw::interface::Location location_;
  legionclaw::interface::ObstacleList obstacle_list_input_;
};
} // namespace fusion
} // namespace perception
} // namespace legionclaw
