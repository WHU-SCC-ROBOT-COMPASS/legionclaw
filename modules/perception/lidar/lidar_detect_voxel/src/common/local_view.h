/**
 * @file    local_view.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/point_cloud.hpp"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
struct LocalView {
  legionclaw::interface::PointCloud point_cloud_;
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
