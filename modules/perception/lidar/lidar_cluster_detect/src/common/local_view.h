/**
 * @file    local_view.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/point_cloud.hpp"

/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
struct LocalView {
  legion::interface::PointCloud point_cloud_;
};
} // namespace lidar
} // namespace perception
} // namespace legion
