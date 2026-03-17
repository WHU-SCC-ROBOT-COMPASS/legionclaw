/**
 * @file    local_view.h
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/location.hpp"
#include "modules/common/interface/odometry.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/routing_response.hpp"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
struct LocalView {
  legionclaw::interface::Location location_;
  legionclaw::interface::ADCTrajectory adc_trajectory_;
  legionclaw::interface::ObstacleList obstacle_list_;
  legionclaw::interface::Odometry odometry_;
  // legionclaw::interface::TrafficEvents traffic_events_;
  int localiztion_mode_;
  int map_mode_;
  legionclaw::interface::RoutingResponse routing_response_;
  legionclaw::interface::LaneList lane_list_;
};
} // namespace prediction
} // namespace legionclaw
