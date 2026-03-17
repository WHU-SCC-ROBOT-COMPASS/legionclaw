/**
 * @file    local_view.h
 * @author  zdhy
 * @date    2021-09-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/drivable_region.hpp"
#include "modules/common/interface/guide_info.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/parking_out_info.hpp"
#include "modules/common/interface/prediction_obstacles.hpp"
#include "modules/common/interface/prediction_out_array.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/sotif_monitor_result.hpp"
#include "modules/common/interface/stop_info.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

struct LocalView {
  legionclaw::interface::RoutingResponse routing_response_;
  legionclaw::interface::TrafficEvents traffic_events_;
  legionclaw::interface::GuideInfo guide_info_;
  legionclaw::interface::ParkingInfo parking_info_;
  legionclaw::interface::StopInfo stop_info_;
  legionclaw::interface::TrafficLightMsg traffic_light_msg_;
  legionclaw::interface::Location location_;
  legionclaw::interface::PredictionObstacles prediction_obstacles_;
  legionclaw::interface::LaneList lane_list_;
  legionclaw::interface::Chassis chassis_;
  legionclaw::interface::SotifMonitorResult sotif_monitor_result_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;
  legionclaw::interface::DrivableRegion drivable_region_;
  legionclaw::interface::ParkingOutInfo parking_out_info_;
};
}  // namespace planning
}  // namespace legionclaw
