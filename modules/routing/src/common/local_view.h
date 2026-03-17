/**
 * @file    local_view.h
 * @author  zdhy
 * @date    2021-10-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/odometry.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/routing_request.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
struct LocalView {
  legionclaw::interface::TrafficLightMsg traffic_light_msg_;
  legionclaw::interface::Location location_;
  legionclaw::interface::Chassis chassis_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;
  legionclaw::interface::TrafficEvents traffic_events_input_;
  legionclaw::interface::Odometry odometry_;
  legionclaw::interface::RoutingRequest routing_request_;
};
} // namespace routing
} // namespace legionclaw
