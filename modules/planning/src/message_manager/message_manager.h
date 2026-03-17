/**
 * @file    message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#include "modules/common/enum/enum.h"

#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/events.hpp"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/stop_info.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/guide_info.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/drivable_region.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/parking_out_info.hpp"
#include "modules/common/interface/trajectory_array.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"
#include "modules/common/interface/planning_analysis.hpp"
#include "modules/common/interface/prediction_obstacles.hpp"
#include "modules/common/interface/sotif_monitor_result.hpp"
#include "modules/common/interface/parking_state_display.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T* t) = 0;
  virtual void PublishADCTrajectory(legionclaw::interface::ADCTrajectory msg) = 0;
  virtual void PublishPlanningCmd(legionclaw::interface::PlanningCmd msg) = 0;
  virtual void
  PublishPlanningAnalysis(legionclaw::interface::PlanningAnalysis msg) = 0;
  virtual void
  PublishParkingStateDisplay(legionclaw::interface::ParkingStateDisplay msg) = 0;
  virtual void
  PublishTrajectoryArray(legionclaw::interface::TrajectoryArray msg) = 0;
  virtual void PublishFaults(legionclaw::interface::Faults msg) = 0;
  virtual void PublishEvents(legionclaw::interface::Events msg) = 0;
  virtual bool Activate() = 0;
  virtual bool DeActivate() = 0;
};
} // namespace planning
} // namespace legionclaw
