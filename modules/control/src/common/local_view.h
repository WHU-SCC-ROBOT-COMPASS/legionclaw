/**
 * @file    local_view.h
 * @author  jiangchengjie
 * @date    2021-07-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/chassis.hpp"
// #include "modules/common/interface/location.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/control/src/interface/localization_estimate.hpp"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
struct LocalView {
  legionclaw::interface::ADCTrajectory adc_trajectory_;
  legionclaw::interface::Chassis chassis_;
  legionclaw::interface::ObstacleList obstacle_list_;
  // legionclaw::interface::Location location_;
  legionclaw::interface::LocalizationEstimate localization_;
  legionclaw::interface::PlanningCmd planning_cmd_;
};
} // namespace control
} // namespace legionclaw
