/**
 * @file    message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/events.hpp"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/control_command.hpp"
#include "modules/common/interface/control_analysis.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
using namespace legionclaw::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T* t) = 0;
  virtual void PublishControlCommand(legionclaw::interface::ControlCommand msg) = 0;
  virtual void
  PublishControlAnalysis(legionclaw::interface::ControlAnalysis msg) = 0;
  virtual void PublishFaults(legionclaw::interface::Faults msg) = 0;
  virtual void PublishEvents(legionclaw::interface::Events msg) = 0;
  virtual bool Activate() = 0;
  virtual bool DeActivate() = 0;
};
} // namespace control
} // namespace legionclaw
