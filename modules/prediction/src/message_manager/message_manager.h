/**
 * @file    message_manager.h
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#include "modules/common/enum/enum.h"

#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/odometry.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/prediction_obstacles.hpp"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
using namespace legionclaw::common;
template <typename T> class MessageManager {
public:
  MessageManager() = default;
  virtual ~MessageManager() = default;

  virtual void Init(T* t) = 0;
  virtual void
  PublishPredictionObstacles(legionclaw::interface::PredictionObstacles msg) = 0;
  virtual void PublishFaults(legionclaw::interface::Faults msg) = 0;
  virtual bool Activate() = 0;
  virtual bool DeActivate() = 0;
};
} // namespace prediction
} // namespace legionclaw
