/**
 * @file    obstacle_intent.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ObstacleIntent {
public:
  ObstacleIntent() {
    type_ = legionclaw::interface::ObstacleIntent::Type::INTENT_UNKNOWN_STATE;
  }
  ~ObstacleIntent() = default;

  enum Type {
    INTENT_UNKNOWN_STATE = 0,
    INTENT_FORWARD_STATE = 1,
    INTENT_BRANCH_LEFT_STATE = 2,
    INTENT_BRANCH_RIGHT_STATE = 3,
    INTENT_YIELDING_STATE = 4,
    INTENT_ACCELERATING_STATE = 5,
    INTENT_SLOWDOWN_STATE = 6,
    INTENT_TURN_LEFT_STATE = 7,
    INTENT_TURN_RIGHT_STATE = 8,
    INTENT_CHANGE_LANE_LEFT_STATE = 9,
    INTENT_CHANGE_LANE_RIGHT_STATE = 10,
    INTENT_CROSS_ROAD_STATE = 11,
    INTENT_ALONG_ROAD_STATE = 12,
    INTENT_STOPPING_STATE = 13,
    INTENT_ONCOMMING_STATE = 14,
    INTENT_KEEP_SPEED_STATE = 15,
  };

  inline void set_type(const legionclaw::interface::ObstacleIntent::Type &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const legionclaw::interface::ObstacleIntent::Type &type() const {
    return type_;
  }

  inline legionclaw::interface::ObstacleIntent::Type *mutable_type() {
    return &type_;
  }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  void operator=(const ObstacleIntent &obstacle_intent) {
    CopyFrom(obstacle_intent);
  }

  void CopyFrom(const ObstacleIntent &obstacle_intent) {
    type_ = obstacle_intent.type();
  }

protected:
  //估计障碍物的意图        0-UNKNOWN, 1-STOP, 2-STATIONARY, 3-移动 MOVING,
  //4-HANGE_LANE, 5-LOW_ACCELERATION, 6-HIGH_ACCELERATION, 7-LOW_DECELERATION,
  //8-HIGH_DECELERATION,
  legionclaw::interface::ObstacleIntent::Type type_;
  legionclaw::interface::ObstacleIntent::Type *type_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
