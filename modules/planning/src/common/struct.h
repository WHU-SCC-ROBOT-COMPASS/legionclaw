
#pragma once
#include "modules/common/interface/adc_trajectory.hpp"
namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
typedef struct {
  legionclaw::interface::ADCTrajectory::BehaviourLatState lat_state;
  legionclaw::interface::ADCTrajectory::BehaviourLonState lon_state;
  std::string lat_state_name;
  std::string lon_state_name;
  double collision_distance;
  double relative_distance;
  double follow_velocity;
  double follow_acceleration;
} BehaviourState;
}  // namespace planning
}  // namespace legionclaw
