
/**
 * @file dependency_injector.h
 * @brief Defines the PIDController class.
 */

#pragma once

#include "modules/control/src/common/vehicle_state/vehicle_state_provider.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

using legionclaw::control::common::VehicleStateProvider;

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  VehicleStateProvider* vehicle_state() 
  {
    return &vehicle_state_;
  }

 protected:
  VehicleStateProvider vehicle_state_;
};

}  // namespace control
}  // namespace legionclaw
