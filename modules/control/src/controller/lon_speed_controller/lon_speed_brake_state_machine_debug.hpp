#pragma once
#include "modules/common/interface/chassis.hpp"

namespace legionclaw
{
  namespace control
  {
    class BrakeStateMachineDebug
    {
    public:
      BrakeStateMachineDebug() = default;
      ~BrakeStateMachineDebug() = default;

      void set_is_obs_stop(bool is_obs_stop) 
      {
        is_obs_stop_ = is_obs_stop;
      }
      bool is_obs_stop() const {
        return is_obs_stop_;
      }

      void set_is_geartransition_stop(bool is_geartransition_stop) 
      {
        is_geartransition_stop_ = is_geartransition_stop;
      }
      bool is_geartransition_stop() const {
        return is_geartransition_stop_;
      }

    protected:
      bool is_obs_stop_ = false;
      bool is_geartransition_stop_ = false;

    };
  }
}
