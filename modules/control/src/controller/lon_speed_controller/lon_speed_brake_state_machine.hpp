#pragma once

#include "lon_speed_controller.h"

namespace legionclaw 
{
  namespace control 
  {
    legionclaw::interface::ControlCommand *control_cmd_;
    void LonSpeedController::BrakeStatusInit() 
    {
      brake_sm_.reset(new state_machine::StateContext("./conf/control/brake_state_machine.json", true,"./log/state_machine/", "brake_state_machine"));
      brake_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Init",std::bind(&LonSpeedController::BrakeInitEntry, this, std::placeholders::_1, 0));
      brake_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Init",std::bind(&LonSpeedController::BrakeInitUpdate, this,std::placeholders::_1, 0));

      brake_sm_->SetCallback(state_machine::CallbackType::ENTRY, "ON",std::bind(&LonSpeedController::BrakeOnEntry, this, std::placeholders::_1, 0));
      brake_sm_->SetCallback(state_machine::CallbackType::UPDATE, "ON",std::bind(&LonSpeedController::BrakeOnUpdate, this, std::placeholders::_1, 0));
      brake_sm_->SetCallback(state_machine::CallbackType::EXIT, "ON",std::bind(&LonSpeedController::BrakeOnExit, this, std::placeholders::_1, 0));

      brake_sm_->SetCallback(state_machine::CallbackType::ENTRY, "OFF",std::bind(&LonSpeedController::BrakeOffEntry, this, std::placeholders::_1, 0));
      brake_sm_->SetCallback(state_machine::CallbackType::UPDATE, "OFF",std::bind(&LonSpeedController::BrakeOffUpdate, this, std::placeholders::_1, 0));
      brake_sm_->SetCallback(state_machine::CallbackType::EXIT, "OFF",std::bind(&LonSpeedController::BrakeOffExit, this, std::placeholders::_1, 0));
      brake_sm_->NextState("started");
    }

    void LonSpeedController::BrakeTransition(legionclaw::interface::ControlCommand *cmd) 
    {
      control_cmd_ = cmd;
      if (brake_sm_debug_.is_obs_stop() || brake_sm_debug_.is_geartransition_stop())
      {
        brake_sm_ -> NextState("on");
      }
      else
      {
        brake_sm_ -> NextState("off");
      }
    }

    void LonSpeedController::BrakeInitEntry(const std::string &state_name, int state)
    {
      AINFO << "Brake state machine init ";
    }

    void LonSpeedController::BrakeInitUpdate(const std::string &state_name, int state)
    {
      brake_sm_->NextState("off");
    }

    void LonSpeedController::BrakeOnEntry(const std::string &state_name, int state)
    {
      AINFO << "Brake state machine ON Entry";
    }

    void LonSpeedController::BrakeOnUpdate(const std::string &state_name, int state)
    {
      //先后顺序
      if (brake_sm_debug_.is_geartransition_stop())
      {
        control_cmd_ ->set_brake_value(vehicle_param_.brake_value_when_gear_transitioning());
      }
      if (brake_sm_debug_.is_obs_stop())
      {
        control_cmd_ ->set_brake_value(control_conf_->soft_estop_brake());
      }
    }

    void LonSpeedController::BrakeOnExit(const std::string &state_name, int state)
    {
      AINFO << "Brake state machine ON Exit";
    }

    void LonSpeedController::BrakeOffEntry(const std::string &state_name, int state)
    {
      AINFO << "Brake state machine OFF Entry";
    }

    void LonSpeedController::BrakeOffUpdate(const std::string &state_name, int state)
    {
      control_cmd_ ->set_brake_value(0);
    }

    void LonSpeedController::BrakeOffExit(const std::string &state_name, int state)
    {
      AINFO << "Brake state machine OFF Exit";
    }
  }
}