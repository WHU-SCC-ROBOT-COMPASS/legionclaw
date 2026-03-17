
/**
 * @file gear_state_machine.hpp
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2020-08-03
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#pragma once

#include "lon_speed_controller.h"
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw 
{
  namespace control 
  {
    legionclaw::interface::ControlCommand *lon_speed_cmd_;

    void LonSpeedController::GearTransitionInit() 
    { 
      gear_sm_.reset(new state_machine::StateContext("./conf/control/gear_state_machine.json", true,"./log/state_machine/", "gear_state_machine"));
      epb_sm_.reset(new state_machine::StateContext("./conf/control/epb_state_machine.json", true,"./log/state_machine/", "epb_state_machine"));

      gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Init",std::bind(&LonSpeedController::GearInitEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Init",std::bind(&LonSpeedController::GearInitUpdate, this,std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Neutral",std::bind(&LonSpeedController::GearNEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Neutral",std::bind(&LonSpeedController::GearNUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Neutral",std::bind(&LonSpeedController::GearNExit, this, std::placeholders::_1, 0));

      // gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Parking",std::bind(&LonSpeedController::GearPEntry, this, std::placeholders::_1, 0));
      // gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Parking",std::bind(&LonSpeedController::GearPUpdate, this, std::placeholders::_1, 0));
      // gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Parking",std::bind(&LonSpeedController::GearPExit, this, std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Drive",std::bind(&LonSpeedController::GearDEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Drive",std::bind(&LonSpeedController::GearDUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Drive",std::bind(&LonSpeedController::GearDExit, this, std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Reverse",std::bind(&LonSpeedController::GearREntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Reverse",std::bind(&LonSpeedController::GearRUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Reverse",std::bind(&LonSpeedController::GearRExit, this, std::placeholders::_1, 0));

      

      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Init",std::bind(&LonSpeedController::EPBInitEntry, this, std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Init",std::bind(&LonSpeedController::EPBInitUpdate, this, std::placeholders::_1, 0));

      //进入状态执行一次
      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Applied",std::bind(&LonSpeedController::EPBAppliedEntry, this,std::placeholders::_1, 0));
      //处于当前状态频繁执行
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Applied",std::bind(&LonSpeedController::EPBAppliedUpdate, this,std::placeholders::_1, 0));
      //退出时执行一次
      epb_sm_->SetCallback(state_machine::CallbackType::EXIT, "Applied",std::bind(&LonSpeedController::EPBAppliedExit, this,std::placeholders::_1, 0));

      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Release",std::bind(&LonSpeedController::EPBReleaseEntry, this,std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Release",std::bind(&LonSpeedController::EPBReleaseUpdate, this,std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::EXIT, "Release",std::bind(&LonSpeedController::EPBReleaseExit, this,std::placeholders::_1, 0));
      gear_sm_->NextState("started");
      epb_sm_->NextState("started");
    }

    void LonSpeedController::GearTransition(legionclaw::interface::ControlCommand *cmd) 
    {
      if (cmd == nullptr) 
      {
        AWARN<<"GearTransition cmd == nullptr";
      }
      lon_speed_cmd_ = cmd;

      // EPB切换逻辑
      if (gear_sm_debug_.is_stopped() == true &&gear_sm_debug_.enable_epb_applied() == true &&
          (gear_sm_debug_.tar_gear() ==legionclaw::common::GearPosition::GEAR_NEUTRAL || gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_PARKING)) 
      {
        epb_sm_->NextState("applied");
      } 
      else 
      {
        // if (gear_sm_debug_.cur_gear() ==
        //       legionclaw::common::GearPosition::GEAR_DRIVE ||
        //     gear_sm_debug_.cur_gear() ==
        //       legionclaw::common::GearPosition::GEAR_REVERSE)   //EV350给刹车的时候无法挂档问题
        epb_sm_->NextState("release");
      }

      //目标档位和当前档位不一致  并且当前EPB处于非拉起状态  设置刹车值
      if (gear_sm_debug_.tar_gear() != gear_sm_debug_.cur_gear() && gear_sm_debug_.cur_epb_state() != legionclaw::common::EPBLevel::APPLIED) 
      {
        brake_sm_debug_.set_is_geartransition_stop(true);
        // cmd->set_brake_value(gear_sm_debug_.brake_value_when_gear_transitioning());
      }

      //目标档位和当前档位一致  并且目前档位为N或者P 并且EPB处于非拉起状态
      //设置刹车值
      if ((gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear() &&
          (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_NEUTRAL ||
            gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_PARKING)) &&
          gear_sm_debug_.cur_epb_state() != legionclaw::common::EPBLevel::APPLIED) 
      {
        brake_sm_debug_.set_is_geartransition_stop(true);
        // cmd->set_brake_value(gear_sm_debug_.brake_value_when_gear_transitioning());
      }
    }

    void LonSpeedController::GearInitEntry(const std::string &state_name, int state) {
      AINFO << "Init NNNNNNNNNNNNNN";
    }

    void LonSpeedController::GearInitUpdate(const std::string &state_name, int state) {
      gear_sm_->NextState("N");
    }

    void LonSpeedController::GearNEntry(const std::string &state_name, int state) 
    {
      AINFO << "Entry NNNNNNNNNNNNNN";
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
    }

    void LonSpeedController::GearNUpdate(const std::string &state_name, int state) 
    {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
      if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) 
      {
        return;
      }

      if (gear_sm_debug_.cur_gear() == legionclaw::common::GearPosition::GEAR_NEUTRAL) 
      {
        if (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_DRIVE) 
        {
          gear_sm_->NextState("D");
        } 
        else if (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_REVERSE) 
        {
          gear_sm_->NextState("R");
        }
      }
    }

    void LonSpeedController::GearNExit(const std::string &state_name, int state) {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
    }

    // void LonSpeedController::GearPEntry(const std::string &state_name, int state) 
    // {
    //   AINFO << "Entry PPPPPPPPPPPPPPPPPPPP";
    //   lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_PARKING);
    // }

    // void LonSpeedController::GearPUpdate(const std::string &state_name, int state) 
    // {
    //   lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_PARKING);
    //   if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) 
    //   {
    //     return;
    //   }

    //   if (gear_sm_debug_.cur_gear() == legionclaw::common::GearPosition::GEAR_PARKING) 
    //   {
    //     if (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_DRIVE) 
    //     {
    //       gear_sm_->NextState("D");
    //     } 
    //     else if (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_REVERSE) 
    //     {
    //       gear_sm_->NextState("R");
    //     }
    //   }
    // }

    // void LonSpeedController::GearPExit(const std::string &state_name, int state) 
    // {
    //   lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_PARKING);
    // }

    void LonSpeedController::GearDEntry(const std::string &state_name, int state) {
      AINFO << "Entry DDDDDDDDDDDDDD";
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
    }

    void LonSpeedController::GearDUpdate(const std::string &state_name, int state) 
    {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
      if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) {
        return;
      }

      if (gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_DRIVE &&
          gear_sm_debug_.is_stopped() == true)
        gear_sm_->NextState("N");
    }

    void LonSpeedController::GearDExit(const std::string &state_name, int state) {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
    }

    void LonSpeedController::GearREntry(const std::string &state_name, int state) {
      AINFO << "Entry RRRRRRRRRRRRRRR";
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
    }

    void LonSpeedController::GearRUpdate(const std::string &state_name, int state) 
    {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
      if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) {
        return;
      }

      if (gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_REVERSE &&
          gear_sm_debug_.is_stopped() == true)
        gear_sm_->NextState("N");
    }

    void LonSpeedController::GearRExit(const std::string &state_name, int state) {
      lon_speed_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
    }

    void LonSpeedController::EPBInitEntry(const std::string &state_name, int state) {
    }

    void LonSpeedController::EPBInitUpdate(const std::string &state_name, int state) {
    }

    void LonSpeedController::EPBAppliedEntry(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonSpeedController::EPBAppliedUpdate(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonSpeedController::EPBAppliedExit(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonSpeedController::EPBReleaseEntry(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }

    void LonSpeedController::EPBReleaseUpdate(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }

    void LonSpeedController::EPBReleaseExit(const std::string &state_name, int state) {
      lon_speed_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }
  }  // namespace control
}  // namespace legionclaw