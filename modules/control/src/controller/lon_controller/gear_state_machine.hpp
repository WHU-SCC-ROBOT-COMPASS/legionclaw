
/**
 * @file gear_state_machine.hpp
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2020-08-03
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#pragma once

#include "lon_controller.h"
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw 
{
  namespace control 
  {
    legionclaw::interface::ControlCommand *lon_cmd_;

    void LonController::GearTransitionInit() 
    { 
      gear_sm_.reset(new state_machine::StateContext(FLAGS_state_machine_app_gear_state_machine_file, true,FLAGS_state_machine_log_dir, "gear_state_machine"));
      epb_sm_.reset(new state_machine::StateContext(FLAGS_state_machine_app_epb_state_machine_file, true,FLAGS_state_machine_log_dir, "epb_state_machine"));

      gear_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Init",std::bind(&LonController::GearInitEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Init",std::bind(&LonController::GearInitUpdate, this,std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Neutral",std::bind(&LonController::GearNEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Neutral",std::bind(&LonController::GearNUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Neutral",std::bind(&LonController::GearNExit, this, std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Drive",std::bind(&LonController::GearDEntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Drive",std::bind(&LonController::GearDUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Drive",std::bind(&LonController::GearDExit, this, std::placeholders::_1, 0));

      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Reverse",std::bind(&LonController::GearREntry, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Reverse",std::bind(&LonController::GearRUpdate, this, std::placeholders::_1, 0));
      gear_sm_->SetCallback(state_machine::CallbackType::EXIT, "Reverse",std::bind(&LonController::GearRExit, this, std::placeholders::_1, 0));

      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Init",std::bind(&LonController::EPBInitEntry, this, std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Init",std::bind(&LonController::EPBInitUpdate, this, std::placeholders::_1, 0));

      //进入状态执行一次
      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Applied",std::bind(&LonController::EPBAppliedEntry, this,std::placeholders::_1, 0));
      //处于当前状态频繁执行
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Applied",std::bind(&LonController::EPBAppliedUpdate, this,std::placeholders::_1, 0));
      //退出时执行一次
      epb_sm_->SetCallback(state_machine::CallbackType::EXIT, "Applied",std::bind(&LonController::EPBAppliedExit, this,std::placeholders::_1, 0));

      epb_sm_->SetCallback(state_machine::CallbackType::ENTRY, "Release",std::bind(&LonController::EPBReleaseEntry, this,std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Release",std::bind(&LonController::EPBReleaseUpdate, this,std::placeholders::_1, 0));
      epb_sm_->SetCallback(state_machine::CallbackType::EXIT, "Release",std::bind(&LonController::EPBReleaseExit, this,std::placeholders::_1, 0));
      gear_sm_->NextState("started");
      epb_sm_->NextState("started");
    }

    void LonController::GearTransition(legionclaw::interface::ControlCommand *cmd) 
    {
      if (cmd == nullptr) 
      {
        AWARN<<"GearTransition cmd == nullptr";
      }
      lon_cmd_ = cmd;

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
        cmd->set_brake_value(gear_sm_debug_.brake_value_when_gear_transitioning());
      }

      //目标档位和当前档位一致  并且目前档位为N或者P 并且EPB处于非拉起状态
      //设置刹车值
      if ((gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear() &&
          (gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_NEUTRAL ||
            gear_sm_debug_.tar_gear() == legionclaw::common::GearPosition::GEAR_PARKING)) &&
          gear_sm_debug_.cur_epb_state() != legionclaw::common::EPBLevel::APPLIED) 
      {
        cmd->set_brake_value(gear_sm_debug_.brake_value_when_gear_transitioning());
      }
    }

    void LonController::GearInitEntry(const std::string &state_name, int state) {
      AINFO << "Init NNNNNNNNNNNNNN";
    }

    void LonController::GearInitUpdate(const std::string &state_name, int state) {
      gear_sm_->NextState("N");
    }

    void LonController::GearNEntry(const std::string &state_name, int state) {
      AINFO << "Entry NNNNNNNNNNNNNN";
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
    }

    void LonController::GearNUpdate(const std::string &state_name, int state) 
    {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
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

    void LonController::GearNExit(const std::string &state_name, int state) {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_NEUTRAL);
    }

    void LonController::GearDEntry(const std::string &state_name, int state) {
      AINFO << "Entry DDDDDDDDDDDDDD";
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
    }

    void LonController::GearDUpdate(const std::string &state_name, int state) 
    {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
      if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) {
        return;
      }

      if (gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_DRIVE &&
          gear_sm_debug_.is_stopped() == true)
        gear_sm_->NextState("N");
    }

    void LonController::GearDExit(const std::string &state_name, int state) {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_DRIVE);
    }

    void LonController::GearREntry(const std::string &state_name, int state) {
      AINFO << "Entry RRRRRRRRRRRRRRR";
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
    }

    void LonController::GearRUpdate(const std::string &state_name, int state) 
    {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
      if (gear_sm_debug_.tar_gear() == gear_sm_debug_.cur_gear()) {
        return;
      }

      if (gear_sm_debug_.tar_gear() != legionclaw::common::GearPosition::GEAR_REVERSE &&
          gear_sm_debug_.is_stopped() == true)
        gear_sm_->NextState("N");
    }

    void LonController::GearRExit(const std::string &state_name, int state) {
      lon_cmd_->set_gear_location(legionclaw::common::GearPosition::GEAR_REVERSE);
    }

    void LonController::EPBInitEntry(const std::string &state_name, int state) {
    }

    void LonController::EPBInitUpdate(const std::string &state_name, int state) {
    }

    void LonController::EPBAppliedEntry(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonController::EPBAppliedUpdate(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonController::EPBAppliedExit(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::APPLIED);
    }

    void LonController::EPBReleaseEntry(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }

    void LonController::EPBReleaseUpdate(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }

    void LonController::EPBReleaseExit(const std::string &state_name, int state) {
      lon_cmd_->set_epb_level(legionclaw::common::EPBLevel::RELEASED);
    }
  }  // namespace control
}  // namespace legionclaw