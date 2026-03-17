/**
 * @file    lcm_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if LCM_ENABLE
#include <thread>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_interface/Faults.hpp"
#include "lcm_interface/Events.hpp"
#include "lcm_interface/Chassis.hpp"
#include "lcm_interface/Location.hpp"
#include "lcm_interface/PlanningCmd.hpp"
#include "lcm_interface/ADCTrajectory.hpp"
#include "lcm_interface/ControlCommand.hpp"
#include "lcm_interface/ControlAnalysis.hpp"

#include "message_manager/message_manager.h"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "lcm_interface/ObuCmdMsg.hpp"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
/**
 * @class LcmMessageManager
 * @brief LCM消息管理器.
 */
template <typename T> class LcmMessageManager : public MessageManager<T> {
public:
  LcmMessageManager() = default;
  ~LcmMessageManager() = default;

  /**
   * @brief     初始化。
   * @param[in] obu_url LCM组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  /**
   * @brief     ControlCommand消息发送.
   * @param[in] control_command
   * @return    void.
   */
  void PublishControlCommand(legionclaw::interface::ControlCommand msg) override;

  /**
   * @brief     ControlAnalysis消息发送.
   * @param[in] control_analysis
   * @return    void.
   */
  void PublishControlAnalysis(legionclaw::interface::ControlAnalysis msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Events消息发送.
   * @param[in] events
   * @return    void.
   */
  void PublishEvents(legionclaw::interface::Events msg) override;

  /**
   * @brief     行泊功能相关的订阅和去订阅.
   * @param[in] 
   * @return    bool.
   */
  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  std::shared_ptr<lcm::LCM> lcm_;
  bool is_active_;
  std::mutex mutex_;

  lcm::Subscription* location_sub_;
  lcm::Subscription* chassis_sub_;
  lcm::Subscription* ADCTrajectory_sub_;
  lcm::Subscription* PlanningCmd_sub_;

  std::unique_ptr<std::thread> handle_message_thread_;

  void HandleObuCmdMsgMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const lcm_interface::ObuCmdMsg* msg);
                              
  void HandleADCTrajectoryMessage(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const lcm_interface::ADCTrajectory* msg);

  void HandleChassisMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcm_interface::Chassis* msg);

  void HandleLocationMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Location* msg);

  void HandlePlanningCmdMessage(const lcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const lcm_interface::PlanningCmd* msg);

  /**
   * @brief     线程运行函数.
   * @return    void.
   */
  void Run();

  /**
   * @brief     线程结束函数.
   * @return    void.
   */
  void Stop();
};
} // namespace control
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif