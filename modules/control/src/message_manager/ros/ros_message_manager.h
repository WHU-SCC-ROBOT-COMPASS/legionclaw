/**
 * @file    ros_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS_ENABLE
#include <thread>

#include <ros/ros.h>

#include "ros_interface/Faults.h"
#include "ros_interface/Events.h"
#include "ros_interface/Chassis.h"
#include "ros_interface/Location.h"
#include "ros_interface/PlanningCmd.h"
#include "ros_interface/ADCTrajectory.h"
#include "ros_interface/ControlCommand.h"
#include "ros_interface/ControlAnalysis.h"
#include "ros_interface/ObuCmdMsg.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
using namespace legionclaw::common;
/**
 * @class RosMessageManager
 * @brief ROS消息管理器.
 */
template <typename T> class RosMessageManager : public MessageManager<T> {
public:
  RosMessageManager() = default;
  ~RosMessageManager() = default;

  /**
   * @brief     初始化。
   * @param[in] obu_url LCM组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  /**
   * @brief     ControlCommand消息发布.
   * @param[in] control_command.
   * @return    void.
   */
  void PublishControlCommand(legionclaw::interface::ControlCommand msg) override;

  /**
   * @brief     ControlAnalysis消息发布.
   * @param[in] control_analysis.
   * @return    void.
   */
  void PublishControlAnalysis(legionclaw::interface::ControlAnalysis msg) override;

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Events消息发布.
   * @param[in] events.
   * @return    void.
   */
  void PublishEvents(legionclaw::interface::Events msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;
  std::mutex mode_mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

  std::unique_ptr<std::thread> register_thread_;

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

  void TaskStop();

  void TaskStart();

  void RegisterLoop();

protected:
  std::mutex mutex_;
  ros::NodeHandle nh_;
  ros::Publisher control_command_pub_;
  ros::Publisher control_analysis_pub_;
  ros::Publisher faults_pub_;
  ros::Publisher events_pub_;

  ros::Subscriber adc_trajectory_sub_;
  ros::Subscriber chassis_sub_;
  ros::Subscriber location_sub_;
  ros::Subscriber planning_cmd_sub_;
  ros::Subscriber obu_cmd_msg_sub_;

  void HandleADCTrajectoryMessage(const ros_interface::ADCTrajectory& msg_obj);

  void HandleChassisMessage(const ros_interface::Chassis& msg_obj);

  void HandleLocationMessage(const ros_interface::Location& msg_obj);

  void HandlePlanningCmdMessage(const ros_interface::PlanningCmd& msg_obj);

  void HandleObuCmdMsgMessage(const ros_interface::ObuCmdMsg& msg_obj);
};
} // namespace control
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
