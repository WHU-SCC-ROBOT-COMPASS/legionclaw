/**
 * @file    ros_2_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS2_ENABLE
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "modules/common/enum/enum.h"

#include "ros2_interface/msg/faults.hpp"
#include "ros2_interface/msg/events.hpp"
#include "ros2_interface/msg/chassis.hpp"
#include "ros2_interface/msg/location.hpp"
#include "ros2_interface/msg/planning_cmd.hpp"
#include "ros2_interface/msg/adc_trajectory.hpp"
#include "ros2_interface/msg/control_command.hpp"
#include "ros2_interface/msg/control_analysis.hpp"

#include "message_manager/message_manager.h"
#include "modules/common/enum/enum.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
/**
 * @class Ros2MessageManager
 * @brief ROS2消息管理器.
 */
template <typename T>
class Ros2MessageManager : public MessageManager<T>, public rclcpp::Node {
public:
  Ros2MessageManager();
  ~Ros2MessageManager() = default;

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

  /**
   * @brief     激活消息管理器.
   * @return    bool 成功返回true，失败返回false.
   */
  bool Activate() override;

  /**
   * @brief     停用消息管理器.
   * @return    bool 成功返回true，失败返回false.
   */
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;
  std::mutex mode_mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

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

protected:
  rclcpp::Publisher<::ros2_interface::msg::ControlCommand>::SharedPtr
      control_command_pub_;
  rclcpp::Publisher<::ros2_interface::msg::ControlAnalysis>::SharedPtr
      control_analysis_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Events>::SharedPtr events_pub_;

  rclcpp::Subscription<::ros2_interface::msg::ADCTrajectory>::SharedPtr
      adc_trajectory_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Chassis>::SharedPtr chassis_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Location>::SharedPtr
      location_sub_;
  rclcpp::Subscription<::ros2_interface::msg::PlanningCmd>::SharedPtr
      planning_cmd_sub_;

  void HandleADCTrajectoryMessage(
      const ros2_interface::msg::ADCTrajectory::SharedPtr msg_obj);

  void
  HandleChassisMessage(const ros2_interface::msg::Chassis::SharedPtr msg_obj);

  void
  HandleLocationMessage(const ros2_interface::msg::Location::SharedPtr msg_obj);

  void HandlePlanningCmdMessage(
      const ros2_interface::msg::PlanningCmd::SharedPtr msg_obj);
};
} // namespace control
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
