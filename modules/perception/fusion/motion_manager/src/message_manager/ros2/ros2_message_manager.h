/**
 * @file    ros_2_message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS2_ENABLE
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "ros2_interface/msg/location.hpp"
#include "ros2_interface/msg/obstacle_list.hpp"
#include "ros2_interface/msg/obu_cmd_msg.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::fusion
 * @brief legionclaw::perception::fusion
 */

namespace legionclaw {
namespace perception {
namespace fusion {
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
  void Init(T *t) override;

  /**
   * @brief     ObstacleList消息发布.
   * @param[in] obstacle_list.
   * @return    void.
   */
  void PublishObstacleListOutput(legionclaw::interface::ObstacleList msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T *instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;
  std::mutex mode_mutex_;

  std::unique_ptr<std::thread> handle_avtive_thread_;

  std::unique_ptr<std::thread> handle_message_thread_;

  /**
   * @brief     线程运行函数.
   * @return    void.
   */
  void Run();

  /**
   * @brief     线程运行函数.
   * @return    void.
   */
  void ActivateMode();

  /**
   * @brief     线程结束函数.
   * @return    void.
   */
  void Stop();

  void TaskStop();

  void TaskStart();

protected:
  std::mutex mutex_;
  rclcpp::Publisher<::ros2_interface::msg::ObstacleList>::SharedPtr
      obstacle_list_output_pub_;

  rclcpp::Subscription<::ros2_interface::msg::Location>::SharedPtr
      location_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ObstacleList>::SharedPtr
      obstacle_list_input_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ObstacleList>::SharedPtr
      lcd_obstacle_list_sub_;
      
  rclcpp::Subscription<::ros2_interface::msg::ObuCmdMsg>::SharedPtr
      obu_cmd_msg_sub_;

  void HandleObuCmdMsgMessage(
      const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj);

  void
  HandleLocationMessage(const ros2_interface::msg::Location::SharedPtr msg_obj);

  void HandleObstacleListInputMessage(
      const ros2_interface::msg::ObstacleList::SharedPtr msg_obj);

  void HandleLCDObstacleListMessage(
      const ros2_interface::msg::ObstacleList::SharedPtr msg_obj);
};
} // namespace fusion
} // namespace perception
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
