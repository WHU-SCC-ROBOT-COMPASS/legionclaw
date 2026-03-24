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

#include "rclcpp/rclcpp.hpp"

#include "ros2_interface/msg/faults.hpp"
#include "ros2_interface/msg/point_cloud.hpp"
#include "ros2_interface/msg/obu_cmd_msg.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "message_manager/message_manager.h"

/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
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
   * @brief     PointCloud消息发布.
   * @param[in] point_cloud.
   * @return    void.
   */
  void PublishGroundPoints(const legion::interface::PointCloud& msg);

  /**
   * @brief     PointCloud消息发布.
   * @param[in] point_cloud.
   * @return    void.
   */
  void PublishNoGroundPoints(const legion::interface::PointCloud& msg);

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legion::interface::Faults msg) override;

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

  std::unique_ptr<std::thread> handle_avtive_thread_;

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      ground_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      no_ground_points_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_input_sub_;
  
  rclcpp::Subscription<::ros2_interface::msg::ObuCmdMsg>::SharedPtr
      obu_cmd_msg_sub_;

  void HandleObuCmdMsgMessage(
      const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj);

  void HandlePointCloudInputMessage(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
};
} // namespace lidar
} // namespace perception
} // namespace legion
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
