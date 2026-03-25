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

#include "ros2_interface/msg/obstacle_list.hpp"
#include "ros2_interface/msg/point_cloud.hpp"

#include "sensor_msgs/msg/point_cloud.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
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
  void Init(T *t) override;

  /**
   * @brief     ObstacleList消息发布.
   * @param[in] obstacle_list.
   * @return    void.
   */
  void PublishObstacleList(legionclaw::interface::ObstacleList msg) override;

protected:
  T *instance_;
  bool is_init_;

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
  rclcpp::Publisher<::ros2_interface::msg::ObstacleList>::SharedPtr
      obstacle_list_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;

  void HandlePointCloudMessage(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg_obj);
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
