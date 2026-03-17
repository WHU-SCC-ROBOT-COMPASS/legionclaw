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
#include "ros2_interface/msg/obstacle_list.hpp"
#include "ros2_interface/msg/obu_cmd_msg.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

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
  void Init(T* t) override;

  /**
   * @brief     ObstacleList消息发布.
   * @param[in] obstacle_list.
   * @return    void.
   */
  void PublishObstacleList(legionclaw::interface::ObstacleList msg) override;

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     发布聚类点云.
   * @param[in] clusters 聚类结果 (每个聚类包含点云数据).
   * @param[in] frame_id 坐标系.
   * @return    void.
   */
  void PublishClusterPointCloud(
      const std::vector<std::vector<std::tuple<float, float, float, float>>>& clusters,
      const std::string& frame_id);

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
  rclcpp::Publisher<::ros2_interface::msg::ObstacleList>::SharedPtr
      obstacle_list_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cluster_point_cloud_pub_;

  rclcpp::Subscription<::ros2_interface::msg::ObuCmdMsg>::SharedPtr
      obu_cmd_msg_sub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;

  void HandleObuCmdMsgMessage(
      const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj);
      
  void HandlePointCloudMessage(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
