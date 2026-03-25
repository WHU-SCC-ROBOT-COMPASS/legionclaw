/**
 * @file    ros_message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS_ENABLE
#include <thread>

#include <ros/ros.h>

#include "ros_interface/ObstacleList.h"
#include "ros_interface/PointCloud.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
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
  ros::NodeHandle nh_;
  ros::Publisher obstacle_list_pub_;

  ros::Subscriber point_cloud_sub_;

  void HandlePointCloudMessage(const sensor_msgs::PointCloud2 &msg_obj);
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
