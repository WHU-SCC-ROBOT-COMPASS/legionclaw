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
#include "ros_interface/PointCloud.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
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
   * @brief     PointCloud消息发布.
   * @param[in] point_cloud.
   * @return    void.
   */
  void PublishGroundPoints(legionclaw::interface::PointCloud msg) override;

  /**
   * @brief     PointCloud消息发布.
   * @param[in] point_cloud.
   * @return    void.
   */
  void PublishNoGroundPoints(legionclaw::interface::PointCloud msg) override;

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

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
  ros::NodeHandle nh_;
  std::mutex mutex_;
  ros::Publisher ground_points_pub_;
  ros::Publisher no_ground_points_pub_;
  ros::Publisher faults_pub_;

  ros::Subscriber point_cloud_input_sub_;

  void HandlePointCloudInputMessage(const ros_interface::PointCloud& msg_obj);
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
