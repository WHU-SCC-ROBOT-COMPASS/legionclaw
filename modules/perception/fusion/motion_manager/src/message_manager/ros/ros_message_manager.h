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

#include "ros_interface/Location.h"
#include "ros_interface/ObstacleList.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::fusion
 * @brief legionclaw::perception::fusion
 */

namespace legionclaw {
namespace perception {
namespace fusion {
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
  void PublishObstacleListOutput(legionclaw::interface::ObstacleList msg) override;

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
  ros::Publisher obstacle_list_output_pub_;

  ros::Subscriber location_sub_;
  ros::Subscriber obstacle_list_input_sub_;

  void HandleLocationMessage(const ros_interface::Location &msg_obj);

  void
  HandleObstacleListInputMessage(const ros_interface::ObstacleList &msg_obj);
};
} // namespace fusion
} // namespace perception
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
