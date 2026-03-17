/**
 * @file    ros_message_manager.h
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS_ENABLE
#include <thread>

#include <ros/ros.h>

#include "ros_interface/ObuCmdMsg.h"
#include "ros_interface/Faults.h"
#include "ros_interface/Location.h"
#include "ros_interface/Odometry.h"
#include "ros_interface/LaneList.h"
#include "ros_interface/ObstacleList.h"
#include "ros_interface/ADCTrajectory.h"
#include "ros_interface/TrafficEvents.h"
#include "ros_interface/RoutingResponse.h"
#include "ros_interface/PredictionObstacles.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
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
   * @brief     PredictionObstacles消息发布.
   * @param[in] prediction_obstacles.
   * @return    void.
   */
  void PublishPredictionObstacles(
      legionclaw::interface::PredictionObstacles msg) override;

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
  int action_mode_;
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
  ros::Publisher prediction_obstacles_pub_;
  ros::Publisher faults_pub_;

  ros::Subscriber location_sub_;
  ros::Subscriber adc_trajectory_sub_;
  ros::Subscriber obstacle_list_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber traffic_events_sub_;
  ros::Subscriber routing_response_sub_;
  ros::Subscriber lane_list_sub_;
  ros::Subscriber obu_cmd_msg_sub_;


  void HandleLocationMessage(const ros_interface::Location& msg_obj);

  void HandleADCTrajectoryMessage(const ros_interface::ADCTrajectory& msg_obj);

  void HandleObstacleListMessage(const ros_interface::ObstacleList& msg_obj);

  void HandleOdometryMessage(const ros_interface::Odometry& msg_obj);

  void HandleTrafficEventsMessage(const ros_interface::TrafficEvents& msg_obj);

  void
  HandleRoutingResponseMessage(const ros_interface::RoutingResponse& msg_obj);

  void HandleLaneListMessage(const ros_interface::LaneList& msg_obj);


  void HandleObuCmdMsgMessage(const ros_interface::ObuCmdMsg& msg_obj);
};
} // namespace prediction
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
