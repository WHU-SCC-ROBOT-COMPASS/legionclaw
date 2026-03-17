/**
 * @file    ros_2_message_manager.h
 * @author  legionclaw
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
#include "ros2_interface/msg/location.hpp"
#include "ros2_interface/msg/odometry.hpp"
#include "ros2_interface/msg/lane_list.hpp"
#include "ros2_interface/msg/obstacle_list.hpp"
#include "ros2_interface/msg/adc_trajectory.hpp"
#include "ros2_interface/msg/traffic_events.hpp"
#include "ros2_interface/msg/routing_response.hpp"
#include "ros2_interface/msg/prediction_obstacles.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
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
  int action_mode_;
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
  rclcpp::Publisher<::ros2_interface::msg::PredictionObstacles>::SharedPtr
      prediction_obstacles_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;

  rclcpp::Subscription<::ros2_interface::msg::Location>::SharedPtr
      location_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ADCTrajectory>::SharedPtr
      adc_trajectory_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ObstacleList>::SharedPtr
      obstacle_list_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Odometry>::SharedPtr
      odometry_sub_;
  rclcpp::Subscription<::ros2_interface::msg::TrafficEvents>::SharedPtr
      traffic_events_sub_;
  rclcpp::Subscription<::ros2_interface::msg::RoutingResponse>::SharedPtr
      routing_response_sub_;
  rclcpp::Subscription<::ros2_interface::msg::LaneList>::SharedPtr
      lane_list_sub_;

  void
  HandleLocationMessage(const ros2_interface::msg::Location::SharedPtr msg_obj);

  void HandleADCTrajectoryMessage(
      const ros2_interface::msg::ADCTrajectory::SharedPtr msg_obj);

  void HandleObstacleListMessage(
      const ros2_interface::msg::ObstacleList::SharedPtr msg_obj);

  void
  HandleOdometryMessage(const ros2_interface::msg::Odometry::SharedPtr msg_obj);

  void HandleTrafficEventsMessage(
      const ros2_interface::msg::TrafficEvents::SharedPtr msg_obj);

  void HandleRoutingResponseMessage(
      const ros2_interface::msg::RoutingResponse::SharedPtr msg_obj);

  void
  HandleLaneListMessage(const ros2_interface::msg::LaneList::SharedPtr msg_obj);
};
} // namespace prediction
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
