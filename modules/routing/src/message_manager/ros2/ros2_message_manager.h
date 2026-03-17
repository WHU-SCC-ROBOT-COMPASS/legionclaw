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
#include "ros2_interface/msg/chassis.hpp"
#include "ros2_interface/msg/location.hpp"
#include "ros2_interface/msg/odometry.hpp"
#include "ros2_interface/msg/lane_list.hpp"
#include "ros2_interface/msg/stop_info.hpp"
#include "ros2_interface/msg/guide_info.hpp"
#include "ros2_interface/msg/polygon2_d.hpp"
#include "ros2_interface/msg/obu_cmd_msg.hpp"
#include "ros2_interface/msg/parking_info.hpp"
#include "ros2_interface/msg/navi_info_msg.hpp"
#include "ros2_interface/msg/traffic_events.hpp"
#include "ros2_interface/msg/routing_request.hpp"
#include "ros2_interface/msg/routing_response.hpp"
#include "ros2_interface/msg/global_route_msg.hpp"
#include "ros2_interface/msg/traffic_light_msg.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
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
   * @brief     RoutingResponse消息发布.
   * @param[in] routing_response.
   * @return    void.
   */
  void PublishRoutingResponse(legionclaw::interface::RoutingResponse msg) override;

  /**
   * @brief     GuideInfo消息发布.
   * @param[in] guide_info.
   * @return    void.
   */
  void PublishGuideInfo(legionclaw::interface::GuideInfo msg) override;

  /**
   * @brief     LaneList消息发布.
   * @param[in] lane_list.
   * @return    void.
   */
  void PublishLaneList(legionclaw::interface::LaneList msg) override;

  /**
   * @brief     ParkingInfo消息发布.
   * @param[in] parking_info.
   * @return    void.
   */
  void PublishParkingInfo(legionclaw::interface::ParkingInfo msg) override;

  /**
   * @brief     StopInfo消息发布.
   * @param[in] stop_info.
   * @return    void.
   */
  void PublishStopInfo(legionclaw::interface::StopInfo msg) override;

  /**
   * @brief     GlobalRouteMsg消息发布.
   * @param[in] global_route_msg.
   * @return    void.
   */
  void PublishGlobalRouteMsg(legionclaw::interface::GlobalRouteMsg msg) override;

  void PublishNaviInfoMsg(legionclaw::interface::NaviInfoMsg msg) override;

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Polygon2D消息发布.
   * @param[in] polygon_2d.
   * @return    void.
   */
  void PublishPolygon2D(legionclaw::interface::Polygon2D msg) override;

  /**
   * @brief     TrafficEvents消息发布.
   * @param[in] traffic_events.
   * @return    void.
   */
  void
  PublishTrafficEventsOutput(legionclaw::interface::TrafficEvents msg) override;
 
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
  rclcpp::Publisher<::ros2_interface::msg::RoutingResponse>::SharedPtr
      routing_response_pub_;
  rclcpp::Publisher<::ros2_interface::msg::GuideInfo>::SharedPtr guide_info_pub_;
  rclcpp::Publisher<::ros2_interface::msg::NaviInfoMsg>::SharedPtr navi_info_msg_pub_;
  rclcpp::Publisher<::ros2_interface::msg::LaneList>::SharedPtr lane_list_pub_;
  rclcpp::Publisher<::ros2_interface::msg::ParkingInfo>::SharedPtr
      parking_info_pub_;
  rclcpp::Publisher<::ros2_interface::msg::StopInfo>::SharedPtr stop_info_pub_;
  rclcpp::Publisher<::ros2_interface::msg::GlobalRouteMsg>::SharedPtr
      global_route_msg_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Polygon2D>::SharedPtr
      polygon_2d_pub_;
  rclcpp::Publisher<::ros2_interface::msg::TrafficEvents>::SharedPtr
      traffic_events_output_pub_;

  rclcpp::Subscription<::ros2_interface::msg::TrafficLightMsg>::SharedPtr
      traffic_light_msg_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Location>::SharedPtr
      location_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Chassis>::SharedPtr chassis_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ObuCmdMsg>::SharedPtr
      obu_cmd_msg_sub_;
  rclcpp::Subscription<::ros2_interface::msg::TrafficEvents>::SharedPtr
      traffic_events_input_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Odometry>::SharedPtr
      odometry_sub_;
  rclcpp::Subscription<::ros2_interface::msg::RoutingRequest>::SharedPtr
      routing_request_sub_;

  void HandleTrafficLightMsgMessage(
      const ros2_interface::msg::TrafficLightMsg::SharedPtr msg_obj);

  void
  HandleLocationMessage(const ros2_interface::msg::Location::SharedPtr msg_obj);

  void
  HandleChassisMessage(const ros2_interface::msg::Chassis::SharedPtr msg_obj);

  void HandleObuCmdMsgMessage(
      const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj);

  void HandleTrafficEventsInputMessage(
      const ros2_interface::msg::TrafficEvents::SharedPtr msg_obj);

  void
  HandleOdometryMessage(const ros2_interface::msg::Odometry::SharedPtr msg_obj);

  void HandleRoutingRequestMessage(
      const ros2_interface::msg::RoutingRequest::SharedPtr msg_obj);
};
} // namespace routing
} // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif // ROS2_ENABLE
