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
#include "ros_interface/Chassis.h"
#include "ros_interface/Location.h"
#include "ros_interface/Odometry.h"
#include "ros_interface/LaneList.h"
#include "ros_interface/StopInfo.h"
#include "ros_interface/ObuCmdMsg.h"
#include "ros_interface/GuideInfo.h"
#include "ros_interface/Polygon2D.h"
#include "ros_interface/ParkingInfo.h"
#include "ros_interface/TrafficEvents.h"
#include "ros_interface/RoutingRequest.h"
#include "ros_interface/GlobalRouteMsg.h"
#include "ros_interface/TrafficLightMsg.h"
#include "ros_interface/RoutingResponse.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
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
  ros::Publisher routing_response_pub_;
  ros::Publisher guide_info_pub_;
  ros::Publisher lane_list_pub_;
  ros::Publisher parking_info_pub_;
  ros::Publisher stop_info_pub_;
  ros::Publisher global_route_msg_pub_;
  ros::Publisher faults_pub_;
  ros::Publisher polygon_2d_pub_;
  ros::Publisher traffic_events_output_pub_;

  ros::Subscriber traffic_light_msg_sub_;
  ros::Subscriber location_sub_;
  ros::Subscriber chassis_sub_;
  ros::Subscriber obu_cmd_msg_sub_;
  ros::Subscriber traffic_events_input_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber routing_request_sub_;

  void
  HandleTrafficLightMsgMessage(const ros_interface::TrafficLightMsg& msg_obj);

  void HandleLocationMessage(const ros_interface::Location& msg_obj);

  void HandleChassisMessage(const ros_interface::Chassis& msg_obj);

  void HandleObuCmdMsgMessage(const ros_interface::ObuCmdMsg& msg_obj);

  void
  HandleTrafficEventsInputMessage(const ros_interface::TrafficEvents& msg_obj);

  void HandleOdometryMessage(const ros_interface::Odometry& msg_obj);

  void
  HandleRoutingRequestMessage(const ros_interface::RoutingRequest& msg_obj);
};
} // namespace routing
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
