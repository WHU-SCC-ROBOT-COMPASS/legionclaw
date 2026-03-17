/**
 * @file    lcm_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if LCM_ENABLE
#include <thread>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_interface/Faults.hpp"
#include "lcm_interface/Chassis.hpp"
#include "lcm_interface/Location.hpp"
#include "lcm_interface/Odometry.hpp"
#include "lcm_interface/StopInfo.hpp"
#include "lcm_interface/ObuCmdMsg.hpp"
#include "lcm_interface/Polygon2D.hpp"
#include "lcm_interface/ParkingInfo.hpp"
#include "lcm_interface/TrafficEvents.hpp"
#include "lcm_interface/RoutingRequest.hpp"
#include "lcm_interface/GlobalRouteMsg.hpp"
#include "lcm_interface/TrafficLightMsg.hpp"
#include "lcm_interface/RoutingResponse.hpp"
#include "lcm_interface/GuideInfo.hpp"
#include "lcm_interface/CurvatureInfo.hpp"
#include "lcm_interface/GuideRoad.hpp"
#include "lcm_interface/LaneList.hpp"
#include "lcm_interface/NaviInfoMsg.hpp"
#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
/**
 * @class LcmMessageManager
 * @brief LCM消息管理器.
 */
template <typename T> class LcmMessageManager : public MessageManager<T> {
public:
  LcmMessageManager() = default;
  ~LcmMessageManager() = default;

  /**
   * @brief     初始化。
   * @param[in] obu_url LCM组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  /**
   * @brief     RoutingResponse消息发送.
   * @param[in] routing_response
   * @return    void.
   */
  void PublishRoutingResponse(legionclaw::interface::RoutingResponse msg) override;

  /**
   * @brief     ParkingInfo消息发送.
   * @param[in] parking_info
   * @return    void.
   */
  void PublishParkingInfo(legionclaw::interface::ParkingInfo msg) override;

  /**
   * @brief     StopInfo消息发送.
   * @param[in] stop_info
   * @return    void.
   */
  void PublishStopInfo(legionclaw::interface::StopInfo msg) override;

  /**
   * @brief     GlobalRouteMsg消息发送.
   * @param[in] global_route_msg
   * @return    void.
   */
  void PublishGlobalRouteMsg(legionclaw::interface::GlobalRouteMsg msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Polygon2D消息发送.
   * @param[in] polygon_2d
   * @return    void.
   */
  void PublishPolygon2D(legionclaw::interface::Polygon2D msg) override;

  /**
   * @brief     TrafficEvents消息发送.
   * @param[in] traffic_events
   * @return    void.
   */
  void PublishTrafficEventsOutput(legionclaw::interface::TrafficEvents msg) override;

  /**
   * @brief     GuideInfo消息发送.
   * @param[in] guide_info
   * @return    void.
   */
  void PublishGuideInfo(legionclaw::interface::GuideInfo msg) override;

  /**
   * @brief     NaviInfoMsg消息发送.
   * @param[in] navi_info_msg
   * @return    void.
   */
  void PublishNaviInfoMsg(legionclaw::interface::NaviInfoMsg msg) override;
  
  /**
   * @brief     TrafficLightMsg消息发送.
   * @param[in] traffic_light_msg
   * @return    void.
   */
  // void PublishTrafficLightMsgOutput(legionclaw::interface::TrafficLightMsg msg) override;

  /**
   * @brief     LaneList消息发送.
   * @param[in] lane_list
   * @return    void.
   */
  void PublishLaneList(legionclaw::interface::LaneList msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  std::shared_ptr<lcm::LCM> lcm_;
  std::mutex mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

  lcm::Subscription* traffic_light_msg_sub_;
  void HandleTrafficLightMsgMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::TrafficLightMsg* msg);

  lcm::Subscription* location_sub_;
  void HandleLocationMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Location* msg);

  lcm::Subscription* chassis_sub_;
  void HandleChassisMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcm_interface::Chassis* msg);

  lcm::Subscription* obu_cmd_msg_sub_;
  void HandleObuCmdMsgMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const lcm_interface::ObuCmdMsg* msg);

  lcm::Subscription* traffic_events_input_sub_;
  void HandleTrafficEventsInputMessage(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& chan,
                                       const lcm_interface::TrafficEvents* msg);

  lcm::Subscription* odometry_sub_;
  void HandleOdometryMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Odometry* msg);

  lcm::Subscription* routing_request_sub_;
  void HandleRoutingRequestMessage(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const lcm_interface::RoutingRequest* msg);

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
};
} // namespace routing
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif