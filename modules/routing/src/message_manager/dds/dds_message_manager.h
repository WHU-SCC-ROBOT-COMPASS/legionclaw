/**
 * @file    dds_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if DDS_ENABLE

#include <mutex>
#include <thread>
#include <fastdds/dds/core/policy/QosPolicies.hpp>

#include "fastrtps/utils/IPLocator.h"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "dds_interface/FaultsPubSubTypes.h"
#include "dds_interface/ChassisPubSubTypes.h"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "dds_interface/LocationPubSubTypes.h"
#include "dds_interface/OdometryPubSubTypes.h"
#include "dds_interface/LaneListPubSubTypes.h"
#include "dds_interface/NaviInfoMsgPubSubTypes.h"
#include "dds_interface/StopInfoPubSubTypes.h"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "dds_interface/ObuCmdMsgPubSubTypes.h"
#include "dds_interface/GuideInfoPubSubTypes.h"
#include "dds_interface/Polygon2DPubSubTypes.h"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "dds_interface/ParkingInfoPubSubTypes.h"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "dds_interface/TrafficEventsPubSubTypes.h"
#include "fastrtps/attributes/PublisherAttributes.h"
#include "dds_interface/RoutingRequestPubSubTypes.h"
#include "dds_interface/GlobalRouteMsgPubSubTypes.h"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastrtps/attributes/SubscriberAttributes.h"
#include "dds_interface/TrafficLightMsgPubSubTypes.h"
#include "dds_interface/RoutingResponsePubSubTypes.h"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/rtps/transport/TCPv4TransportDescriptor.h"
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
using namespace legionclaw::common;
/**
 * @class DdsMessageManager
 * @brief DDS消息管理器.
 */
template <typename T>
class DdsMessageManager : public MessageManager<T>,
                          public eprosima::fastdds::dds::DataReaderListener {
public:
  DdsMessageManager() = default;
  virtual ~DdsMessageManager();

  /**
   * @brief     初始化。
   * @param[in] obu_url DDS组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  bool Activate() override;
  bool DeActivate() override;

  /**
   * @brief     RoutingResponse消息发送.
   * @param[in] routing_response
   * @return    void.
   */
  void PublishRoutingResponse(legionclaw::interface::RoutingResponse msg) override;

  /**
   * @brief     GuideInfo消息发送.
   * @param[in] guide_info
   * @return    void.
   */
  void PublishGuideInfo(legionclaw::interface::GuideInfo msg) override;

  /**
   * @brief     LaneList消息发送.
   * @param[in] lane_list
   * @return    void.
   */
  void PublishLaneList(legionclaw::interface::LaneList msg) override;

  /**
   * @brief     NaviInfoMsg消息发送.
   * @param[in] navi_info_msg
   * @return    void.
   */
  void PublishNaviInfoMsg(legionclaw::interface::NaviInfoMsg msg) override;

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
   * @brief     TrafficLightMsg消息发送.
   * @param[in] traffic_light_msg
   * @return    void.
   */
  // void PublishTrafficLightMsgOutput(legionclaw::interface::TrafficLightMsg msg) override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;

  std::unique_ptr<std::thread> handle_message_thread_;

protected:
  std::mutex r_mutex_;
  std::mutex mode_mutex_;

  /**
   * @brief     线程运行函数.
   * @return    void.
   */
  void Run();

  void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;

  /**
   * @brief     线程结束函数.
   * @return    void.
   */
  void Stop();

  void TaskStop();

  void TaskStart();

  void
  HandleTrafficLightMsgMessage(const ros2_interface::msg::TrafficLightMsg* msg);

  void HandleLocationMessage(const ros2_interface::msg::Location* msg);

  void HandleChassisMessage(const ros2_interface::msg::Chassis* msg);

  void HandleObuCmdMsgMessage(const ros2_interface::msg::ObuCmdMsg* msg);

  void HandleSpeedLimitMessage(const ros2_interface::msg::TrafficEvents* msg);

  void HandleMap2LocalTFMessage(const ros2_interface::msg::Odometry* msg);

  void
  HandleRoutingRequestMessage(const ros2_interface::msg::RoutingRequest* msg);

private:
  ros2_interface::msg::RoutingResponse routing_response;
  ros2_interface::msg::GuideInfo guide_info;
  ros2_interface::msg::NaviInfoMsg navi_info_msg;
  ros2_interface::msg::LaneList lane_list;
  ros2_interface::msg::ParkingInfo parking_info;
  ros2_interface::msg::StopInfo stop_info;
  ros2_interface::msg::GlobalRouteMsg global_route_msg;
  ros2_interface::msg::Faults faults;
  ros2_interface::msg::Polygon2D map_boundary;
  ros2_interface::msg::TrafficEvents traffic_events;
  ros2_interface::msg::TrafficLightMsg traffic_light_msg;
  eprosima::fastdds::dds::DomainParticipant *participant_;
  eprosima::fastdds::dds::Publisher *publisher_;

  eprosima::fastdds::dds::DataWriter* routing_response_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> routing_response_type_;
  eprosima::fastdds::dds::Topic* routing_response_topic_;
  eprosima::fastdds::dds::DataWriter *guide_info_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> guide_info_type_;
  eprosima::fastdds::dds::Topic* guide_info_topic_;
  eprosima::fastdds::dds::DataWriter *navi_info_msg_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> navi_info_msg_type_;
  eprosima::fastdds::dds::Topic *navi_info_msg_topic_;
  eprosima::fastdds::dds::DataWriter *lane_list_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> lane_list_type_;
  eprosima::fastdds::dds::Topic* lane_list_topic_;
  eprosima::fastdds::dds::DataWriter* parking_info_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> parking_info_type_;
  eprosima::fastdds::dds::Topic* parking_info_topic_;
  eprosima::fastdds::dds::DataWriter* stop_info_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> stop_info_type_;
  eprosima::fastdds::dds::Topic* stop_info_topic_;
  eprosima::fastdds::dds::DataWriter* global_route_msg_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> global_route_msg_type_;
  eprosima::fastdds::dds::Topic* global_route_msg_topic_;
  eprosima::fastdds::dds::DataWriter* faults_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> faults_type_;
  eprosima::fastdds::dds::Topic* faults_topic_;
  eprosima::fastdds::dds::DataWriter* map_boundary_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> map_boundary_type_;
  eprosima::fastdds::dds::Topic* map_boundary_topic_;
  eprosima::fastdds::dds::DataWriter* traffic_events_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> traffic_events_type_;
  eprosima::fastdds::dds::Topic* traffic_events_topic_;

  eprosima::fastdds::dds::DataWriter *traffic_light_msg_writer_;
  // std::unique_ptr<eprosima::fastdds::dds::TypeSupport> traffic_light_msg_type_;
  // eprosima::fastdds::dds::Topic* traffic_light_msg_topic_;

  eprosima::fastdds::dds::Subscriber* subscriber_;
  eprosima::fastdds::dds::Subscriber* command_subscriber_;
  eprosima::fastdds::dds::SampleInfo info_;

  eprosima::fastdds::dds::DataReader* traffic_light_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> traffic_light_msg_type_;
  eprosima::fastdds::dds::Topic* traffic_light_msg_topic_;
  eprosima::fastdds::dds::DataReader* location_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> location_type_;
  eprosima::fastdds::dds::Topic* location_topic_;
  eprosima::fastdds::dds::DataReader* chassis_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> chassis_type_;
  eprosima::fastdds::dds::Topic* chassis_topic_;
  eprosima::fastdds::dds::DataReader* obu_cmd_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> obu_cmd_msg_type_;
  eprosima::fastdds::dds::Topic* obu_cmd_msg_topic_;
  eprosima::fastdds::dds::DataReader* speed_limit_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> speed_limit_type_;
  eprosima::fastdds::dds::Topic* speed_limit_topic_;
  eprosima::fastdds::dds::DataReader* map_2local_tf_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> map_2local_tf_type_;
  eprosima::fastdds::dds::Topic* map_2local_tf_topic_;
  eprosima::fastdds::dds::DataReader* routing_request_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> routing_request_type_;
  eprosima::fastdds::dds::Topic* routing_request_topic_;
};
} // namespace routing
} // namespace legionclaw
#include "dds_message_manager.hpp"
#endif