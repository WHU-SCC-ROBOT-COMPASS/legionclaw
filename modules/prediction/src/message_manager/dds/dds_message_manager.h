/**
 * @file    dds_message_manager.h
 * @author  legionclaw
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
#include "dds_interface/ObuCmdMsgPubSubTypes.h"
#include "fastrtps/utils/IPLocator.h"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "dds_interface/FaultsPubSubTypes.h"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "dds_interface/LocationPubSubTypes.h"
#include "dds_interface/OdometryPubSubTypes.h"
#include "dds_interface/LaneListPubSubTypes.h"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "dds_interface/ObstacleListPubSubTypes.h"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "dds_interface/ADCTrajectoryPubSubTypes.h"
#include "dds_interface/TrafficEventsPubSubTypes.h"
#include "fastrtps/attributes/PublisherAttributes.h"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastrtps/attributes/SubscriberAttributes.h"
#include "dds_interface/RoutingResponsePubSubTypes.h"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "dds_interface/PredictionObstaclesPubSubTypes.h"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/rtps/transport/TCPv4TransportDescriptor.h"
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
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

  /**
   * @brief     PredictionObstacles消息发送.
   * @param[in] prediction_obstacles
   * @return    void.
   */
  void PublishPredictionObstacles(
      legionclaw::interface::PredictionObstacles msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
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

  void HandleLocationMessage(const ros2_interface::msg::Location* msg);

  void
  HandleADCTrajectoryMessage(const ros2_interface::msg::ADCTrajectory* msg);

  void
  HandleMMObstacleListMessage(const ros2_interface::msg::ObstacleList* msg);

  void HandleMap2LocalTFMessage(const ros2_interface::msg::Odometry* msg);

  void HandleTrafficEventsLocalMessage(
      const ros2_interface::msg::TrafficEvents* msg);

  void HandleLocalRoutingResponseMessage(
      const ros2_interface::msg::RoutingResponse* msg);

  void HandleMFLaneListMessage(const ros2_interface::msg::LaneList* msg);

  void HandleObuCmdMsgMessage(const ros2_interface::msg::ObuCmdMsg* msg);

private:
  ros2_interface::msg::PredictionObstacles prediction_obstacles;
  ros2_interface::msg::Faults faults;
  eprosima::fastdds::dds::DomainParticipant* participant_;
  eprosima::fastdds::dds::Publisher* publisher_;
  eprosima::fastdds::dds::Topic* topic_;

  eprosima::fastdds::dds::DataWriter* prediction_obstacles_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      prediction_obstacles_type_;
  eprosima::fastdds::dds::Topic* prediction_obstacles_topic_;

  eprosima::fastdds::dds::DataWriter* faults_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> faults_type_;
  eprosima::fastdds::dds::Topic* faults_topic_;

  eprosima::fastdds::dds::Subscriber* subscriber_;
  eprosima::fastdds::dds::Subscriber* command_subscriber_;
  eprosima::fastdds::dds::SampleInfo info_;

  eprosima::fastdds::dds::DataReader* location_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> location_type_;
  eprosima::fastdds::dds::Topic* location_topic_;

  eprosima::fastdds::dds::DataReader* adc_trajectory_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> adc_trajectory_type_;
  eprosima::fastdds::dds::Topic* adc_trajectory_topic_;
  
  eprosima::fastdds::dds::DataReader* mm_obstacle_list_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> mm_obstacle_list_type_;
  eprosima::fastdds::dds::Topic* mm_obstacle_list_topic_;

  eprosima::fastdds::dds::DataReader* map_2local_tf_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> map_2local_tf_type_;
  eprosima::fastdds::dds::Topic* map_2local_tf_topic_;

  eprosima::fastdds::dds::DataReader* traffic_events_local_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      traffic_events_local_type_;
  eprosima::fastdds::dds::Topic* traffic_events_local_topic_;

  eprosima::fastdds::dds::DataReader* local_routing_response_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      local_routing_response_type_;
  eprosima::fastdds::dds::Topic* local_routing_response_topic_;

  eprosima::fastdds::dds::DataReader* mf_lane_list_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> mf_lane_list_type_;
  eprosima::fastdds::dds::Topic* mf_lane_list_topic_;

  eprosima::fastdds::dds::DataReader* obu_cmd_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> obu_cmd_msg_type_;
  eprosima::fastdds::dds::Topic* obu_cmd_msg_topic_;

};
} // namespace prediction
} // namespace legionclaw
#include "dds_message_manager.hpp"
#endif
