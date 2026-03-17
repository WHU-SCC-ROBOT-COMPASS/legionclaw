/**
 * @file    dds_message_manager.h
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if DDS_ENABLE

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <mutex>
#include <thread>

#include "dds_interface/ADCTrajectoryPubSubTypes.h"
#include "dds_interface/ChassisPubSubTypes.h"
#include "dds_interface/DrivableRegionPubSubTypes.h"
#include "dds_interface/EventsPubSubTypes.h"
#include "dds_interface/FaultsPubSubTypes.h"
#include "dds_interface/GuideInfoPubSubTypes.h"
#include "dds_interface/LaneListPubSubTypes.h"
#include "dds_interface/LocationPubSubTypes.h"
#include "dds_interface/ObuCmdMsgPubSubTypes.h"
#include "dds_interface/ParkingInfoPubSubTypes.h"
#include "dds_interface/ParkingOutInfoPubSubTypes.h"
#include "dds_interface/ParkingStateDisplayPubSubTypes.h"
#include "dds_interface/PlanningAnalysisPubSubTypes.h"
#include "dds_interface/PlanningCmdPubSubTypes.h"
#include "dds_interface/PredictionObstaclesPubSubTypes.h"
#include "dds_interface/RoutingResponsePubSubTypes.h"
#include "dds_interface/SotifMonitorResultPubSubTypes.h"
#include "dds_interface/StopInfoPubSubTypes.h"
#include "dds_interface/TrafficEventsPubSubTypes.h"
#include "dds_interface/TrafficLightMsgPubSubTypes.h"
#include "dds_interface/TrajectoryArrayPubSubTypes.h"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "fastdds/rtps/transport/TCPv4TransportDescriptor.h"
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "fastrtps/attributes/PublisherAttributes.h"
#include "fastrtps/attributes/SubscriberAttributes.h"
#include "fastrtps/utils/IPLocator.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
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
  void Init(T *t) override;

  bool Activate() override;
  bool DeActivate() override;

  /**
   * @brief     ADCTrajectory消息发送.
   * @param[in] adctrajectory
   * @return    void.
   */
  void PublishADCTrajectory(legionclaw::interface::ADCTrajectory msg) override;

  /**
   * @brief     PlanningCmd消息发送.
   * @param[in] planning_cmd
   * @return    void.
   */
  void PublishPlanningCmd(legionclaw::interface::PlanningCmd msg) override;

  /**
   * @brief     PlanningAnalysis消息发送.
   * @param[in] planning_analysis
   * @return    void.
   */
  void PublishPlanningAnalysis(legionclaw::interface::PlanningAnalysis msg) override;

  /**
   * @brief     ParkingStateDisplay消息发送.
   * @param[in] parking_state_display
   * @return    void.
   */
  void PublishParkingStateDisplay(
      legionclaw::interface::ParkingStateDisplay msg) override;

  /**
   * @brief     TrajectoryArray消息发送.
   * @param[in] trajectory_array
   * @return    void.
   */
  void PublishTrajectoryArray(legionclaw::interface::TrajectoryArray msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Events消息发送.
   * @param[in] events
   * @return    void.
   */
  void PublishEvents(legionclaw::interface::Events msg) override;

protected:
  T *instance_;
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

  void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;

  /**
   * @brief     线程结束函数.
   * @return    void.
   */
  void Stop();

  void TaskStop();

  void TaskStart();

  void
  HandleRoutingResponseMessage(const ros2_interface::msg::RoutingResponse *msg);

  void HandleLocalRoutingResponseMessage(
      const ros2_interface::msg::RoutingResponse *msg);

  void HandleParkingInfoMessage(const ros2_interface::msg::ParkingInfo *msg);

  void HandleStopInfoMessage(const ros2_interface::msg::StopInfo *msg);

  void
  HandleTrafficLightMsgMessage(const ros2_interface::msg::TrafficLightMsg *msg);

  void HandleLocationMessage(const ros2_interface::msg::Location *msg);

  void HandlePredictionObstaclesMessage(
      const ros2_interface::msg::PredictionObstacles *msg);

  void HandleLaneListMessage(const ros2_interface::msg::LaneList *msg);

  void HandleChassisMessage(const ros2_interface::msg::Chassis *msg);

  void HandleSotifMonitorResultMessage(
      const ros2_interface::msg::SotifMonitorResult *msg);

  void HandleObuCmdMsgMessage(const ros2_interface::msg::ObuCmdMsg *msg);

  void
  HandleDrivableRegionMessage(const ros2_interface::msg::DrivableRegion *msg);

  void
  HandleParkingOutInfoMessage(const ros2_interface::msg::ParkingOutInfo *msg);

  void HandleGuideInfoMessage(const ros2_interface::msg::GuideInfo *msg);

  void
  HandleTrafficEventsMessage(const ros2_interface::msg::TrafficEvents *msg);

private:
  ros2_interface::msg::ADCTrajectory adc_trajectory;
  ros2_interface::msg::PlanningCmd planning_cmd;
  ros2_interface::msg::PlanningAnalysis planning_analysis;
  ros2_interface::msg::ParkingStateDisplay parking_state_display;
  ros2_interface::msg::TrajectoryArray trajectory_array;
  ros2_interface::msg::Faults faults;
  ros2_interface::msg::Events events;
  eprosima::fastdds::dds::DomainParticipant *participant_;
  eprosima::fastdds::dds::Publisher *publisher_;

  eprosima::fastdds::dds::DataWriter *adc_trajectory_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> adc_trajectory_type_;
  eprosima::fastdds::dds::Topic *adc_trajectory_topic_;
  eprosima::fastdds::dds::DataWriter *planning_cmd_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> planning_cmd_type_;
  eprosima::fastdds::dds::Topic *planning_cmd_topic_;
  eprosima::fastdds::dds::DataWriter *planning_analysis_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> planning_analysis_type_;
  eprosima::fastdds::dds::Topic *planning_analysis_topic_;
  eprosima::fastdds::dds::DataWriter *parking_state_display_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      parking_state_display_type_;
  eprosima::fastdds::dds::Topic *parking_state_display_topic_;
  eprosima::fastdds::dds::DataWriter *trajectory_array_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> trajectory_array_type_;
  eprosima::fastdds::dds::Topic *trajectory_array_topic_;
  eprosima::fastdds::dds::DataWriter *faults_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> faults_type_;
  eprosima::fastdds::dds::Topic *faults_topic_;
  eprosima::fastdds::dds::DataWriter *events_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> events_type_;
  eprosima::fastdds::dds::Topic *events_topic_;

  eprosima::fastdds::dds::Subscriber *subscriber_;
  eprosima::fastdds::dds::Subscriber *command_subscriber_;
  eprosima::fastdds::dds::SampleInfo info_;

  eprosima::fastdds::dds::DataReader *routing_response_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> routing_response_type_;
  eprosima::fastdds::dds::Topic *routing_response_topic_;
  eprosima::fastdds::dds::DataReader *local_routing_response_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      local_routing_response_type_;
  eprosima::fastdds::dds::Topic *local_routing_response_topic_;
  eprosima::fastdds::dds::DataReader *parking_info_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> parking_info_type_;
  eprosima::fastdds::dds::Topic *parking_info_topic_;
  eprosima::fastdds::dds::DataReader *stop_info_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> stop_info_type_;
  eprosima::fastdds::dds::Topic *stop_info_topic_;
  eprosima::fastdds::dds::DataReader *traffic_light_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> traffic_light_msg_type_;
  eprosima::fastdds::dds::Topic *traffic_light_msg_topic_;
  eprosima::fastdds::dds::DataReader *location_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> location_type_;
  eprosima::fastdds::dds::Topic *location_topic_;
  eprosima::fastdds::dds::DataReader *prediction_obstacles_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      prediction_obstacles_type_;
  eprosima::fastdds::dds::Topic *prediction_obstacles_topic_;
  eprosima::fastdds::dds::DataReader *lane_list_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> lane_list_type_;
  eprosima::fastdds::dds::Topic *lane_list_topic_;
  eprosima::fastdds::dds::DataReader *chassis_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> chassis_type_;
  eprosima::fastdds::dds::Topic *chassis_topic_;
  eprosima::fastdds::dds::DataReader *sotif_monitor_result_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      sotif_monitor_result_type_;
  eprosima::fastdds::dds::Topic *sotif_monitor_result_topic_;
  eprosima::fastdds::dds::DataReader *obu_cmd_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> obu_cmd_msg_type_;
  eprosima::fastdds::dds::Topic *obu_cmd_msg_topic_;
  eprosima::fastdds::dds::DataReader *drivable_region_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> drivable_region_type_;
  eprosima::fastdds::dds::Topic *drivable_region_topic_;
  eprosima::fastdds::dds::DataReader *parking_out_info_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> parking_out_info_type_;
  eprosima::fastdds::dds::Topic *parking_out_info_topic_;
  eprosima::fastdds::dds::DataReader *guide_info_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> guide_info_type_;
  eprosima::fastdds::dds::Topic *guide_info_topic_;
  eprosima::fastdds::dds::DataReader *traffic_events_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> traffic_events_type_;
  eprosima::fastdds::dds::Topic *traffic_events_topic_;
};
} // namespace planning
} // namespace legionclaw
#include "dds_message_manager.hpp"
#endif