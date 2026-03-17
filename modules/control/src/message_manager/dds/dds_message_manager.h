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
#include "dds_interface/EventsPubSubTypes.h"
#include "dds_interface/ChassisPubSubTypes.h"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "dds_interface/LocationPubSubTypes.h"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "dds_interface/PlanningCmdPubSubTypes.h"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "dds_interface/ADCTrajectoryPubSubTypes.h"
#include "fastrtps/attributes/PublisherAttributes.h"
#include "dds_interface/ControlCommandPubSubTypes.h"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastrtps/attributes/SubscriberAttributes.h"
#include "dds_interface/ControlAnalysisPubSubTypes.h"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastrtps/attributes/ParticipantAttributes.h"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/rtps/transport/TCPv4TransportDescriptor.h"
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"

#include "message_manager/message_manager.h"
#include "dds_interface/ObuCmdMsgPubSubTypes.h"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */

namespace legionclaw {
namespace control {
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
   * @brief     ControlCommand消息发送.
   * @param[in] control_command
   * @return    void.
   */
  void PublishControlCommand(legionclaw::interface::ControlCommand msg) override;

  /**
   * @brief     ControlAnalysis消息发送.
   * @param[in] control_analysis
   * @return    void.
   */
  void PublishControlAnalysis(legionclaw::interface::ControlAnalysis msg) override;

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

  void
  HandleADCTrajectoryMessage(const ros2_interface::msg::ADCTrajectory* msg);

  void HandleChassisMessage(const ros2_interface::msg::Chassis* msg);

  void HandleLocationMessage(const ros2_interface::msg::Location* msg);

  void HandlePlanningCmdMessage(const ros2_interface::msg::PlanningCmd* msg);

  void HandleObuCmdMsgMessage(const ros2_interface::msg::ObuCmdMsg* msg);

private:
  ros2_interface::msg::ControlCommand control_command;
  ros2_interface::msg::ControlAnalysis control_analysis;
  ros2_interface::msg::Faults faults;
  ros2_interface::msg::Events events;
  eprosima::fastdds::dds::DomainParticipant* participant_;
  eprosima::fastdds::dds::Publisher* publisher_;
  eprosima::fastdds::dds::Topic* topic_;

  eprosima::fastdds::dds::DataWriter* control_command_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> control_command_type_;
  eprosima::fastdds::dds::Topic* control_command_topic_;

  eprosima::fastdds::dds::DataWriter* control_analysis_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> control_analysis_type_;
  eprosima::fastdds::dds::Topic* control_analysis_topic_;
  
  eprosima::fastdds::dds::DataWriter* faults_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> faults_type_;
  eprosima::fastdds::dds::Topic* faults_topic_;

  eprosima::fastdds::dds::DataWriter* events_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> events_type_;
  eprosima::fastdds::dds::Topic* events_topic_;

  eprosima::fastdds::dds::Subscriber* subscriber_;
  eprosima::fastdds::dds::Subscriber* command_subscriber_;
  eprosima::fastdds::dds::SampleInfo info_;

  eprosima::fastdds::dds::DataReader* adc_trajectory_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> adc_trajectory_type_;
  eprosima::fastdds::dds::Topic* adc_trajectory_topic_;

  eprosima::fastdds::dds::DataReader* chassis_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> chassis_type_;
  eprosima::fastdds::dds::Topic* chassis_topic_;

  eprosima::fastdds::dds::DataReader* location_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> location_type_;
  eprosima::fastdds::dds::Topic* location_topic_;

  eprosima::fastdds::dds::DataReader* planning_cmd_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> planning_cmd_type_;
  eprosima::fastdds::dds::Topic* planning_cmd_topic_;

  eprosima::fastdds::dds::DataReader* obu_cmd_msg_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> obu_cmd_msg_type_;
  eprosima::fastdds::dds::Topic* obu_cmd_msg_topic_;
};
} // namespace control
} // namespace legionclaw
#include "dds_message_manager.hpp"
#endif
