/**
 * @file    dds_message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if DDS_ENABLE

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <mutex>
#include <thread>

#include "dds_interface/ObstacleListPubSubTypes.h"
#include "dds_interface/PointCloudPubSubTypes.h"
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
 * @namespace athena::perception::lidar
 * @brief athena::perception::lidar
 */

namespace athena {
namespace perception {
namespace lidar {
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

  /**
   * @brief     ObstacleList消息发送.
   * @param[in] obstacle_list
   * @return    void.
   */
  void PublishObstacleList(athena::interface::ObstacleList msg) override;

protected:
  T *instance_;
  bool is_init_;

  std::unique_ptr<std::thread> handle_message_thread_;

protected:
  std::mutex r_mutex_;

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

  void HandlePointCloud_TopMessage(const ros2_interface::msg::PointCloud *msg);

private:
  ros2_interface::msg::ObstacleList lidar_obstacle_list;
  eprosima::fastdds::dds::DomainParticipant *participant_;
  eprosima::fastdds::dds::Publisher *publisher_;
  eprosima::fastdds::dds::Topic *topic_;

  eprosima::fastdds::dds::DataWriter *lidar_obstacle_list_writer_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport>
      lidar_obstacle_list_type_;

  eprosima::fastdds::dds::Subscriber *subscriber_;
  eprosima::fastdds::dds::SampleInfo info_;

  eprosima::fastdds::dds::DataReader *point_cloud_top_reader_;
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> point_cloud_top_type_;
};
} // namespace lidar
} // namespace perception
} // namespace athena
#include "dds_message_manager.hpp"
#endif