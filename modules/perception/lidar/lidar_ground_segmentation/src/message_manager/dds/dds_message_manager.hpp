/**
 * @file    dds_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "dds_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include "modules/common/math/euler_angles_zxy.h"

#if DDS_ENABLE
/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;
template <typename T> void DdsMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  action_mode_ = MessageActionMode::DO_NOTHING;
  instance_ = t;

  DomainParticipantQos pqos;
  auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
  udp_transport->sendBufferSize = 0;
  udp_transport->receiveBufferSize = 0;
  udp_transport->TTL = 3;
  udp_transport->non_blocking_send = true;
  // Link the Transport Layer to the Participant.
  pqos.transport().user_transports.push_back(udp_transport);
  // Avoid using the default transport
  pqos.transport().use_builtin_transports = false;
  pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
      DiscoveryProtocol_t::SIMPLE;
  pqos.wire_protocol()
      .builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
  pqos.wire_protocol()
      .builtin.discovery_config.m_simpleEDP
      .use_PublicationReaderANDSubscriptionWriter = true;
  pqos.wire_protocol()
      .builtin.discovery_config.m_simpleEDP
      .use_PublicationWriterANDSubscriptionReader = true;
  pqos.name("Participant_pub");
  participant_ =
      DomainParticipantFactory::get_instance()->create_participant(200, pqos);
  if (participant_ == nullptr) {
    return;
  }
  DataWriterQos wqos;
  wqos.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos.history().depth = 10;
  wqos.resource_limits().max_samples = 10000;
  // wqos.resource_limits().max_instances = 400;
  // wqos.resource_limits().allocated_samples = 100;
  wqos.reliable_writer_qos().times.heartbeatPeriod.seconds = 1;
  wqos.reliable_writer_qos().times.heartbeatPeriod.fraction(0);
  wqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  wqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;
  wqos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
  // PointCloud
  ground_points_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::PointCloudPubSubType()));
  ground_points_type_->register_type(participant_);
  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  ground_points_topic_ = participant_->create_topic(
      "rt/perception/lidar/lidar_ground_segmentation/GroundPoints",
      ground_points_type_->get_type_name(), TOPIC_QOS_DEFAULT);
  ground_points_writer_ =
      publisher_->create_datawriter(ground_points_topic_, wqos, nullptr);

  // PointCloud
  no_ground_points_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::PointCloudPubSubType()));
  no_ground_points_type_->register_type(participant_);
  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  no_ground_points_topic_ = participant_->create_topic(
      "rt/perception/lidar/lidar_ground_segmentation/NoGroundPoints",
      no_ground_points_type_->get_type_name(), TOPIC_QOS_DEFAULT);
  no_ground_points_writer_ =
      publisher_->create_datawriter(no_ground_points_topic_, wqos, nullptr);

  // Faults
  faults_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::FaultsPubSubType()));
  faults_type_->register_type(participant_);
  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  faults_topic_ = participant_->create_topic(
      "rt/perception/lidar/lidar_ground_segmentation/Faults",
      faults_type_->get_type_name(), TOPIC_QOS_DEFAULT);
  faults_writer_ = publisher_->create_datawriter(faults_topic_, wqos, nullptr);

  DataReaderQos rqos;
  rqos.history().kind = KEEP_LAST_HISTORY_QOS;
  rqos.history().depth = 10;
  rqos.resource_limits().max_samples = 10000;
  rqos.reliable_reader_qos().times.heartbeatResponseDelay.seconds = 1;
  rqos.reliable_reader_qos().times.heartbeatResponseDelay.fraction(0);
  // rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  rqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;

  command_subscriber_ =
      participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
  subscriber_ =
      participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

  mlc_point_cloud_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::PointCloudPubSubType()));
  mlc_point_cloud_type_->register_type(participant_);
  mlc_point_cloud_topic_ = participant_->create_topic(
      "rt/perception/lidar/multi_lidar_concate/MLCPointCloud",
      mlc_point_cloud_type_->get_type_name(), TOPIC_QOS_DEFAULT);

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool DdsMessageManager<T>::Activate() {
  std::unique_lock<std::mutex> lock(mode_mutex_);
  if (is_active_ == true) {
    std::cout << "already start" << std::endl;
    action_mode_ = MessageActionMode::DO_NOTHING;
    return false;
  } else {
    action_mode_ = MessageActionMode::TO_ACTIVATE;
  }
  return true;
}

template <typename T> bool DdsMessageManager<T>::DeActivate() {
  std::unique_lock<std::mutex> lock(mode_mutex_);
  if (is_active_ == false) {
    std::cout << "already stop" << std::endl;
    action_mode_ = MessageActionMode::DO_NOTHING;
    return false;
  } else {
    action_mode_ = MessageActionMode::TO_DEACTIVATE;
  }
  return true;
}

template <typename T> void DdsMessageManager<T>::TaskStart() {
  if (is_active_ == true) {
    return;
  }
  DataReaderQos rqos;
  rqos.history().kind = KEEP_LAST_HISTORY_QOS;
  rqos.history().depth = 30;
  rqos.resource_limits().max_samples = 5000;
  rqos.resource_limits().allocated_samples = 400;
  rqos.resource_limits().max_instances = 100;
  // rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  rqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;
  mlc_point_cloud_reader_ =
      subscriber_->create_datareader(mlc_point_cloud_topic_, rqos, this);
  std::cout << "dds activate" << std::endl;
  action_mode_ = MessageActionMode::DO_NOTHING;
  is_active_ = true;
  return;
}

template <typename T> void DdsMessageManager<T>::TaskStop() {
  // std::lock_guard<std::mutex> lock(r_mutex_);
  if (is_active_ == false) {
    return;
  }
  subscriber_->delete_contained_entities();
  std::cout << "dds deactivate" << std::endl;
  action_mode_ = MessageActionMode::DO_NOTHING;
  is_active_ = false;
  return;
}
template <typename T>
void DdsMessageManager<T>::PublishGroundPoints(
    legionclaw::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  ros2_interface::msg::PointCloud point_cloud;
  MESSAGE_DDS_HEADER_ASSIGN(ros2_interface::msg, point_cloud)
  point_cloud.frame_id() = msg.frame_id();
  point_cloud.is_dense() = msg.is_dense();
  std::vector<ros2_interface::msg::PointXYZIRT> dds_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ros2_interface::msg::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x() = it_point.x();
    point_cloud_point_xyzirt.y() = it_point.y();
    point_cloud_point_xyzirt.z() = it_point.z();
    point_cloud_point_xyzirt.intensity() = it_point.intensity();
    point_cloud_point_xyzirt.ring_id() = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp() = it_point.timestamp();
    dds_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point() = dds_point;
  point_cloud.measurement_time() = msg.measurement_time();
  point_cloud.width() = msg.width();
  point_cloud.height() = msg.height();

  ground_points_writer_->write(&point_cloud);
}

template <typename T>
void DdsMessageManager<T>::PublishNoGroundPoints(
    legionclaw::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  ros2_interface::msg::PointCloud point_cloud;
  MESSAGE_DDS_HEADER_ASSIGN(ros2_interface::msg, point_cloud)
  point_cloud.frame_id() = msg.frame_id();
  point_cloud.is_dense() = msg.is_dense();
  std::vector<ros2_interface::msg::PointXYZIRT> dds_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ros2_interface::msg::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x() = it_point.x();
    point_cloud_point_xyzirt.y() = it_point.y();
    point_cloud_point_xyzirt.z() = it_point.z();
    point_cloud_point_xyzirt.intensity() = it_point.intensity();
    point_cloud_point_xyzirt.ring_id() = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp() = it_point.timestamp();
    dds_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point() = dds_point;
  point_cloud.measurement_time() = msg.measurement_time();
  point_cloud.width() = msg.width();
  point_cloud.height() = msg.height();

  no_ground_points_writer_->write(&point_cloud);
}

template <typename T>
void DdsMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  MESSAGE_DDS_HEADER_ASSIGN(ros2_interface::msg, faults)
  DDS_FAULTS_PARSER(ros2, faults)

  faults_writer_->write(&faults);
}

template <typename T>
void DdsMessageManager<T>::HandleMLCPointCloudMessage(
    const ros2_interface::msg::PointCloud* msg) {
  legionclaw::interface::PointCloud point_cloud;
  MESSAGE_DDS_HEADER_PARSER(point_cloud)
  point_cloud.set_frame_id(msg->frame_id());
  point_cloud.set_is_dense(msg->is_dense());
  std::vector<legionclaw::interface::PointXYZIRT> point;
  for (auto it_point : msg->point()) {
    legionclaw::interface::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.set_x(it_point.x());
    point_cloud_point_xyzirt.set_y(it_point.y());
    point_cloud_point_xyzirt.set_z(it_point.z());
    point_cloud_point_xyzirt.set_intensity(it_point.intensity());
    point_cloud_point_xyzirt.set_ring_id(it_point.ring_id());
    point_cloud_point_xyzirt.set_timestamp(it_point.timestamp());
    point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.set_point(&point);
  point_cloud.set_measurement_time(msg->measurement_time());
  point_cloud.set_width(msg->width());
  point_cloud.set_height(msg->height());

  instance_->HandlePointCloudInput(point_cloud);
}

template <typename T>
void DdsMessageManager<T>::on_data_available(
    eprosima::fastdds::dds::DataReader* reader) {
  std::lock_guard<std::mutex> lock(r_mutex_);
  if (is_init_ == false)
    return;
  if (!reader->get_topicdescription()->get_name().compare(
          "rt/vui_client/ObuCmdMsg")) {

    ros2_interface::msg::ObuCmdMsg msg;
    eprosima::fastdds::dds::SampleInfo info;
    if (obu_cmd_msg_reader_->take_next_sample(&msg, &info) ==
        ReturnCode_t::RETCODE_OK) {
      HandleObuCmdMsgMessage(&msg);
    }
    return;
  }

  if (is_active_ == false) {
    return;
  }
  if (!reader->get_topicdescription()->get_name().compare(
          "rt/perception/lidar/multi_lidar_concate/MLCPointCloud")) {

    ros2_interface::msg::PointCloud msg;
    eprosima::fastdds::dds::SampleInfo info;
    if (mlc_point_cloud_reader_->take_next_sample(&msg, &info) ==
        ReturnCode_t::RETCODE_OK) {
      HandleMLCPointCloudMessage(&msg);
    }
  }
  return;
}
template <typename T> DdsMessageManager<T>::~DdsMessageManager() {
  participant_->delete_contained_entities();
  DomainParticipantFactory::get_instance()->delete_participant(participant_);
}
template <typename T> void DdsMessageManager<T>::Run() {
  while (true) {
    std::unique_lock<std::mutex> lock(mode_mutex_);
    switch (action_mode_) {
    case MessageActionMode::DO_NOTHING:
      /* code sleep */
      break;
    case MessageActionMode::TO_ACTIVATE:
      TaskStart();
      break;
    case MessageActionMode::TO_DEACTIVATE:
      TaskStop();
      break;
    default:
      // sleep
      break;
    }
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <typename T> void DdsMessageManager<T>::Stop() {}
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#endif
