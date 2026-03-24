/**
 * @file    dds_message_manager.hpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "dds_message_manager.h"
#include "modules/common/logging/logging.h"
#include "modules/common/macros/macros.h"
#include "modules/common/math/euler_angles_zxy.h"

#if DDS_ENABLE
/**
 * @namespace athena::perception::lidar
 * @brief athena::perception::lidar
 */

namespace athena {
namespace perception {
namespace lidar {
using namespace athena::common;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;
template <typename T> void DdsMessageManager<T>::Init(T *t) {
  is_init_ = false;
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
  pqos.name("Participant_pub");
  participant_ =
      DomainParticipantFactory::get_instance()->create_participant(0, pqos);
  if (participant_ == nullptr) {
    return;
  }
  DataWriterQos wqos;
  wqos.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos.history().depth = 30;
  wqos.resource_limits().max_samples = 5000;
  wqos.resource_limits().max_instances = 400;
  wqos.resource_limits().allocated_samples = 100;
  wqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  wqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;
  wqos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
  // ObstacleList
  lidar_obstacle_list_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::ObstacleListPubSubType()));
  lidar_obstacle_list_type_->register_type(participant_);
  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  topic_ = participant_->create_topic(
      "rt/perception/lidar/lidar_detect/LidarObstacleList",
      lidar_obstacle_list_type_->get_type_name(), TOPIC_QOS_DEFAULT);
  lidar_obstacle_list_writer_ =
      publisher_->create_datawriter(topic_, wqos, nullptr);

  DataReaderQos rqos;
  rqos.history().kind = KEEP_LAST_HISTORY_QOS;
  rqos.history().depth = 30;
  rqos.resource_limits().max_samples = 5000;
  rqos.resource_limits().allocated_samples = 400;
  rqos.resource_limits().max_instances = 100;
  // rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  rqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;
  point_cloud_top_type_.reset(new eprosima::fastdds::dds::TypeSupport(
      new ros2_interface::msg::PointCloudPubSubType()));
  point_cloud_top_type_->register_type(participant_);
  subscriber_ =
      participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
  topic_ = participant_->create_topic("rt/drivers/lidar/PointCloud_Top",
                                      point_cloud_top_type_->get_type_name(),
                                      TOPIC_QOS_DEFAULT);
  point_cloud_top_reader_ = subscriber_->create_datareader(topic_, rqos, this);

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T>
void DdsMessageManager<T>::PublishObstacleList(
    athena::interface::ObstacleList msg) {
  if (is_init_ == false)
    return;
  ros2_interface::msg::ObstacleList obstacle_list;
  MESSAGE_DDS_HEADER_ASSIGN(ros2_interface::msg, obstacle_list)
  obstacle_list.sensor_id() = msg.sensor_id();
  std::vector<ros2_interface::msg::Obstacle> dds_obstacle;
  std::vector<athena::interface::Obstacle> athena_obstacle;
  msg.obstacle(athena_obstacle);
  for (auto it_obstacle : athena_obstacle) {
    ros2_interface::msg::Obstacle obstacle_list_obstacle;
    ros2_interface::msg::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.sec() = it_obstacle.timestamp().sec();
    obstacle_list_obstacle_timestamp.nsec() = it_obstacle.timestamp().nsec();
    obstacle_list_obstacle.timestamp() = obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle.id() = it_obstacle.id();
    obstacle_list_obstacle.existence_prob() = it_obstacle.existence_prob();
    ros2_interface::msg::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.sec() = it_obstacle.create_time().sec();
    obstacle_list_obstacle_create_time.nsec() =
        it_obstacle.create_time().nsec();
    obstacle_list_obstacle.create_time() = obstacle_list_obstacle_create_time;
    ros2_interface::msg::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.sec() =
        it_obstacle.last_updated_time().sec();
    obstacle_list_obstacle_last_updated_time.nsec() =
        it_obstacle.last_updated_time().nsec();
    obstacle_list_obstacle.last_updated_time() =
        obstacle_list_obstacle_last_updated_time;
    ros2_interface::msg::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.x() =
        it_obstacle.center_pos_vehicle().x();
    obstacle_list_obstacle_center_pos_vehicle.y() =
        it_obstacle.center_pos_vehicle().y();
    obstacle_list_obstacle_center_pos_vehicle.z() =
        it_obstacle.center_pos_vehicle().z();
    obstacle_list_obstacle.center_pos_vehicle() =
        obstacle_list_obstacle_center_pos_vehicle;
    ros2_interface::msg::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.x() =
        it_obstacle.center_pos_abs().x();
    obstacle_list_obstacle_center_pos_abs.y() =
        it_obstacle.center_pos_abs().y();
    obstacle_list_obstacle_center_pos_abs.z() =
        it_obstacle.center_pos_abs().z();
    obstacle_list_obstacle.center_pos_abs() =
        obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle.theta_vehicle() = it_obstacle.theta_vehicle();
    obstacle_list_obstacle.theta_abs() = it_obstacle.theta_abs();
    ros2_interface::msg::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.x() =
        it_obstacle.velocity_vehicle().x();
    obstacle_list_obstacle_velocity_vehicle.y() =
        it_obstacle.velocity_vehicle().y();
    obstacle_list_obstacle_velocity_vehicle.z() =
        it_obstacle.velocity_vehicle().z();
    obstacle_list_obstacle.velocity_vehicle() =
        obstacle_list_obstacle_velocity_vehicle;
    ros2_interface::msg::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.x() = it_obstacle.velocity_abs().x();
    obstacle_list_obstacle_velocity_abs.y() = it_obstacle.velocity_abs().y();
    obstacle_list_obstacle_velocity_abs.z() = it_obstacle.velocity_abs().z();
    obstacle_list_obstacle.velocity_abs() = obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle.length() = it_obstacle.length();
    obstacle_list_obstacle.width() = it_obstacle.width();
    obstacle_list_obstacle.height() = it_obstacle.height();
    std::vector<ros2_interface::msg::ImageKeyPoint> dds_image_key_points;
    std::vector<athena::interface::ImageKeyPoint> athena_image_key_points;
    it_obstacle.image_key_points(athena_image_key_points);
    for (auto it_image_key_points : athena_image_key_points) {
      ros2_interface::msg::ImageKeyPoint obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.x() = it_image_key_points.x();
      obstacle_list_obstacle_image_key_point.y() = it_image_key_points.y();
      obstacle_list_obstacle_image_key_point.confidence() =
          it_image_key_points.confidence();
      dds_image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.image_key_points() = dds_image_key_points;
    std::vector<ros2_interface::msg::Point3D> dds_polygon_point_abs;
    std::vector<athena::interface::Point3D> athena_polygon_point_abs;
    it_obstacle.polygon_point_abs(athena_polygon_point_abs);
    for (auto it_polygon_point_abs : athena_polygon_point_abs) {
      ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x() = it_polygon_point_abs.x();
      obstacle_list_obstacle_point_3d.y() = it_polygon_point_abs.y();
      obstacle_list_obstacle_point_3d.z() = it_polygon_point_abs.z();
      dds_polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_abs() = dds_polygon_point_abs;
    std::vector<ros2_interface::msg::Point3D> dds_polygon_point_vehicle;
    std::vector<athena::interface::Point3D> athena_polygon_point_vehicle;
    it_obstacle.polygon_point_vehicle(athena_polygon_point_vehicle);
    for (auto it_polygon_point_vehicle : athena_polygon_point_vehicle) {
      ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x() = it_polygon_point_vehicle.x();
      obstacle_list_obstacle_point_3d.y() = it_polygon_point_vehicle.y();
      obstacle_list_obstacle_point_3d.z() = it_polygon_point_vehicle.z();
      dds_polygon_point_vehicle.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_vehicle() = dds_polygon_point_vehicle;
    obstacle_list_obstacle.tracking_time() = it_obstacle.tracking_time();
    obstacle_list_obstacle.type() = it_obstacle.type();
    obstacle_list_obstacle.confidence() = it_obstacle.confidence();
    obstacle_list_obstacle.confidence_type() = it_obstacle.confidence_type();
    std::vector<ros2_interface::msg::Point3D> dds_drops;
    std::vector<athena::interface::Point3D> athena_drops;
    it_obstacle.drops(athena_drops);
    for (auto it_drops : athena_drops) {
      ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x() = it_drops.x();
      obstacle_list_obstacle_point_3d.y() = it_drops.y();
      obstacle_list_obstacle_point_3d.z() = it_drops.z();
      dds_drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.drops() = dds_drops;
    ros2_interface::msg::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.x() =
        it_obstacle.acceleration_vehicle().x();
    obstacle_list_obstacle_acceleration_vehicle.y() =
        it_obstacle.acceleration_vehicle().y();
    obstacle_list_obstacle_acceleration_vehicle.z() =
        it_obstacle.acceleration_vehicle().z();
    obstacle_list_obstacle.acceleration_vehicle() =
        obstacle_list_obstacle_acceleration_vehicle;
    ros2_interface::msg::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.x() =
        it_obstacle.acceleration_abs().x();
    obstacle_list_obstacle_acceleration_abs.y() =
        it_obstacle.acceleration_abs().y();
    obstacle_list_obstacle_acceleration_abs.z() =
        it_obstacle.acceleration_abs().z();
    obstacle_list_obstacle.acceleration_abs() =
        obstacle_list_obstacle_acceleration_abs;
    ros2_interface::msg::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.x() =
        it_obstacle.anchor_point_image().x();
    obstacle_list_obstacle_anchor_point_image.y() =
        it_obstacle.anchor_point_image().y();
    obstacle_list_obstacle.anchor_point_image() =
        obstacle_list_obstacle_anchor_point_image;
    ros2_interface::msg::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.x() =
        it_obstacle.anchor_point_vehicle().x();
    obstacle_list_obstacle_anchor_point_vehicle.y() =
        it_obstacle.anchor_point_vehicle().y();
    obstacle_list_obstacle_anchor_point_vehicle.z() =
        it_obstacle.anchor_point_vehicle().z();
    obstacle_list_obstacle.anchor_point_vehicle() =
        obstacle_list_obstacle_anchor_point_vehicle;
    ros2_interface::msg::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.x() =
        it_obstacle.anchor_point_abs().x();
    obstacle_list_obstacle_anchor_point_abs.y() =
        it_obstacle.anchor_point_abs().y();
    obstacle_list_obstacle_anchor_point_abs.z() =
        it_obstacle.anchor_point_abs().z();
    obstacle_list_obstacle.anchor_point_abs() =
        obstacle_list_obstacle_anchor_point_abs;
    ros2_interface::msg::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.xmin() = it_obstacle.bbox2d().xmin();
    obstacle_list_obstacle_bbox2d.ymin() = it_obstacle.bbox2d().ymin();
    obstacle_list_obstacle_bbox2d.xmax() = it_obstacle.bbox2d().xmax();
    obstacle_list_obstacle_bbox2d.ymax() = it_obstacle.bbox2d().ymax();
    obstacle_list_obstacle.bbox2d() = obstacle_list_obstacle_bbox2d;
    ros2_interface::msg::BBox2D obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle_bbox2d_rear.xmin() =
        it_obstacle.bbox2d_rear().xmin();
    obstacle_list_obstacle_bbox2d_rear.ymin() =
        it_obstacle.bbox2d_rear().ymin();
    obstacle_list_obstacle_bbox2d_rear.xmax() =
        it_obstacle.bbox2d_rear().xmax();
    obstacle_list_obstacle_bbox2d_rear.ymax() =
        it_obstacle.bbox2d_rear().ymax();
    obstacle_list_obstacle.bbox2d_rear() = obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle.sub_type() = it_obstacle.sub_type();
    obstacle_list_obstacle.height_above_ground() =
        it_obstacle.height_above_ground();
    std::vector<double> position_abs_covariance;
    it_obstacle.position_abs_covariance(position_abs_covariance);
    obstacle_list_obstacle.position_abs_covariance() = position_abs_covariance;
    std::vector<double> velocity_abs_covariance;
    it_obstacle.velocity_abs_covariance(velocity_abs_covariance);
    obstacle_list_obstacle.velocity_abs_covariance() = velocity_abs_covariance;
    std::vector<double> acceleration_abs_covariance;
    it_obstacle.acceleration_abs_covariance(acceleration_abs_covariance);
    obstacle_list_obstacle.acceleration_abs_covariance() =
        acceleration_abs_covariance;
    obstacle_list_obstacle.theta_abs_covariance() =
        it_obstacle.theta_abs_covariance();
    std::vector<double> position_vehicle_covariance;
    it_obstacle.position_vehicle_covariance(position_vehicle_covariance);
    obstacle_list_obstacle.position_vehicle_covariance() =
        position_vehicle_covariance;
    std::vector<double> velocity_vehicle_covariance;
    it_obstacle.velocity_vehicle_covariance(velocity_vehicle_covariance);
    obstacle_list_obstacle.velocity_vehicle_covariance() =
        velocity_vehicle_covariance;
    std::vector<double> acceleration_vehicle_covariance;
    it_obstacle.acceleration_vehicle_covariance(
        acceleration_vehicle_covariance);
    obstacle_list_obstacle.acceleration_vehicle_covariance() =
        acceleration_vehicle_covariance;
    obstacle_list_obstacle.theta_vehicle_covariance() =
        it_obstacle.theta_vehicle_covariance();
    ros2_interface::msg::SensorCalibrator
        obstacle_list_obstacle_sensor_calibrator;
    ros2_interface::msg::Point3D obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.x() =
        it_obstacle.sensor_calibrator().pose().x();
    obstacle_list_obstacle_sensor_calibrator_pose.y() =
        it_obstacle.sensor_calibrator().pose().y();
    obstacle_list_obstacle_sensor_calibrator_pose.z() =
        it_obstacle.sensor_calibrator().pose().z();
    obstacle_list_obstacle_sensor_calibrator.pose() =
        obstacle_list_obstacle_sensor_calibrator_pose;
    ros2_interface::msg::Point3D obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle_sensor_calibrator_angle.x() =
        it_obstacle.sensor_calibrator().angle().x();
    obstacle_list_obstacle_sensor_calibrator_angle.y() =
        it_obstacle.sensor_calibrator().angle().y();
    obstacle_list_obstacle_sensor_calibrator_angle.z() =
        it_obstacle.sensor_calibrator().angle().z();
    obstacle_list_obstacle_sensor_calibrator.angle() =
        obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle.sensor_calibrator() =
        obstacle_list_obstacle_sensor_calibrator;
    obstacle_list_obstacle.cipv_flag() = it_obstacle.cipv_flag();
    obstacle_list_obstacle.lane_position() = it_obstacle.lane_position();
    obstacle_list_obstacle.pihp_percentage() = it_obstacle.pihp_percentage();
    obstacle_list_obstacle.blinker_flag() = it_obstacle.blinker_flag();
    obstacle_list_obstacle.fusion_type() = it_obstacle.fusion_type();
    dds_obstacle.emplace_back(obstacle_list_obstacle);
  }
  obstacle_list.obstacle() = dds_obstacle;
  obstacle_list.error_code() = msg.error_code();
  obstacle_list.is_valid() = msg.is_valid();
  obstacle_list.change_origin_flag() = msg.change_origin_flag();

  lidar_obstacle_list_writer_->write(&obstacle_list);
}

template <typename T>
void DdsMessageManager<T>::HandlePointCloud_TopMessage(
    const ros2_interface::msg::PointCloud *msg) {
  athena::interface::PointCloud point_cloud;
  MESSAGE_DDS_HEADER_PARSER(point_cloud)
  point_cloud.set_frame_id(msg->frame_id());
  point_cloud.set_is_dense(msg->is_dense());
  std::vector<athena::interface::PointXYZIRT> point;
  for (auto it_point : msg->point()) {
    athena::interface::PointXYZIRT point_cloud_point_xyzirt;
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

  instance_->HandlePointCloud(point_cloud);
}

template <typename T>
void DdsMessageManager<T>::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  std::lock_guard<std::mutex> lock(r_mutex_);
  if (is_init_ == false)
    return;
  if (!reader->get_topicdescription()->get_name().compare(
          "rt/drivers/lidar/PointCloud_Top")) {

    ros2_interface::msg::PointCloud msg;
    eprosima::fastdds::dds::SampleInfo info;
    if (point_cloud_top_reader_->take_next_sample(&msg, &info) ==
        ReturnCode_t::RETCODE_OK) {
      HandlePointCloud_TopMessage(&msg);
    }
  }
}
template <typename T> DdsMessageManager<T>::~DdsMessageManager() {
  if (lidar_obstacle_list_writer_ != nullptr) {
    publisher_->delete_datawriter(lidar_obstacle_list_writer_);
  }
  if (publisher_ != nullptr) {
    participant_->delete_publisher(publisher_);
  }
  if (topic_ != nullptr) {
    participant_->delete_topic(topic_);
  }
  if (point_cloud_top_reader_ != nullptr) {
    subscriber_->delete_datareader(point_cloud_top_reader_);
  }
  if (topic_ != nullptr) {
    participant_->delete_topic(topic_);
  }
  if (subscriber_ != nullptr) {
    participant_->delete_subscriber(subscriber_);
  }
  DomainParticipantFactory::get_instance()->delete_participant(participant_);
}
template <typename T> void DdsMessageManager<T>::Run() {}

template <typename T> void DdsMessageManager<T>::Stop() {}
} // namespace lidar
} // namespace perception
} // namespace athena
#endif
