/**
 * @file    lcm_message_manager.hpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "lcm_message_manager.h"
#include "modules/common/base_message/message.h"
#include "modules/common/logging/logging.h"
#include "modules/common/macros/macros.h"
#include "modules/common/math/euler_angles_zxy.h"

#if LCM_ENABLE
/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;
template <typename T> void LcmMessageManager<T>::Init(T *t) {
  is_init_ = false;
  instance_ = t;
  std::map<std::string, legionclaw::common::Message> messages =
      instance_->GetConf()->messages();
  lcm_ = std::make_shared<lcm::LCM>(messages["LCM"].url);

  if (!lcm_->good()) {
    AERROR << "lcm init error!";
    return;
  }

  lcm_->subscribe("/drivers/lidar/PointCloud_Top",
                  &LcmMessageManager::HandlePointCloudMessage, this);

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T>
void LcmMessageManager<T>::PublishObstacleList(
    legionclaw::interface::ObstacleList msg) {
  if (is_init_ == false)
    return;
  lcm_interface::ObstacleList obstacle_list;
  MESSAGE_HEADER_ASSIGN(lcm_interface, obstacle_list)
  obstacle_list.sensor_id = msg.sensor_id();
  std::vector<lcm_interface::Obstacle> lcm_obstacle;
  std::vector<legionclaw::interface::Obstacle> legion_obstacle;
  msg.obstacle(legion_obstacle);
  for (auto it_obstacle : legion_obstacle) {
    lcm_interface::Obstacle obstacle_list_obstacle;
    lcm_interface::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.sec = it_obstacle.timestamp().sec();
    obstacle_list_obstacle_timestamp.nsec = it_obstacle.timestamp().nsec();
    obstacle_list_obstacle.timestamp = obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle.id = it_obstacle.id();
    obstacle_list_obstacle.existence_prob = it_obstacle.existence_prob();
    lcm_interface::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.sec = it_obstacle.create_time().sec();
    obstacle_list_obstacle_create_time.nsec = it_obstacle.create_time().nsec();
    obstacle_list_obstacle.create_time = obstacle_list_obstacle_create_time;
    lcm_interface::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.sec =
        it_obstacle.last_updated_time().sec();
    obstacle_list_obstacle_last_updated_time.nsec =
        it_obstacle.last_updated_time().nsec();
    obstacle_list_obstacle.last_updated_time =
        obstacle_list_obstacle_last_updated_time;
    lcm_interface::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.x =
        it_obstacle.center_pos_vehicle().x();
    obstacle_list_obstacle_center_pos_vehicle.y =
        it_obstacle.center_pos_vehicle().y();
    obstacle_list_obstacle_center_pos_vehicle.z =
        it_obstacle.center_pos_vehicle().z();
    obstacle_list_obstacle.center_pos_vehicle =
        obstacle_list_obstacle_center_pos_vehicle;
    lcm_interface::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.x = it_obstacle.center_pos_abs().x();
    obstacle_list_obstacle_center_pos_abs.y = it_obstacle.center_pos_abs().y();
    obstacle_list_obstacle_center_pos_abs.z = it_obstacle.center_pos_abs().z();
    obstacle_list_obstacle.center_pos_abs =
        obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle.theta_vehicle = it_obstacle.theta_vehicle();
    obstacle_list_obstacle.theta_abs = it_obstacle.theta_abs();
    lcm_interface::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.x =
        it_obstacle.velocity_vehicle().x();
    obstacle_list_obstacle_velocity_vehicle.y =
        it_obstacle.velocity_vehicle().y();
    obstacle_list_obstacle_velocity_vehicle.z =
        it_obstacle.velocity_vehicle().z();
    obstacle_list_obstacle.velocity_vehicle =
        obstacle_list_obstacle_velocity_vehicle;
    lcm_interface::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.x = it_obstacle.velocity_abs().x();
    obstacle_list_obstacle_velocity_abs.y = it_obstacle.velocity_abs().y();
    obstacle_list_obstacle_velocity_abs.z = it_obstacle.velocity_abs().z();
    obstacle_list_obstacle.velocity_abs = obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle.length = it_obstacle.length();
    obstacle_list_obstacle.width = it_obstacle.width();
    obstacle_list_obstacle.height = it_obstacle.height();
    std::vector<lcm_interface::ImageKeyPoint> lcm_image_key_points;
    std::vector<legionclaw::interface::ImageKeyPoint> legion_image_key_points;
    it_obstacle.image_key_points(legion_image_key_points);
    for (auto it_image_key_points : legion_image_key_points) {
      lcm_interface::ImageKeyPoint obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.x = it_image_key_points.x();
      obstacle_list_obstacle_image_key_point.y = it_image_key_points.y();
      obstacle_list_obstacle_image_key_point.confidence =
          it_image_key_points.confidence();
      lcm_image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.image_key_points_size = lcm_image_key_points.size();
    obstacle_list_obstacle.image_key_points = lcm_image_key_points;
    std::vector<lcm_interface::Point3D> lcm_polygon_point_abs;
    std::vector<legionclaw::interface::Point3D> legion_polygon_point_abs;
    it_obstacle.polygon_point_abs(legion_polygon_point_abs);
    for (auto it_polygon_point_abs : legion_polygon_point_abs) {
      lcm_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_polygon_point_abs.x();
      obstacle_list_obstacle_point_3d.y = it_polygon_point_abs.y();
      obstacle_list_obstacle_point_3d.z = it_polygon_point_abs.z();
      lcm_polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_abs_size =
        lcm_polygon_point_abs.size();
    obstacle_list_obstacle.polygon_point_abs = lcm_polygon_point_abs;
    std::vector<lcm_interface::Point3D> lcm_polygon_point_vehicle;
    std::vector<legionclaw::interface::Point3D> legion_polygon_point_vehicle;
    it_obstacle.polygon_point_vehicle(legion_polygon_point_vehicle);
    for (auto it_polygon_point_vehicle : legion_polygon_point_vehicle) {
      lcm_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_polygon_point_vehicle.x();
      obstacle_list_obstacle_point_3d.y = it_polygon_point_vehicle.y();
      obstacle_list_obstacle_point_3d.z = it_polygon_point_vehicle.z();
      lcm_polygon_point_vehicle.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_vehicle_size =
        lcm_polygon_point_vehicle.size();
    obstacle_list_obstacle.polygon_point_vehicle = lcm_polygon_point_vehicle;
    obstacle_list_obstacle.tracking_time = it_obstacle.tracking_time();
    obstacle_list_obstacle.type = it_obstacle.type();
    obstacle_list_obstacle.confidence = it_obstacle.confidence();
    obstacle_list_obstacle.confidence_type = it_obstacle.confidence_type();
    std::vector<lcm_interface::Point3D> lcm_drops;
    std::vector<legionclaw::interface::Point3D> legion_drops;
    it_obstacle.drops(legion_drops);
    for (auto it_drops : legion_drops) {
      lcm_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_drops.x();
      obstacle_list_obstacle_point_3d.y = it_drops.y();
      obstacle_list_obstacle_point_3d.z = it_drops.z();
      lcm_drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.drops_size = lcm_drops.size();
    obstacle_list_obstacle.drops = lcm_drops;
    lcm_interface::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.x =
        it_obstacle.acceleration_vehicle().x();
    obstacle_list_obstacle_acceleration_vehicle.y =
        it_obstacle.acceleration_vehicle().y();
    obstacle_list_obstacle_acceleration_vehicle.z =
        it_obstacle.acceleration_vehicle().z();
    obstacle_list_obstacle.acceleration_vehicle =
        obstacle_list_obstacle_acceleration_vehicle;
    lcm_interface::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.x =
        it_obstacle.acceleration_abs().x();
    obstacle_list_obstacle_acceleration_abs.y =
        it_obstacle.acceleration_abs().y();
    obstacle_list_obstacle_acceleration_abs.z =
        it_obstacle.acceleration_abs().z();
    obstacle_list_obstacle.acceleration_abs =
        obstacle_list_obstacle_acceleration_abs;
    lcm_interface::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.x =
        it_obstacle.anchor_point_image().x();
    obstacle_list_obstacle_anchor_point_image.y =
        it_obstacle.anchor_point_image().y();
    obstacle_list_obstacle.anchor_point_image =
        obstacle_list_obstacle_anchor_point_image;
    lcm_interface::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.x =
        it_obstacle.anchor_point_vehicle().x();
    obstacle_list_obstacle_anchor_point_vehicle.y =
        it_obstacle.anchor_point_vehicle().y();
    obstacle_list_obstacle_anchor_point_vehicle.z =
        it_obstacle.anchor_point_vehicle().z();
    obstacle_list_obstacle.anchor_point_vehicle =
        obstacle_list_obstacle_anchor_point_vehicle;
    lcm_interface::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.x =
        it_obstacle.anchor_point_abs().x();
    obstacle_list_obstacle_anchor_point_abs.y =
        it_obstacle.anchor_point_abs().y();
    obstacle_list_obstacle_anchor_point_abs.z =
        it_obstacle.anchor_point_abs().z();
    obstacle_list_obstacle.anchor_point_abs =
        obstacle_list_obstacle_anchor_point_abs;
    lcm_interface::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.xmin = it_obstacle.bbox2d().xmin();
    obstacle_list_obstacle_bbox2d.ymin = it_obstacle.bbox2d().ymin();
    obstacle_list_obstacle_bbox2d.xmax = it_obstacle.bbox2d().xmax();
    obstacle_list_obstacle_bbox2d.ymax = it_obstacle.bbox2d().ymax();
    obstacle_list_obstacle.bbox2d = obstacle_list_obstacle_bbox2d;
    lcm_interface::BBox2D obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle_bbox2d_rear.xmin = it_obstacle.bbox2d_rear().xmin();
    obstacle_list_obstacle_bbox2d_rear.ymin = it_obstacle.bbox2d_rear().ymin();
    obstacle_list_obstacle_bbox2d_rear.xmax = it_obstacle.bbox2d_rear().xmax();
    obstacle_list_obstacle_bbox2d_rear.ymax = it_obstacle.bbox2d_rear().ymax();
    obstacle_list_obstacle.bbox2d_rear = obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle.sub_type = it_obstacle.sub_type();
    obstacle_list_obstacle.height_above_ground =
        it_obstacle.height_above_ground();
    std::vector<double> position_abs_covariance;
    it_obstacle.position_abs_covariance(position_abs_covariance);
    obstacle_list_obstacle.position_abs_covariance_size =
        position_abs_covariance.size();
    obstacle_list_obstacle.position_abs_covariance = position_abs_covariance;
    std::vector<double> velocity_abs_covariance;
    it_obstacle.velocity_abs_covariance(velocity_abs_covariance);
    obstacle_list_obstacle.velocity_abs_covariance_size =
        velocity_abs_covariance.size();
    obstacle_list_obstacle.velocity_abs_covariance = velocity_abs_covariance;
    std::vector<double> acceleration_abs_covariance;
    it_obstacle.acceleration_abs_covariance(acceleration_abs_covariance);
    obstacle_list_obstacle.acceleration_abs_covariance_size =
        acceleration_abs_covariance.size();
    obstacle_list_obstacle.acceleration_abs_covariance =
        acceleration_abs_covariance;
    obstacle_list_obstacle.theta_abs_covariance =
        it_obstacle.theta_abs_covariance();
    std::vector<double> position_vehicle_covariance;
    it_obstacle.position_vehicle_covariance(position_vehicle_covariance);
    obstacle_list_obstacle.position_vehicle_covariance_size =
        position_vehicle_covariance.size();
    obstacle_list_obstacle.position_vehicle_covariance =
        position_vehicle_covariance;
    std::vector<double> velocity_vehicle_covariance;
    it_obstacle.velocity_vehicle_covariance(velocity_vehicle_covariance);
    obstacle_list_obstacle.velocity_vehicle_covariance_size =
        velocity_vehicle_covariance.size();
    obstacle_list_obstacle.velocity_vehicle_covariance =
        velocity_vehicle_covariance;
    std::vector<double> acceleration_vehicle_covariance;
    it_obstacle.acceleration_vehicle_covariance(
        acceleration_vehicle_covariance);
    obstacle_list_obstacle.acceleration_vehicle_covariance_size =
        acceleration_vehicle_covariance.size();
    obstacle_list_obstacle.acceleration_vehicle_covariance =
        acceleration_vehicle_covariance;
    obstacle_list_obstacle.theta_vehicle_covariance =
        it_obstacle.theta_vehicle_covariance();
    lcm_interface::SensorCalibrator obstacle_list_obstacle_sensor_calibrator;
    lcm_interface::Point3D obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.x =
        it_obstacle.sensor_calibrator().pose().x();
    obstacle_list_obstacle_sensor_calibrator_pose.y =
        it_obstacle.sensor_calibrator().pose().y();
    obstacle_list_obstacle_sensor_calibrator_pose.z =
        it_obstacle.sensor_calibrator().pose().z();
    obstacle_list_obstacle_sensor_calibrator.pose =
        obstacle_list_obstacle_sensor_calibrator_pose;
    lcm_interface::Point3D obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle_sensor_calibrator_angle.x =
        it_obstacle.sensor_calibrator().angle().x();
    obstacle_list_obstacle_sensor_calibrator_angle.y =
        it_obstacle.sensor_calibrator().angle().y();
    obstacle_list_obstacle_sensor_calibrator_angle.z =
        it_obstacle.sensor_calibrator().angle().z();
    obstacle_list_obstacle_sensor_calibrator.angle =
        obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle.sensor_calibrator =
        obstacle_list_obstacle_sensor_calibrator;
    obstacle_list_obstacle.cipv_flag = it_obstacle.cipv_flag();
    obstacle_list_obstacle.lane_position = it_obstacle.lane_position();
    obstacle_list_obstacle.pihp_percentage = it_obstacle.pihp_percentage();
    obstacle_list_obstacle.blinker_flag = it_obstacle.blinker_flag();
    obstacle_list_obstacle.fusion_type = it_obstacle.fusion_type();
    lcm_obstacle.emplace_back(obstacle_list_obstacle);
  }
  obstacle_list.obstacle_size = lcm_obstacle.size();
  obstacle_list.obstacle = lcm_obstacle;
  obstacle_list.error_code = msg.error_code();
  obstacle_list.is_valid = msg.is_valid();
  obstacle_list.change_origin_flag = msg.change_origin_flag();

  lcm_->publish("/perception/PObstacleList",
                &obstacle_list);
}

template <typename T>
void LcmMessageManager<T>::HandlePointCloudMessage(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const lcm_interface::PointCloud *msg) {
  if (is_init_ == false)
    return;

  legionclaw::interface::PointCloud point_cloud;
  MESSAGE_HEADER_PARSER(point_cloud)
  point_cloud.set_frame_id(msg->frame_id);
  point_cloud.set_is_dense(msg->is_dense);
  std::vector<legionclaw::interface::PointXYZIRT> point;
  for (auto it_point : msg->point) {
    legionclaw::interface::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.set_x(it_point.x);
    point_cloud_point_xyzirt.set_y(it_point.y);
    point_cloud_point_xyzirt.set_z(it_point.z);
    point_cloud_point_xyzirt.set_intensity(it_point.intensity);
    point_cloud_point_xyzirt.set_ring_id(it_point.ring_id);
    point_cloud_point_xyzirt.set_timestamp(it_point.timestamp);
    point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.set_point(&point);
  point_cloud.set_measurement_time(msg->measurement_time);
  point_cloud.set_width(msg->width);
  point_cloud.set_height(msg->height);

  instance_->HandlePointCloud(point_cloud);
}

template <typename T> void LcmMessageManager<T>::Run() {
  while (0 == lcm_->handle())
    ;
}

template <typename T> void LcmMessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#endif
