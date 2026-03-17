/**
 * @file    ros_message_manager.hpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "modules/common/logging/logging.h"
#include "modules/common/macros/macros.h"
#include "ros_message_manager.h"

#if ROS_ENABLE
/**
 * @namespace legion::perception::fusion
 * @brief legion::perception::fusion
 */

namespace legion {
namespace perception {
namespace fusion {
using namespace legion::common;
template <typename T> void RosMessageManager<T>::Init(T *t) {
  is_init_ = false;
  instance_ = t;

  obstacle_list_output_pub_ = nh_.advertise<::ros_interface::ObstacleList>(
      "/perception/fusion/motion_manager/MMObstacleList", 1);

  location_sub_ =
      nh_.subscribe("/111localization/global_fusion/Location", 1,
                    &RosMessageManager::HandleLocationMessage, this);

  obstacle_list_input_sub_ =
      nh_.subscribe("/perception/PObstacleList", 1,
                    &RosMessageManager::HandleObstacleListInputMessage, this);

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T>
void RosMessageManager<T>::PublishObstacleListOutput(
    legion::interface::ObstacleList msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::ObstacleList obstacle_list;
  MESSAGE_HEADER_ASSIGN(std_msgs, obstacle_list)
  obstacle_list.sensor_id = msg.sensor_id();

  obstacle_list.header.stamp.sec = msg.header().stamp().sec();
  obstacle_list.header.stamp.nsec = msg.header().stamp().nsec();

  std::vector<ros_interface::Obstacle> ros_obstacle;
  std::vector<legion::interface::Obstacle> legion_obstacle;
  msg.obstacle(legion_obstacle);
  for (auto it_obstacle : legion_obstacle) {
    ::ros_interface::Obstacle obstacle_list_obstacle;
    ::ros_interface::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.sec = it_obstacle.timestamp().sec();
    obstacle_list_obstacle_timestamp.nsec = it_obstacle.timestamp().nsec();
    obstacle_list_obstacle.timestamp = obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle.id = it_obstacle.id();
    obstacle_list_obstacle.existence_prob = it_obstacle.existence_prob();
    ::ros_interface::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.sec = it_obstacle.create_time().sec();
    obstacle_list_obstacle_create_time.nsec = it_obstacle.create_time().nsec();
    obstacle_list_obstacle.create_time = obstacle_list_obstacle_create_time;
    ::ros_interface::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.sec =
        it_obstacle.last_updated_time().sec();
    obstacle_list_obstacle_last_updated_time.nsec =
        it_obstacle.last_updated_time().nsec();
    obstacle_list_obstacle.last_updated_time =
        obstacle_list_obstacle_last_updated_time;
    ::ros_interface::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.x =
        it_obstacle.center_pos_vehicle().x();
    obstacle_list_obstacle_center_pos_vehicle.y =
        it_obstacle.center_pos_vehicle().y();
    obstacle_list_obstacle_center_pos_vehicle.z =
        it_obstacle.center_pos_vehicle().z();
    obstacle_list_obstacle.center_pos_vehicle =
        obstacle_list_obstacle_center_pos_vehicle;
    ::ros_interface::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.x = it_obstacle.center_pos_abs().x();
    obstacle_list_obstacle_center_pos_abs.y = it_obstacle.center_pos_abs().y();
    obstacle_list_obstacle_center_pos_abs.z = it_obstacle.center_pos_abs().z();
    obstacle_list_obstacle.center_pos_abs =
        obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle.theta_vehicle = it_obstacle.theta_vehicle();
    obstacle_list_obstacle.theta_abs = it_obstacle.theta_abs();
    ::ros_interface::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.x =
        it_obstacle.velocity_vehicle().x();
    obstacle_list_obstacle_velocity_vehicle.y =
        it_obstacle.velocity_vehicle().y();
    obstacle_list_obstacle_velocity_vehicle.z =
        it_obstacle.velocity_vehicle().z();
    obstacle_list_obstacle.velocity_vehicle =
        obstacle_list_obstacle_velocity_vehicle;
    ::ros_interface::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.x = it_obstacle.velocity_abs().x();
    obstacle_list_obstacle_velocity_abs.y = it_obstacle.velocity_abs().y();
    obstacle_list_obstacle_velocity_abs.z = it_obstacle.velocity_abs().z();
    obstacle_list_obstacle.velocity_abs = obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle.length = it_obstacle.length();
    obstacle_list_obstacle.width = it_obstacle.width();
    obstacle_list_obstacle.height = it_obstacle.height();
    std::vector<ros_interface::ImageKeyPoint> ros_image_key_points;
    std::vector<legion::interface::ImageKeyPoint> legion_image_key_points;
    it_obstacle.image_key_points(legion_image_key_points);
    for (auto it_image_key_points : legion_image_key_points) {
      ::ros_interface::ImageKeyPoint obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.x = it_image_key_points.x();
      obstacle_list_obstacle_image_key_point.y = it_image_key_points.y();
      obstacle_list_obstacle_image_key_point.confidence =
          it_image_key_points.confidence();
      ros_image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.image_key_points = ros_image_key_points;
    std::vector<ros_interface::Point3D> ros_polygon_point_abs;
    std::vector<legion::interface::Point3D> legion_polygon_point_abs;
    it_obstacle.polygon_point_abs(legion_polygon_point_abs);
    for (auto it_polygon_point_abs : legion_polygon_point_abs) {
      ::ros_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_polygon_point_abs.x();
      obstacle_list_obstacle_point_3d.y = it_polygon_point_abs.y();
      obstacle_list_obstacle_point_3d.z = it_polygon_point_abs.z();
      ros_polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_abs = ros_polygon_point_abs;
    std::vector<ros_interface::Point3D> ros_polygon_point_vehicle;
    std::vector<legion::interface::Point3D> legion_polygon_point_vehicle;
    it_obstacle.polygon_point_vehicle(legion_polygon_point_vehicle);
    for (auto it_polygon_point_vehicle : legion_polygon_point_vehicle) {
      ::ros_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_polygon_point_vehicle.x();
      obstacle_list_obstacle_point_3d.y = it_polygon_point_vehicle.y();
      obstacle_list_obstacle_point_3d.z = it_polygon_point_vehicle.z();
      ros_polygon_point_vehicle.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_vehicle = ros_polygon_point_vehicle;
    obstacle_list_obstacle.tracking_time = it_obstacle.tracking_time();
    obstacle_list_obstacle.type = it_obstacle.type();
    obstacle_list_obstacle.confidence = it_obstacle.confidence();
    obstacle_list_obstacle.confidence_type = it_obstacle.confidence_type();
    std::vector<ros_interface::Point3D> ros_drops;
    std::vector<legion::interface::Point3D> legion_drops;
    it_obstacle.drops(legion_drops);
    for (auto it_drops : legion_drops) {
      ::ros_interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_drops.x();
      obstacle_list_obstacle_point_3d.y = it_drops.y();
      obstacle_list_obstacle_point_3d.z = it_drops.z();
      ros_drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.drops = ros_drops;
    ::ros_interface::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.x =
        it_obstacle.acceleration_vehicle().x();
    obstacle_list_obstacle_acceleration_vehicle.y =
        it_obstacle.acceleration_vehicle().y();
    obstacle_list_obstacle_acceleration_vehicle.z =
        it_obstacle.acceleration_vehicle().z();
    obstacle_list_obstacle.acceleration_vehicle =
        obstacle_list_obstacle_acceleration_vehicle;
    ::ros_interface::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.x =
        it_obstacle.acceleration_abs().x();
    obstacle_list_obstacle_acceleration_abs.y =
        it_obstacle.acceleration_abs().y();
    obstacle_list_obstacle_acceleration_abs.z =
        it_obstacle.acceleration_abs().z();
    obstacle_list_obstacle.acceleration_abs =
        obstacle_list_obstacle_acceleration_abs;
    ::ros_interface::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.x =
        it_obstacle.anchor_point_image().x();
    obstacle_list_obstacle_anchor_point_image.y =
        it_obstacle.anchor_point_image().y();
    obstacle_list_obstacle.anchor_point_image =
        obstacle_list_obstacle_anchor_point_image;
    ::ros_interface::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.x =
        it_obstacle.anchor_point_vehicle().x();
    obstacle_list_obstacle_anchor_point_vehicle.y =
        it_obstacle.anchor_point_vehicle().y();
    obstacle_list_obstacle_anchor_point_vehicle.z =
        it_obstacle.anchor_point_vehicle().z();
    obstacle_list_obstacle.anchor_point_vehicle =
        obstacle_list_obstacle_anchor_point_vehicle;
    ::ros_interface::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.x =
        it_obstacle.anchor_point_abs().x();
    obstacle_list_obstacle_anchor_point_abs.y =
        it_obstacle.anchor_point_abs().y();
    obstacle_list_obstacle_anchor_point_abs.z =
        it_obstacle.anchor_point_abs().z();
    obstacle_list_obstacle.anchor_point_abs =
        obstacle_list_obstacle_anchor_point_abs;
    ::ros_interface::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.xmin = it_obstacle.bbox2d().xmin();
    obstacle_list_obstacle_bbox2d.ymin = it_obstacle.bbox2d().ymin();
    obstacle_list_obstacle_bbox2d.xmax = it_obstacle.bbox2d().xmax();
    obstacle_list_obstacle_bbox2d.ymax = it_obstacle.bbox2d().ymax();
    obstacle_list_obstacle.bbox2d = obstacle_list_obstacle_bbox2d;
    ::ros_interface::BBox2D obstacle_list_obstacle_bbox2d_rear;
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
    obstacle_list_obstacle.position_abs_covariance = position_abs_covariance;
    std::vector<double> velocity_abs_covariance;
    it_obstacle.velocity_abs_covariance(velocity_abs_covariance);
    obstacle_list_obstacle.velocity_abs_covariance = velocity_abs_covariance;
    std::vector<double> acceleration_abs_covariance;
    it_obstacle.acceleration_abs_covariance(acceleration_abs_covariance);
    obstacle_list_obstacle.acceleration_abs_covariance =
        acceleration_abs_covariance;
    obstacle_list_obstacle.theta_abs_covariance =
        it_obstacle.theta_abs_covariance();
    std::vector<double> position_vehicle_covariance;
    it_obstacle.position_vehicle_covariance(position_vehicle_covariance);
    obstacle_list_obstacle.position_vehicle_covariance =
        position_vehicle_covariance;
    std::vector<double> velocity_vehicle_covariance;
    it_obstacle.velocity_vehicle_covariance(velocity_vehicle_covariance);
    obstacle_list_obstacle.velocity_vehicle_covariance =
        velocity_vehicle_covariance;
    std::vector<double> acceleration_vehicle_covariance;
    it_obstacle.acceleration_vehicle_covariance(
        acceleration_vehicle_covariance);
    obstacle_list_obstacle.acceleration_vehicle_covariance =
        acceleration_vehicle_covariance;
    obstacle_list_obstacle.theta_vehicle_covariance =
        it_obstacle.theta_vehicle_covariance();
    ::ros_interface::SensorCalibrator obstacle_list_obstacle_sensor_calibrator;
    ::ros_interface::Point3D obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.x =
        it_obstacle.sensor_calibrator().pose().x();
    obstacle_list_obstacle_sensor_calibrator_pose.y =
        it_obstacle.sensor_calibrator().pose().y();
    obstacle_list_obstacle_sensor_calibrator_pose.z =
        it_obstacle.sensor_calibrator().pose().z();
    obstacle_list_obstacle_sensor_calibrator.pose =
        obstacle_list_obstacle_sensor_calibrator_pose;
    ::ros_interface::Point3D obstacle_list_obstacle_sensor_calibrator_angle;
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
    ros_obstacle.emplace_back(obstacle_list_obstacle);
  }
  obstacle_list.obstacle = ros_obstacle;
  obstacle_list.error_code = msg.error_code();
  obstacle_list.is_valid = msg.is_valid();
  obstacle_list.change_origin_flag = msg.change_origin_flag();

  obstacle_list_output_pub_.publish(obstacle_list);
}

template <typename T>
void RosMessageManager<T>::HandleLocationMessage(
    const ros_interface::Location &msg_obj) {
  if (is_init_ == false)
    return;
  const ros_interface::Location *msg_obj_ptr = &msg_obj;
  ros_interface::Location *msg =
      const_cast<ros_interface::Location *>(msg_obj_ptr);

  legion::interface::Location location;
  MESSAGE_HEADER_PARSER(location)
  legion::interface::PointLLH location_position;
  location_position.set_lon(msg->position.lon);
  location_position.set_lat(msg->position.lat);
  location_position.set_height(msg->position.height);
  location.set_position(location_position);
  location.set_pitch(msg->pitch);
  location.set_roll(msg->roll);
  location.set_heading(msg->heading);
  legion::interface::Point3D location_linear_velocity;
  location_linear_velocity.set_x(msg->linear_velocity.x);
  location_linear_velocity.set_y(msg->linear_velocity.y);
  location_linear_velocity.set_z(msg->linear_velocity.z);
  location.set_linear_velocity(location_linear_velocity);
  legion::interface::Point3D location_linear_acceleration;
  location_linear_acceleration.set_x(msg->linear_acceleration.x);
  location_linear_acceleration.set_y(msg->linear_acceleration.y);
  location_linear_acceleration.set_z(msg->linear_acceleration.z);
  location.set_linear_acceleration(location_linear_acceleration);
  legion::interface::Point3D location_angular_velocity;
  location_angular_velocity.set_x(msg->angular_velocity.x);
  location_angular_velocity.set_y(msg->angular_velocity.y);
  location_angular_velocity.set_z(msg->angular_velocity.z);
  location.set_angular_velocity(location_angular_velocity);
  location.set_rtk_flag((legion::interface::Location::RTKFlag)msg->rtk_flag);
  location.set_odom_type((legion::interface::Location::OdomType)msg->odom_type);
  location.set_auxiliary_type(
      (legion::interface::Location::AuxiliaryType)msg->auxiliary_type);
  location.set_location_valid_flag(
      (legion::common::IsValid)msg->location_valid_flag);
  location.set_origin_lat(msg->origin_lat);
  location.set_origin_lon(msg->origin_lon);
  legion::interface::PointENU location_utm_position;
  location_utm_position.set_x(msg->utm_position.x);
  location_utm_position.set_y(msg->utm_position.y);
  location_utm_position.set_z(msg->utm_position.z);
  location.set_utm_position(location_utm_position);
  location.set_change_origin_flag(
      (legion::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  legion::interface::PointENU location_utm_position_next;
  location_utm_position_next.set_x(msg->utm_position_next.x);
  location_utm_position_next.set_y(msg->utm_position_next.y);
  location_utm_position_next.set_z(msg->utm_position_next.z);
  location.set_utm_position_next(location_utm_position_next);
  legion::interface::Point3D location_position_std_dev;
  location_position_std_dev.set_x(msg->position_std_dev.x);
  location_position_std_dev.set_y(msg->position_std_dev.y);
  location_position_std_dev.set_z(msg->position_std_dev.z);
  location.set_position_std_dev(location_position_std_dev);
  legion::interface::Point3D location_orientation_std_dev;
  location_orientation_std_dev.set_x(msg->orientation_std_dev.x);
  location_orientation_std_dev.set_y(msg->orientation_std_dev.y);
  location_orientation_std_dev.set_z(msg->orientation_std_dev.z);
  location.set_orientation_std_dev(location_orientation_std_dev);
  legion::interface::Point3D location_linear_velocity_std_dev;
  location_linear_velocity_std_dev.set_x(msg->linear_velocity_std_dev.x);
  location_linear_velocity_std_dev.set_y(msg->linear_velocity_std_dev.y);
  location_linear_velocity_std_dev.set_z(msg->linear_velocity_std_dev.z);
  location.set_linear_velocity_std_dev(location_linear_velocity_std_dev);
  legion::interface::Point3D location_linear_acceleration_std_dev;
  location_linear_acceleration_std_dev.set_x(
      msg->linear_acceleration_std_dev.x);
  location_linear_acceleration_std_dev.set_y(
      msg->linear_acceleration_std_dev.y);
  location_linear_acceleration_std_dev.set_z(
      msg->linear_acceleration_std_dev.z);
  location.set_linear_acceleration_std_dev(
      location_linear_acceleration_std_dev);
  legion::interface::Point3D location_angular_velocity_std_dev;
  location_angular_velocity_std_dev.set_x(msg->angular_velocity_std_dev.x);
  location_angular_velocity_std_dev.set_y(msg->angular_velocity_std_dev.y);
  location_angular_velocity_std_dev.set_z(msg->angular_velocity_std_dev.z);
  location.set_angular_velocity_std_dev(location_angular_velocity_std_dev);
  instance_->HandleLocation(location);
}

template <typename T>
void RosMessageManager<T>::HandleObstacleListInputMessage(
    const ros_interface::ObstacleList &msg_obj) {      
  if (is_init_ == false)
    return;
  const ros_interface::ObstacleList *msg_obj_ptr = &msg_obj;
  ros_interface::ObstacleList *msg =
      const_cast<ros_interface::ObstacleList *>(msg_obj_ptr);
  legion::interface::ObstacleList obstacle_list;
  MESSAGE_HEADER_PARSER(obstacle_list)
  obstacle_list.set_sensor_id((legion::common::SensorID)msg->sensor_id);
  std::vector<legion::interface::Obstacle> obstacle;
  for (auto it_obstacle : msg->obstacle) {
    legion::interface::Obstacle obstacle_list_obstacle;
    legion::interface::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.set_sec(it_obstacle.timestamp.sec);
    obstacle_list_obstacle_timestamp.set_nsec(it_obstacle.timestamp.nsec);
    obstacle_list_obstacle.set_timestamp(obstacle_list_obstacle_timestamp);
    obstacle_list_obstacle.set_id(it_obstacle.id);
    obstacle_list_obstacle.set_existence_prob(it_obstacle.existence_prob);
    legion::interface::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.set_sec(it_obstacle.create_time.sec);
    obstacle_list_obstacle_create_time.set_nsec(it_obstacle.create_time.nsec);
    obstacle_list_obstacle.set_create_time(obstacle_list_obstacle_create_time);
    legion::interface::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.set_sec(
        it_obstacle.last_updated_time.sec);
    obstacle_list_obstacle_last_updated_time.set_nsec(
        it_obstacle.last_updated_time.nsec);
    obstacle_list_obstacle.set_last_updated_time(
        obstacle_list_obstacle_last_updated_time);
    legion::interface::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.set_x(
        it_obstacle.center_pos_vehicle.x);
    obstacle_list_obstacle_center_pos_vehicle.set_y(
        it_obstacle.center_pos_vehicle.y);
    obstacle_list_obstacle_center_pos_vehicle.set_z(
        it_obstacle.center_pos_vehicle.z);
    obstacle_list_obstacle.set_center_pos_vehicle(
        obstacle_list_obstacle_center_pos_vehicle);
    legion::interface::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.set_x(it_obstacle.center_pos_abs.x);
    obstacle_list_obstacle_center_pos_abs.set_y(it_obstacle.center_pos_abs.y);
    obstacle_list_obstacle_center_pos_abs.set_z(it_obstacle.center_pos_abs.z);
    obstacle_list_obstacle.set_center_pos_abs(
        obstacle_list_obstacle_center_pos_abs);
    obstacle_list_obstacle.set_theta_vehicle(it_obstacle.theta_vehicle);
    obstacle_list_obstacle.set_theta_abs(it_obstacle.theta_abs);
    legion::interface::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.set_x(
        it_obstacle.velocity_vehicle.x);
    obstacle_list_obstacle_velocity_vehicle.set_y(
        it_obstacle.velocity_vehicle.y);
    obstacle_list_obstacle_velocity_vehicle.set_z(
        it_obstacle.velocity_vehicle.z);
    obstacle_list_obstacle.set_velocity_vehicle(
        obstacle_list_obstacle_velocity_vehicle);
    legion::interface::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.set_x(it_obstacle.velocity_abs.x);
    obstacle_list_obstacle_velocity_abs.set_y(it_obstacle.velocity_abs.y);
    obstacle_list_obstacle_velocity_abs.set_z(it_obstacle.velocity_abs.z);
    obstacle_list_obstacle.set_velocity_abs(
        obstacle_list_obstacle_velocity_abs);
    obstacle_list_obstacle.set_length(it_obstacle.length);
    obstacle_list_obstacle.set_width(it_obstacle.width);
    obstacle_list_obstacle.set_height(it_obstacle.height);
    std::vector<legion::interface::ImageKeyPoint> image_key_points;
    for (auto it_image_key_points : it_obstacle.image_key_points) {
      legion::interface::ImageKeyPoint obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.set_x(it_image_key_points.x);
      obstacle_list_obstacle_image_key_point.set_y(it_image_key_points.y);
      obstacle_list_obstacle_image_key_point.set_confidence(
          it_image_key_points.confidence);
      image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.set_image_key_points(&image_key_points);
    std::vector<legion::interface::Point3D> polygon_point_abs;
    for (auto it_polygon_point_abs : it_obstacle.polygon_point_abs) {
      legion::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_polygon_point_abs.x);
      obstacle_list_obstacle_point_3d.set_y(it_polygon_point_abs.y);
      obstacle_list_obstacle_point_3d.set_z(it_polygon_point_abs.z);
      polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_polygon_point_abs(&polygon_point_abs);
    std::vector<legion::interface::Point3D> polygon_point_vehicle;
    for (auto it_polygon_point_vehicle : it_obstacle.polygon_point_vehicle) {
      legion::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_polygon_point_vehicle.x);
      obstacle_list_obstacle_point_3d.set_y(it_polygon_point_vehicle.y);
      obstacle_list_obstacle_point_3d.set_z(it_polygon_point_vehicle.z);
      polygon_point_vehicle.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_polygon_point_vehicle(&polygon_point_vehicle);
    obstacle_list_obstacle.set_tracking_time(it_obstacle.tracking_time);
    obstacle_list_obstacle.set_type(
        (legion::common::ObstacleType)it_obstacle.type);
    obstacle_list_obstacle.set_confidence(it_obstacle.confidence);
    obstacle_list_obstacle.set_confidence_type(
        (legion::interface::Obstacle::ConfidenceType)
            it_obstacle.confidence_type);
    std::vector<legion::interface::Point3D> drops;
    for (auto it_drops : it_obstacle.drops) {
      legion::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_drops.x);
      obstacle_list_obstacle_point_3d.set_y(it_drops.y);
      obstacle_list_obstacle_point_3d.set_z(it_drops.z);
      drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_drops(&drops);
    legion::interface::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.set_x(
        it_obstacle.acceleration_vehicle.x);
    obstacle_list_obstacle_acceleration_vehicle.set_y(
        it_obstacle.acceleration_vehicle.y);
    obstacle_list_obstacle_acceleration_vehicle.set_z(
        it_obstacle.acceleration_vehicle.z);
    obstacle_list_obstacle.set_acceleration_vehicle(
        obstacle_list_obstacle_acceleration_vehicle);
    legion::interface::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.set_x(
        it_obstacle.acceleration_abs.x);
    obstacle_list_obstacle_acceleration_abs.set_y(
        it_obstacle.acceleration_abs.y);
    obstacle_list_obstacle_acceleration_abs.set_z(
        it_obstacle.acceleration_abs.z);
    obstacle_list_obstacle.set_acceleration_abs(
        obstacle_list_obstacle_acceleration_abs);
    legion::interface::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.set_x(
        it_obstacle.anchor_point_image.x);
    obstacle_list_obstacle_anchor_point_image.set_y(
        it_obstacle.anchor_point_image.y);
    obstacle_list_obstacle.set_anchor_point_image(
        obstacle_list_obstacle_anchor_point_image);
    legion::interface::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.set_x(
        it_obstacle.anchor_point_vehicle.x);
    obstacle_list_obstacle_anchor_point_vehicle.set_y(
        it_obstacle.anchor_point_vehicle.y);
    obstacle_list_obstacle_anchor_point_vehicle.set_z(
        it_obstacle.anchor_point_vehicle.z);
    obstacle_list_obstacle.set_anchor_point_vehicle(
        obstacle_list_obstacle_anchor_point_vehicle);
    legion::interface::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.set_x(
        it_obstacle.anchor_point_abs.x);
    obstacle_list_obstacle_anchor_point_abs.set_y(
        it_obstacle.anchor_point_abs.y);
    obstacle_list_obstacle_anchor_point_abs.set_z(
        it_obstacle.anchor_point_abs.z);
    obstacle_list_obstacle.set_anchor_point_abs(
        obstacle_list_obstacle_anchor_point_abs);
    legion::interface::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.set_xmin(it_obstacle.bbox2d.xmin);
    obstacle_list_obstacle_bbox2d.set_ymin(it_obstacle.bbox2d.ymin);
    obstacle_list_obstacle_bbox2d.set_xmax(it_obstacle.bbox2d.xmax);
    obstacle_list_obstacle_bbox2d.set_ymax(it_obstacle.bbox2d.ymax);
    obstacle_list_obstacle.set_bbox2d(obstacle_list_obstacle_bbox2d);
    legion::interface::BBox2D obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle_bbox2d_rear.set_xmin(it_obstacle.bbox2d_rear.xmin);
    obstacle_list_obstacle_bbox2d_rear.set_ymin(it_obstacle.bbox2d_rear.ymin);
    obstacle_list_obstacle_bbox2d_rear.set_xmax(it_obstacle.bbox2d_rear.xmax);
    obstacle_list_obstacle_bbox2d_rear.set_ymax(it_obstacle.bbox2d_rear.ymax);
    obstacle_list_obstacle.set_bbox2d_rear(obstacle_list_obstacle_bbox2d_rear);
    obstacle_list_obstacle.set_sub_type(
        (legion::common::ObstacleSubType)it_obstacle.sub_type);
    obstacle_list_obstacle.set_height_above_ground(
        it_obstacle.height_above_ground);
    std::vector<double> position_abs_covariance;
    for (auto it_position_abs_covariance :
         it_obstacle.position_abs_covariance) {
      double position_abs_covariance_item;
      position_abs_covariance_item = it_position_abs_covariance;
      position_abs_covariance.emplace_back(position_abs_covariance_item);
    }
    obstacle_list_obstacle.set_position_abs_covariance(
        &position_abs_covariance);
    std::vector<double> velocity_abs_covariance;
    for (auto it_velocity_abs_covariance :
         it_obstacle.velocity_abs_covariance) {
      double velocity_abs_covariance_item;
      velocity_abs_covariance_item = it_velocity_abs_covariance;
      velocity_abs_covariance.emplace_back(velocity_abs_covariance_item);
    }
    obstacle_list_obstacle.set_velocity_abs_covariance(
        &velocity_abs_covariance);
    std::vector<double> acceleration_abs_covariance;
    for (auto it_acceleration_abs_covariance :
         it_obstacle.acceleration_abs_covariance) {
      double acceleration_abs_covariance_item;
      acceleration_abs_covariance_item = it_acceleration_abs_covariance;
      acceleration_abs_covariance.emplace_back(
          acceleration_abs_covariance_item);
    }
    obstacle_list_obstacle.set_acceleration_abs_covariance(
        &acceleration_abs_covariance);
    obstacle_list_obstacle.set_theta_abs_covariance(
        it_obstacle.theta_abs_covariance);
    std::vector<double> position_vehicle_covariance;
    for (auto it_position_vehicle_covariance :
         it_obstacle.position_vehicle_covariance) {
      double position_vehicle_covariance_item;
      position_vehicle_covariance_item = it_position_vehicle_covariance;
      position_vehicle_covariance.emplace_back(
          position_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_position_vehicle_covariance(
        &position_vehicle_covariance);
    std::vector<double> velocity_vehicle_covariance;
    for (auto it_velocity_vehicle_covariance :
         it_obstacle.velocity_vehicle_covariance) {
      double velocity_vehicle_covariance_item;
      velocity_vehicle_covariance_item = it_velocity_vehicle_covariance;
      velocity_vehicle_covariance.emplace_back(
          velocity_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_velocity_vehicle_covariance(
        &velocity_vehicle_covariance);
    std::vector<double> acceleration_vehicle_covariance;
    for (auto it_acceleration_vehicle_covariance :
         it_obstacle.acceleration_vehicle_covariance) {
      double acceleration_vehicle_covariance_item;
      acceleration_vehicle_covariance_item = it_acceleration_vehicle_covariance;
      acceleration_vehicle_covariance.emplace_back(
          acceleration_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_acceleration_vehicle_covariance(
        &acceleration_vehicle_covariance);
    obstacle_list_obstacle.set_theta_vehicle_covariance(
        it_obstacle.theta_vehicle_covariance);
    legion::interface::SensorCalibrator
        obstacle_list_obstacle_sensor_calibrator;
    legion::interface::Point3D obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.set_x(
        it_obstacle.sensor_calibrator.pose.x);
    obstacle_list_obstacle_sensor_calibrator_pose.set_y(
        it_obstacle.sensor_calibrator.pose.y);
    obstacle_list_obstacle_sensor_calibrator_pose.set_z(
        it_obstacle.sensor_calibrator.pose.z);
    obstacle_list_obstacle_sensor_calibrator.set_pose(
        obstacle_list_obstacle_sensor_calibrator_pose);
    legion::interface::Point3D obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle_sensor_calibrator_angle.set_x(
        it_obstacle.sensor_calibrator.angle.x);
    obstacle_list_obstacle_sensor_calibrator_angle.set_y(
        it_obstacle.sensor_calibrator.angle.y);
    obstacle_list_obstacle_sensor_calibrator_angle.set_z(
        it_obstacle.sensor_calibrator.angle.z);
    obstacle_list_obstacle_sensor_calibrator.set_angle(
        obstacle_list_obstacle_sensor_calibrator_angle);
    obstacle_list_obstacle.set_sensor_calibrator(
        obstacle_list_obstacle_sensor_calibrator);
    obstacle_list_obstacle.set_cipv_flag(it_obstacle.cipv_flag);
    obstacle_list_obstacle.set_lane_position(
        (legion::common::LanePosition)it_obstacle.lane_position);
    obstacle_list_obstacle.set_pihp_percentage(it_obstacle.pihp_percentage);
    obstacle_list_obstacle.set_blinker_flag(
        (legion::common::BlinkerFlag)it_obstacle.blinker_flag);
    obstacle_list_obstacle.set_fusion_type(
        (legion::interface::Obstacle::FusionType)it_obstacle.fusion_type);
    obstacle.emplace_back(obstacle_list_obstacle);
  }
  obstacle_list.set_obstacle(&obstacle);
  obstacle_list.set_error_code((legion::common::ErrorCode)msg->error_code);
  obstacle_list.set_is_valid(msg->is_valid);
  obstacle_list.set_change_origin_flag(
      (legion::interface::Location::ChangeOriginFlag)msg->change_origin_flag);

  instance_->HandleObstacleListInput(obstacle_list);
}

template <typename T> void RosMessageManager<T>::Run() {
  ros::spin();
  ros::shutdown();
  exit(0);
}

template <typename T> void RosMessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace fusion
} // namespace perception
} // namespace legion
#endif
