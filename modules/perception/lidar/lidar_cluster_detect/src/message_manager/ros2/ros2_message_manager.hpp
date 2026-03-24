/**
 * @file    ros_2_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "ros2_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include <vector>
#include <tuple>
#include <string>

#if ROS2_ENABLE
/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
using namespace legion::common;

using ::ros2_interface::msg::Faults;
using ::ros2_interface::msg::ObstacleList;
using ::ros2_interface::msg::PointCloud;

template <typename T>
Ros2MessageManager<T>::Ros2MessageManager() : Node{"perception"} {}
template <typename T> void Ros2MessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  action_mode_ = MessageActionMode::DO_NOTHING;

  using rclcpp::QoS;
  using SubAllocT =
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

  obstacle_list_pub_ = create_publisher<::ros2_interface::msg::ObstacleList>(
      "/perception/lidar/lidar_cluster_detect/LCDObstacleList", QoS{10},
      PubAllocT{});

  faults_pub_ = create_publisher<::ros2_interface::msg::Faults>(
      "/perception/lidar/lidar_cluster_detect/Faults", QoS{10}, PubAllocT{});

  cluster_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/perception/lidar/lidar_cluster_detect/ClusterPointCloud", QoS{10},
      PubAllocT{});

  obu_cmd_msg_sub_ = create_subscription<::ros2_interface::msg::ObuCmdMsg>(
      "/vui_client/ObuCmdMsg", QoS{30},
      [this](const ros2_interface::msg::ObuCmdMsg::SharedPtr msg) {
        Ros2MessageManager::HandleObuCmdMsgMessage(msg);
      },
      SubAllocT{});

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  handle_avtive_thread_.reset(new std::thread([this] { ActivateMode(); }));
  if (handle_avtive_thread_ == nullptr) {
    AERROR << "Unable to create handle_avtive_thread_ thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool Ros2MessageManager<T>::Activate() {
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

template <typename T> void Ros2MessageManager<T>::TaskStart() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_active_ == true) {
    return;
  }
  using rclcpp::QoS;
  using SubAllocT =
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

  std::cout << "ros2 activate" << std::endl;
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/lidar/lidar_ground_segmentation/NoGroundPoints", QoS{30},
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        Ros2MessageManager::HandlePointCloudMessage(msg);
      },
      SubAllocT{});

  action_mode_ = MessageActionMode::DO_NOTHING;
  is_active_ = true;
  return;
}

template <typename T> bool Ros2MessageManager<T>::DeActivate() {
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

template <typename T> void Ros2MessageManager<T>::TaskStop() {
  if (is_active_ == false) {
    return;
  }

  point_cloud_sub_ = nullptr;
  action_mode_ = MessageActionMode::DO_NOTHING;
  is_active_ = false;
  return;
}

template <typename T>
void Ros2MessageManager<T>::PublishObstacleList(
    legion::interface::ObstacleList msg) {
  if (is_active_ == false)
    return;
  ::ros2_interface::msg::ObstacleList obstacle_list;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, obstacle_list)
  obstacle_list.sensor_id = msg.sensor_id();
  std::vector<ros2_interface::msg::Obstacle> ros_obstacle;
  std::vector<legion::interface::Obstacle> legion_obstacle;
  msg.obstacle(legion_obstacle);
  for (auto it_obstacle : legion_obstacle) {
    ::ros2_interface::msg::Obstacle obstacle_list_obstacle;
    ::ros2_interface::msg::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.sec = it_obstacle.timestamp().sec();
    obstacle_list_obstacle_timestamp.nsec = it_obstacle.timestamp().nsec();
    obstacle_list_obstacle.timestamp = obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle.id = it_obstacle.id();
    obstacle_list_obstacle.existence_prob = it_obstacle.existence_prob();
    ::ros2_interface::msg::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.sec = it_obstacle.create_time().sec();
    obstacle_list_obstacle_create_time.nsec = it_obstacle.create_time().nsec();
    obstacle_list_obstacle.create_time = obstacle_list_obstacle_create_time;
    ::ros2_interface::msg::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.sec =
        it_obstacle.last_updated_time().sec();
    obstacle_list_obstacle_last_updated_time.nsec =
        it_obstacle.last_updated_time().nsec();
    obstacle_list_obstacle.last_updated_time =
        obstacle_list_obstacle_last_updated_time;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.x =
        it_obstacle.center_pos_vehicle().x();
    obstacle_list_obstacle_center_pos_vehicle.y =
        it_obstacle.center_pos_vehicle().y();
    obstacle_list_obstacle_center_pos_vehicle.z =
        it_obstacle.center_pos_vehicle().z();
    obstacle_list_obstacle.center_pos_vehicle =
        obstacle_list_obstacle_center_pos_vehicle;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.x = it_obstacle.center_pos_abs().x();
    obstacle_list_obstacle_center_pos_abs.y = it_obstacle.center_pos_abs().y();
    obstacle_list_obstacle_center_pos_abs.z = it_obstacle.center_pos_abs().z();
    obstacle_list_obstacle.center_pos_abs =
        obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle.theta_vehicle = it_obstacle.theta_vehicle();
    obstacle_list_obstacle.theta_abs = it_obstacle.theta_abs();
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.x =
        it_obstacle.velocity_vehicle().x();
    obstacle_list_obstacle_velocity_vehicle.y =
        it_obstacle.velocity_vehicle().y();
    obstacle_list_obstacle_velocity_vehicle.z =
        it_obstacle.velocity_vehicle().z();
    obstacle_list_obstacle.velocity_vehicle =
        obstacle_list_obstacle_velocity_vehicle;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.x = it_obstacle.velocity_abs().x();
    obstacle_list_obstacle_velocity_abs.y = it_obstacle.velocity_abs().y();
    obstacle_list_obstacle_velocity_abs.z = it_obstacle.velocity_abs().z();
    obstacle_list_obstacle.velocity_abs = obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle.length = it_obstacle.length();
    obstacle_list_obstacle.width = it_obstacle.width();
    obstacle_list_obstacle.height = it_obstacle.height();
    std::vector<ros2_interface::msg::ImageKeyPoint> ros_image_key_points;
    std::vector<legion::interface::ImageKeyPoint> legion_image_key_points;
    it_obstacle.image_key_points(legion_image_key_points);
    for (auto it_image_key_points : legion_image_key_points) {
      ::ros2_interface::msg::ImageKeyPoint
          obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.x = it_image_key_points.x();
      obstacle_list_obstacle_image_key_point.y = it_image_key_points.y();
      obstacle_list_obstacle_image_key_point.confidence =
          it_image_key_points.confidence();
      ros_image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.image_key_points = ros_image_key_points;
    std::vector<ros2_interface::msg::Point3D> ros_polygon_point_abs;
    std::vector<legion::interface::Point3D> legion_polygon_point_abs;
    it_obstacle.polygon_point_abs(legion_polygon_point_abs);
    for (auto it_polygon_point_abs : legion_polygon_point_abs) {
      ::ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_polygon_point_abs.x();
      obstacle_list_obstacle_point_3d.y = it_polygon_point_abs.y();
      obstacle_list_obstacle_point_3d.z = it_polygon_point_abs.z();
      ros_polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.polygon_point_abs = ros_polygon_point_abs;
    std::vector<ros2_interface::msg::Point3D> ros_polygon_point_vehicle;
    std::vector<legion::interface::Point3D> legion_polygon_point_vehicle;
    it_obstacle.polygon_point_vehicle(legion_polygon_point_vehicle);
    for (auto it_polygon_point_vehicle : legion_polygon_point_vehicle) {
      ::ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
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
    std::vector<ros2_interface::msg::Point3D> ros_drops;
    std::vector<legion::interface::Point3D> legion_drops;
    it_obstacle.drops(legion_drops);
    for (auto it_drops : legion_drops) {
      ::ros2_interface::msg::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.x = it_drops.x();
      obstacle_list_obstacle_point_3d.y = it_drops.y();
      obstacle_list_obstacle_point_3d.z = it_drops.z();
      ros_drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.drops = ros_drops;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.x =
        it_obstacle.acceleration_vehicle().x();
    obstacle_list_obstacle_acceleration_vehicle.y =
        it_obstacle.acceleration_vehicle().y();
    obstacle_list_obstacle_acceleration_vehicle.z =
        it_obstacle.acceleration_vehicle().z();
    obstacle_list_obstacle.acceleration_vehicle =
        obstacle_list_obstacle_acceleration_vehicle;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.x =
        it_obstacle.acceleration_abs().x();
    obstacle_list_obstacle_acceleration_abs.y =
        it_obstacle.acceleration_abs().y();
    obstacle_list_obstacle_acceleration_abs.z =
        it_obstacle.acceleration_abs().z();
    obstacle_list_obstacle.acceleration_abs =
        obstacle_list_obstacle_acceleration_abs;
    ::ros2_interface::msg::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.x =
        it_obstacle.anchor_point_image().x();
    obstacle_list_obstacle_anchor_point_image.y =
        it_obstacle.anchor_point_image().y();
    obstacle_list_obstacle.anchor_point_image =
        obstacle_list_obstacle_anchor_point_image;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.x =
        it_obstacle.anchor_point_vehicle().x();
    obstacle_list_obstacle_anchor_point_vehicle.y =
        it_obstacle.anchor_point_vehicle().y();
    obstacle_list_obstacle_anchor_point_vehicle.z =
        it_obstacle.anchor_point_vehicle().z();
    obstacle_list_obstacle.anchor_point_vehicle =
        obstacle_list_obstacle_anchor_point_vehicle;
    ::ros2_interface::msg::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.x =
        it_obstacle.anchor_point_abs().x();
    obstacle_list_obstacle_anchor_point_abs.y =
        it_obstacle.anchor_point_abs().y();
    obstacle_list_obstacle_anchor_point_abs.z =
        it_obstacle.anchor_point_abs().z();
    obstacle_list_obstacle.anchor_point_abs =
        obstacle_list_obstacle_anchor_point_abs;
    ::ros2_interface::msg::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.xmin = it_obstacle.bbox2d().xmin();
    obstacle_list_obstacle_bbox2d.ymin = it_obstacle.bbox2d().ymin();
    obstacle_list_obstacle_bbox2d.xmax = it_obstacle.bbox2d().xmax();
    obstacle_list_obstacle_bbox2d.ymax = it_obstacle.bbox2d().ymax();
    obstacle_list_obstacle.bbox2d = obstacle_list_obstacle_bbox2d;
    ::ros2_interface::msg::BBox2D obstacle_list_obstacle_bbox2d_rear;
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
    ::ros2_interface::msg::SensorCalibrator
        obstacle_list_obstacle_sensor_calibrator;
    ::ros2_interface::msg::Point3D
        obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.x =
        it_obstacle.sensor_calibrator().pose().x();
    obstacle_list_obstacle_sensor_calibrator_pose.y =
        it_obstacle.sensor_calibrator().pose().y();
    obstacle_list_obstacle_sensor_calibrator_pose.z =
        it_obstacle.sensor_calibrator().pose().z();
    obstacle_list_obstacle_sensor_calibrator.pose =
        obstacle_list_obstacle_sensor_calibrator_pose;
    ::ros2_interface::msg::Point3D
        obstacle_list_obstacle_sensor_calibrator_angle;
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

  obstacle_list_pub_->publish(obstacle_list);
}

template <typename T>
void Ros2MessageManager<T>::PublishFaults(legion::interface::Faults msg) {
  if (is_init_ == false)
    return;
  ::ros2_interface::msg::Faults faults;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, faults)
  FAULTS_PARSER_ROS2(ros2, faults)

  faults_pub_->publish(faults);
}

template <typename T>
void Ros2MessageManager<T>::HandleObuCmdMsgMessage(
    const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj_ptr) {
  if (is_init_ == false)
    return;
  std::shared_ptr<ros2_interface::msg::ObuCmdMsg> msg =
      std::const_pointer_cast<ros2_interface::msg::ObuCmdMsg>(msg_obj_ptr);

  legion::interface::ObuCmdMsg obu_cmd_msg;
  MESSAGE_HEADER_ROS2_PARSER(obu_cmd_msg)
  obu_cmd_msg.set_id(msg->id);
  obu_cmd_msg.set_name(msg->name);
  std::vector<legion::interface::ObuCmd> obu_cmd_list;
  for (auto it_obu_cmd_list : msg->obu_cmd_list) {
    legion::interface::ObuCmd obu_cmd_msg_obu_cmd;
    obu_cmd_msg_obu_cmd.set_code(it_obu_cmd_list.code);
    obu_cmd_msg_obu_cmd.set_val(it_obu_cmd_list.val);
    obu_cmd_list.emplace_back(obu_cmd_msg_obu_cmd);
  }
  obu_cmd_msg.set_obu_cmd_list(&obu_cmd_list);

  instance_->HandleObuCmdMsg(obu_cmd_msg);
}

template <typename T>
void Ros2MessageManager<T>::HandlePointCloudMessage(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (is_active_ == false)
    return;

  legion::interface::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_PARSER(point_cloud)
  // point_cloud.set_frame_id(msg->header.frame_id);
  point_cloud.set_is_dense(msg->is_dense);
  point_cloud.set_width(msg->width);
  point_cloud.set_height(msg->height);

  // Parse PointCloud2 fields to find offsets
  int x_offset = -1, y_offset = -1, z_offset = -1;
  int intensity_offset = -1, ring_offset = -1, timestamp_offset = -1;
  
  for (const auto& field : msg->fields) {
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      x_offset = field.offset;
    } else if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      y_offset = field.offset;
    } else if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      z_offset = field.offset;
    } else if (field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      intensity_offset = field.offset;
    } else if (field.name == "ring" && field.datatype == sensor_msgs::msg::PointField::UINT16) {
      ring_offset = field.offset;
    } else if (field.name == "timestamp" && field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
      timestamp_offset = field.offset;
    }
  }

  // Check if required fields exist
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    AERROR << "PointCloud2 missing required fields (x, y, z)";
    return;
  }

  // Convert point cloud data
  std::vector<legion::interface::PointXYZIRT> point;
  const uint8_t* data_ptr = msg->data.data();
  size_t point_count = msg->width * msg->height;
  
  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* point_data = data_ptr + i * msg->point_step;
    
    legion::interface::PointXYZIRT point_xyzirt;
    
    // Extract x, y, z (required)
    float x = *reinterpret_cast<const float*>(point_data + x_offset);
    float y = *reinterpret_cast<const float*>(point_data + y_offset);
    float z = *reinterpret_cast<const float*>(point_data + z_offset);
    
    point_xyzirt.set_x(x);
    point_xyzirt.set_y(y);
    point_xyzirt.set_z(z);
    
    // Extract intensity (optional, default to 0)
    if (intensity_offset >= 0) {
      float intensity = *reinterpret_cast<const float*>(point_data + intensity_offset);
      point_xyzirt.set_intensity(intensity);
    } else {
      point_xyzirt.set_intensity(0.0f);
    }
    
    // Extract ring_id (optional, default to 0)
    if (ring_offset >= 0) {
      uint16_t ring_id = *reinterpret_cast<const uint16_t*>(point_data + ring_offset);
      point_xyzirt.set_ring_id(ring_id);
    } else {
      point_xyzirt.set_ring_id(0);
    }
    
    // Extract timestamp (optional, default to 0)
    if (timestamp_offset >= 0) {
      double timestamp = *reinterpret_cast<const double*>(point_data + timestamp_offset);
      point_xyzirt.set_timestamp(timestamp);
    } else {
      point_xyzirt.set_timestamp(0.0);
    }
    
    point.emplace_back(point_xyzirt);
  }
  
  point_cloud.set_point(&point);
  
  // Set measurement time from header if available
  if (msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0) {
    point_cloud.set_measurement_time(
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  } else {
    point_cloud.set_measurement_time(0.0);
  }

  instance_->HandlePointCloud(point_cloud);
}

template <typename T>
void Ros2MessageManager<T>::PublishClusterPointCloud(
    const legion::interface::PointCloud& msg) {
  if (is_active_ == false || cluster_point_cloud_pub_ == nullptr)
    return;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, cloud_msg)
  cloud_msg.height = 1;
  cloud_msg.width = msg.point_size();
  
  if (cloud_msg.width == 0) {
    return;
  }

  // Define point fields: x, y, z, intensity
  cloud_msg.fields.resize(4);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;
  
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count = 1;
  
  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count = 1;
  
  cloud_msg.fields[3].name = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[3].count = 1;
  
  cloud_msg.point_step = 16; // 4 floats * 4 bytes
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;
  
  // Allocate data buffer
  cloud_msg.data.resize(cloud_msg.row_step);
  
  // Fill point cloud data
  uint8_t* data_ptr = cloud_msg.data.data();

  for (uint32_t i = 0; i < msg.point_size(); ++i) {
    const auto& p = msg.point(i);

    uint8_t* point_data = data_ptr + i * cloud_msg.point_step;
    *reinterpret_cast<float*>(point_data + 0) =
        static_cast<float>(p.x());
    *reinterpret_cast<float*>(point_data + 4) =
        static_cast<float>(p.y());
    *reinterpret_cast<float*>(point_data + 8) =
        static_cast<float>(p.z());
    *reinterpret_cast<float*>(point_data + 12) =
        static_cast<float>(p.intensity());
  }
  
  cluster_point_cloud_pub_->publish(cloud_msg);
}

template <typename T> void Ros2MessageManager<T>::Run() {
  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
  exit(0);
}

template <typename T> void Ros2MessageManager<T>::ActivateMode() {
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

template <typename T> void Ros2MessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace lidar
} // namespace perception
} // namespace legion
#endif
