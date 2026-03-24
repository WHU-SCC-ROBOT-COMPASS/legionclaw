/**
 * @file    ros_2_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "ros2_message_manager.h"
// #include "Utils.hpp"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include <std_msgs/msg/header.hpp>

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

  ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/perception/lidar/lidar_ground_segmentation/GroundPoints", QoS{10},
      PubAllocT{});

  no_ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/perception/lidar/lidar_ground_segmentation/NoGroundPoints", QoS{10},
      PubAllocT{});

  faults_pub_ = create_publisher<::ros2_interface::msg::Faults>(
      "/perception/lidar/lidar_ground_segmentation/Faults", QoS{10}, PubAllocT{});

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
  point_cloud_input_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", QoS{30},
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        Ros2MessageManager::HandlePointCloudInputMessage(msg);
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

  point_cloud_input_sub_ = nullptr;
  action_mode_ = MessageActionMode::DO_NOTHING;
  is_active_ = false;
  return;
}

template <typename T>
void Ros2MessageManager<T>::PublishGroundPoints(
    const legion::interface::PointCloud& msg) {
  if (is_active_ == false)
    return;

  // Convert legion::interface::PointCloud to sensor_msgs::msg::PointCloud2
  sensor_msgs::msg::PointCloud2 cloud_msg;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, cloud_msg)
  // Get point data
  std::vector<legion::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);

  uint32_t width  = msg.width();
  uint32_t height = msg.height();
  const uint32_t point_count =
      static_cast<uint32_t>(legion_point.size());

  if (width == 0 || height == 0 ||
      width * height != point_count) {
    width  = point_count;
    height = 1;
  }

  cloud_msg.height = height;
  cloud_msg.width  = width;

  // Define fields: x, y, z, intensity, ring, timestamp
  cloud_msg.fields.resize(6);

  cloud_msg.fields[0].name   = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count  = 1;

  cloud_msg.fields[1].name   = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count  = 1;

  cloud_msg.fields[2].name   = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count  = 1;

  cloud_msg.fields[3].name   = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[3].count  = 1;

  cloud_msg.fields[4].name   = "ring";
  cloud_msg.fields[4].offset = 16;
  cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  cloud_msg.fields[4].count  = 1;

  cloud_msg.fields[5].name   = "timestamp";
  cloud_msg.fields[5].offset = 18;
  cloud_msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
  cloud_msg.fields[5].count  = 1;

  cloud_msg.point_step = 26;  // 4+4+4+4+2+8 bytes
  cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense     = msg.is_dense();

  cloud_msg.data.resize(static_cast<std::size_t>(cloud_msg.row_step) *
                        cloud_msg.height);

  uint8_t *data_ptr = cloud_msg.data.data();
  for (uint32_t i = 0; i < point_count; ++i) {
    const auto &p = legion_point[i];
    uint8_t *point_data = data_ptr + i * cloud_msg.point_step;

    *reinterpret_cast<float *>(point_data + 0)  =
        static_cast<float>(p.x());
    *reinterpret_cast<float *>(point_data + 4)  =
        static_cast<float>(p.y());
    *reinterpret_cast<float *>(point_data + 8)  =
        static_cast<float>(p.z());
    *reinterpret_cast<float *>(point_data + 12) =
        static_cast<float>(p.intensity());
    *reinterpret_cast<uint16_t *>(point_data + 16) =
        static_cast<uint16_t>(p.ring_id());
    *reinterpret_cast<double *>(point_data + 18) =
        static_cast<double>(p.timestamp());
  }

  ground_points_pub_->publish(cloud_msg);
}

template <typename T>
void Ros2MessageManager<T>::PublishNoGroundPoints(
    const legion::interface::PointCloud& msg) {
  if (is_active_ == false)
    return;

  // Convert legion::interface::PointCloud to sensor_msgs::msg::PointCloud2
  sensor_msgs::msg::PointCloud2 cloud_msg;

  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, cloud_msg)
  
  std::vector<legion::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);

  uint32_t width  = msg.width();
  uint32_t height = msg.height();
  const uint32_t point_count =
      static_cast<uint32_t>(legion_point.size());

  if (width == 0 || height == 0 ||
      width * height != point_count) {
    width  = point_count;
    height = 1;
  }

  cloud_msg.height = height;
  cloud_msg.width  = width;

  cloud_msg.fields.resize(6);

  cloud_msg.fields[0].name   = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count  = 1;

  cloud_msg.fields[1].name   = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count  = 1;

  cloud_msg.fields[2].name   = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count  = 1;

  cloud_msg.fields[3].name   = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[3].count  = 1;

  cloud_msg.fields[4].name   = "ring";
  cloud_msg.fields[4].offset = 16;
  cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  cloud_msg.fields[4].count  = 1;

  cloud_msg.fields[5].name   = "timestamp";
  cloud_msg.fields[5].offset = 18;
  cloud_msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
  cloud_msg.fields[5].count  = 1;

  cloud_msg.point_step = 26;
  cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense     = msg.is_dense();

  cloud_msg.data.resize(static_cast<std::size_t>(cloud_msg.row_step) *
                        cloud_msg.height);

  uint8_t *data_ptr = cloud_msg.data.data();
  for (uint32_t i = 0; i < point_count; ++i) {
    const auto &p = legion_point[i];
    uint8_t *point_data = data_ptr + i * cloud_msg.point_step;

    *reinterpret_cast<float *>(point_data + 0)  =
        static_cast<float>(p.x());
    *reinterpret_cast<float *>(point_data + 4)  =
        static_cast<float>(p.y());
    *reinterpret_cast<float *>(point_data + 8)  =
        static_cast<float>(p.z());
    *reinterpret_cast<float *>(point_data + 12) =
        static_cast<float>(p.intensity());
    *reinterpret_cast<uint16_t *>(point_data + 16) =
        static_cast<uint16_t>(p.ring_id());
    *reinterpret_cast<double *>(point_data + 18) =
        static_cast<double>(p.timestamp());
  }

  no_ground_points_pub_->publish(cloud_msg);
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
void Ros2MessageManager<T>::HandlePointCloudInputMessage(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (is_active_ == false)
    return;
  legion::interface::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_PARSER(point_cloud)
  point_cloud.set_frame_id(msg->header.frame_id);
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
  instance_->HandlePointCloudInput(point_cloud);
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
