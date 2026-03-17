/**
 * @file    ros_2_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "ros2_message_manager.h"
#include "Utils.hpp"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include <std_msgs/msg/header.hpp>

#if ROS2_ENABLE
/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;

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
      "/sensor/lidar/mid/PointCloud2", QoS{30},
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
    legionclaw::interface::PointCloud msg) {
  if (is_active_ == false)
    return;
  ::ros2_interface::msg::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<ros2_interface::msg::PointXYZIRT> ros_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ::ros2_interface::msg::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x = it_point.x();
    point_cloud_point_xyzirt.y = it_point.y();
    point_cloud_point_xyzirt.z = it_point.z();
    point_cloud_point_xyzirt.intensity = it_point.intensity();
    point_cloud_point_xyzirt.ring_id = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp = it_point.timestamp();
    ros_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point = ros_point;
  point_cloud.measurement_time = msg.measurement_time();
  point_cloud.width = msg.width();
  point_cloud.height = msg.height();

  // ground_points_pub_->publish(point_cloud);
}

template <typename T>
void Ros2MessageManager<T>::PublishNoGroundPoints(
    legionclaw::interface::PointCloud msg) {
  if (is_active_ == false)
    return;
  ::ros2_interface::msg::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_ASSIGN(std_msgs::msg, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<ros2_interface::msg::PointXYZIRT> ros_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ::ros2_interface::msg::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x = it_point.x();
    point_cloud_point_xyzirt.y = it_point.y();
    point_cloud_point_xyzirt.z = it_point.z();
    point_cloud_point_xyzirt.intensity = it_point.intensity();
    point_cloud_point_xyzirt.ring_id = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp = it_point.timestamp();
    ros_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point = ros_point;
  point_cloud.measurement_time = msg.measurement_time();
  point_cloud.width = msg.width();
  point_cloud.height = msg.height();

  // no_ground_points_pub_->publish(point_cloud);
}

template <typename T>
void Ros2MessageManager<T>::PublishGroundPoints(
    const Eigen::MatrixX3f& ground_points) {
  if (is_active_ == false)
    return;
  
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "lidar";
  
  auto point_cloud_msg = patchworkpp_ros::utils::EigenMatToPointCloud2(ground_points, header);
  ground_points_pub_->publish(std::move(*point_cloud_msg));
}

template <typename T>
void Ros2MessageManager<T>::PublishNoGroundPoints(
    const Eigen::MatrixX3f& no_ground_points) {
  if (is_active_ == false)
    return;
  
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "lidar";
  
  auto point_cloud_msg = patchworkpp_ros::utils::EigenMatToPointCloud2(no_ground_points, header);
  no_ground_points_pub_->publish(std::move(*point_cloud_msg));
}

template <typename T>
void Ros2MessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
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

  legionclaw::interface::ObuCmdMsg obu_cmd_msg;
  MESSAGE_HEADER_ROS2_PARSER(obu_cmd_msg)
  obu_cmd_msg.set_id(msg->id);
  obu_cmd_msg.set_name(msg->name);
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  for (auto it_obu_cmd_list : msg->obu_cmd_list) {
    legionclaw::interface::ObuCmd obu_cmd_msg_obu_cmd;
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
  const auto &cloud = patchworkpp_ros::utils::PointCloud2ToEigenMat(msg);
  instance_->HandlePointCloudInput(cloud);
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
} // namespace legionclaw
#endif
