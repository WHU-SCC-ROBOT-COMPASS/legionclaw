/**
 * @file    ros_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "ros_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"

#if ROS_ENABLE
/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;
template <typename T> void RosMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  action_mode_ = MessageActionMode::DO_NOTHING;

  ground_points_pub_ = nh_.advertise<::ros_interface::PointCloud>(
      "/perception/lidar/lidar_ground_segmentation/GroundPoints", 10);

  no_ground_points_pub_ = nh_.advertise<::ros_interface::PointCloud>(
      "/perception/lidar/lidar_ground_segmentation/NoGroundPoints", 10);

  faults_pub_ = nh_.advertise<::ros_interface::Faults>(
      "/perception/lidar/lidar_ground_segmentation/Faults", 10);

  obu_cmd_msg_sub_ =
      nh_.subscribe("/vui_client/ObuCmdMsg", 30,
                    &RosMessageManager::HandleObuCmdMsgMessage, this);
  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  register_thread_.reset(new std::thread([this] { RegisterLoop(); }));
  if (register_thread_ == nullptr) {
    AERROR << "Unable to create register_thread_ thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool RosMessageManager<T>::Activate() {
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

template <typename T> void RosMessageManager<T>::TaskStart() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_active_ == true) {
    return;
  }
  point_cloud_input_sub_ =
      nh_.subscribe("/perception/lidar/multi_lidar_concate/MLCPointCloud", 30,
                    &RosMessageManager::HandlePointCloudInputMessage, this);

  std::cout << "ros1 activate" << std::endl;
  is_active_ = true;
  action_mode_ = MessageActionMode::DO_NOTHING;
  return;
}
template <typename T> bool RosMessageManager<T>::DeActivate() {
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

template <typename T> void RosMessageManager<T>::TaskStop() {
  if (is_active_ == false) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  point_cloud_input_sub_.shutdown();

  std::cout << "ros1 deactivate" << std::endl;
  is_active_ = false;
  action_mode_ = MessageActionMode::DO_NOTHING;
  return;
}
template <typename T>
void RosMessageManager<T>::PublishGroundPoints(
    legionclaw::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::PointCloud point_cloud;
  MESSAGE_HEADER_ASSIGN(std_msgs, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<ros_interface::PointXYZIRT> ros_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ::ros_interface::PointXYZIRT point_cloud_point_xyzirt;
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

  ground_points_pub_.publish(point_cloud);
}

template <typename T>
void RosMessageManager<T>::PublishNoGroundPoints(
    legionclaw::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::PointCloud point_cloud;
  MESSAGE_HEADER_ASSIGN(std_msgs, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<ros_interface::PointXYZIRT> ros_point;
  std::vector<legionclaw::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    ::ros_interface::PointXYZIRT point_cloud_point_xyzirt;
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

  no_ground_points_pub_.publish(point_cloud);
}

template <typename T>
void RosMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  ::ros_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(std_msgs, faults)
  FAULTS_PARSER_1(ros, faults)

  faults_pub_.publish(faults);
}

template <typename T>
void RosMessageManager<T>::HandlePointCloudInputMessage(
    const ros_interface::PointCloud& msg_obj) {
  if (is_active_ == false)
    return;
  const ros_interface::PointCloud* msg_obj_ptr = &msg_obj;
  ros_interface::PointCloud* msg =
      const_cast<ros_interface::PointCloud*>(msg_obj_ptr);

  legionclaw::interface::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_PARSER(point_cloud)
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

  instance_->HandlePointCloudInput(point_cloud);
}

template <typename T> void RosMessageManager<T>::RegisterLoop() {
  //   eprosima::fastrtps::Duration_t timeout(0.1, 0);
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
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#endif
