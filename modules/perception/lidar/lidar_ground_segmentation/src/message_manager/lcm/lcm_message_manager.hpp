/**
 * @file    lcm_message_manager.hpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "lcm_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include "modules/common/base_message/message.h"
#include "modules/common/math/euler_angles_zxy.h"

#if LCM_ENABLE
/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
using namespace legion::common;
template <typename T> void LcmMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  std::map<std::string, legion::common::Message> messages =
      instance_->GetConf()->messages();
  lcm_ = std::make_shared<lcm::LCM>(messages["LCM"].url);

  if (!lcm_->good()) {
    AERROR << "lcm init error!";
    return;
  }

  lcm_->subscribe("/vui_client/ObuCmdMsg",
                  &LcmMessageManager::HandleObuCmdMsgMessage, this);
  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T> bool LcmMessageManager<T>::Activate() {
  if (is_active_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  point_cloud_input_sub_ =
      lcm_->subscribe("/perception/lidar/multi_lidar_concate/MLCPointCloud",
                      &LcmMessageManager::HandlePointCloudInputMessage, this);

  std::cout << "lcm activate" << std::endl;
  is_active_ = true;
  return true;
}

template <typename T> bool LcmMessageManager<T>::DeActivate() {
  if (is_active_ == false) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  lcm_->unsubscribe(point_cloud_input_sub_);

  is_active_ = false;
  std::cout << "lcm deactivate" << std::endl;
  return true;
}

template <typename T>
void LcmMessageManager<T>::PublishGroundPoints(
    legion::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  lcm_interface::PointCloud point_cloud;
  MESSAGE_HEADER_ASSIGN(lcm_interface, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<lcm_interface::PointXYZIRT> lcm_point;
  std::vector<legion::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    lcm_interface::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x = it_point.x();
    point_cloud_point_xyzirt.y = it_point.y();
    point_cloud_point_xyzirt.z = it_point.z();
    point_cloud_point_xyzirt.intensity = it_point.intensity();
    point_cloud_point_xyzirt.ring_id = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp = it_point.timestamp();
    lcm_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point_size = lcm_point.size();
  point_cloud.point = lcm_point;
  point_cloud.measurement_time = msg.measurement_time();
  point_cloud.width = msg.width();
  point_cloud.height = msg.height();

  lcm_->publish("/perception/lidar/lidar_ground_segmentation/GroundPoints",
                &point_cloud);
}

template <typename T>
void LcmMessageManager<T>::PublishNoGroundPoints(
    legion::interface::PointCloud msg) {
  if (is_init_ == false)
    return;
  lcm_interface::PointCloud point_cloud;
  MESSAGE_HEADER_ASSIGN(lcm_interface, point_cloud)
  point_cloud.frame_id = msg.frame_id();
  point_cloud.is_dense = msg.is_dense();
  std::vector<lcm_interface::PointXYZIRT> lcm_point;
  std::vector<legion::interface::PointXYZIRT> legion_point;
  msg.point(legion_point);
  for (auto it_point : legion_point) {
    lcm_interface::PointXYZIRT point_cloud_point_xyzirt;
    point_cloud_point_xyzirt.x = it_point.x();
    point_cloud_point_xyzirt.y = it_point.y();
    point_cloud_point_xyzirt.z = it_point.z();
    point_cloud_point_xyzirt.intensity = it_point.intensity();
    point_cloud_point_xyzirt.ring_id = it_point.ring_id();
    point_cloud_point_xyzirt.timestamp = it_point.timestamp();
    lcm_point.emplace_back(point_cloud_point_xyzirt);
  }
  point_cloud.point_size = lcm_point.size();
  point_cloud.point = lcm_point;
  point_cloud.measurement_time = msg.measurement_time();
  point_cloud.width = msg.width();
  point_cloud.height = msg.height();

  lcm_->publish("/perception/lidar/lidar_ground_segmentation/NoGroundPoints",
                &point_cloud);
}

template <typename T>
void LcmMessageManager<T>::PublishFaults(legion::interface::Faults msg) {
  if (is_init_ == false)
    return;
  lcm_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(lcm_interface, faults)
  FAULTS_PARSER(lcm, faults)
  faults.faults_size = faults.faults.size();

  lcm_->publish("/perception/lidar/lidar_ground_segmentation/Faults", &faults);
}

template <typename T>
void LcmMessageManager<T>::HandlePointCloudInputMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::PointCloud* msg) {
  if (is_active_ == false)
    return;

  legion::interface::PointCloud point_cloud;
  MESSAGE_HEADER_ROS2_PARSER(point_cloud)
  point_cloud.set_frame_id(msg->frame_id);
  point_cloud.set_is_dense(msg->is_dense);
  std::vector<legion::interface::PointXYZIRT> point;
  for (auto it_point : msg->point) {
    legion::interface::PointXYZIRT point_cloud_point_xyzirt;
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
} // namespace legion
#endif