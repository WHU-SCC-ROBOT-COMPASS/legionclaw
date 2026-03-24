/**
 * @file    lidar_detect.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iomanip>
#include <mutex>
#include <thread>

#include "message_manager/message_manager.h"
#include "modules/common/base_message/message_status.hpp"
#include "modules/common/json/json.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/status/status.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/perception/lidar/lidar_detect_voxel/src/common/local_view.h"

#include "common.h"
#include "centerpoint.h"

#if LCM_ENABLE
#include "message_manager/lcm/lcm_message_manager.h"
#endif
#if DDS_ENABLE
#include "message_manager/dds/dds_message_manager.h"
#endif
#if ROS_ENABLE
#include "message_manager/ros/ros_message_manager.h"
#endif
#if ROS2_ENABLE
#include "message_manager/ros2/ros2_message_manager.h"
#endif

#include "conf/lidar_detect_conf.hpp"
/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
using namespace legion::common;
using json = nlohmann::json;
/**
 * @class LidarDetect
 * @brief 控制类.
 */
class LidarDetect {
public:
  LidarDetect(std::string file_path) : config_file_path_(file_path){};
  ~LidarDetect() = default;
  /**
   * @brief     初始化．
   * @param[in] void．
   * @return    void.
   */
  void Init();

  /**
   * @brief     join．
   * @param[in] void.
   * @return    void.
   */
  void Join();

  /**
   * @brief Get the Conf object
   * @return std::shared_ptr<LidarDetectConf>
   */
  std::shared_ptr<LidarDetectConf> GetConf() const;

protected:
  //初始化状态
  bool is_init_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json lidar_detect_json_;
  //控制逻辑设置
  std::shared_ptr<LidarDetectConf> lidar_detect_conf_;
  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<LidarDetect>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<LidarDetect>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<LidarDetect>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<LidarDetect>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<LidarDetect>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<LidarDetect>> adsfi_message_manager_;
#endif

protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<LidarDetect>> message_manager);

  /**
   * @brief
   *
   */
  void VariableInit();

  /**
   * @brief     消息初始化.
   * @return    void.
   */
  void MessagesInit();

  /**
   * @brief 故障码监控初始化
   */
  void FaultMonitorInit();

  /**
   * @brief 周期发送控制输出到底盘,以及广播lidar_detect状态
   * @return void.
   */
  void Task10ms(void *param);

  /**
   * @brief
   * @param  obstacle_list
   */
  void PublishObstacleList(legion::interface::ObstacleList obstacle_list);

  /**
   * @brief     打印调试.
   * @return    void.
   */
  void Print();

  /**
   * @brief     日志调试.
   * @return    void.
   */
  void Log();

public:
  /**
   * @brief     PointCloud消息接收.
   * @param[in] point_cloud_top .
   * @return    void.
   */
  void HandlePointCloud(legion::interface::PointCloud point_cloud);

  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputeLidarDetectCommandOnTimer();

  /**
   * @brief
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView *local_view);

  /**
   * @brief     状态监测.
   * @return    void.
   */
  void StatusDetectOnTimer();

protected:
  legion::interface::PointCloud point_cloud_;
  legion::interface::ObstacleList obstacle_list_;

  //控制命令生产周期
  int32_t produce_lidar_detect_command_duration_;
  //控制命令发送周期
  int32_t publish_lidar_detect_command_duration_;
  //状态检测周期
  uint32_t status_detect_duration_;
  //消息状态
  std::map<std::string, legion::common::MessageStatus> message_status_;
  // task线程
  std::unique_ptr<std::thread> task_thread_;

  std::mutex mutex_;

  LocalView local_view_;

protected:
  //定时器
  std::shared_ptr<ADTimerManager<LidarDetect, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<LidarDetect, void>> task_10ms_;
  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();
protected:
  CenterPoint* centerpoint;
public:
  string scn_engine_file_path;
  string rpn_engine_file_path;
  cudaStream_t stream = NULL;
  float *d_points = nullptr;

  double general_threshold;
  double big_vehicle_threshold;
  double unknow_threshold;

public:
  void PointToBuffer(float* points, legion::interface::PointCloud pointcloud);
  void LidarDetect::BoxesToObstacleList(std::vector<Bndbox> preboxes, legion::interface::ObstacleList &result_obstacle_list);
  void GetPolygon(legion::interface::ObstacleList &result_obstacle_list);
};
} // namespace lidar
} // namespace perception
} // namespace legion
