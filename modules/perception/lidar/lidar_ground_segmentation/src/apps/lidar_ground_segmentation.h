/**
 * @file    lidar_ground_segmentation.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <thread>
#include <iomanip>

#include "modules/common/json/json.hpp"
#include "modules/common/status/status.h"
#include "modules/common/logging/logging.h"
#include "message_manager/message_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/fault/fault_client.hpp"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/base_message/message_status.hpp"
#include "modules/perception/lidar/lidar_ground_segmentation/src/common/local_view.h"

#include "patchworkpp/patchworkpp.h"

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

#include "conf/lidar_ground_segmentation_conf.hpp"
/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
using namespace legionclaw::common;
using json = nlohmann::json;
/**
 * @class LidarGroundSegmentation
 * @brief 控制类.
 */
class LidarGroundSegmentation {
public:
  LidarGroundSegmentation(std::string file_path) : config_file_path_(file_path){};
  ~LidarGroundSegmentation() = default;
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
   * @return std::shared_ptr<LidarSegmentGroundConf>
   */
  std::shared_ptr<LidarSegmentGroundConf> GetConf() const;

protected:
  //初始化状态
  bool is_init_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json lidar_ground_segmentation_json_;
  //控制逻辑设置
  std::shared_ptr<LidarSegmentGroundConf> lidar_ground_segmentation_conf_;
  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<LidarGroundSegmentation>>>
      message_manager_;
  // Patchwork++
  std::unique_ptr<patchwork::PatchWorkpp> patchworkpp_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<LidarGroundSegmentation>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<LidarGroundSegmentation>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<LidarGroundSegmentation>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<LidarGroundSegmentation>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<LidarGroundSegmentation>>
      adsfi_message_manager_;
#endif

protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<LidarGroundSegmentation>> message_manager);

  /**
   * @brief
   *
   */
  void VariableInit();

  /**
   * @brief
   *
   */
  patchwork::Params LoadParamsFromJSON(const std::string& config_file);

  /**
   * @brief     消息初始化.
   * @return    void.
   */
  void MessagesInit();

  /**
   * @brief     消息激活.
   * @return    void.
   */
  void MessagesActivate();

  /**
   * @brief     消息去激活.
   * @return    void.
   */
  void MessagesDeActivate();

  /**
   * @brief 定时器任务激活
   */
  void TaskActivate();

  /**
   * @brief 定时器任务停止
   */
  void TaskStop();

  /**
   * @brief 故障码监控初始化
   */
  void FaultMonitorInit();

  /**
   * @brief 周期发送控制输出到底盘,以及广播lidar_ground_segmentation状态
   * @return void.
   */
  void Task1000ms(void* param);

  /**
   * @brief
   * @param  ground_points Eigen Matrix格式的地面点云
   */
  void PublishGroundPoints(const Eigen::MatrixX3f& ground_points);

  /**
   * @brief
   * @param  no_ground_points Eigen Matrix格式的非地面点云
   */
  void PublishNoGroundPoints(const Eigen::MatrixX3f& no_ground_points);

  /**
   * @brief
   * @param  faults
   */
  void PublishFaults();

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
   * @brief     ObuCmdMsg消息接收.
   * @param[in] obu_cmd_msg .
   * @return    void.
   */
  void HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg);

  /**
   * @brief     Eigen Matrix点云消息接收（用于ROS2）.
   * @param[in] cloud Eigen Matrix格式的点云数据.
   * @return    void.
   */
  void HandlePointCloudInput(const Eigen::MatrixXf& cloud);

  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputeLidarSegmentGroundCommandOnTimer();

  /**
   * @brief
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView* local_view);

  /**
   * @brief     状态监测.
   * @return    void.
   */
  void StatusDetectOnTimer();

protected:
  legionclaw::interface::PointCloud point_cloud_input_;
  legionclaw::interface::PointCloud ground_points_;
  legionclaw::interface::PointCloud no_ground_points_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;
  legionclaw::interface::Faults faults_;

  legionclaw::interface::FaultCodeSet* faultcodeset_;
  //控制命令生产周期
  int32_t produce_lidar_ground_segmentation_command_duration_;
  //控制命令发送周期
  int32_t publish_lidar_ground_segmentation_command_duration_;
  //状态检测周期
  uint32_t status_detect_duration_;
  // 功能激活状态,激活为true，未激活为false
  bool function_activation_;
  //消息状态
  std::map<std::string, legionclaw::common::MessageStatus> message_status_;
  // task线程
  std::unique_ptr<std::thread> task_thread_;

  std::mutex mutex_;

  LocalView local_view_;

protected:
  //定时器
  std::shared_ptr<ADTimerManager<LidarGroundSegmentation, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<LidarGroundSegmentation, void>> task_1000ms_;
  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();
};
} // namespace lidar
} // namespace perception
} // namespace legionclaw
