/**
 * @file    prediction.h
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <thread>
#include <iomanip>

#include "modules/common/file/file.h"
#include "modules/common/json/json.hpp"
#include "modules/common/status/status.h"
#include "modules/common/logging/logging.h"
#include "message_manager/message_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/prediction/src/common/local_view.h"
#include "modules/common/base_message/message.pb.h"
#include "modules/prediction/src/predictor/vector_map_predictor.h"
#include "modules/common/fault/fault_client.hpp"
#include "modules/prediction/src/proto/prediction_conf.pb.h"

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

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
using namespace legionclaw::common;
using json = nlohmann::json;
/**
 * @class Prediction
 * @brief 控制类.
 */
class Prediction {
public:
  Prediction(std::string file_path) : config_file_path_(file_path){};
  ~Prediction() = default;
  // struct LocalView {
  //   legionclaw::interface::Location location_;
  //   legionclaw::interface::ADCTrajectory adc_trajectory_;
  //   legionclaw::interface::ObstacleList obstacle_list_;
  //   legionclaw::interface::LaneList lane_list_;
  // };
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
   * @return std::shared_ptr<PredictionConf>
   */
  std::shared_ptr<PredictionConf> GetConf() const;

protected:
  //初始化状态
  bool is_init_;
  bool location_init_;
  bool map_loaded_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json prediction_json_;
  //控制逻辑设置
  std::shared_ptr<legionclaw::prediction::PredictionConf> prediction_conf_;
  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<Prediction>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<Prediction>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<Prediction>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<Prediction>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<Prediction>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<Prediction>> adsfi_message_manager_;
#endif

protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<Prediction>> message_manager);

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
   * @brief 周期发送控制输出到底盘,以及广播prediction状态
   * @return void.
   */
  void Task10ms(void* param);

  /**
   * @brief
   * @param  prediction_obstacles
   */
  void PublishPredictionObstacles(
      legionclaw::interface::PredictionObstacles prediction_obstacles);

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
   * @brief     Location消息接收.
   * @param[in] location .
   * @return    void.
   */
  void HandleLocation(legionclaw::interface::Location location);

  /**
   * @brief     ADCTrajectory消息接收.
   * @param[in] adc_trajectory .
   * @return    void.
   */
  void HandleADCTrajectory(legionclaw::interface::ADCTrajectory adc_trajectory);

  /**
   * @brief     ObstacleList消息接收.
   * @param[in] mm_obstacle_list .
   * @return    void.
   */
  void HandleObstacleList(legionclaw::interface::ObstacleList obstacle_list);

  /**
   * @brief     Odometry消息接收.
   * @param[in] map_2local_tf .
   * @return    void.
   */
  void HandleOdometry(legionclaw::interface::Odometry odometry);

    /**
   * @brief     ObuCmdMsg消息接收.
   * @param[in] obu_cmd_msg .
   * @return    void.
   */
  void HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg);

  /**
   * @brief     TrafficEvents消息接收.
   * @param[in] traffic_events_local .
   * @return    void.
   */
  void HandleTrafficEvents(legionclaw::interface::TrafficEvents traffic_events);

  /**
   * @brief     RoutingResponse消息接收.
   * @param[in] local_routing_response .
   * @return    void.
   */
  void
  HandleRoutingResponse(legionclaw::interface::RoutingResponse routing_response);

  /**
   * @brief     LaneList消息接收.
   * @param[in] mf_lane_list .
   * @return    void.
   */
  void HandleLaneList(legionclaw::interface::LaneList lane_list);

  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputePredictionCommandOnTimer();

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
  legionclaw::interface::Location location_;
  legionclaw::interface::ADCTrajectory adc_trajectory_;
  legionclaw::interface::ObstacleList obstacle_list_;
  legionclaw::interface::Odometry odometry_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;
  legionclaw::interface::Faults faults_;
  legionclaw::interface::RoutingResponse routing_response_;
  legionclaw::interface::LaneList lane_list_;
  legionclaw::interface::PredictionObstacles prediction_obstacles_;
  legionclaw::interface::FaultCodeSet  * faultcodeset_ ;

  //控制命令生产周期
  int32_t produce_prediction_command_duration_;
  //控制命令发送周期
  int32_t publish_prediction_command_duration_;
  //状态检测周期
  uint32_t status_detect_duration_;
  // 功能激活状态,激活为true，未激活为false
  bool function_activation_;
  //消息状态
  std::map<std::string, legionclaw::common::Status> message_status_;
  // task线程
  std::unique_ptr<std::thread> task_thread_;

  std::mutex mutex_;

  LocalView local_view_;

  //predictor
  VectorMapPredictor vector_map_predictor_;

protected:
  //定时器
  std::shared_ptr<ADTimerManager<Prediction, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<Prediction, void>> task_10ms_;
  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();
};
} // namespace prediction
} // namespace legionclaw
