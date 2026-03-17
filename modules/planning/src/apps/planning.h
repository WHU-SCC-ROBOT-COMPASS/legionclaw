/**
 * @file    planning.h
 * @author  zdhy
 * @date    2021-09-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#pragma once

#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>

#include "message_manager/message_manager.h"
#include "modules/common/interface/drivable_region.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/events.hpp"
#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"    
#include "modules/common/json/json.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/macros/macros.h"
#include "modules/common/status/status.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/fault/fault_client.hpp"      
#include "modules/planning/src/common/frame.h"
#include "modules/planning/src/common/local_view.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/common/file/file.h"
#include "modules/planning/src/planner/lattice_planner/behaviour_selector/behaviour_selector.h"
#include "modules/planning/src/planner/lattice_planner/lattice_planner.h"
#include "modules/planning/src/planner/lattice_planner/trajectory_evaluator/trajectory_evaluator.h"
#include "modules/planning/src/planner/lattice_planner/trajectory_generator/trajectory_generator.h"
#include "modules/planning/src/planner/parking_planner/parking_planner.h"
#include "modules/planning/src/planner/planner.h"
#include "modules/planning/src/common/reference_line/reference_line_provider.h"
#include "modules/planning/src/common/dependency_injector.h"
#include "modules/planning/src/common/longitude_info_provider/longitude_info_provider.h"

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
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
using namespace legionclaw::interface;
using json = nlohmann::json;
/**
 * @class Planning
 * @brief 规划类.
 */
class Planning {
 public:
  Planning(std::string file_path) : config_file_path_(file_path){};
  ~Planning() = default;

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
   * @return std::shared_ptr<legionclaw::planning::PlanningConf>
   */
  std::shared_ptr<legionclaw::planning::PlanningConf> GetConf() const;

  /**
   * @brief     读配置文件（保留用于兼容）
   * @param[in] file_path 配置文件路径..
   * @return    true or false.
   */
  bool ReadConfigFile(const std::string &file_path);

  /**
   * @brief
   *
   */
  void VariableInit();

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

 protected:
   //初始化状态
  bool is_init_;
  Status frame_status_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json planning_json_;
  //lattice配置文件操作类
  json lattice_planner_json_;
  //parking配置文件操作类
  json parking_planner_json_;
  //车辆配置文件
  json vehicle_param_json_;
  //车辆配置文件路径
  std::string vehicle_param_file_path;
  //车辆行车路径
  std::string driving_state_machine_file;
  //车辆停车路径
  std::string parking_state_machine_file;
  //控制逻辑设置（使用 protobuf 生成的类）
  std::shared_ptr<legionclaw::planning::PlanningConf> planning_conf_;
  
  /*********************
   *  消息域
   *********************/
 protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<Planning>> message_manager);

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

  /*************
   *  消息发送
   *************/
  /**
   * @brief
   * @param  adc_trajectory
   */
  void PublishADCTrajectory(legionclaw::interface::ADCTrajectory adc_trajectory);

  /**
   * @brief
   * @param  planning_cmd
   */
  void PublishPlanningCmd(legionclaw::interface::PlanningCmd planning_cmd);

  /**
   * @brief
   * @param  planning_analysis
   */
  void PublishPlanningAnalysis(
      legionclaw::interface::PlanningAnalysis planning_analysis);

  /**
   * @brief
   * @param  trajectory_array
   */
  void PublishTrajectoryArray(
      legionclaw::interface::TrajectoryArray trajectory_array);

  /**
   * @brief
   * @param  faults
   */
  void PublishFaults();

  /**
   * @brief
   * @param  events
   */
  void PublishEvents(legionclaw::interface::Events events);

 public:
  /*************
   *  消息接收
   *************/
  /**
   * @brief     RoutingResponse消息接收.
   * @param[in] routing_response .
   * @return    void.
   */
  void HandleRoutingResponse(
      legionclaw::interface::RoutingResponse routing_response);

    /**
   * @brief     RoutingResponse消息接收.
   * @param[in] routing_response .
   * @return    void.
   */
  void HandleLocalRoutingResponse(
      legionclaw::interface::RoutingResponse routing_response);

    /**
   * @brief     TrafficEvents消息接收.
   * @param[in]  .
   * @return    void.
   */    
  void HandleTrafficEvents(legionclaw::interface::TrafficEvents traffic_events);

  /**
   * @brief     GuideInfo消息接收.
   * @param[in] guide_info .
   * @return    void.
   */
  void HandleGuideInfo(legionclaw::interface::GuideInfo guide_info);
  
  /**
   * @brief     ParkingInfo消息接收.
   * @param[in] parking_info .
   * @return    void.
   */
  void HandleParkingInfo(legionclaw::interface::ParkingInfo parking_info);

  /**
   * @brief     StopInfo消息接收.
   * @param[in] stop_info .
   * @return    void.
   */
  void HandleStopInfo(legionclaw::interface::StopInfo stop_info);

  /**
   * @brief     TrafficLightMsg消息接收.
   * @param[in] traffic_light_msg .
   * @return    void.
   */
  void HandleTrafficLightMsg(
      legionclaw::interface::TrafficLightMsg traffic_light_msg);

  /**
   * @brief     Location消息接收.
   * @param[in] location .
   * @return    void.
   */
  void HandleLocation(legionclaw::interface::Location location);

  /**
   * @brief     PredictionObstacles消息接收.
   * @param[in] prediction_obstacles .
   * @return    void.
   */
  void HandlePredictionObstacles(
      legionclaw::interface::PredictionObstacles prediction_obstacles);

  /**
   * @brief     LaneList消息接收.
   * @param[in] lane_list .
   * @return    void.
   */
  void HandleLaneList(legionclaw::interface::LaneList lane_list);

  /**
   * @brief     Chassis消息接收.
   * @param[in] chassis .
   * @return    void.
   */
  void HandleChassis(legionclaw::interface::Chassis chassis);

  /**
   * @brief     SotifMonitorResult消息接收.
   * @param[in] sotif_monitor_result .
   * @return    void.
   */
  void HandleSotifMonitorResult(
      legionclaw::interface::SotifMonitorResult sotif_monitor_result);

  /**
   * @brief     ObuCmdMsg消息接收.
   * @param[in] obu_cmd_msg .
   * @return    void.
   */
  void HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg);

  /**
   * @brief     DrivableRegion消息接收.
   * @param[in] drivable_region .
   * @return    void.
   */
  void HandleDrivableRegion(legionclaw::interface::DrivableRegion drivable_region);

  /**
   * @brief     ParkingOutInfo消息接收.
   * @param[in] parking_out_info .
   * @return    void.
   */
  void HandleParkingOutInfo(legionclaw::interface::ParkingOutInfo parking_out_info);

  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<Planning>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<Planning>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<Planning>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<Planning>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<Planning>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<Planning>> adsfi_message_manager_;
#endif

  /*********************
   *  算法域
   *********************/
  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputePlanningCommandOnTimer(void *param);

    /**
   * @brief     消息发布.
   * @return    void.
   */
  void PublishPlanningCommandOnTimer(void *param);

  /**
   * @brief
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView *local_view);

  /**
   * @brief
   * @param
   * @return
   */
  void UpdateHMICommand();

  /**
   * @brief
   * @param local_view
   * @return Status
   */
  Status UpdateFrame(const LocalView &local_view);

  /**
   * @brief     道路参考线处理
   * @param 
   * @return Status
   */
  Status FrameCreateReferenceLineInfo();

  /**
   * @brief     状态监测.
   * @return    void.
   */
  void StatusDetectOnTimer();

  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();

  std::string GetLogFileName();

  void WriteHeaders(std::ofstream &file_stream);
  void CloseLogFile();
  void ResetFlag() {
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    task_status_ = TaskStatus::TASK_INVALID;
    planning_flag_ = PlanningFlag::PLANNING_INVALID;
  }
  /**
   * @brief     自动驾驶切换人工驾驶时，处理
   * @param[in] void.
   * @return    void.
   */
  void Reset();
  /**
   * @brief
   * @param[in]
   * @return
   */
  Frame *mutable_frame() { return &frame_; }
protected:
  //接入
  legionclaw::interface::RoutingResponse routing_response_;
  legionclaw::interface::GuideInfo guide_info_;
  legionclaw::interface::TrafficEvents traffic_events_;
  legionclaw::interface::ParkingInfo parking_info_;
  legionclaw::interface::StopInfo stop_info_;
  legionclaw::interface::TrafficLightMsg traffic_light_msg_;
  legionclaw::interface::Location location_;
  legionclaw::interface::PredictionObstacles prediction_obstacles_;
  legionclaw::interface::LaneList lane_list_;
  legionclaw::interface::Chassis chassis_;
  legionclaw::interface::SotifMonitorResult sotif_monitor_result_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;
  legionclaw::interface::DrivableRegion drivable_region_;
  legionclaw::interface::ParkingOutInfo parking_out_info_;
  //发出
  legionclaw::interface::ADCTrajectory adc_trajectory_;
  legionclaw::interface::ADCTrajectory last_adc_trajectory_;
  legionclaw::interface::PlanningCmd planning_cmd_;
  legionclaw::interface::PlanningAnalysis planning_analysis_;
  legionclaw::interface::TrajectoryArray trajectory_array_;
  legionclaw::interface::Faults faults_;
  legionclaw::interface::Events events_;
  legionclaw::interface::VehicleState vehicle_state_;

  legionclaw::interface::FaultCodeSet  *faultcodeset_;

  //
  int32_t produce_planning_command_duration_;  //控制命令生产周期
  int32_t publish_planning_command_duration_;  //控制命令发送周期
  uint32_t status_detect_duration_;  //状态检测周期
  std::ofstream trajectory_point_log_file_;
  bool enable_log_debug_;
  bool enable_local_map_topic_;
  
  // 功能激活状态,激活为true，未激活为false
  bool function_activation_;

  //
  // std::map<std::string, legionclaw::common::MessageStatus> message_status_;  //消息状态
  std::unique_ptr<std::thread> task_thread_;  // task线程
  std::mutex mutex_;
  LocalView local_view_;
  std::shared_ptr<DependencyInjector> injector_;
  Frame frame_;
  LongitudeInfoProvider lon_info_provider_;
  legionclaw::common::Status plan_status_;

  // planner
  LatticePlanner lattice_planner_;
  ParkingPlanner parking_planner_;
  // ParkingOutPlanner parking_out_planner_;
protected:
  //定时器
  std::shared_ptr<ADTimerManager<Planning, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<Planning, void>> task_compute_;
  std::shared_ptr<WheelTimer<Planning, void>> task_publish_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  
  /*********************
   *  全局状态机域
   *********************/
protected:
  bool PlanningStateMachineInit();
  bool AutoDrivingEnvInit();
  void AutoDrivingStateUpdate(const std::string &state_name, int state);
  void HumanDrivingStateUpdate(const std::string &state_name, int state);
  void PlanningStartStateUpdate(const std::string &state_name, int state);
  void DrivingPlanningStateUpdate(const std::string &state_name, int state);
  void LatticePlannerStateUpdate(const std::string &state_name, int state);
  void ESlowStopPlanStateUpdate(const std::string &state_name, int state);
  void EQuickStopPlanStateUpdate(const std::string &state_name, int state);
  void ParkingPlanningStateUpdate(const std::string &state_name, int state);
  void ParkingPlannerStateUpdate(const std::string &state_name, int state);
  void ParkingPublisherStateUpdate(const std::string &state_name, int state);
  void ParkingOutPlanningStateUpdate(const std::string &state_name, int state);
  void ParkingOutPlannerStateUpdate(const std::string &state_name, int state);
  void ParkingOutPublishStateUpdate(const std::string &state_name, int state);

  void AfterHumanDriving();
  void ChangeDrivingModeAndBehaviourLatState();
  void TrajectoryToTakeOver();
private:
  std::unique_ptr<state_machine::StateContext> planning_sm_;
  //状态切换Flag
  DrivingFlag driving_flag_;
  TaskStatus task_status_;
  PlanningFlag planning_flag_;
  PlannerFlag planner_flag_;
  int parking_out_state_ = 0; // 0:invalid; 1:planning; 2:publish; 
  ParkingManager::OutDirection parking_out_direction_; // 0:invalid; 1:left; 2:forward; 3:right; 4:back;
  int emergency_state_ = 0;
  int waitting_state_ = 0;

};
}  // namespace planning
}  // namespace legionclaw
