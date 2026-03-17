/**
 * @file    ros_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS_ENABLE
#include <thread>

#include <ros/ros.h>

#include "ros_interface/Faults.h"
#include "ros_interface/Events.h"
#include "ros_interface/Chassis.h"
#include "ros_interface/StopInfo.h"
#include "ros_interface/Location.h"
#include "ros_interface/LaneList.h"
#include "ros_interface/ObuCmdMsg.h"
#include "ros_interface/ParkingInfo.h"
#include "ros_interface/PlanningCmd.h"
#include "ros_interface/TrafficEvents.h"
#include "ros_interface/ADCTrajectory.h"
#include "ros_interface/DrivableRegion.h"
#include "ros_interface/ParkingOutInfo.h"
#include "ros_interface/RoutingResponse.h"
#include "ros_interface/TrafficLightMsg.h"
#include "ros_interface/TrajectoryArray.h"
#include "ros_interface/PlanningAnalysis.h"
#include "ros_interface/SotifMonitorResult.h"
#include "ros_interface/PredictionObstacles.h"
#include "ros_interface/ParkingStateDisplay.h"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
/**
 * @class RosMessageManager
 * @brief ROS消息管理器.
 */
template <typename T> class RosMessageManager : public MessageManager<T> {
public:
  RosMessageManager() = default;
  ~RosMessageManager() = default;

  /**
   * @brief     初始化。
   * @param[in] obu_url LCM组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  /**
   * @brief     ADCTrajectory消息发布.
   * @param[in] adc_trajectory.
   * @return    void.
   */
  void PublishADCTrajectory(legionclaw::interface::ADCTrajectory msg) override;

  /**
   * @brief     PlanningCmd消息发布.
   * @param[in] planning_cmd.
   * @return    void.
   */
  void PublishPlanningCmd(legionclaw::interface::PlanningCmd msg) override;

  /**
   * @brief     PlanningAnalysis消息发布.
   * @param[in] planning_analysis.
   * @return    void.
   */
  void
  PublishPlanningAnalysis(legionclaw::interface::PlanningAnalysis msg) override;

  /**
   * @brief     ParkingStateDisplay消息发布.
   * @param[in] parking_state_display.
   * @return    void.
   */
  void PublishParkingStateDisplay(
      legionclaw::interface::ParkingStateDisplay msg) override;

  /**
   * @brief     TrajectoryArray消息发布.
   * @param[in] trajectory_array.
   * @return    void.
   */
  void PublishTrajectoryArray(legionclaw::interface::TrajectoryArray msg) override;

  /**
   * @brief     Faults消息发布.
   * @param[in] faults.
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Events消息发布.
   * @param[in] events.
   * @return    void.
   */
  void PublishEvents(legionclaw::interface::Events msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;
  std::mutex mode_mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

  std::unique_ptr<std::thread> register_thread_;

  /**
   * @brief     线程运行函数.
   * @return    void.
   */
  void Run();

  /**
   * @brief     线程结束函数.
   * @return    void.
   */
  void Stop();

  void TaskStop();

  void TaskStart();

  void RegisterLoop();

protected:
  ros::NodeHandle nh_;
  std::mutex mutex_;
  ros::Publisher adc_trajectory_pub_;
  ros::Publisher planning_cmd_pub_;
  ros::Publisher planning_analysis_pub_;
  ros::Publisher parking_state_display_pub_;
  ros::Publisher trajectory_array_pub_;
  ros::Publisher faults_pub_;
  ros::Publisher events_pub_;

  ros::Subscriber routing_response_sub_;
  ros::Subscriber local_routing_response_sub_;
  ros::Subscriber parking_info_sub_;
  ros::Subscriber stop_info_sub_;
  ros::Subscriber traffic_light_msg_sub_;
  ros::Subscriber location_sub_;
  ros::Subscriber prediction_obstacles_sub_;
  ros::Subscriber lane_list_sub_;
  ros::Subscriber chassis_sub_;
  ros::Subscriber sotif_monitor_result_sub_;
  ros::Subscriber obu_cmd_msg_sub_;
  ros::Subscriber drivable_region_sub_;
  ros::Subscriber parking_out_info_sub_;
  ros::Subscriber traffic_events_sub_;

  void
  HandleRoutingResponseMessage(const ros_interface::RoutingResponse& msg_obj);

  void HandleLocalRoutingResponseMessage(
      const ros_interface::RoutingResponse& msg_obj);

  void HandleParkingInfoMessage(const ros_interface::ParkingInfo& msg_obj);

  void HandleStopInfoMessage(const ros_interface::StopInfo& msg_obj);

  void
  HandleTrafficLightMsgMessage(const ros_interface::TrafficLightMsg& msg_obj);

  void HandleLocationMessage(const ros_interface::Location& msg_obj);

  void HandlePredictionObstaclesMessage(
      const ros_interface::PredictionObstacles& msg_obj);

  void HandleLaneListMessage(const ros_interface::LaneList& msg_obj);

  void HandleChassisMessage(const ros_interface::Chassis& msg_obj);

  void HandleSotifMonitorResultMessage(
      const ros_interface::SotifMonitorResult& msg_obj);

  void HandleObuCmdMsgMessage(const ros_interface::ObuCmdMsg& msg_obj);

  void
  HandleDrivableRegionMessage(const ros_interface::DrivableRegion& msg_obj);

  void
  HandleParkingOutInfoMessage(const ros_interface::ParkingOutInfo& msg_obj);

  void HandleTrafficEventsMessage(const ros_interface::TrafficEvents& msg_obj);
};
} // namespace planning
} // namespace legionclaw
#include "ros_message_manager.hpp"
#endif // ROS_ENABLE
