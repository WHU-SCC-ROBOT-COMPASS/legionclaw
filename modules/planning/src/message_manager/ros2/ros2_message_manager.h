/**
 * @file    ros_2_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once
#if ROS2_ENABLE
#include <mutex>
#include <thread>

#include "message_manager/message_manager.h"
#include "modules/common/enum/enum.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_interface/msg/adc_trajectory.hpp"
#include "ros2_interface/msg/chassis.hpp"
#include "ros2_interface/msg/drivable_region.hpp"
#include "ros2_interface/msg/events.hpp"
#include "ros2_interface/msg/faults.hpp"
#include "ros2_interface/msg/guide_info.hpp"
#include "ros2_interface/msg/lane_list.hpp"
#include "ros2_interface/msg/location.hpp"
#include "ros2_interface/msg/obu_cmd_msg.hpp"
#include "ros2_interface/msg/parking_info.hpp"
#include "ros2_interface/msg/parking_out_info.hpp"
#include "ros2_interface/msg/parking_state_display.hpp"
#include "ros2_interface/msg/planning_analysis.hpp"
#include "ros2_interface/msg/planning_cmd.hpp"
#include "ros2_interface/msg/prediction_obstacles.hpp"
#include "ros2_interface/msg/routing_response.hpp"
#include "ros2_interface/msg/sotif_monitor_result.hpp"
#include "ros2_interface/msg/stop_info.hpp"
#include "ros2_interface/msg/traffic_events.hpp"
#include "ros2_interface/msg/traffic_light_msg.hpp"
#include "ros2_interface/msg/trajectory_array.hpp"
#include "std_msgs/msg/header.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
/**
 * @class Ros2MessageManager
 * @brief ROS2消息管理器.
 */
template <typename T>
class Ros2MessageManager : public MessageManager<T>, public rclcpp::Node {
 public:
  Ros2MessageManager();
  ~Ros2MessageManager() = default;

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
  void PublishPlanningAnalysis(
      legionclaw::interface::PlanningAnalysis msg) override;

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

  /**
   * @brief     激活消息管理器.
   * @return    bool 成功返回true，失败返回false.
   */
  bool Activate() override;

  /**
   * @brief     停用消息管理器.
   * @return    bool 成功返回true，失败返回false.
   */
  bool DeActivate() override;

 protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  // 0 do_nothing ; 1 activate ; 2 deactivate
  MessageActionMode action_mode_;
  std::mutex mode_mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

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

 protected:
  rclcpp::Publisher<::ros2_interface::msg::ADCTrajectory>::SharedPtr
      adc_trajectory_pub_;
  rclcpp::Publisher<::ros2_interface::msg::PlanningCmd>::SharedPtr
      planning_cmd_pub_;
  rclcpp::Publisher<::ros2_interface::msg::PlanningAnalysis>::SharedPtr
      planning_analysis_pub_;
  rclcpp::Publisher<::ros2_interface::msg::ParkingStateDisplay>::SharedPtr
      parking_state_display_pub_;
  rclcpp::Publisher<::ros2_interface::msg::TrajectoryArray>::SharedPtr
      trajectory_array_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<::ros2_interface::msg::Events>::SharedPtr events_pub_;

  rclcpp::Subscription<::ros2_interface::msg::RoutingResponse>::SharedPtr
      routing_response_sub_;
  rclcpp::Subscription<::ros2_interface::msg::RoutingResponse>::SharedPtr
      local_routing_response_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ParkingInfo>::SharedPtr
      parking_info_sub_;
  rclcpp::Subscription<::ros2_interface::msg::StopInfo>::SharedPtr
      stop_info_sub_;
  rclcpp::Subscription<::ros2_interface::msg::TrafficLightMsg>::SharedPtr
      traffic_light_msg_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Location>::SharedPtr
      location_sub_;
  rclcpp::Subscription<::ros2_interface::msg::PredictionObstacles>::SharedPtr
      prediction_obstacles_sub_;
  rclcpp::Subscription<::ros2_interface::msg::LaneList>::SharedPtr
      lane_list_sub_;
  rclcpp::Subscription<::ros2_interface::msg::Chassis>::SharedPtr chassis_sub_;
  rclcpp::Subscription<::ros2_interface::msg::SotifMonitorResult>::SharedPtr
      sotif_monitor_result_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ObuCmdMsg>::SharedPtr
      obu_cmd_msg_sub_;
  rclcpp::Subscription<::ros2_interface::msg::DrivableRegion>::SharedPtr
      drivable_region_sub_;
  rclcpp::Subscription<::ros2_interface::msg::ParkingOutInfo>::SharedPtr
      parking_out_info_sub_;
  rclcpp::Subscription<::ros2_interface::msg::GuideInfo>::SharedPtr
      guide_info_sub_;
  rclcpp::Subscription<::ros2_interface::msg::TrafficEvents>::SharedPtr
      traffic_events_sub_;

  void HandleRoutingResponseMessage(
      const ros2_interface::msg::RoutingResponse::SharedPtr msg_obj);

  void HandleLocalRoutingResponseMessage(
      const ros2_interface::msg::RoutingResponse::SharedPtr msg_obj);

  void HandleParkingInfoMessage(
      const ros2_interface::msg::ParkingInfo::SharedPtr msg_obj);

  void HandleStopInfoMessage(
      const ros2_interface::msg::StopInfo::SharedPtr msg_obj);

  void HandleTrafficLightMsgMessage(
      const ros2_interface::msg::TrafficLightMsg::SharedPtr msg_obj);

  void HandleLocationMessage(
      const ros2_interface::msg::Location::SharedPtr msg_obj);

  void HandlePredictionObstaclesMessage(
      const ros2_interface::msg::PredictionObstacles::SharedPtr msg_obj);

  void HandleLaneListMessage(
      const ros2_interface::msg::LaneList::SharedPtr msg_obj);

  void HandleChassisMessage(
      const ros2_interface::msg::Chassis::SharedPtr msg_obj);

  void HandleSotifMonitorResultMessage(
      const ros2_interface::msg::SotifMonitorResult::SharedPtr msg_obj);

  void HandleObuCmdMsgMessage(
      const ros2_interface::msg::ObuCmdMsg::SharedPtr msg_obj);

  void HandleDrivableRegionMessage(
      const ros2_interface::msg::DrivableRegion::SharedPtr msg_obj);

  void HandleParkingOutInfoMessage(
      const ros2_interface::msg::ParkingOutInfo::SharedPtr msg_obj);

  void HandleGuideInfoMessage(
      const ros2_interface::msg::GuideInfo::SharedPtr msg_obj);

  void HandleTrafficEventsMessage(
      const ros2_interface::msg::TrafficEvents::SharedPtr msg_obj);
};
}  // namespace planning
}  // namespace legionclaw
#include "ros2_message_manager.hpp"
#endif  // ROS2_ENABLE
