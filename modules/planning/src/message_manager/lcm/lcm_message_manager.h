/**
 * @file    lcm_message_manager.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if LCM_ENABLE
#include <thread>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_interface/Faults.hpp"
#include "lcm_interface/Events.hpp"
#include "lcm_interface/Chassis.hpp"
#include "lcm_interface/StopInfo.hpp"
#include "lcm_interface/Location.hpp"
#include "lcm_interface/LaneList.hpp"
#include "lcm_interface/ObuCmdMsg.hpp"
#include "lcm_interface/GuideInfo.hpp"
#include "lcm_interface/ParkingInfo.hpp"
#include "lcm_interface/PlanningCmd.hpp"
#include "lcm_interface/TrafficEvents.hpp"
#include "lcm_interface/ADCTrajectory.hpp"
#include "lcm_interface/DrivableRegion.hpp"
#include "lcm_interface/ParkingOutInfo.hpp"
#include "lcm_interface/RoutingResponse.hpp"
#include "lcm_interface/TrafficLightMsg.hpp"
#include "lcm_interface/TrajectoryArray.hpp"
#include "lcm_interface/PlanningAnalysis.hpp"
#include "lcm_interface/SotifMonitorResult.hpp"
#include "lcm_interface/PredictionObstacles.hpp"
#include "lcm_interface/ParkingStateDisplay.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */

namespace legionclaw {
namespace planning {
/**
 * @class LcmMessageManager
 * @brief LCM消息管理器.
 */
template <typename T> class LcmMessageManager : public MessageManager<T> {
public:
  LcmMessageManager() = default;
  ~LcmMessageManager() = default;

  /**
   * @brief     初始化。
   * @param[in] obu_url LCM组播信息.
   * @return    void.
   */
  void Init(T* t) override;

  /**
   * @brief     ADCTrajectory消息发送.
   * @param[in] adctrajectory
   * @return    void.
   */
  void PublishADCTrajectory(legionclaw::interface::ADCTrajectory msg) override;

  /**
   * @brief     PlanningCmd消息发送.
   * @param[in] planning_cmd
   * @return    void.
   */
  void PublishPlanningCmd(legionclaw::interface::PlanningCmd msg) override;

  /**
   * @brief     PlanningAnalysis消息发送.
   * @param[in] planning_analysis
   * @return    void.
   */
  void
  PublishPlanningAnalysis(legionclaw::interface::PlanningAnalysis msg) override;

  /**
   * @brief     ParkingStateDisplay消息发送.
   * @param[in] parking_state_display
   * @return    void.
   */
  void PublishParkingStateDisplay(
      legionclaw::interface::ParkingStateDisplay msg) override;

  /**
   * @brief     TrajectoryArray消息发送.
   * @param[in] trajectory_array
   * @return    void.
   */
  void PublishTrajectoryArray(legionclaw::interface::TrajectoryArray msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  /**
   * @brief     Events消息发送.
   * @param[in] events
   * @return    void.
   */
  void PublishEvents(legionclaw::interface::Events msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  std::shared_ptr<lcm::LCM> lcm_;
  std::mutex mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

  lcm::Subscription* routing_response_sub_;
  void HandleRoutingResponseMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::RoutingResponse* msg);

  lcm::Subscription* local_routing_response_sub_;
  void
  HandleLocalRoutingResponseMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::RoutingResponse* msg);

  lcm::Subscription* parking_info_sub_;
  void HandleParkingInfoMessage(const lcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const lcm_interface::ParkingInfo* msg);

  lcm::Subscription* stop_info_sub_;
  void HandleStopInfoMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::StopInfo* msg);

  lcm::Subscription* traffic_light_msg_sub_;
  void HandleTrafficLightMsgMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::TrafficLightMsg* msg);

  lcm::Subscription* location_sub_;
  void HandleLocationMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Location* msg);

  lcm::Subscription* prediction_obstacles_sub_;
  void HandlePredictionObstaclesMessage(
      const lcm::ReceiveBuffer* rbuf, const std::string& chan,
      const lcm_interface::PredictionObstacles* msg);

  lcm::Subscription* lane_list_sub_;
  void HandleLaneListMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::LaneList* msg);

  lcm::Subscription* chassis_sub_;
  void HandleChassisMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcm_interface::Chassis* msg);

  lcm::Subscription* sotif_monitor_result_sub_;
  void
  HandleSotifMonitorResultMessage(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const lcm_interface::SotifMonitorResult* msg);

  lcm::Subscription* obu_cmd_msg_sub_;
  void HandleObuCmdMsgMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const lcm_interface::ObuCmdMsg* msg);

  lcm::Subscription* drivable_region_sub_;
  void HandleDrivableRegionMessage(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const lcm_interface::DrivableRegion* msg);

  lcm::Subscription* parking_out_info_sub_;
  void HandleParkingOutInfoMessage(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& chan,
                                   const lcm_interface::ParkingOutInfo* msg);

  lcm::Subscription* guide_info_sub_;
  void HandleGuideInfoMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const lcm_interface::GuideInfo* msg);

  lcm::Subscription* traffic_events_sub_;
  void HandleTrafficEventsMessage(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const lcm_interface::TrafficEvents* msg);

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
};
} // namespace planning
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif
