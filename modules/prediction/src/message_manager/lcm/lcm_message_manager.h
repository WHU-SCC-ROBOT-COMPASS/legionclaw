/**
 * @file    lcm_message_manager.h
 * @author  legionclaw
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

#include "lcm_interface/ObuCmdMsg.hpp"
#include "lcm_interface/Faults.hpp"
#include "lcm_interface/Location.hpp"
#include "lcm_interface/Odometry.hpp"
#include "lcm_interface/LaneList.hpp"
#include "lcm_interface/ObstacleList.hpp"
#include "lcm_interface/ADCTrajectory.hpp"
#include "lcm_interface/TrafficEvents.hpp"
#include "lcm_interface/RoutingResponse.hpp"
#include "lcm_interface/PredictionObstacles.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
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
   * @brief     PredictionObstacles消息发送.
   * @param[in] prediction_obstacles
   * @return    void.
   */
  void PublishPredictionObstacles(
      legionclaw::interface::PredictionObstacles msg) override;

  /**
   * @brief     Faults消息发送.
   * @param[in] faults
   * @return    void.
   */
  void PublishFaults(legionclaw::interface::Faults msg) override;

  bool Activate() override;
  bool DeActivate() override;

protected:
  T* instance_;
  bool is_init_;
  bool is_active_;
  std::shared_ptr<lcm::LCM> lcm_;
  std::mutex mutex_;

  std::unique_ptr<std::thread> handle_message_thread_;

  lcm::Subscription* routing_reponse_input_sub_;
  lcm::Subscription* location_sub_;
  lcm::Subscription* traffic_events_sub_;
  lcm::Subscription* adc_trajectory_sub_;
  lcm::Subscription* lanelist_input_sub_;
  lcm::Subscription* obstacle_list_sub_;
  lcm::Subscription* odometry_sub_;

  void HandleObuCmdMsgMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcm_interface::ObuCmdMsg* msg);

  void HandleLocationMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Location* msg);

  void HandleADCTrajectoryMessage(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const lcm_interface::ADCTrajectory* msg);

  void HandleObstacleListMessage(const lcm::ReceiveBuffer* rbuf,
                                 const std::string& chan,
                                 const lcm_interface::ObstacleList* msg);

  void HandleOdometryMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::Odometry* msg);

  void HandleTrafficEventsMessage(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const lcm_interface::TrafficEvents* msg);

  void HandleRoutingResponseMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::RoutingResponse* msg);

  void HandleLaneListMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const lcm_interface::LaneList* msg);

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
} // namespace prediction
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif