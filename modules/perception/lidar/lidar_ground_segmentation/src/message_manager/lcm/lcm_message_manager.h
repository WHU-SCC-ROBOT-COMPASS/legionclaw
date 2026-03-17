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
#include "lcm_interface/PointCloud.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {
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
   * @brief     PointCloud消息发送.
   * @param[in] point_cloud
   * @return    void.
   */
  void PublishGroundPoints(legionclaw::interface::PointCloud msg) override;

  /**
   * @brief     PointCloud消息发送.
   * @param[in] point_cloud
   * @return    void.
   */
  void PublishNoGroundPoints(legionclaw::interface::PointCloud msg) override;

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

  lcm::Subscription* point_cloud_input_sub_;
  void HandlePointCloudInputMessage(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const lcm_interface::PointCloud* msg);

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
} // namespace lidar
} // namespace perception
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif