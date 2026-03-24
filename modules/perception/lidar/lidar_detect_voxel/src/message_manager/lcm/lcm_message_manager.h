/**
 * @file    lcm_message_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#if LCM_ENABLE
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include <thread>

#include "lcm_interface/ObstacleList.hpp"
#include "lcm_interface/PointCloud.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
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
  void Init(T *t) override;

  /**
   * @brief     ObstacleList消息发送.
   * @param[in] obstacle_list
   * @return    void.
   */
  void PublishObstacleList(legion::interface::ObstacleList msg) override;

protected:
  T *instance_;
  bool is_init_;
  std::shared_ptr<lcm::LCM> lcm_;

  std::unique_ptr<std::thread> handle_message_thread_;

  void HandlePointCloudMessage(const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const lcm_interface::PointCloud *msg);

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
} // namespace legion
#include "lcm_message_manager.hpp"
#endif