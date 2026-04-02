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

#include "lcm_interface/Location.hpp"
#include "lcm_interface/ObstacleList.hpp"

#include "message_manager/message_manager.h"

/**
 * @namespace legionclaw::perception::fusion
 * @brief legionclaw::perception::fusion
 */

namespace legionclaw {
namespace perception {
namespace fusion {
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
  void PublishObstacleListOutput(legionclaw::interface::ObstacleList msg) override;

protected:
  T *instance_;
  bool is_init_;
  std::shared_ptr<lcm::LCM> lcm_;

  std::unique_ptr<std::thread> handle_message_thread_;

  double receive_time;
  double publish_time;

  void HandleLocationMessage(const lcm::ReceiveBuffer *rbuf,
                             const std::string &chan,
                             const lcm_interface::Location *msg);

  void HandleObstacleListInputMessage(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const lcm_interface::ObstacleList *msg);

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
} // namespace fusion
} // namespace perception
} // namespace legionclaw
#include "lcm_message_manager.hpp"
#endif