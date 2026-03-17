/**
 * @file         timer_tool_test.h
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#pragma once

#include "common/timer/timer_manager.h"

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

/**
 * @class TimeToolTest
 *
 * @brief  TimeToolTest.
 */
class TimeToolTest {
public:
  TimeToolTest() = default;
  ~TimeToolTest() = default;
  void Init();
  void Join();
  void Spin();

  std::unique_ptr<std::thread> task_thread_;
};
} // namespace common
} // namespace legionclaw
