/**
 * @file         timer_manager_test.h
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
 * @class CommonTimer
 *
 * @brief  Common Timer.
 */
class TimerManagerTest {
public:
  TimerManagerTest() = default;
  ~TimerManagerTest() = default;
  void Init();
  void Join();
  void Spin();
  void T1();
  void T2();
  void T3();
  void T4();
  void T5();

  std::unique_ptr<std::thread> task_thread_;

  // std::unique_ptr<TimerManager<TimerManagerTest> > timer_manager_;
};
} // namespace common
} // namespace legionclaw
