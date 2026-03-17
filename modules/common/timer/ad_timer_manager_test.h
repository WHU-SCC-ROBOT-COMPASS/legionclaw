/**
 * @file         ad_timer_manager_test.h
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#pragma once
#include <memory>
#include <thread>
#include "common/timer/ad_timer_manager.h"

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
class ADTimerManagerTest {
public:
  ADTimerManagerTest() = default;
  ~ADTimerManagerTest() = default;
  void Init();
  void Join();
  void Spin();
  void HDSpin();
  void T1(void* param);
  void T2(void* param);
  void T3(void* param);
  void T4(void* param);
  void T5(void* param);
  void T6(void* param);
  void T7(void* param);
  void T8(void* param);
  void T9(void* param);
  void T10(void* param);

  std::unique_ptr<std::thread> task_thread_;
  std::unique_ptr<std::thread> hd_task_thread_;
  ///时间轮定时器mananger
  std::shared_ptr<ADTimerManager<ADTimerManagerTest, void>> ad_timer_manager_;
  std::shared_ptr<ADTimerManager<ADTimerManagerTest, void>>
      ad_hd_timer_manager_;
  ///发送can总线控制命令定时器
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer1_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer2_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer3_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer4_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer5_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer6_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer7_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer8_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer9_;
  std::shared_ptr<WheelTimer<ADTimerManagerTest, void>> ad_timer10_;
};
} // namespace common
} // namespace legionclaw
