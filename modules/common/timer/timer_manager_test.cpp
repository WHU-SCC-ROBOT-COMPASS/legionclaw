/**
 * @file         timer_manager_test.cpp
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#include <iostream>
#include <unistd.h>
#include "common/timer/timer_manager_test.h"

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

void TimerManagerTest::Init() {
  TimerManager<TimerManagerTest>::AddTimer(10, &TimerManagerTest::T1, this);
  TimerManager<TimerManagerTest>::AddTimer(20, &TimerManagerTest::T2, this);
  TimerManager<TimerManagerTest>::AddTimer(30, &TimerManagerTest::T3, this);
  TimerManager<TimerManagerTest>::AddTimer(40, &TimerManagerTest::T4, this);
  TimerManager<TimerManagerTest>::AddTimer(50, &TimerManagerTest::T5, this);
  task_thread_.reset(new std::thread([this] { Spin(); }));
  if (task_thread_ == nullptr) {
    std::cout << "Unable to create can task_thread_ thread." << "\n";
    return;
  }
}

void TimerManagerTest::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    std::cout << "task_thread_ stopped [ok]." << "\n";
  }
}

void TimerManagerTest::Spin() {
  while (1) {
    sleep(1);
  }
}

void TimerManagerTest::T1() {
  std::cout << "T1" << "\n";
  usleep(5000);
}

void TimerManagerTest::T2() {
  std::cout << "T2" << "\n";
  usleep(5000);
}

void TimerManagerTest::T3() {
  std::cout << "T3" << "\n";
  usleep(5000);
}

void TimerManagerTest::T4() {
  std::cout << "T4" << "\n";
  usleep(5000);
}

void TimerManagerTest::T5() {
  std::cout << "T5" << "\n";
  usleep(5000);
}
} // namespace common
} // namespace legionclaw
