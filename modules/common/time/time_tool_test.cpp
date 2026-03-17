/**
 * @file         time_tool_test.cpp
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#include <iostream>
#include <unistd.h>
#include "common/time/time_tool.h"
#include "common/time/time_tool_test.h"

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

void TimeToolTest::Init() {
  task_thread_.reset(new std::thread([this] { Spin(); }));
  if (task_thread_ == nullptr) {
    std::cout << "Unable to create can task_thread_ thread." << "\n";
    return;
  }
}

void TimeToolTest::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    std::cout << "task_thread_ stopped [ok]." << "\n";
  }
}

void TimeToolTest::Spin() {
  while (1) {
    std::chrono::system_clock::time_point start_time;
    std::cout << "ElapseSeconds: " << TimeTool::ElapseSeconds(start_time)
              << "\n";
    std::cout << "ElapseMs: " << TimeTool::ElapseMs(start_time) << "\n";
    std::cout << "ElapseUs: " << TimeTool::ElapseUs(start_time) << "\n";
    std::cout << "Now2Seconds: " << TimeTool::Now2Seconds() << "\n";
    std::cout << "Now2Ms: " << TimeTool::Now2Ms() << "\n";
    std::cout << "Now2Us: " << TimeTool::Now2Us() << "\n";

    // std::chrono::system_clock::time_point t(std::chrono::seconds(2));
    // std::cout << "ToSeconds: " << TimeTool::ToSeconds(t) << "\n";
    // std::cout << "ToMs: " << TimeTool::ToMs(t) << "\n";
    // std::cout << "ToUs: " << TimeTool::ToUs(t) << "\n";
    // std::cout << "ToStr: " << TimeTool::ToStr(t) << "\n";
    // std::cout << "ToStr: " << TimeTool::ToStr(t) << "\n";

    std::cout << "Now2MsEx: " << TimeTool::Now2MsEx() << "\n";

    std::cout << "ToTime: " << TimeTool::ToTime("2020-03-26 17:16:32")
              << "\n";

    std::chrono::system_clock::time_point t =
        TimeTool::ToTimeEx("2020-03-26 17:16:32:123");
    std::cout << "ToSeconds: " << TimeTool::ToSeconds(t) << "\n";
    std::cout << "ToMs: " << TimeTool::ToMs(t) << "\n";
    std::cout << "ToUs: " << TimeTool::ToUs(t) << "\n";
    std::cout << "ToStr: " << TimeTool::ToStr(t) << "\n";
    std::cout << "ToStr: " << TimeTool::ToStr(t) << "\n";
    std::cout << "\n";
    sleep(1);
  }
}
} // namespace common
} // namespace legionclaw
