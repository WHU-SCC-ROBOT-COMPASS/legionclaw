/**
 * @file    main.cpp
 * @author  jiangchengjie
 * @date    2021-07-25
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "control.h"

#if GLOG_ENABLE
#include <glog/logging.h>
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
#endif

#if ROS_ENABLE
#include <ros/ros.h>
#endif

using namespace std;
using namespace legionclaw::control;

int main(int argc, char **argv) {
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "control");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  std::cout << "version: 6.1.8.1.20251229_b" << "\n";
  Control *control = new Control(FLAGS_control_config_file);
  control->Init();
  control->Join();
  return 1;
}
