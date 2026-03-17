/**
 * @file    main.cpp
 * @author  zdhy
 * @date    2021-10-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "routing.h"

#if GLOG_ENABLE
#include <glog/logging.h>
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
#endif

#if ROS_ENABLE
#include <ros/ros.h>
#endif

#if ROS2_ENABLE
#include "rclcpp/rclcpp.hpp"
#endif

using namespace std;
using namespace legionclaw::routing;

int main(int argc, char **argv) {
  std::string file_path = "./conf/routing/routing.json";
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "routing");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  std::cout << "version: 6.1.1.5.20251202_b" << "\n";
  Routing *routing = new Routing(file_path);
  routing->Init();
  routing->Join();
  return 1;
}
