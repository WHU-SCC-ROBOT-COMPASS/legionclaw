/**
 * @file    main.cpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "motion_manager.h"

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
using namespace legionclaw::perception::fusion;

int main(int argc, char **argv) {
  std::string file_path =
      "./conf/perception/fusion/motion_manager/motion_manager.json";
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "motion_manager");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  MotionManager *motion_manager = new MotionManager(file_path);
  motion_manager->Init();
  motion_manager->Join();
  return 1;
}