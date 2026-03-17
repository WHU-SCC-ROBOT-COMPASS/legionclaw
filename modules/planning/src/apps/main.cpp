/**
 * @file    main.cpp
 * @author  zdhy
 * @date    2021-09-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "modules/planning/src/apps/planning.h"

#if GLOG_ENABLE
#include <glog/logging.h>
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
#endif

#if ROS_ENABLE
#include <ros/ros.h>
#endif

using namespace std;
using namespace legionclaw::planning;

int main(int argc, char **argv)
{
    std::string file_path = "./conf/planning/planning.json";
    system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "planning");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  std::cout << "version: 6.1.5.1.20251226_b" << "\n";
  Planning *planning = new Planning(file_path);
  planning->Init();
  planning->Join();
  return 1;
}


