/**
 * @file    main.cpp
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "prediction.h"

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
using namespace legionclaw::prediction;

int main(int argc, char** argv) {
  std::string file_path = "./conf/prediction/prediction.json";
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "prediction");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  std::cout << "version: 6.1.2.1.20251202_b" << "\n";
  Prediction* prediction = new Prediction(file_path);
  prediction->Init();
  prediction->Join();
  return 1;
}