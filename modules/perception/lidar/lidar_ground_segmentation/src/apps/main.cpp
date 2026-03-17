/**
 * @file    main.cpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <iostream>
#include <signal.h>

#include "lidar_ground_segmentation.h"

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
using namespace legionclaw::perception::lidar;

int main(int argc, char** argv) {
  std::string file_path =
      "./conf/perception/lidar/lidar_ground_segmentation/lidar_ground_segmentation.json";
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "lidar_ground_segmentation");
#endif

#if ROS2_ENABLE
  rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
  google::InitGoogleLogging(argv[0]);
#endif
  LidarGroundSegmentation* lidar_ground_segmentation = new LidarGroundSegmentation(file_path);
  lidar_ground_segmentation->Init();
  lidar_ground_segmentation->Join();
  return 1;
}