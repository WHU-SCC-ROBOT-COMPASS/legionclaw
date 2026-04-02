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
#include <cstdlib>
#include <filesystem>
#include <string>

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

#if ROS2_ENABLE
static void AppendPathEnv(const char* key, const std::string& value) {
  if (value.empty()) {
    return;
  }
  const char* old_val = std::getenv(key);
  if (old_val == nullptr || std::string(old_val).empty()) {
    setenv(key, value.c_str(), 1);
    return;
  }
  std::string merged = std::string(old_val);
  if (merged.find(value) == std::string::npos) {
    merged = value + ":" + merged;
    setenv(key, merged.c_str(), 1);
  }
}

static void EnsureRos2InterfaceRuntimeLibPath() {
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path exe_path = fs::read_symlink("/proc/self/exe", ec);
  if (ec) {
    return;
  }
  fs::path exe_dir = exe_path.parent_path();

  // bin -> lidar_ground_segmentation -> lidar -> perception -> modules -> message/ros2/install/ros2_interface/lib
  fs::path rel_lib_dir = exe_dir / "../../../../message/ros2/install/ros2_interface/lib";
  rel_lib_dir = fs::weakly_canonical(rel_lib_dir, ec);
  if (!ec && fs::exists(rel_lib_dir)) {
    AppendPathEnv("LD_LIBRARY_PATH", rel_lib_dir.string());
  }

  // 兼容固定工作区路径
  fs::path abs_lib_dir = "/home/lenovo/project/legionclaw/modules/message/ros2/install/ros2_interface/lib";
  if (fs::exists(abs_lib_dir)) {
    AppendPathEnv("LD_LIBRARY_PATH", abs_lib_dir.string());
  }
}
#endif

int main(int argc, char** argv) {
  std::string file_path =
      "./conf/perception/lidar/lidar_ground_segmentation/lidar_ground_segmentation.json";
  system("mkdir -p log");
#if ROS_ENABLE
  ros::init(argc, argv, "lidar_ground_segmentation");
#endif

#if ROS2_ENABLE
  EnsureRos2InterfaceRuntimeLibPath();
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
