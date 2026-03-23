/**
 * @file    main.cpp
 * @brief   FAST-LIO-SAM 主入口 - LiDAR-Inertial SLAM with ISAM2 backend
 * @author  LegionClaw Team (based on Ji Zhang's FAST-LIO-SAM)
 * @license BSD
 */

#include <iostream>
#include <memory>
#include <signal.h>

#include <modules/map/src/slam/fast_lio_sam.h>
#include <modules/map/src/slam/laserMappingNode.h>

#if GLOG_ENABLE
#include <glog/logging.h>
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
#endif

#if ROS_ENABLE
#include <ros/ros.h>
#endif

#if ROS2_ENABLE
#include <rclcpp/rclcpp.hpp>
#endif

int main(int argc, char** argv) {
    system("mkdir -p log");

#if ROS_ENABLE
    ros::init(argc, argv, "laser_mapping");
#endif

#if ROS2_ENABLE
    rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
    google::InitGoogleLogging(argv[0]);
#endif

    std::cout << "map module version: 1.0.0 (FAST-LIO-SAM)" << std::endl;

    // 默认配置
    slam::FastLIOSAM::Config cfg;
    cfg.acc_cov = 0.1;
    cfg.gyr_cov = 0.1;
    cfg.b_acc_cov = 0.0001;
    cfg.b_gyr_cov = 0.0001;
    cfg.max_iteration = 4;
    cfg.filter_size_surf = 0.5;
    cfg.cube_side_length = 200.0;
    cfg.det_range = 300.0;
    cfg.loop_closure_en = false;

    // 使用 laserMapping.cpp 中的 LaserMappingNode
    legionclaw::map::LaserMappingNode node(cfg, "laser_mapping");
    node.spin();

    return 0;
}
