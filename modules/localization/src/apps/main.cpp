/**
 * @file    main.cpp
 * @brief   Localization node - NDT scan matching with UKF pose estimation
 * @author  LegionClaw Team (based on koide3/hdl_localization)
 * @license BSD
 */

#include <iostream>
#include <memory>
#include <signal.h>

#include "modules/localization/src/apps/localization.h"

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
    std::string config_path = "./conf/localization/localization.json";
    system("mkdir -p log");

#if ROS_ENABLE
    ros::init(argc, argv, "localization");
#endif

#if ROS2_ENABLE
    rclcpp::init(argc, argv);
#endif

#if GLOG_ENABLE
    google::InitGoogleLogging(argv[0]);
#endif

    std::cout << "localization module version: 1.0.0 (ROS2 SLAM Integration)" << std::endl;

    legionclaw::localization::LocalizationNode* loc_node =
        new legionclaw::localization::LocalizationNode(config_path);
    loc_node->Init();
    loc_node->Join();

    return 0;
}
