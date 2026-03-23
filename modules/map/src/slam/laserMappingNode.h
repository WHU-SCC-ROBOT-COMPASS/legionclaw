/**
 * @file    laserMappingNode.h
 * @brief   FAST-LIO-SAM 应用层头文件
 * @author  LegionClaw Team
 */

#ifndef LASER_MAPPING_NODE_H
#define LASER_MAPPING_NODE_H

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <fstream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "modules/map/src/slam/fast_lio_sam.h"

// ROS2 头文件
#if ROS2_ENABLE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>

#include <ros2_slam_msgs/srv/save_map.hpp>
#include <ros2_slam_msgs/srv/save_pose.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#endif

// ROS1 头文件
#if ROS_ENABLE
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

#include <fast_lio_sam/save_map.h>
#include <fast_lio_sam/save_pose.h>

#include <tf/transform_broadcaster.h>
#endif

namespace legionclaw {
namespace map {

class LaserMappingNode {
public:
    explicit LaserMappingNode(const slam::FastLIOSAM::Config& cfg,
                              const std::string& node_name = "laser_mapping");

    void spin();

private:
    void initPublishers();
    void initSubscribers();
    void initServices();

    // IMU 回调
#if ROS2_ENABLE
    void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
#endif
#if ROS_ENABLE
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
#endif

    // 点云回调
#if ROS2_ENABLE
    void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
#endif
#if ROS_ENABLE
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
#endif

    void processScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double stamp);

    // 发布
    void publishTF(double stamp, const Eigen::Matrix4f& T_body_world);
    void publishOdometry(double stamp, const Eigen::Matrix4f& T);
    void publishWorldScan(double stamp);
    void publishPath(double stamp, const Eigen::Matrix4f& T);
    void publishOptimizedPath(const std::vector<std::pair<double, Eigen::Matrix4f>>& poses);

    // 服务
#if ROS2_ENABLE
    void saveMapService(
        const std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Request> req,
        std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Response> res);
    void savePoseService(
        const std::shared_ptr<ros2_slam_msgs::srv::SavePose::Request> req,
        std::shared_ptr<ros2_slam_msgs::srv::SavePose::Response> res);
#endif
#if ROS_ENABLE
    bool saveMapServiceROS1(fast_lio_sam::save_mapRequest& req,
                             fast_lio_sam::save_mapResponse& res);
    bool savePoseServiceROS1(fast_lio_sam::save_poseRequest& req,
                              fast_lio_sam::save_poseResponse& res);
#endif

    // ROS2 成员
#if ROS2_ENABLE
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_gnss_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_constraint_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keyframe_submap_;
    rclcpp::Service<ros2_slam_msgs::srv::SaveMap>::SharedPtr srv_save_map_;
    rclcpp::Service<ros2_slam_msgs::srv::SavePose>::SharedPtr srv_save_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
#endif

    // ROS1 成员
#if ROS_ENABLE
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_lidar_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_path_;
    ros::Publisher pub_gnss_path_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_cloud_body_;
    ros::Publisher pub_loop_constraint_;
    ros::Publisher pub_keyframe_submap_;
    ros::ServiceServer srv_save_map_;
    ros::ServiceServer srv_save_pose_;
    tf::TransformBroadcaster tf_broadcaster_;
#endif

    std::shared_ptr<slam::FastLIOSAM> slam_;
};

}  // namespace map
}  // namespace legionclaw

#endif  // LASER_MAPPING_NODE_H
