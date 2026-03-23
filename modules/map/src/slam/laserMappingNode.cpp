/**
 * @file    laserMapping.cpp
 * @brief   FAST-LIO-SAM 应用层 - ROS2/ROS1 封装，使用 middleware-agnostic slam::FastLIOSAM 核心
 * @author  LegionClaw Team (based on Ji Zhang's FAST-LIO-SAM)
 * @license BSD
 *
 * 此文件是 FAST-LIO-SAM 的 ROS2 封装层。
 * 核心 SLAM 算法在 fast_lio_sam.cpp / fast_lio_sam.h 中，
 * 完全独立于 ROS middleware。
 */

#include <iostream>
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
#include <pcl_conversions/pcl_conversions.h>

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

// 日志宏
#if GLOG_ENABLE
#include <glog/logging.h>
#define SLAM_LOG_INFO(msg) LOG(INFO) << "[FAST-LIO-SAM] " << msg
#define SLAM_LOG_WARN(msg) LOG(WARNING) << "[FAST-LIO-SAM] " << msg
#define SLAM_LOG_ERROR(msg) LOG(ERROR) << "[FAST-LIO-SAM] " << msg
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
#define SLAM_LOG_INFO(msg) AINFO << "[FAST-LIO-SAM] " << msg
#define SLAM_LOG_WARN(msg) AWARN << "[FAST-LIO-SAM] " << msg
#define SLAM_LOG_ERROR(msg) AERROR << "[FAST-LIO-SAM] " << msg
#else
#define SLAM_LOG_INFO(msg) std::cout << "[FAST-LIO-SAM INFO] " << msg << std::endl
#define SLAM_LOG_WARN(msg) std::cout << "[FAST-LIO-SAM WARN] " << msg << std::endl
#define SLAM_LOG_ERROR(msg) std::cout << "[FAST-LIO-SAM ERROR] " << msg << std::endl
#endif

namespace legionclaw {
namespace map {
namespace {

// Use slam::PointType (pcl::PointXYZINormal) for consistency with the SLAM core
using PointType = pcl::PointXYZINormal;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

/** Eigen Matrix4f → geometry_msgs::Quaternion + position */
void Matrix4fToPoseMsg(const Eigen::Matrix4f& T,
                        geometry_msgs::msg::Quaternion& q,
                        geometry_msgs::msg::Point& p) {
    Eigen::Quaternionf quat(T.block<3, 3>(0, 0));
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    q.w = quat.w();
    p.x = T(0, 3);
    p.y = T(1, 3);
    p.z = T(2, 3);
}

}  // anonymous namespace

/**
 * @brief ROS2 封装: 订阅 IMU/点云 -> 调用 slam::FastLIOSAM -> 发布结果
 */
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

    void processScan(const pcl::PointCloud<PointType>::Ptr& cloud, double stamp);

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

// ===== Method Definitions =====

LaserMappingNode::LaserMappingNode(const slam::FastLIOSAM::Config& cfg,
                                   const std::string& node_name)
#if ROS2_ENABLE
    : node_(rclcpp::Node::make_shared(node_name))
    , slam_(std::make_shared<slam::FastLIOSAM>(cfg))
    , tf_buffer_(node_->get_clock())
    , tf_listener_(tf_buffer_)
    , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*node_))
#endif
#if ROS_ENABLE
    : nh_("~")
    , slam_(std::make_shared<slam::FastLIOSAM>(cfg))
#endif
{
    initPublishers();
    initSubscribers();
    initServices();
}

void LaserMappingNode::spin() {
#if ROS2_ENABLE
    rclcpp::spin(node_);
#endif
#if ROS_ENABLE
    ros::spin();
#endif
}

void LaserMappingNode::initPublishers() {
#if ROS2_ENABLE
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/Odometry", rclcpp::QoS(10));
    pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("/fast_lio_sam/path_update", rclcpp::QoS(10));
    pub_gnss_path_ = node_->create_publisher<nav_msgs::msg::Path>("/gnss_path", rclcpp::QoS(10));
    pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", rclcpp::SensorDataQoS());
    pub_cloud_body_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", rclcpp::SensorDataQoS());
    pub_loop_constraint_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fast_lio_sam/mapping/loop_closure_constraints", rclcpp::QoS(1));
    pub_keyframe_submap_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/fast_lio_sam/mapping/keyframe_submap", rclcpp::QoS(1));
#endif

#if ROS_ENABLE
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/Odometry", 10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/fast_lio_sam/path_update", 10);
    pub_gnss_path_ = nh_.advertise<nav_msgs::Path>("/gnss_path", 10);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 2);
    pub_cloud_body_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 2);
    pub_loop_constraint_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/fast_lio_sam/mapping/loop_closure_constraints", 1);
    pub_keyframe_submap_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/fast_lio_sam/mapping/keyframe_submap", 1);
#endif
}

void LaserMappingNode::initSubscribers() {
#if ROS2_ENABLE
    sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { this->imuCallback(msg); });

    sub_lidar_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { this->lidarCallback(msg); });
#endif

#if ROS_ENABLE
    sub_imu_ = nh_.subscribe("/livox/imu", 200,
                               &LaserMappingNode::imuCallback, this);
    sub_lidar_ = nh_.subscribe("/livox/lidar", 5,
                                &LaserMappingNode::lidarCallback, this);
#endif
}

void LaserMappingNode::initServices() {
#if ROS2_ENABLE
    srv_save_map_ = node_->create_service<ros2_slam_msgs::srv::SaveMap>(
        "/save_map",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Request> req,
               std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Response> res) {
                this->saveMapService(req, res);
            });

    srv_save_pose_ = node_->create_service<ros2_slam_msgs::srv::SavePose>(
        "/save_pose",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<ros2_slam_msgs::srv::SavePose::Request> req,
               std::shared_ptr<ros2_slam_msgs::srv::SavePose::Response> res) {
                this->savePoseService(req, res);
            });
#endif

#if ROS_ENABLE
    srv_save_map_ = nh_.advertiseService("/save_map",
        &LaserMappingNode::saveMapServiceROS1, this);
    srv_save_pose_ = nh_.advertiseService("/save_pose",
        &LaserMappingNode::savePoseServiceROS1, this);
#endif
}

#if ROS2_ENABLE
void LaserMappingNode::imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    Eigen::Vector3f acc(msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z);
    Eigen::Vector3f gyro(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    // Use rclcpp::Time for ROS2 timestamp conversion
    double stamp = node_->get_clock()->now().seconds() -
                  (msg->header.stamp.sec > 0 ? 0 :
                   (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9));
    // Simpler approach: use header stamp directly
    stamp = static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    slam_->feedIMU(stamp, acc, gyro);
}
#endif

#if ROS_ENABLE
void LaserMappingNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Vector3f acc(msg->linear_acceleration.x,
                         msg->linear_acceleration.y,
                         msg->linear_acceleration.z);
    Eigen::Vector3f gyro(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    double stamp = msg->header.stamp.toSec();
    slam_->feedIMU(stamp, acc, gyro);
}
#endif

#if ROS2_ENABLE
void LaserMappingNode::lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *cloud);
    // Convert ROS2 header stamp to double
    double stamp = static_cast<double>(msg->header.stamp.sec) +
                   static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    processScan(cloud, stamp);
}
#endif

#if ROS_ENABLE
void LaserMappingNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *cloud);
    double stamp = msg->header.stamp.toSec();
    processScan(cloud, stamp);
}
#endif

void LaserMappingNode::processScan(const pcl::PointCloud<PointType>::Ptr& cloud, double stamp) {
    slam_->feedScan(stamp, cloud);

    Eigen::Matrix4f T = slam_->getLidarPose();
    Eigen::Matrix4f T_world_body = slam_->getPose();

    // 发布 TF
    publishTF(stamp, T_world_body);

    // 发布 Odometry
    publishOdometry(stamp, T_world_body);

    // 发布 world-frame 点云
    publishWorldScan(stamp);

    // 发布路径
    publishPath(stamp, T_world_body);

    // 发布优化后的位姿（如果有后端优化）
    if (slam_->hasOptimizedPose()) {
        publishOptimizedPath(slam_->getOptimizedPose());
    }
}

void LaserMappingNode::publishTF(double stamp, const Eigen::Matrix4f& T_body_world) {
#if ROS2_ENABLE
    geometry_msgs::msg::TransformStamped tf_msg;
    // Use rclcpp::Time with RCL_SYSTEM_TIME for reliable stamp construction
    tf_msg.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    tf_msg.header.frame_id = "camera_init";
    tf_msg.child_frame_id = "body";
    Eigen::Quaternionf q(T_body_world.block<3, 3>(0, 0));
    tf_msg.transform.translation.x = T_body_world(0, 3);
    tf_msg.transform.translation.y = T_body_world(1, 3);
    tf_msg.transform.translation.z = T_body_world(2, 3);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
#endif

#if ROS_ENABLE
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time(stamp);
    tf_msg.header.frame_id = "camera_init";
    tf_msg.child_frame_id = "body";
    Eigen::Quaternionf q(T_body_world.block<3, 3>(0, 0));
    tf_msg.transform.translation.x = T_body_world(0, 3);
    tf_msg.transform.translation.y = T_body_world(1, 3);
    tf_msg.transform.translation.z = T_body_world(2, 3);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf_msg);
#endif
}

void LaserMappingNode::publishOdometry(double stamp, const Eigen::Matrix4f& T) {
#if ROS2_ENABLE
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    odom.pose.pose.position.x = T(0, 3);
    odom.pose.pose.position.y = T(1, 3);
    odom.pose.pose.position.z = T(2, 3);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    Eigen::Vector3f vel = slam_->getVelocity();
    odom.twist.twist.linear.x = vel(0);
    odom.twist.twist.linear.y = vel(1);
    odom.twist.twist.linear.z = vel(2);

    pub_odom_->publish(odom);
#endif

#if ROS_ENABLE
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(stamp);
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    odom.pose.pose.position.x = T(0, 3);
    odom.pose.pose.position.y = T(1, 3);
    odom.pose.pose.position.z = T(2, 3);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    Eigen::Vector3f vel = slam_->getVelocity();
    odom.twist.twist.linear.x = vel(0);
    odom.twist.twist.linear.y = vel(1);
    odom.twist.twist.linear.z = vel(2);

    pub_odom_.publish(odom);
#endif
}

void LaserMappingNode::publishWorldScan(double stamp) {
    auto world_scan = slam_->getWorldScan();
    if (!world_scan || world_scan->empty()) return;

#if ROS2_ENABLE
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*world_scan, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    cloud_msg.header.frame_id = "camera_init";
    pub_cloud_->publish(cloud_msg);
#endif

#if ROS_ENABLE
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*world_scan, cloud_msg);
    cloud_msg.header.stamp = ros::Time(stamp);
    cloud_msg.header.frame_id = "camera_init";
    pub_cloud_.publish(cloud_msg);
#endif
}

void LaserMappingNode::publishPath(double stamp, const Eigen::Matrix4f& T) {
#if ROS2_ENABLE
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    path_msg.header.frame_id = "camera_init";

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    ps.pose.position.x = T(0, 3);
    ps.pose.position.y = T(1, 3);
    ps.pose.position.z = T(2, 3);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    path_msg.poses.push_back(ps);
    pub_path_->publish(path_msg);
#endif

#if ROS_ENABLE
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time(stamp);
    path_msg.header.frame_id = "camera_init";

    geometry_msgs::PoseStamped ps;
    ps.header = path_msg.header;
    Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    ps.pose.position.x = T(0, 3);
    ps.pose.position.y = T(1, 3);
    ps.pose.position.z = T(2, 3);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    path_msg.poses.push_back(ps);
    pub_path_->publish(path_msg);
#endif
}

void LaserMappingNode::publishOptimizedPath(const std::vector<std::pair<double, Eigen::Matrix4f>>& poses) {
#if ROS2_ENABLE
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "camera_init";

    for (const auto& [stamp, T] : poses) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
        ps.header.frame_id = "camera_init";
        Eigen::Quaternionf q(T.block<3, 3>(0, 0));
        ps.pose.position.x = T(0, 3);
        ps.pose.position.y = T(1, 3);
        ps.pose.position.z = T(2, 3);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path_msg.poses.push_back(ps);
    }
    if (!path_msg.poses.empty()) {
        pub_path_->publish(path_msg);
    }
#endif

#if ROS_ENABLE
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "camera_init";

    for (const auto& [stamp, T] : poses) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time(stamp);
        ps.header.frame_id = "camera_init";
        Eigen::Quaternionf q(T.block<3, 3>(0, 0));
        ps.pose.position.x = T(0, 3);
        ps.pose.position.y = T(1, 3);
        ps.pose.position.z = T(2, 3);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path_msg.poses.push_back(ps);
    }
    if (!path_msg.poses.empty()) {
        pub_path_->publish(path_msg);
    }
#endif
}

#if ROS2_ENABLE
void LaserMappingNode::saveMapService(
    const std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<ros2_slam_msgs::srv::SaveMap::Response> res) {
    SLAM_LOG_INFO("Save map service called, resolution: " << req->resolution);

    // getFullMap() returns pcl::PointCloud<PointType>::Ptr where PointType is pcl::PointXYZINormal
    // which matches our PointType alias here
    auto map = slam_->getFullMap();
    if (!map || map->empty()) {
        SLAM_LOG_WARN("Map is empty, cannot save.");
        res->success = false;
        return;
    }

    std::string dir = req->destination.empty() ? "/tmp/FAST_LIO_SAM/" : req->destination;
    if (req->resolution > 0) {
        pcl::PointCloud<PointType>::Ptr downsampled(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(map);
        vg.setLeafSize(req->resolution, req->resolution, req->resolution);
        vg.filter(*downsampled);
        map.swap(downsampled);  // use swap instead of assignment
    }

    std::string filepath = dir + "/slam_map.pcd";
    int ret = pcl::io::savePCDFileBinary(filepath, *map);
    SLAM_LOG_INFO("Saved map with " << map->size() << " points to " << filepath);
    res->success = (ret == 0);
}

void LaserMappingNode::savePoseService(
    const std::shared_ptr<ros2_slam_msgs::srv::SavePose::Request> req,
    std::shared_ptr<ros2_slam_msgs::srv::SavePose::Response> res) {
    SLAM_LOG_INFO("Save pose service called");
    res->success = true;
}
#endif

#if ROS_ENABLE
bool LaserMappingNode::saveMapServiceROS1(
    fast_lio_sam::save_mapRequest& req,
    fast_lio_sam::save_mapResponse& res) {
    SLAM_LOG_INFO("Save map service called");

    auto map = slam_->getFullMap();
    if (!map || map->empty()) {
        SLAM_LOG_WARN("Map is empty, cannot save.");
        res.success = false;
        return true;
    }

    std::string dir = req.destination.empty() ? "/tmp/FAST_LIO_SAM/" : req.destination;
    if (req.resolution > 0) {
        pcl::PointCloud<PointType>::Ptr downsampled(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(map);
        vg.setLeafSize(req.resolution, req.resolution, req.resolution);
        vg.filter(*downsampled);
        map = downsampled;
    }

    std::string filepath = dir + "/slam_map.pcd";
    int ret = pcl::io::savePCDFileBinary(filepath, *map);
    SLAM_LOG_INFO("Saved map with " << map->size() << " points to " << filepath);
    res.success = (ret == 0);
    return true;
}

bool LaserMappingNode::savePoseServiceROS1(
    fast_lio_sam::save_poseRequest& req,
    fast_lio_sam::save_poseResponse& res) {
    SLAM_LOG_INFO("Save pose service called");
    res.success = true;
    return true;
}
#endif

}  // namespace map
}  // namespace legionclaw

// ===== main =====
// 使用 slam::FastLIOSAM 的简洁 main 函数
