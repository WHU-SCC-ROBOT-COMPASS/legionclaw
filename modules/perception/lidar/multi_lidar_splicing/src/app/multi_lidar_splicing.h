#pragma once

#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <thread>
#include <future>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensors/lidar.h"

class MultiLidarSplicing : public rclcpp::Node
{
private:
    Lidar lidar_front_;
    Lidar lidar_mid_;
    Lidar lidar_left_;
    Lidar lidar_right_;
    Lidar lidar_back_;

    std::string frame_id_;
    std::string publish_topic_;

    // ROS2 pub/sub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    // message_filters（ROS2）
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_front_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_mid_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_left_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_right_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_back_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 性能监控
    std::chrono::high_resolution_clock::time_point last_publish_time_;
    size_t dropped_frames_ = 0;
    size_t total_frames_ = 0;

public:
    MultiLidarSplicing(std::string lidar_front,
                       std::string lidar_mid,
                       std::string lidar_left,
                       std::string lidar_right,
                       std::string lidar_back,
                       std::string frame_id,
                       std::string publish_topic);
    ~MultiLidarSplicing();

    void run();

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_front_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_mid_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_left_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_right_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_back_msg);

private:
    // 并行处理单个激光雷达的点云转换和变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr processLidarCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const Lidar &lidar);
};
