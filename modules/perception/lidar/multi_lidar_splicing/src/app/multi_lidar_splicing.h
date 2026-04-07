#pragma once

#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <thread>
#include <future>
#include <vector>
#include <string>
#include <map>
#include <mutex>

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

struct LidarConfig {
    std::string name;
    std::string path;
    bool enabled;
};

struct FilterRegion {
    bool enable = false;
    float x_min = -0.4f;
    float x_max = 1.6f;
    float y_min = -0.6f;
    float y_max = 0.61f;
    float z_min = -0.1f;
    float z_max = 1.6f;
};

class MultiLidarSplicing : public rclcpp::Node
{
private:
    std::vector<Lidar> lidars_;
    std::map<std::string, size_t> lidar_name_to_index_;

    std::string frame_id_;
    std::string publish_topic_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    std::map<std::string, std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>> subscribers_;

    int lidar_count_;

    std::chrono::high_resolution_clock::time_point last_publish_time_;
    size_t dropped_frames_ = 0;
    size_t total_frames_ = 0;

    FilterRegion filter_region_;

public:
    explicit MultiLidarSplicing(const std::vector<LidarConfig> &lidar_configs,
                                const std::string &frame_id,
                                const std::string &publish_topic,
                                const FilterRegion &filter_region);
    ~MultiLidarSplicing();

    void run();

    void callback(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &msgs);

private:
    void onLidarMessage(size_t lidar_index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr processLidarCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const Lidar &lidar);

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergeClouds(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
};