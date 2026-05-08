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
#include <deque>
#include <queue>
#include <condition_variable>
#include <atomic>

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

struct SyncConfig {
    bool enable_approximate_time_sync = true;
    double sync_time_tolerance = 0.1;
    int min_lidars_for_fusion = 1;
    double message_timeout = 1.0;
    bool strict_sync = false;
};

class MultiLidarSplicing : public rclcpp::Node
{
private:
    struct BufferedMessage {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
        rclcpp::Time timestamp;
        rclcpp::Time receive_time;
    };

    std::vector<Lidar> lidars_;
    std::map<std::string, size_t> lidar_name_to_index_;

    std::string frame_id_;
    std::string publish_topic_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    std::map<std::string, std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>> subscribers_;

    FilterRegion filter_region_;
    FilterRegion filter_region_outside_;
    SyncConfig sync_config_;

    int lidar_count_;
    std::chrono::high_resolution_clock::time_point last_publish_time_;

    // 有界缓存：每个雷达独立 deque，最大长度 3
    std::vector<std::deque<BufferedMessage>> msg_buffer_;
    std::vector<rclcpp::Time> last_published_stamps_;
    std::mutex msg_mutex_;
    static constexpr size_t MAX_BUFFER_PER_LIDAR = 3;

    // 工作线程
    std::thread worker_thread_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::queue<std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>> task_queue_;
    std::atomic<bool> stop_worker_{false};

    rclcpp::TimerBase::SharedPtr sync_timer_;

    void workerLoop();
    void tryPublishLocked();
    void onLidarMessage(size_t lidar_index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void approximateTimeSync();

    pcl::PointCloud<pcl::PointXYZI>::Ptr processLidarCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
        const Lidar &lidar);

    pcl::PointCloud<pcl::PointXYZI>::Ptr mergeClouds(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

public:
    explicit MultiLidarSplicing(const std::vector<LidarConfig> &lidar_configs,
                                const std::string &frame_id,
                                const std::string &publish_topic,
                                const FilterRegion &filter_region,
                                const FilterRegion &filter_region_outside,
                                const SyncConfig &sync_config);
    ~MultiLidarSplicing();

    void run();

    void callback(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &msgs);
};
