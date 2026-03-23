/**
 * @file    localization.h
 * @brief   NDT scan matching localization header
 * @author  LegionClaw Team (based on koide3/hdl_localization)
 * @license BSD
 */

#ifndef MODULES_LOCALIZATION_SRC_APPS_LOCALIZATION_H_
#define MODULES_LOCALIZATION_SRC_APPS_LOCALIZATION_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <mutex>
#include <deque>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>

#if ROS2_ENABLE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#endif

#if ROS_ENABLE
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#endif

// TF2 / Eigen
#include <tf2_eigen/tf2_eigen.hpp>

// Algorithm headers
#include <modules/localization/src/pose_estimator/pose_estimator.hpp>
#include <modules/localization/src/pose_estimator/pose_system.hpp>
#include <modules/localization/src/pose_estimator/odom_system.hpp>

#ifdef NDT_OMP_ENABLE
#include <pclomp/ndt_omp.h>
#endif

#ifdef FAST_GICP_ENABLE
#include <fast_gicp/ndt/ndt_cuda.hpp>
#endif

namespace legionclaw {
namespace localization {

/**
 * @brief Configuration for localization module
 */
struct LocalizationConfig {
    // Topics
    std::string points_topic = "/velodyne_points";
    std::string imu_topic = "/imu_data";
    std::string odom_child_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::string map_frame_id = "map";

    // IMU
    bool use_imu = true;
    bool invert_acc = false;
    bool invert_gyro = false;
    double cool_time_duration = 2.0;

    // Robot odometry
    bool enable_robot_odometry_prediction = false;
    std::string robot_odom_frame_id = "odom";

    // Registration
    std::string reg_method = "NDT_OMP";
    std::string ndt_neighbor_search_method = "DIRECT7";
    double ndt_neighbor_search_radius = 2.0;
    double ndt_resolution = 1.0;
    double downsample_resolution = 0.3;
    double converged_score = 4.0;

    // Initial pose
    bool specify_init_pose = false;
    double init_pos_x = 0.0, init_pos_y = 0.0, init_pos_z = 0.0;
    double init_ori_x = 0.0, init_ori_y = 0.0, init_ori_z = 0.0, init_ori_w = 1.0;

    // Global localization
    bool use_global_localization = true;
};

/**
 * @brief Delta estimator for relocalization
 */
class DeltaEstimater {
public:
    using PointT = pcl::PointXYZI;

    DeltaEstimater(pcl::Registration<PointT, PointT>::Ptr reg)
        : delta(Eigen::Isometry3f::Identity()), reg(reg) {}

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        delta.setIdentity();
        last_frame.reset();
    }

    void add_frame(pcl::PointCloud<PointT>::ConstPtr frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!last_frame) {
            last_frame = frame;
            return;
        }
        reg->setInputTarget(last_frame);
        reg->setInputSource(frame);
        lock.unlock();
        pcl::PointCloud<PointT> aligned;
        reg->align(aligned);
        lock.lock();
        last_frame = frame;
        delta = delta * Eigen::Isometry3f(reg->getFinalTransformation());
    }

    Eigen::Isometry3f estimated_delta() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return delta;
    }

private:
    mutable std::mutex mutex_;
    Eigen::Isometry3f delta;
    pcl::Registration<PointT, PointT>::Ptr reg;
    pcl::PointCloud<PointT>::ConstPtr last_frame;
};

/**
 * @brief Localization module main class
 * Real-time NDT scan matching with UKF pose estimation
 */
class LocalizationNode {
public:
    explicit LocalizationNode(const std::string& config_path);
    ~LocalizationNode();
    void Init();
    void Join();

private:
    void LoadConfig(const std::string& config_path);

    // Registration factory
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr CreateRegistration();

    // Callbacks
#if ROS2_ENABLE
    void ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg);
    void PointsCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void GlobalmapCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void InitialPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg);
    void RelocalizeCallback(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<const std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);
#endif

#if ROS_ENABLE
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void GlobalmapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    bool RelocalizeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
#endif

    // Publishing
    void PublishOdometry(double stamp);
    void PublishAlignedPoints(double stamp, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned);
    void PublishScanMatchingStatus(double stamp, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned);

    // Relocalization
    bool Relocalize();
    void RelocalizeThread();

    // Helpers
    double GetYawFromQuaternion(double x, double y, double z, double w);
    void QuaternionToRPY(double x, double y, double z, double w,
                       double& roll, double& pitch, double& yaw);

    // ROS2 members
#if ROS2_ENABLE
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
#endif

    // ROS1 members
#if ROS_ENABLE
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber points_sub_;
    ros::Subscriber globalmap_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher aligned_pub_;
    ros::ServiceServer relocalize_srv_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
#endif

    // Core data
    std::mutex pose_estimator_mutex_;
    std::unique_ptr<PoseEstimator> pose_estimator_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_;
    pcl::VoxelGrid<pcl::PointXYZI>::Ptr downsample_filter_;
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration_;

    std::mutex imu_data_mutex_;
    std::deque<std::vector<double>> imu_data_;
    std::atomic_bool relocalizing_;
    std::unique_ptr<DeltaEstimater> delta_estimater_;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr last_scan_;
    std::thread relocalize_thread_;

    // Config
    LocalizationConfig config_;
};

}  // namespace localization
}  // namespace legionclaw

#endif  // MODULES_LOCALIZATION_SRC_APPS_LOCALIZATION_H_
