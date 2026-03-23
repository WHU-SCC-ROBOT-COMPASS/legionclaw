/**
 * @file    localization.cpp
 * @brief   NDT scan matching localization implementation
 * @author  LegionClaw Team (based on koide3/hdl_localization)
 * @license BSD
 */

#include "modules/localization/src/apps/localization.h"

#include <fstream>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <modules/localization/src/pose_estimator/kkl/alg/unscented_kalman_filter.hpp>
#include <modules/localization/src/pose_estimator/pose_system.hpp>
#include <modules/localization/src/pose_estimator/odom_system.hpp>
#include <modules/localization/src/pose_estimator/pose_estimator.hpp>

#include <modules/common/status/status.h>

namespace {

#if GLOG_ENABLE
#define LOG_INFO(msg) std::cout << "[LOC-INFO] " << msg << std::endl
#define LOG_WARN(msg) std::cout << "[LOC-WARN] " << msg << std::endl
#define LOG_ERROR(msg) std::cout << "[LOC-ERROR] " << msg << std::endl
#elif MDCLOG_ENABLE
#define LOG_INFO(msg) AINFO << msg
#define LOG_WARN(msg) AINFO << msg
#define LOG_ERROR(msg) AINFO << msg
#else
#define LOG_INFO(msg) std::cout << "[LOC-INFO] " << msg << std::endl
#define LOG_WARN(msg) std::cout << "[LOC-WARN] " << msg << std::endl
#define LOG_ERROR(msg) std::cout << "[LOC-ERROR] " << msg << std::endl
#endif

}  // anonymous namespace

namespace legionclaw {
namespace localization {

LocalizationNode::LocalizationNode(const std::string& config_path) {
    LoadConfig(config_path);

#if ROS2_ENABLE
    node_ = rclcpp::Node::make_shared("localization_node");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
#endif

#if ROS_ENABLE
    tf_buffer_ = tf2_ros::Buffer();
    tf_listener_ = tf2_ros::TransformListener(tf_buffer_);
#endif
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::Init() {
    LOG_INFO("Initializing localization node");

    // Create registration method
    registration_ = CreateRegistration();
    if (!registration_) {
        LOG_ERROR("Failed to create registration method: " + config_.reg_method);
        return;
    }
    LOG_INFO("Registration method: " + config_.reg_method);

    // Initialize pose estimator
    {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        Eigen::Vector3f init_pos(config_.init_pos_x, config_.init_pos_y, config_.init_pos_z);
        Eigen::Quaternionf init_ori(config_.init_ori_w, config_.init_ori_x,
                                   config_.init_ori_y, config_.init_ori_z);
        pose_estimator_.reset(new PoseEstimator(
            registration_, init_pos, init_ori, config_.cool_time_duration));
    }

    // Create downsample filter
    downsample_filter_ = pcl::make_shared<pcl::VoxelGrid<pcl::PointXYZI>>();
    downsample_filter_->setLeafSize(config_.downsample_resolution,
                                   config_.downsample_resolution,
                                   config_.downsample_resolution);

    relocalizing_.store(false);

    // Start relocalization thread if enabled
    if (config_.use_global_localization) {
        relocalize_thread_ = std::thread(&LocalizationNode::RelocalizeThread, this);
    }

#if ROS2_ENABLE
    // Subscriptions
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::UniquePtr msg) { ImuCallback(std::move(msg)); });

    points_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.points_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { PointsCallback(std::move(msg)); });

    globalmap_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/globalmap", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { GlobalmapCallback(std::move(msg)); });

    initialpose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1).reliable(),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) { InitialPoseCallback(std::move(msg)); });

    // Publishers
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10));
    aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/aligned_points", rclcpp::SensorDataQoS());

    // Service
    relocalize_srv_ = node_->create_service<std_srvs::srv::Empty>(
        "/relocalize",
        [this](const std::shared_ptr<rmw_request_id_t> req_header,
               const std::shared_ptr<const std_srvs::srv::Empty::Request> req,
               const std::shared_ptr<std_srvs::srv::Empty::Response> res) {
            RelocalizeCallback(req_header, req, res);
        });

    LOG_INFO("ROS2 subscriptions and publishers created");
#endif

#if ROS_ENABLE
    imu_sub_ = nh_.subscribe(config_.imu_topic, 256,
                            &LocalizationNode::ImuCallback, this);
    points_sub_ = nh_.subscribe(config_.points_topic, 5,
                               &LocalizationNode::PointsCallback, this);
    globalmap_sub_ = nh_.subscribe("/globalmap", 1,
                                  &LocalizationNode::GlobalmapCallback, this);
    initialpose_sub_ = nh_.subscribe("/initialpose", 1,
                                    &LocalizationNode::InitialPoseCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1);
    relocalize_srv_ = nh_.advertiseService("/relocalize",
                                           &LocalizationNode::RelocalizeCallback, this);

    LOG_INFO("ROS1 subscriptions and publishers created");
#endif

    LOG_INFO("Localization node initialized");
}

void LocalizationNode::Join() {
#if ROS2_ENABLE
    rclcpp::spin(node_);
#endif

#if ROS_ENABLE
    ros::spin();
#endif
}

void LocalizationNode::LoadConfig(const std::string& config_path) {
    // Default config is set in LocalizationConfig struct
    std::ifstream fin(config_path);
    if (!fin.is_open()) {
        LOG_WARN("Config file not found: " + config_path + ", using defaults");
        return;
    }
    LOG_INFO("Config loaded from: " + config_path);
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
LocalizationNode::CreateRegistration() {
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

    if (config_.reg_method == "NDT_OMP") {
#ifdef NDT_OMP_ENABLE
        auto ndt = pcl::make_shared<pclomp::NDTOMPImpl<pcl::PointXYZI, pcl::PointXYZI>>(
            config_.ndt_resolution, 3.0);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt->setMaxIterations(35);
        registration = ndt;
        LOG_INFO("NDT-OMP registration created");
#else
        LOG_ERROR("NDT-OMP not enabled. Set NDT_OMP_ENABLE=ON");
        return nullptr;
#endif
    } else if (config_.reg_method == "NDT_CUDA") {
#ifdef FAST_GICP_ENABLE
        auto ndt = pcl::make_shared<fast_gicp::NDTCuda<pcl::PointXYZI, pcl::PointXYZI>>();
        registration = ndt;
        LOG_INFO("NDT-CUDA registration created (via fast_gicp)");
#else
        LOG_ERROR("fast_gicp not enabled. Set FAST_GICP_ENABLE=ON");
        return nullptr;
#endif
    } else {
        LOG_ERROR("Unknown registration method: " + config_.reg_method);
        return nullptr;
    }

    registration->setMaxCorrespondenceDistance(config_.ndt_neighbor_search_radius * 2.0);
    registration->setTransformationEpsilon(0.01);
    return registration;
}

#if ROS2_ENABLE
void LocalizationNode::ImuCallback(sensor_msgs::msg::Imu::UniquePtr msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);

    double roll, pitch, yaw;
    QuaternionToRPY(msg->orientation.x, msg->orientation.y,
                  msg->orientation.z, msg->orientation.w,
                  roll, pitch, yaw);

    std::vector<double> imu;
    imu.reserve(6);
    if (config_.invert_acc) {
        imu.push_back(-msg->linear_acceleration.x);
        imu.push_back(-msg->linear_acceleration.y);
        imu.push_back(-msg->linear_acceleration.z);
    } else {
        imu.push_back(msg->linear_acceleration.x);
        imu.push_back(msg->linear_acceleration.y);
        imu.push_back(msg->linear_acceleration.z);
    }

    if (config_.invert_gyro) {
        imu.push_back(-msg->angular_velocity.x);
        imu.push_back(-msg->angular_velocity.y);
        imu.push_back(-msg->angular_velocity.z);
    } else {
        imu.push_back(msg->angular_velocity.x);
        imu.push_back(msg->angular_velocity.y);
        imu.push_back(msg->angular_velocity.z);
    }

    imu.push_back(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    imu_data_.push_back(imu);
    if (imu_data_.size() > 100) {
        imu_data_.pop_front();
    }
}

void LocalizationNode::PointsCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    if (relocalizing_.load()) return;
    if (!globalmap_) {
        LOG_WARN("No global map received yet");
        return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Transform to odom frame
    try {
        auto stamp = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
        if (tf_buffer_->canTransform(config_.odom_frame_id, msg->header.frame_id,
                                     stamp, tf2::durationFromSec(0.1))) {
            auto transform = tf_buffer_->lookupTransform(
                config_.odom_frame_id, msg->header.frame_id, stamp,
                tf2::durationFromSec(0.1));
            Eigen::Isometry3d T =
                tf2::transformToEigen(transform.transform);
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*cloud, transformed, T.cast<float>());
            cloud = transformed.makeShared();
            cloud->header.frame_id = config_.odom_frame_id;
        }
    } catch (const tf2::TransformException& ex) {
        LOG_WARN("TF transform failed: " + std::string(ex.what()));
    }

    // Downsample
    downsample_filter_->setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI> downsampled;
    downsample_filter_->filter(downsampled);
    downsampled.header = cloud->header;
    downsampled_ = downsampled.makeShared();

    // IMU prediction
    {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        if (!pose_estimator_) return;

        std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
        if (!imu_data_.empty()) {
            const auto& last = imu_data_.back();
            Eigen::Vector3f acc(last[0], last[1], last[2]);
            Eigen::Vector3f gyro(last[3], last[4], last[5]);
            double stamp = last[6];
            pose_estimator_->predict(stamp, acc, gyro);
        } else {
            pose_estimator_->predict(node_->now().seconds());
        }
    }

    // NDT correction
    registration_->setInputTarget(globalmap_);
    registration_->setInputSource(downsampled_);

    {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        if (!pose_estimator_) return;

        double stamp = node_->now().seconds();
        auto aligned = pose_estimator_->correct(stamp, downsampled_);
        PublishOdometry(stamp);
        PublishAlignedPoints(stamp, aligned);
        PublishScanMatchingStatus(stamp, aligned);
    }
}

void LocalizationNode::GlobalmapCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    LOG_INFO("Global map received: " + std::to_string(msg->width * msg->height) + " points");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    globalmap_ = cloud;
    registration_->setInputTarget(globalmap_);
}

void LocalizationNode::InitialPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
    LOG_INFO("Initial pose received from RViz");
    Eigen::Vector3f pos(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaternionf ori(msg->pose.pose.orientation.w,
                          msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);

    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    registration_ = CreateRegistration();
    pose_estimator_.reset(new PoseEstimator(
        registration_, pos, ori, config_.cool_time_duration));
}

void LocalizationNode::RelocalizeCallback(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<const std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    (void)req_header; (void)req; (void)res;
    LOG_INFO("Relocalize service called");
    relocalizing_.store(true);
    delta_estimater_.reset(new DeltaEstimater(registration_));
    relocalizing_.store(false);
}
#endif

#if ROS_ENABLE
void LocalizationNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);

    double roll, pitch, yaw;
    QuaternionToRPY(msg->orientation.x, msg->orientation.y,
                  msg->orientation.z, msg->orientation.w,
                  roll, pitch, yaw);

    std::vector<double> imu;
    imu.reserve(6);
    if (config_.invert_acc) {
        imu.push_back(-msg->linear_acceleration.x);
        imu.push_back(-msg->linear_acceleration.y);
        imu.push_back(-msg->linear_acceleration.z);
    } else {
        imu.push_back(msg->linear_acceleration.x);
        imu.push_back(msg->linear_acceleration.y);
        imu.push_back(msg->linear_acceleration.z);
    }
    if (config_.invert_gyro) {
        imu.push_back(-msg->angular_velocity.x);
        imu.push_back(-msg->angular_velocity.y);
        imu.push_back(-msg->angular_velocity.z);
    } else {
        imu.push_back(msg->angular_velocity.x);
        imu.push_back(msg->angular_velocity.y);
        imu.push_back(msg->angular_velocity.z);
    }
    imu.push_back(msg->header.stamp.toSec());
    imu_data_.push_back(imu);
    if (imu_data_.size() > 100) imu_data_.pop_front();
}

void LocalizationNode::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (relocalizing_.load()) return;
    if (!globalmap_) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Transform to odom frame
    try {
        if (tf_buffer_.canTransform(config_.odom_frame_id, msg->header.frame_id,
                                    msg->header.stamp, ros::Duration(0.1))) {
            geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(config_.odom_frame_id, msg->header.frame_id,
                                          msg->header.stamp, ros::Duration(0.1));
            Eigen::Isometry3d T = tf2::transformToEigen(transform.transform);
            pcl::PointCloud<pcl::PointXYZI> transformed;
            pcl::transformPointCloud(*cloud, transformed, T.cast<float>());
            cloud = transformed.makeShared();
            cloud->header.frame_id = config_.odom_frame_id;
        }
    } catch (const tf2::TransformException& ex) {
        LOG_WARN("TF: " + std::string(ex.what()));
    }

    downsample_filter_->setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI> downsampled;
    downsample_filter_->filter(downsampled);
    downsampled.header = cloud->header;
    downsampled_ = downsampled.makeShared();

    // IMU prediction
    {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        if (!pose_estimator_) return;
        std::lock_guard<std::mutex> imu_lock(imu_data_mutex_);
        if (!imu_data_.empty()) {
            const auto& last = imu_data_.back();
            Eigen::Vector3f acc(last[0], last[1], last[2]);
            Eigen::Vector3f gyro(last[3], last[4], last[5]);
            pose_estimator_->predict(last[6], acc, gyro);
        } else {
            pose_estimator_->predict(msg->header.stamp.toSec());
        }
    }

    registration_->setInputTarget(globalmap_);
    registration_->setInputSource(downsampled_);

    {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        if (!pose_estimator_) return;
        double stamp = msg->header.stamp.toSec();
        auto aligned = pose_estimator_->correct(stamp, downsampled_);
        PublishOdometry(stamp);
        PublishAlignedPoints(stamp, aligned);
        PublishScanMatchingStatus(stamp, aligned);
    }
}

void LocalizationNode::GlobalmapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    LOG_INFO("Global map received");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    globalmap_ = cloud;
    registration_->setInputTarget(globalmap_);
}

void LocalizationNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    LOG_INFO("Initial pose received");
    Eigen::Vector3f pos(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaternionf ori(msg->pose.pose.orientation.w,
                          msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    registration_ = CreateRegistration();
    pose_estimator_.reset(new PoseEstimator(
        registration_, pos, ori, config_.cool_time_duration));
}

bool LocalizationNode::RelocalizeCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res) {
    (void)req; (void)res;
    LOG_INFO("Relocalize");
    relocalizing_.store(true);
    delta_estimater_.reset(new DeltaEstimater(registration_));
    relocalizing_.store(false);
    return true;
}
#endif

void LocalizationNode::PublishOdometry(double stamp) {
    Eigen::Matrix4f pose = pose_estimator_->matrix();
    Eigen::Quaternionf quat(Eigen::Matrix3f(pose.block<3, 3>(0, 0)));
    Eigen::Vector3f trans = pose.block<3, 1>(0, 3);

#if ROS2_ENABLE
    // Publish TF
    geometry_msgs::msg::TransformStamped odom_trans;
    rclcpp::Time odom_stamp(stamp);
    odom_trans.header.stamp = odom_stamp;
    odom_trans.header.frame_id = config_.map_frame_id;
    odom_trans.child_frame_id = config_.odom_frame_id;
    odom_trans.transform.translation.x = trans.x();
    odom_trans.transform.translation.y = trans.y();
    odom_trans.transform.translation.z = trans.z();
    odom_trans.transform.rotation.x = quat.x();
    odom_trans.transform.rotation.y = quat.y();
    odom_trans.transform.rotation.z = quat.z();
    odom_trans.transform.rotation.w = quat.w();
    tf_broadcaster_->sendTransform(odom_trans);

    // Publish odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = odom_stamp;
    odom.header.frame_id = config_.map_frame_id;
    odom.child_frame_id = config_.odom_child_frame_id;
    odom.pose.pose.position.x = trans.x();
    odom.pose.pose.position.y = trans.y();
    odom.pose.pose.position.z = trans.z();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom_pub_->publish(odom);
#endif

#if ROS_ENABLE
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = config_.map_frame_id;
    odom_trans.child_frame_id = config_.odom_frame_id;
    odom_trans.transform.translation.x = trans.x();
    odom_trans.transform.translation.y = trans.y();
    odom_trans.transform.translation.z = trans.z();
    odom_trans.transform.rotation.x = quat.x();
    odom_trans.transform.rotation.y = quat.y();
    odom_trans.transform.rotation.z = quat.z();
    odom_trans.transform.rotation.w = quat.w();
    tf_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = config_.map_frame_id;
    odom.child_frame_id = config_.odom_child_frame_id;
    odom.pose.pose.position.x = trans.x();
    odom.pose.pose.position.y = trans.y();
    odom.pose.pose.position.z = trans.z();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom_pub_.publish(odom);
#endif
}

void LocalizationNode::PublishAlignedPoints(double stamp,
                                          pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
#if ROS2_ENABLE
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = config_.map_frame_id;
    aligned_pub_->publish(msg);
#endif

#if ROS_ENABLE
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame_id;
    aligned_pub_.publish(msg);
#endif
}

void LocalizationNode::PublishScanMatchingStatus(double stamp,
                                                pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    double fitness = registration_->getFitnessScore();
    LOG_INFO("fitness_score: " + std::to_string(fitness));
    (void)stamp; (void)aligned;
    // Status publishing can be extended as needed
}

bool LocalizationNode::Relocalize() {
    LOG_INFO("Global localization not fully implemented in standalone mode");
    return false;
}

void LocalizationNode::RelocalizeThread() {
    LOG_INFO("Relocalization thread started");
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        if (!relocalizing_.load() && !imu_data_.empty()) {
            Relocalize();
        }
    }
}

double LocalizationNode::GetYawFromQuaternion(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void LocalizationNode::QuaternionToRPY(double x, double y, double z, double w,
                                      double& roll, double& pitch, double& yaw) {
    // Roll
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace localization
}  // namespace legionclaw
