/**
 * @file fast_lio_sam.h
 * @brief Middleware-agnostic FAST-LIO-SAM SLAM core.
 *
 * NO ROS dependencies. Pure C++ with Eigen, PCL.
 *
 * Uses:
 *   - IKFoM toolkit for iESKF state estimation
 *   - iKD-Tree for incremental map management
 *   - ImuProcess for IMU preprocessing
 */

#ifndef FAST_LIO_SAM_H
#define FAST_LIO_SAM_H

#include <modules/map/src/slam/so3_math.h>
#include <modules/map/src/slam/use-ikfom.hpp>
#include <modules/map/src/slam/common_lib.h>
#include <modules/map/src/slam/ikd-Tree/ikd_Tree.h>
#include <modules/map/src/slam/IMU_Processing.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <memory>
#include <atomic>

namespace slam {

/**
 * @brief Keyframe data stored for loop closure and GTSAM backend.
 */
struct KeyFrame {
    double stamp;
    Eigen::Matrix4f pose;       // world T body
    pcl::PointCloud<PointType>::Ptr cloud;
    Eigen::Vector3f gps_xyz;    // optional ENU coordinates
    bool has_gps;

    KeyFrame() : stamp(0.0), has_gps(false) {
        pose = Eigen::Matrix4f::Identity();
        cloud.reset(new pcl::PointCloud<PointType>);
    }

    explicit KeyFrame(double t) : stamp(t), has_gps(false) {
        pose = Eigen::Matrix4f::Identity();
        cloud.reset(new pcl::PointCloud<PointType>);
    }
};

/**
 * @brief Main FastLIOSAM SLAM class.
 */
class FastLIOSAM {
public:
    /** @brief Tuning parameters. */
    struct Config {
        double acc_cov = 0.1;
        double gyr_cov = 0.1;
        double b_acc_cov = 0.0001;
        double b_gyr_cov = 0.0001;
        std::array<double, 3> lidar_to_imu_trans = {0.0, 0.0, 0.0};
        std::array<double, 9> lidar_to_imu_rot = {1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0};
        double filter_size_surf = 0.5;
        double filter_size_corner = 0.5;
        int    max_iteration = 4;
        double cube_side_length = 200.0;
        double det_range = 300.0;
        bool   loop_closure_en = false;
    };

    explicit FastLIOSAM(const Config& config);
    ~FastLIOSAM();

    void feedIMU(double stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);
    void feedScan(double stamp, const pcl::PointCloud<PointType>::Ptr& scan);
    void feedScanFeature(double stamp, const pcl::PointCloud<PointType>::Ptr& scan);
    void feedGNSS(double stamp, double x, double y, double z);
    void initGNSSOrigin(double latitude, double longitude, double altitude);

    Eigen::Matrix4f getPose();
    Eigen::Matrix4f getLidarPose();
    Eigen::Vector3f getVelocity();
    double getTimestamp();
    pcl::PointCloud<PointType>::Ptr getWorldScan();
    int getMapSize();
    int getKeyFrameCount();
    bool hasOptimizedPose();
    std::vector<std::pair<double, Eigen::Matrix4f>> getOptimizedPose();
    pcl::PointCloud<PointType>::Ptr getFullMap();
    std::vector<KeyFrame> getRecentKeyFrames(int count) const;
    bool shouldAddKeyFrame();
    void forceAddKeyFrame();
    void reset();

private:
    Config config_;

    std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> state_;

    ImuProcess* imu_process_;
    std::shared_ptr<KD_TREE> kdtree_ptr_;

    pcl::VoxelGrid<PointType>::Ptr downsize_filter_surf_;
    pcl::VoxelGrid<PointType>::Ptr downsize_filter_corner_;

    pcl::PointCloud<PointType>::Ptr cur_scan_;
    pcl::PointCloud<PointType>::Ptr cur_scan_world_;
    pcl::PointCloud<PointType>::Ptr local_map_;

    Eigen::Matrix4f last_pose_;
    double cur_stamp_ = 0.0;
    bool first_scan_ = true;
    bool imu_initialized_ = false;

    std::deque<ImuData> imu_buffer_;
    std::vector<KeyFrame> keyframes_;

    std::mutex imu_mutex_;
    std::mutex scan_mutex_;
    std::mutex kf_mutex_;
};

}  // namespace slam

#endif  // FAST_LIO_SAM_H
