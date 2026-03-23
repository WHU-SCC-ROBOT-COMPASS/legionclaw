/**
 * @file fast_lio_sam.cpp
 * @brief Middleware-agnostic FAST-LIO-SAM SLAM core (stub implementation).
 *
 * This is a simplified version that provides the API contract.
 * The actual SLAM logic is integrated directly in LaserMappingNode using
 * the original FAST-LIO-SAM components (IKFoM, iKD-Tree, IMU_Processing).
 */

#include <modules/map/src/slam/fast_lio_sam.h>

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

namespace slam {

FastLIOSAM::FastLIOSAM(const Config& cfg)
    : config_(cfg),
      state_(std::make_shared<esekfom::esekf<state_ikfom, 12, input_ikfom>>()),
      imu_process_(new ImuProcess()),
      kdtree_ptr_(std::make_shared<KD_TREE>()),
      downsize_filter_surf_(new pcl::VoxelGrid<PointType>()),
      downsize_filter_corner_(new pcl::VoxelGrid<PointType>()),
      cur_scan_(new pcl::PointCloud<PointType>()),
      cur_scan_world_(new pcl::PointCloud<PointType>()),
      local_map_(new pcl::PointCloud<PointType>()),
      last_pose_(Eigen::Matrix4f::Identity()),
      cur_stamp_(0.0),
      first_scan_(true),
      imu_initialized_(false) {

    // Configure IMU processing
    imu_process_->lidar_type = 1;  // AVIA
    imu_process_->blind = 0.01;
    imu_process_->filter_size_corner = cfg.filter_size_corner;
    imu_process_->filter_size_surf = cfg.filter_size_surf;
    imu_process_->feature_extract_en = false;
    imu_process_->point_filter_num = 2;

    // IMU-LiDAR extrinsic - use double types for MTK manifold compatibility
    V3D T(cfg.lidar_to_imu_trans[0],
          cfg.lidar_to_imu_trans[1],
          cfg.lidar_to_imu_trans[2]);
    M3D R;
    R << cfg.lidar_to_imu_rot[0], cfg.lidar_to_imu_rot[1], cfg.lidar_to_imu_rot[2],
         cfg.lidar_to_imu_rot[3], cfg.lidar_to_imu_rot[4], cfg.lidar_to_imu_rot[5],
         cfg.lidar_to_imu_rot[6], cfg.lidar_to_imu_rot[7], cfg.lidar_to_imu_rot[8];
    imu_process_->set_extrinsic(T, R);
    imu_process_->set_max_iteration(cfg.max_iteration);

    // Initialize iESKF state using ESKF API
    state_ikfom init_state = state_->get_x();
    init_state.pos = V3D::Zero();
    init_state.rot = SO3();
    init_state.offset_T_L_I = T;
    init_state.offset_R_L_I = R;
    init_state.vel = V3D::Zero();
    init_state.bg = V3D::Zero();
    init_state.ba = V3D::Zero();
    // S2 manifold for gravity - initialize with downward gravity (negative Z in world frame)
    // S2 is constructed from a 3D vector representing the gravity direction
    init_state.grav = S2(V3D(0, 0, -G_m_s2));
    state_->change_x(init_state);

    // Initialize covariance using ESKF API
    auto init_P = state_->get_P();
    init_P.setIdentity();
    init_P *= 0.0001;
    state_->change_P(init_P);

    // Voxel filters
    downsize_filter_surf_->setLeafSize(cfg.filter_size_surf, cfg.filter_size_surf, cfg.filter_size_surf);
    downsize_filter_corner_->setLeafSize(cfg.filter_size_corner, cfg.filter_size_corner, cfg.filter_size_corner);
}

FastLIOSAM::~FastLIOSAM() {
    if (imu_process_) delete imu_process_;
}

void FastLIOSAM::feedIMU(double stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    ImuData data;
    data.stamp = stamp;
    data.acc = acc;
    data.gyro = gyro;
    imu_buffer_.push_back(data);
}

void FastLIOSAM::feedScan(double stamp, const pcl::PointCloud<PointType>::Ptr& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    cur_stamp_ = stamp;

    // Build measure group
    MeasureGroup meas;
    meas.lidar_beg_time = stamp;
    meas.lidar_end_time = stamp + 0.1;  // approximate lidar scan duration
    meas.lidar = scan;
    meas.imu.swap(imu_buffer_);

    // IMU processing and undistortion - Process() takes MeasureGroup, ESKF state, and output cloud
    pcl::PointCloud<PointType>::Ptr undistorted(new pcl::PointCloud<PointType>());
    imu_process_->Process(meas, *state_, undistorted);

    if (undistorted->empty()) return;

    // Downsample
    cur_scan_->clear();
    downsize_filter_surf_->setInputCloud(undistorted);
    downsize_filter_surf_->filter(*cur_scan_);

    // If map has points, do scan-to-map
    if (kdtree_ptr_->size() > 10) {
        // Simple scan-to-map using nearest neighbor
        Eigen::Matrix4f T_lidar_world = getLidarPose();
        pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cur_scan_, *transformed, T_lidar_world);

        // Add to map - use Add_Points with PointVector
        PointVector points_to_add;
        points_to_add.reserve(transformed->size());
        for (const auto& pt : transformed->points) {
            points_to_add.push_back(pt);
        }
        kdtree_ptr_->Add_Points(points_to_add, true);
    } else {
        // Initialize map - Build takes PointVector
        PointVector points_to_build;
        points_to_build.reserve(cur_scan_->size());
        for (const auto& pt : cur_scan_->points) {
            points_to_build.push_back(pt);
        }
        kdtree_ptr_->Build(points_to_build);
    }

    *cur_scan_world_ = *cur_scan_;
    first_scan_ = false;
}

Eigen::Matrix4f FastLIOSAM::getPose() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    state_ikfom s = state_->get_x();
    // Get rotation matrices as double, convert to float for final pose
    Eigen::Matrix3d R_world_lidar_d = s.rot.toRotationMatrix() * s.offset_R_L_I.toRotationMatrix();
    Eigen::Matrix3f R_world_lidar = R_world_lidar_d.cast<float>();
    Eigen::Matrix4f T_world_lidar = Eigen::Matrix4f::Identity();
    T_world_lidar.block<3, 3>(0, 0) = R_world_lidar;
    T_world_lidar.block<3, 1>(0, 3) = s.pos.cast<float>();
    return T_world_lidar;
}

Eigen::Matrix4f FastLIOSAM::getLidarPose() {
    return getPose();
}

Eigen::Vector3f FastLIOSAM::getVelocity() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return state_->get_x().vel.cast<float>();
}

double FastLIOSAM::getTimestamp() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return cur_stamp_;
}

pcl::PointCloud<PointType>::Ptr FastLIOSAM::getWorldScan() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return cur_scan_world_;
}

bool FastLIOSAM::hasOptimizedPose() {
    return false;
}

std::vector<std::pair<double, Eigen::Matrix4f>> FastLIOSAM::getOptimizedPose() {
    return {};
}

int FastLIOSAM::getMapSize() {
    return static_cast<int>(kdtree_ptr_->size());
}

int FastLIOSAM::getKeyFrameCount() {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    return static_cast<int>(keyframes_.size());
}

pcl::PointCloud<PointType>::Ptr FastLIOSAM::getFullMap() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    // Return current world scan as a proxy for the map
    return cur_scan_world_;
}

std::vector<KeyFrame> FastLIOSAM::getRecentKeyFrames(int n) const {
    (void)n;
    return {};
}

void FastLIOSAM::feedScanFeature(double stamp, const pcl::PointCloud<PointType>::Ptr& scan) {
    feedScan(stamp, scan);
}

void FastLIOSAM::feedGNSS(double stamp, double lat, double lon, double alt) {
    (void)stamp;
    (void)lat;
    (void)lon;
    (void)alt;
}

void FastLIOSAM::initGNSSOrigin(double lat, double lon, double alt) {
    (void)lat;
    (void)lon;
    (void)alt;
}

bool FastLIOSAM::shouldAddKeyFrame() {
    return false;
}

void FastLIOSAM::forceAddKeyFrame() {
}

void FastLIOSAM::reset() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    cur_scan_->clear();
    cur_scan_world_->clear();
    local_map_->clear();
    first_scan_ = true;
    imu_initialized_ = false;
    imu_buffer_.clear();

    // Reset ESKF state using API methods
    state_ikfom reset_state = state_->get_x();
    reset_state.pos = V3D::Zero();
    reset_state.rot = SO3();
    reset_state.vel = V3D::Zero();
    reset_state.bg = V3D::Zero();
    reset_state.ba = V3D::Zero();
    reset_state.grav = S2(V3D(0, 0, -G_m_s2));
    state_->change_x(reset_state);

    auto reset_P = state_->get_P();
    reset_P.setIdentity();
    reset_P *= 0.0001;
    state_->change_P(reset_P);
}

}  // namespace slam
