#include <pcl_conversions/pcl_conversions.h>

// ROS1 includes
#if ROS_ENABLE
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// livox_ros_driver only available in ROS1
#include <livox_ros_driver/CustomMsg.h>

// Type aliases for ROS1 sensor messages
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointCloud2MsgPtr = sensor_msgs::PointCloud2::ConstPtr;
#endif

// ROS2 includes
#if ROS2_ENABLE
#include <sensor_msgs/msg/point_cloud2.hpp>

// Type aliases for ROS2 sensor messages
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

// livox_ros_driver ROS2 support (if available)
#ifdef HAVE_LIVOX_ROS2
#include <livox_ros_driver/msg/custom_msg.hpp>
#endif
#endif

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, RS128}; //{1, 2, 3, 4}
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};//未判断，可能平面，平面，跳跃边，平面交接边,细线
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind}; // 未判断，接近0度，接近180度，接近远端，接近近端

//用于记录每个点的距离、角度、特征种类等属性
struct orgtype
{
  double range; //平面距离
  double dista; //与后一个点的间距平方
  double angle[2]; // cos(当前点指向前一点或后一点的向量, ray)
  double intersect;// // 当前点与相邻两点的夹角cos值
  E_jump edj[2]; // 点前后两个方向的edge_jump类型
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

#if ROS_ENABLE
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace rslidar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, curvature)
                                  (float, time, normal_x)
                                  (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
#endif // ROS_ENABLE


class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

#if ROS_ENABLE
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
#endif

#if ROS2_ENABLE
  #ifdef HAVE_LIVOX_ROS2
  void process(const livox_ros_driver::msg::CustomMsg::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  #endif
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
#endif

  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf; //储存全部点(特征提取或间隔采样后）、角点、面特征点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;
  double blind; //xy平面距离，小于此阈值不计算特征
  bool feature_enabled, given_offset_time;

#if ROS_ENABLE
  ros::Publisher pub_full, pub_surf, pub_corn;
#endif


  private:
#if ROS_ENABLE
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void rs_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
#endif

#if ROS2_ENABLE
  #ifdef HAVE_LIVOX_ROS2
  void avia_handler(const livox_ros_driver::msg::CustomMsg::ConstSharedPtr &msg);
  #endif
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void rs_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
#endif

  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types); // 当前扫描线点云， 扫描点属性
#if ROS_ENABLE || ROS2_ENABLE
  void pub_func(PointCloudXYZI &pl, double timestamp);
#else
  // ROS-agnostic version: no publishing
  inline void pub_func(PointCloudXYZI &pl, double timestamp) { (void)pl; (void)timestamp; }
#endif
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size; //计算平面特征时需要的最少局部点数
  double disA, disB, inf_bound; //
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;//??
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
