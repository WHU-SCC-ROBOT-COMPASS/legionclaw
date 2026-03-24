/**
 * @file    lidar_cluster_detect.h
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <thread>
#include <iomanip>

#include "modules/common/json/json.hpp"
#include "modules/common/status/status.h"
#include "modules/common/logging/logging.h"
#include "message_manager/message_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/fault/fault_client.hpp"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/base_message/message_status.hpp"
#include "modules/perception/lidar/lidar_cluster_detect/src/common/local_view.h"

#if LCM_ENABLE
#include "message_manager/lcm/lcm_message_manager.h"
#endif
#if DDS_ENABLE
#include "message_manager/dds/dds_message_manager.h"
#endif
#if ROS_ENABLE
#include "message_manager/ros/ros_message_manager.h"
#endif
#if ROS2_ENABLE
#include "message_manager/ros2/ros2_message_manager.h"
#endif

#include "conf/lidar_cluster_detect_conf.hpp"
/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {
using namespace legion::common;
using json = nlohmann::json;
/**
 * @class LidarClusterDetect
 * @brief 控制类.
 */
class LidarClusterDetect {
public:
  LidarClusterDetect(std::string file_path) : config_file_path_(file_path){};
  ~LidarClusterDetect() = default;
  /**
   * @brief     初始化．
   * @param[in] void．
   * @return    void.
   */
  void Init();

  /**
   * @brief     join．
   * @param[in] void.
   * @return    void.
   */
  void Join();

  /**
   * @brief Get the Conf object
   * @return std::shared_ptr<LidarClusterDetectConf>
   */
  std::shared_ptr<LidarClusterDetectConf> GetConf() const;

protected:
  //初始化状态
  bool is_init_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json lidar_cluster_detect_json_;
  //控制逻辑设置
  std::shared_ptr<LidarClusterDetectConf> lidar_cluster_detect_conf_;
  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<LidarClusterDetect>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<LidarClusterDetect>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<LidarClusterDetect>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<LidarClusterDetect>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<LidarClusterDetect>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<LidarClusterDetect>>
      adsfi_message_manager_;
#endif

protected:
  // Clustering helper structures
  struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
  };

  struct PointsCell {
    std::vector<PointXYZI> cell_points_;
    int find_flag_;
    PointsCell() : find_flag_(0) {}
  };

  struct Cluster {
    std::vector<PointXYZI> cluster_points_;
    std::vector<int> cell_col_;
    std::vector<int> cell_row_;
    float min_x, max_x, min_y, max_y, min_z, max_z;
    float center_x, center_y, center_z;
    float length, width, height;
    Cluster() : min_x(0), max_x(0), min_y(0), max_y(0), min_z(0), max_z(0),
                center_x(0), center_y(0), center_z(0),
                length(0), width(0), height(0) {}
  };

  typedef std::vector<PointsCell> CellVector;
  typedef std::vector<CellVector> CellMatrix;
  typedef std::shared_ptr<CellMatrix> CellMatrixPtr;
  typedef std::vector<Cluster> ClusterVector;
  typedef std::shared_ptr<ClusterVector> ClusterVectorPtr;

  // Clustering parameters
  float cell_map_min_x_;
  float cell_map_max_x_;
  float cell_map_min_y_;
  float cell_map_max_y_;
  float cell_map_min_z_;
  float cell_map_max_z_;
  float cell_width_;
  float cell_length_;
  int point_min_size_for_cell_;
  int point_min_size_for_box_;
  int cell_max_size_for_big_box_;
  float obstacle_min_height_;
  float delta_x_;
  float delta_y_;
  bool in_estimate_pose_;
  int col_size_;
  int row_size_;
  int max_polygon_points_;  // 外包围框最大点数限制

  // Clustering helper functions
  CellMatrixPtr BuildCellMap(const legion::interface::PointCloud& point_cloud, 
                             int col_size, int row_size);
  void ClusterConnectedPoints(CellMatrixPtr& cell_matrix_ptr,
                              ClusterVectorPtr& cluster_vector_ptr);
  void Find8(int col_id, int row_id, CellMatrixPtr& cell_matrix_ptr,
             Cluster& clusteri);
  void BuildCluster(ClusterVectorPtr& cluster_vector_ptr,
                    ClusterVectorPtr& filter_cluster_vector_ptr,
                    const legion::interface::Header& header);
  void ConvertClustersToObstacleList(ClusterVectorPtr& filter_cluster_vector_ptr,
                                     const legion::interface::Header& header,
                                     legion::interface::ObstacleList& obstacle_list);

protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<LidarClusterDetect>> message_manager);

  /**
   * @brief
   *
   */
  void VariableInit();

  /**
   * @brief     消息初始化.
   * @return    void.
   */
  void MessagesInit();

  /**
   * @brief     消息激活.
   * @return    void.
   */
  void MessagesActivate();

  /**
   * @brief     消息去激活.
   * @return    void.
   */
  void MessagesDeActivate();

  /**
   * @brief 定时器任务激活
   */
  void TaskActivate();

  /**
   * @brief 定时器任务停止
   */
  void TaskStop();

  /**
   * @brief 故障码监控初始化
   */
  void FaultMonitorInit();

  /**
   * @brief 周期发送控制输出到底盘,以及广播lidar_cluster_detect状态
   * @return void.
   */
  void Task100ms(void* param);

  /**
   * @brief
   * @param  obstacle_list
   */
  void PublishObstacleList(legion::interface::ObstacleList obstacle_list);

  /**
   * @brief
   * @param  faults
   */
  void PublishFaults();

  /**
   * @brief     发布聚类点云.
   * @param[in] filter_cluster_vector_ptr 过滤后的聚类结果.
   * @param[in] frame_id 坐标系.
   * @return    void.
   */
  void PublishClusterPointCloud(ClusterVectorPtr& filter_cluster_vector_ptr,
                                const legion::interface::Header& header);

  /**
   * @brief     打印调试.
   * @return    void.
   */
  void Print();

  /**
   * @brief     日志调试.
   * @return    void.
   */
  void Log();

public:
  /**
   * @brief     ObuCmdMsg消息接收.
   * @param[in] obu_cmd_msg .
   * @return    void.
   */
  void HandleObuCmdMsg(legion::interface::ObuCmdMsg obu_cmd_msg);

  /**
   * @brief     PointCloud消息接收.
   * @param[in] no_ground_points .
   * @return    void.
   */
  void HandlePointCloud(legion::interface::PointCloud point_cloud);

  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputeLidarClusterDetectCommandOnTimer();

  /**
   * @brief
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView* local_view);

  /**
   * @brief     状态监测.
   * @return    void.
   */
  void StatusDetectOnTimer();

protected:
  legion::interface::PointCloud point_cloud_;
  legion::interface::ObstacleList obstacle_list_;
  legion::interface::ObuCmdMsg obu_cmd_msg_;
  legion::interface::Faults faults_;

  legion::interface::FaultCodeSet* faultcodeset_;
  //控制命令生产周期
  int32_t produce_lidar_cluster_detect_command_duration_;
  //控制命令发送周期
  int32_t publish_lidar_cluster_detect_command_duration_;
  //状态检测周期
  uint32_t status_detect_duration_;
  // 功能激活状态,激活为true，未激活为false
  bool function_activation_;
  //消息状态
  std::map<std::string, legion::common::MessageStatus> message_status_;
  // task线程
  std::unique_ptr<std::thread> task_thread_;

  std::mutex mutex_;

  LocalView local_view_;

protected:
  //定时器
  std::shared_ptr<ADTimerManager<LidarClusterDetect, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<LidarClusterDetect, void>> task_100ms_;
  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();
};
} // namespace lidar
} // namespace perception
} // namespace legion
