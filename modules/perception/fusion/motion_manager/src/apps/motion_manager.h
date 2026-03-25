/**
 * @file    motion_manager.h
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iomanip>
#include <mutex>
#include <thread>
#include <queue>
#include <stack>
#include <set>
#include <deque>
#include <tuple>

#include "message_manager/message_manager.h"
#include "modules/common/base_message/message_status.hpp"
#include "modules/common/json/json.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/status/status.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "common/local_view.h"

#include "modules/common/math/math_utils.h"

#include "hungarian/Hungarian.h"

#include "converter/convert.hpp"

// #include <legion_perception_msgs/DetectedObjects.h>
// #include "tf/tf.h"
#include "common/interface/detected_objects.hpp"
#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include "common/tf2/include/tf2/transform_datatypes.h"


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

#include "conf/motion_manager_conf.hpp"
/**
 * @namespace legion::perception::fusion
 * @brief legion::perception::fusion
 */

namespace legion {
namespace perception {
namespace fusion {
using namespace legionclaw::common;
using json = nlohmann::json;
/**
 * @class MotionManager
 * @brief 控制类.
 */

class MotionManager {
public:
  MotionManager(std::string file_path) : config_file_path_(file_path){};
  ~MotionManager() = default;
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
   * @return std::shared_ptr<MotionManagerConf>
   */
  std::shared_ptr<MotionManagerConf> GetConf() const;

protected:
  //初始化状态
  bool is_init_;
  //配置文件路径
  std::string config_file_path_;
  //配置文件操作类
  json motion_manager_json_;
  //控制逻辑设置
  std::shared_ptr<MotionManagerConf> motion_manager_conf_;
  //消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<MotionManager>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<MotionManager>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  // DDS消息控制器
  std::shared_ptr<DdsMessageManager<MotionManager>> dds_message_manager_;
#endif
#if ROS_ENABLE
  std::shared_ptr<RosMessageManager<MotionManager>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<MotionManager>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  std::shared_ptr<AdsfiMessageManager<MotionManager>>
      adsfi_message_manager_;
#endif

protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<MotionManager>> message_manager);

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
   * @brief 周期发送控制输出到底盘,以及广播motion_manager状态
   * @return void.
   */
  void Task10ms(void *param);

  /**
   * @brief
   * @param  obstacle_list_output
   */
  void PublishObstacleListOutput(
      legionclaw::interface::ObstacleList obstacle_list_output);

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
  void HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg);

  /**
   * @brief     Location消息接收.
   * @param[in] location .
   * @return    void.
   */
  void HandleLocation(legionclaw::interface::Location location);

  /**
   * @brief     ObstacleList消息接收.
   * @param[in] in_obstacle_list .
   * @return    void.
   */
  void
  HandleObstacleListInput(legionclaw::interface::ObstacleList obstacle_list_input);

  /**
   * @brief     LCDObstacleList消息接收（来自lidar_cluster_detect）.
   * @param[in] lcd_obstacle_list .
   * @return    void.
   */
  void
  HandleLCDObstacleList(legionclaw::interface::ObstacleList lcd_obstacle_list);

  /**
   * @brief     计算算法输出.
   * @return    void.
   */
  void ComputeMotionManagerCommandOnTimer(void*);

  /**
   * @brief
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView *local_view);

  /**
   * @brief     状态监测.
   * @return    void.
   */
  void StatusDetectOnTimer();

protected:
  legionclaw::interface::Location location_;
  legionclaw::interface::ObstacleList obstacle_list_input_;
  legionclaw::interface::ObstacleList lcd_obstacle_list_input_;
  legionclaw::interface::ObstacleList obstacle_list_output_;
  legionclaw::interface::ObstacleList fusion_obstacle_list_;
  legionclaw::interface::ObuCmdMsg obu_cmd_msg_;

  //控制命令生产周期
  int32_t produce_motion_manager_command_duration_;
  //控制命令发送周期
  int32_t publish_motion_manager_command_duration_;
  //状态检测周期
  uint32_t status_detect_duration_;
  // 功能激活状态,激活为true，未激活为false
  bool function_activation_;
  //消息状态
  std::map<std::string, legionclaw::common::MessageStatus> message_status_;
  // task线程
  std::unique_ptr<std::thread> task_thread_;

  std::mutex mutex_;

  LocalView local_view_;

protected:
  //定时器
  std::shared_ptr<ADTimerManager<MotionManager, void>> ad_timer_manager_;
  std::shared_ptr<WheelTimer<MotionManager, void>> task_compute_;
  /**
   * @brief     Spin．
   * @param[in] void.
   * @return    void.
   */
  void Spin();

protected:
  set<int> id_set;
  int id_num_max;
  int use_location_time;
  int heading_error;
  int use_sync;
  double obj_rescale;
  // ========== 融合匹配相关参数 ==========
  double iou_threshold;  // IOU阈值，用于判断两个障碍物是否匹配
  double data_timeout_ms;  // 数据超时时间（毫秒），超过此时间未更新则认为数据中断
  double grid_size;  // 位置网格化精度（米），用于位置键值生成
  double history_match_max_distance;  // 历史匹配的最大距离（米），超过此距离不进行匹配
  double close_match_distance;  // 近距离匹配的距离阈值（米），小于此距离且IOU>0时认为匹配成功
  double very_close_match_distance;  // 非常近距离匹配的距离阈值（米），用于处理大幅形状变化
  
  // ========== ID管理相关参数 ==========
  int lcd_id_offset;  // LCD障碍物ID偏移量，LCD障碍物ID = 原始ID + lcd_id_offset
  
  // ========== 时间同步相关参数 ==========
  double duplicate_frame_threshold;  // 重复帧检测阈值（秒），时间差小于此值认为是重复帧
  double time_sync_max_diff;  // 时间同步最大时间差（秒），超过此值跳过时间同步但仍进行融合
  double match_quality_distance_offset;  // 匹配质量计算中的距离偏移量，用于避免除零
  std::string fixed_obstacle;
  std::string config_path;
  json confjson;
  legionclaw::interface::Time time;
  
  // 记录每个数据源的最后更新时间（毫秒时间戳）
  int64_t last_obstacle_list_input_time_ms_;
  int64_t last_lcd_obstacle_list_input_time_ms_;

  MultiObjectTracker tracker_;

  // int use_sync = 1;
  std::vector<legionclaw::interface::Location> location_list_;
  legionclaw::interface::Location location_current;

  int is_first_frame_ = 1;
  legionclaw::interface::ObstacleList last_frame_obs_;

  int use_kalman = 0;

  //tracking setup(Hungrain)
  std::vector<legionclaw::interface::Obstacle> last_frame_obs;
  int global_max_idx = 0;
  int idx = 0;
  std::set<int> global_id_set;
  
  //s
  int frame_c_ = 0;
  std::map<int, int> obl_count_map_;
  std::map<int, int>::iterator obl_count_map_iter_;
  std::vector<legionclaw::interface::Obstacle> obl_out_;
  std::map<int, std::list<legionclaw::interface::Obstacle>> ob_id_tracker_map_;
  std::map<int, std::list<legionclaw::interface::Obstacle>>::iterator ob_id_tracker_map_iter_;
  std::map<int, vector<legionclaw::interface::Point3D>> ob_vel_tracker_map_;
  std::map<int, vector<legionclaw::interface::Point3D>>::iterator ob_vel_tracker_map_iter_;

  std::map<int,int> frame_interpolation_map_;

  //compute acc
  std::map<int,std::vector<legionclaw::interface::Obstacle>> frames_obs_for_acc_;

  std::map<double,double> pos_map_;

  int global_idx = 0;
  map<int,int> id_convert_map_;
  
  // 用于融合后ID稳定化的全局ID计数器
  int fusion_global_idx_ = 0;
  // 用于融合后ID稳定化的ID映射表（从上一帧ID到稳定ID的映射）
  map<int,int> fusion_id_map_;
  
  // 用于维护历史Lidar ID的映射表（基于位置）
  // key: 位置哈希值（基于中心点坐标的网格化），value: 稳定Lidar ID
  struct PositionKey {
    int grid_x;
    int grid_y;
    bool operator<(const PositionKey& other) const {
      if (grid_x != other.grid_x) return grid_x < other.grid_x;
      return grid_y < other.grid_y;
    }
  };
  std::map<PositionKey, int> lidar_id_history_;  // 位置到Lidar ID的映射
  std::map<int, PositionKey> id_to_position_;    // ID到位置的映射（用于更新位置）
  
  // 历史障碍物信息结构
  struct HistoricalObstacle {
    int id;  // 障碍物ID
    legionclaw::interface::Point3D center_pos;  // 中心位置
    std::vector<legionclaw::interface::Point3D> polygon_points;  // 多边形点
    int frame_age;  // 帧年龄（距离当前帧的帧数，0表示当前帧）
  };
  
  // 历史帧窗口（最多10帧）
  static constexpr int HISTORY_WINDOW_SIZE = 30;
  std::deque<std::map<int, HistoricalObstacle>> obstacle_history_;  // 每帧的障碍物历史

  
  void MotionManagerRun();

  void SyncStamp(legionclaw::interface::ObstacleList &obl_list, legionclaw::interface::Location &location);
  void VehicleToWorld(legionclaw::interface::ObstacleList &data, legionclaw::interface::Location &location);
  void ConvertPoint(legionclaw::interface::Point3D &point, legionclaw::interface::Location &location);
  void GetPolygon(legionclaw::interface::ObstacleList &result_obstacle_list);
  void GetObstacleHeader(legionclaw::interface::ObstacleList &obstacle_list);
  double GetAngle(double theta);
  void set_polygon_abs(double len ,double wid ,double hei,double x ,double y ,double z,double theta ,legionclaw::interface::Obstacle &ob_);
  void TransObstacle(motion_manager::interface::DetectedObjects &objects , legionclaw::interface::ObstacleList &obl_list);
  void TransObstacle(legionclaw::interface::ObstacleList &obl_list , motion_manager::interface::TrackedObjects &objects);
  
  /**
   * @brief 融合 lcd_obstacle_list_input_ 和 obstacle_list_output_
   * @return void
   */
  void FuseObstacleLists();
  
  /**
   * @brief 计算两个障碍物多边形的 IOU
   * @param ob1 障碍物1
   * @param ob2 障碍物2
   * @return IOU 值 (0.0 ~ 1.0)
   */
  double CalculatePolygonIoU(const legionclaw::interface::Obstacle &ob1, const legionclaw::interface::Obstacle &ob2);
  
  /**
   * @brief 计算障碍物多边形的面积
   * @param ob 障碍物
   * @return 面积值
   */
  double CalculateObstacleArea(const legionclaw::interface::Obstacle &ob);
  
  /**
   * @brief 稳定化融合后的障碍物ID，通过IOU匹配上一帧来保持ID一致性
   * @param current_fusion_list 当前帧的融合结果
   * @param last_fusion_list 上一帧的融合结果
   */
  void StabilizeFusionIds(legionclaw::interface::ObstacleList &current_fusion_list, 
                          const legionclaw::interface::ObstacleList &last_fusion_list);
  
  /**
   * @brief 根据障碍物位置生成网格化键值
   * @param ob 障碍物
   * @return 位置键值
   */
  PositionKey GetPositionKey(const legionclaw::interface::Obstacle &ob);
  
  /**
   * @brief 计算两个障碍物基于最小矩形框的IOU
   * @param ob1 障碍物1
   * @param ob2 障碍物2
   * @return IOU值 (0.0 ~ 1.0)
   */
  double CalculateBoundingBoxIoU(const legionclaw::interface::Obstacle &ob1, 
                                  const legionclaw::interface::Obstacle &ob2);
  
  /**
   * @brief 从多边形计算最小矩形框（轴对齐）
   * @param polygon 多边形点
   * @return 矩形框 (min_x, min_y, max_x, max_y)
   */
  std::tuple<double, double, double, double> GetBoundingBox(
      const std::vector<legionclaw::interface::Point3D> &polygon);
  
  /**
   * @brief 更新历史障碍物记录
   * @param current_fusion_list 当前帧的融合结果
   */
  void UpdateObstacleHistory(const legionclaw::interface::ObstacleList &current_fusion_list);
  
  /**
   * @brief 从历史帧中匹配障碍物ID
   * @param current_ob 当前障碍物
   * @return 匹配到的历史ID，如果没有匹配返回-1
   */
  int MatchHistoricalObstacle(const legionclaw::interface::Obstacle &current_ob);

  void Getacc(legionclaw::interface::ObstacleList &obstacle_list_output);
  void Settype(legionclaw::interface::Obstacle &ob, motion_manager::interface::ObjectClassification &cls);
  void ResetId(legionclaw::interface::ObstacleList &obstacle_list_output_);
  void FilterVel(legionclaw::interface::ObstacleList &obstacle_list_output_);
};
} // namespace fusion
} // namespace perception
} // namespace legion
