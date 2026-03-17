/**
 * @file
 *
 * @brief 泊车状态管理
 */
#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/planning_analysis.hpp"
#include "modules/common/interface/trajectory.hpp"
#include "modules/common/interface/trajectory_point.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/state_machine/state_context.hpp"
#include "modules/common/status/status.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/planning/src/common/frame.h"
#include "modules/planning/src/common/map_matcher/map_matcher.h"
#include "modules/planning/src/common/planning_gflags.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/planning/src/planner/parking_planner/geometry/parallel_parking_path_generator.h"
#include "modules/planning/src/planner/parking_planner/geometry/vertical_parking_path_generator.h"
namespace legionclaw {
namespace planning {
using namespace std;
using namespace legionclaw::common;
using namespace legionclaw::interface;
using namespace legionclaw::common::math;
/**
 * @class ParkingManager
 * @brief 生成泊车路径信息
 */
class ParkingManager {
 public:
  /**
   * @brief 构造函数
   */
  ParkingManager() = default;
  /**
   * @brief 析构函数
   */
  ~ParkingManager() = default;

  enum ParkingStage {
    STAGE_NONE = 0,
    STAGE1 = 1,
    STAGE2 = 2,
    STAGE3 = 3,
  };
  enum Stage1Move {
    MOVE_NONE = -1,
    MOVE0 = 0,
    MOVE1 = 1,
    MOVE2 = 2,
    MOVE3 = 3,
    MOVE4 = 4,
    MOVE_FINISH = 5,
    MOVE3_3_1 = 6,
    MOVE4_3_1 = 7,
    MOVE3_3_2 = 8,
    MOVE4_3_2 = 9,
  };
  enum Stage3Move {
    OUT_MOVE_NONE = 0,
    OUT_MOVE1 = 1,
    OUT_MOVE2 = 2,
    OUT_MOVE3 = 3,
    OUT_MOVE4 = 4,
    OUT_MOVE5 = 5,
    OUT_MOVE_FINISH = 6,
  };
  enum OutDirection {
    DirectionNone = 0,
    UP = 2,
    Down = 4,
    Left = 1,
    Right = 3,
  };
  /**
   * @brief 初始化函数
   * @param vehicle_params 输入量：车辆配置。
   * @param parking_manager_params 输入量：泊车管理配置。
   */
  void Init(const PlanningConf *planning_conf);

  bool Reset();

  void SetOutDirection(OutDirection out_direction);
  legionclaw::common::Status IsGeometryOK(Frame *frame);
  /**
   * @brief 计算泊车轨迹的主要处理函数
   * @param parking_info 输入量：泊车位信息。
   * @param cur_pose 输入量：当前位置信息。
   * @return 1成功，0不成功。
   */
  legionclaw::common::Status Process(Frame *frame, legionclaw::interface::ADCTrajectory *ptr_adc_trajectory,
                                legionclaw::interface::PlanningAnalysis *ptr_analysis);

  /**
   * @brief 泊车坐标系转换到全局坐标系。
   * @param path_p 输入量：泊车坐标系下路径
   * @param path_g 输入量：全局坐标系下路径
   * @return 1计算成功，0不成功。
   */
  bool TransferPark2Global(const std::vector<legionclaw::interface::PathPoint> &path_p,
                           std::vector<legionclaw::interface::PathPoint> &path_g);

  /**
   * @brief 计算本车安全包围盒
   * @param curr_state 输入量：车当前位姿。
   * @param car_info 输入量：车辆信息。
   * @param safe_width 输入量：安全宽度。
   * @param safe_length 输入量：安全长度。
   * @param polygon 输出量：本车安全包围盒。
   */
  void GetSafetyBorderPolygon(const legionclaw::interface::PathPoint &curr_state,
                              const legionclaw::interface::VehicleParam &car_info,
                              const double &safe_width, const double &safe_length,
                              math::Polygon2d &polygon);

  /**
   * @brief 计算纵向信息，产生泊车轨迹
   * @param parking_path 输入量：泊车路径。
   * @param gear_flag 输入量：档位标志。
   * @param in_collision 输入量：。
   * @return 带有纵向信息的泊车轨迹。
   */
  bool ComputeLonInfoLinear(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                            const double &cur_speed, const double &cur_acc,
                            const double &path_interval, const GearPosition &gear_flag,
                            bool in_collision, Trajectory *ptr_trajectory);

  /**
   * @brief 计算纵向信息，产生泊车轨迹
   * @param parking_path 输入量：泊车路径。
   * @param gear_flag 输入量：档位标志。
   * @param in_collision 输入量：。
   * @return 带有纵向信息的泊车轨迹。
   */
  bool ComputeLonInfo(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                      const double &path_interval, const GearPosition &gear_flag, bool in_collision,
                      Trajectory *ptr_trajectory);

  /**
   * @brief 计算纵向信息，产生泊车轨迹
   * @param parking_path 输入量：泊车路径。
   * @param gear_flag 输入量：档位标志。
   * @param collision_distance 输入量：碰撞距离（为负数表示没有碰撞）。
   * @return 带有纵向信息的泊车轨迹。
   */
  bool ComputeLonInfo(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                      const double &path_interval, const GearPosition &gear_flag,
                      const double &collision_distance, Trajectory *ptr_trajectory);

  /**
   * @brief 向后延长轨迹
   * @param start_pos 输入量：起始点。
   * @param extend_length 输入量：延长长度（单位：米）。
   * @param interval 输入量：延长轨迹的点间隔（单位：米）。
   * @param gear_flag 输入量：档位标志。
   * @param extend_trajectory 输出量：延长的轨迹。
   * @return true：成功，false：失败
   */
  bool ExtendTrajectory(const legionclaw::interface::PathPoint &start_pos, const double &extend_length,
                        const double &interval, const GearPosition &gear_flag,
                        Trajectory &extend_trajectory);

  /**
   * @brief 计算泊车路径与障碍物是否存在碰撞
   * @param safe_width 输入量：安全宽度。
   * @param safe_length 输入量：安全长度。
   * @param safe_distance 输入量：检测车前方一定距离内是否有碰撞（从车头开始）。
   * @return 碰撞距离（为负数表示没有碰撞）。
   */
  bool CollisionCheck(const std::vector<legionclaw::interface::PathPoint> &parking_path,
                      const double &safe_width, const double &safe_length,
                      const double &safe_distance, double &collision_distance);

  /**
   * @brief 计算泊车路径与障碍物是否存在碰撞
   * @param safe_width 输入量：安全宽度。
   * @param safe_length 输入量：安全长度。
   * @param safe_distance 输入量：检测车前方一定距离内是否有碰撞（从车头开始）。
   * @return 碰撞距离（为负数表示没有碰撞）。
   */
  bool InCollision(const double &ttc_window_size, legionclaw::interface::PlanningAnalysis *ptr_analysis,
                   legionclaw::interface::Trajectory trajectory);

  /**
   * @brief 点格式转换：PathPoint 转 TrajectoryPoint
   */
  legionclaw::interface::TrajectoryPoint PathPoint2WayPoint(legionclaw::interface::PathPoint in);
  void UpdateVehicleCurrentPose(const legionclaw::interface::PathPoint vehicle_current_pose);
  void UpdateTargetParkingPose(const legionclaw::interface::PathPoint target_parking_pos);
  void UpdateTargetParkingPose(const double &x, const double &y, const double &yaw);
  void UpdateTargetParkingPose(legionclaw::interface::ParkingInfo parking_info);
  void UpdateTargetParkingPose(legionclaw::interface::ParkingInfo parking_info, double length);
  void SetParkingPot(legionclaw::interface::ParkingInfo parking_info);
  legionclaw::interface::Polygon2D Polygon2d_to_Polygon2D(const legionclaw::common::math::Polygon2d p1);
  bool JudgeNeedMaxTurn(const legionclaw::interface::PathPoint p1, const legionclaw::interface::PathPoint p2);
  bool IsHalfDestination();
  legionclaw::common::Status UpdateParkingInfo(legionclaw::common::ParkingType parking_type);
  legionclaw::common::Status UpdateContext();
  void UpdateParkingOutStartPose();
  double safe_width;
  double safe_length;
  double lat_error_, lon_error_, yaw_error_;
  int num_size;
  bool need_max_turn;
  int parking_scheme_;
  double origin_to_up;
  double dtc;
  bool park_in_;
  bool steer_first_ = true;
  bool keep_steer = true;
  bool IsVehicleSteerNearCmd();

 protected:
  VehicleParam vehicle_params_;
  bool is_init_;
  VerticalParkingPathGenerator
      vertical_parking_path_generator_;  // 垂直泊车路径生成器（用于计算泊车路径）
  ParallelParkingPathGenerator
      parallel_parking_path_generator_;  // 平行泊车路径生成器（用于计算泊车路径）
  Transfer transfer_pg_;                 // 泊车坐标系和全局坐标系之间的转换

  ParallelParkingConf parallel_parking_conf_;
  VerticalParkingConf vertical_parking_conf_;  // 泊车配置参数
  legionclaw::interface::VehicleParam vehicle_param_;
  SpiralCurveConf spiral_curve_conf_;
  ParkingManagerConf parking_manager_conf_;  // 泊车管理参数
  legionclaw::interface::ParkingInfo parking_info_;
  legionclaw::interface::StopInfo stop_info_;
  int32_t max_re_planning_count_;

  VehicleState vehicle_state_;
  double vehicle_speed_;                              // 自车车速
  legionclaw::interface::PathPoint vehicle_current_pose_;  // 自车当前位姿（全局坐标系）
  legionclaw::interface::PathPoint local_vehicle_current_pose_;
  legionclaw::interface::PathPoint
      target_parking_pose_;  // 目标位姿最终停车点（车辆后轴中心点）（全局坐标系）
  std::vector<PredictionObstacleMsg> obstacle_list_;  // 障碍物列表（全局坐标系）
  legionclaw::interface::Trajectory parking_trajectory_;   // 泊车轨迹（全局坐标系）
  std::vector<legionclaw::interface::PathPoint> parking_path_;
  legionclaw::common::GearPosition target_gear_position_;
  legionclaw::common::Status parking_status_;

  double remain_length_;  // 离停车位置（轨迹末端）剩余的（沿轨迹纵向）距离
  legionclaw::common::ParkingType parking_type_;  // 泊车类型

 protected:
  bool first_hit_;
  double start_time_;
  // 自动驾驶状态（主要指横向状态）
  legionclaw::interface::ADCTrajectory::BehaviourLatState behaviour_lat_state_;
  // 自动驾驶状态（主要指纵向状态）
  legionclaw::interface::ADCTrajectory::BehaviourLonState behaviour_lon_state_;
  volatile ParkingStage parking_stage_;
  volatile Stage1Move stage1_move_;
  volatile Stage3Move stage3_move_;
  volatile OutDirection out_direction_;
  std::unique_ptr<state_machine::StateContext> parking_sm_;
  bool ParkingStateMachineInit();
  void ParkingInitialStateUpdate(const std::string &state_name, int state);
  void ParkingFinishedStateUpdate(const std::string &state_name, int state);

  void ParallelParkingBeginStateUpdate(const std::string &state_name, int state);
  void ParallelParkingForwardBeginStateUpdate(const std::string &state_name, int state);
  void ParallelParkingForwardStateUpdate(const std::string &state_name, int state);
  void ParallelParkingForwardFinishStateUpdate(const std::string &state_name, int state);
  void ParallelParkingBackwardBeginStateUpdate(const std::string &state_name, int state);
  void ParallelParkingBackwardStateUpdate(const std::string &state_name, int state);
  void ParallelParkingBackwardFinishStateUpdate(const std::string &state_name, int state);
  void ParallelParkingFinishStateUpdate(const std::string &state_name, int state);

  void VerticalParkingBeginStateUpdate(const std::string &state_name, int state);
  void VerticalParkingForwardBeginStateUpdate(const std::string &state_name, int state);
  void VerticalParkingForwardStateUpdate(const std::string &state_name, int state);
  void VerticalParkingForwardFinishStateUpdate(const std::string &state_name, int state);
  void VerticalParkingBackwardBeginStateUpdate(const std::string &state_name, int state);
  void VerticalParkingBackwardStateUpdate(const std::string &state_name, int state);
  void VerticalParkingBackwardFinishStateUpdate(const std::string &state_name, int state);
  void VerticalParkingFinishStateUpdate(const std::string &state_name, int state);

 protected:
  /**
   * @brief 车辆是否停止
   * @return true
   * @return false
   */
  bool IsTheVehicleStopped();
  /**
   * @brief 车辆是否接近停车点
   * @return true
   * @return false
   */
  bool IsNearDestination();

  /**
   * @brief 是否精确停车
   * @return true
   * @return false
   */
  bool IsParkingPrecisely();

  /**
   * @brief
   * @param  ptr_adc_trajectory
   */
  void UpdateADCTrajectory(legionclaw::interface::Trajectory *trajectory,
                           legionclaw::interface::ADCTrajectory *ptr_adc_trajectory);

  void ParkingPathEraseUpdate();
};

}  // namespace planning
}  // namespace legionclaw
