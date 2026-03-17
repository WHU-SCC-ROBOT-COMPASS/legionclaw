/******************************************************************************
 * Copyright 2018 The LegionClaw Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief This file provides the declaration of the class "TrajectoryEvaluator".
 */

#pragma once

#include <omp.h>

#include <cstddef>
#include <fstream>

#include "float.h"
// #include <nav_msgs/OccupancyGrid.h>
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/status/status.h"
// #include <visualization_msgs/MarkerArray.h>
// #include "RoadNetwork.h"
// #include "PlannerCommonDef.h"
// #include "PlanningHelpers.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/polygon2d.h"
// #include "tf/tf.h"
// #include "interface/path_point.hpp"
#include "modules/planning/src/proto/driving/lattice_planner_conf.pb.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "frame.h"
#include "interface/planning_trajectory.hpp"
#include "modules/common/interface/planning_analysis.hpp"
#include "potential_field.h"
#include "reference_line/reference_line.h"

using namespace std;
using namespace legionclaw::interface;
using namespace legionclaw::common::math;
using namespace legionclaw::common;

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

class TrajectoryEvaluator {
 public:
  TrajectoryEvaluator() = default;
  virtual ~TrajectoryEvaluator() = default;

  /**
   * @brief        初始化函数
   */
  void Init(const legionclaw::planning::PlanningConf *planning_conf);

  /**
   * @brief        参数初始化函数
   */
  void EvaluatorParamsInit(const legionclaw::planning::PlanningConf *planning_conf);

  /**
   * @brief        全局变量初始化函数
   */
  void EvaluatorVariableInit();

  /**
   * @brief       更新evaluator 本地的部分配置参数的函数
   * @param
   * @param     &部分参数与实时变化的车辆状态相关，可能需要实时更新
   */
  void UpdateKeyParams();
  void FindLinearCFromAToB(const double &a, const double &b, const double &x, const double &y,
                           const double input, double &c);

  /**
   * @brief        初始化函数:初始化单条轨迹的代价值结构
   */
  void InitTrajectoryCost(TrajectoryCost &tc);

  /**
   * @brief       轨迹代价值初始化函数
   */
  int InitializeTrajectoryCosts();

  /**
   * @brief       轨迹跃迁评价函数：描述从当前轨迹切换到其他轨迹的代价
   */
  bool CalculateRightmostCosts();

  /**
   * @brief       偏离当前车道参考线评价函数
   * @param[in]   &biasPreferCurrentlane 偏离当前车道参考线的惩罚值
   */
  int CalculatePriorityCosts(const double &biasPreferCurrentlane);

  /**
   * @brief       轨迹长度即纵向距离评价函数
   */
  int CalculateHorizonCosts(const std::vector<reference_line::ReferenceLine> &reference_lines);

  /**
   * @brief       轨迹路口接近评价函数
   * @param[in]
   */
  int CalculateJunctionCloseCosts(const legionclaw::interface::JunctionInfo &junction_info);

  /**
   * @brief       轨迹曲率评价函数
   * @param[in]  	&turning_radius 车辆最小转弯半径，用作比较
   */
  int CalculateCurvatureCosts(const double &turning_radius);

  /**
   * @brief       轨迹跃迁评价函数：描述从当前轨迹切换到其他轨迹的代价
   */
  int CalculateTransitionCosts();

  /**
   * @brief       轨迹簇代价值归一化函数
   */
  int NormalizeCosts();

  /**
   * @brief       计算动态障碍物的碰撞检测函数
   * @param[in]   &obj 预测障碍物信息
   * @param[in]   &path_pose 轨迹点
   * @param[in]   &adc_polygon 车辆包围盒
   * @param[out]  &collision_point 碰撞点
   * @param[out]  &trajectorycost 轨迹代价值等信息
   * @param[out]  &col_obj_polygon 有碰撞的障碍物多边形
   */
  bool CalculateDynamicObjPolygonCollision(const PredictionObstacleMsg &obj,
                                           const TrajectoryPoint &path_pose,
                                           const Polygon2d &adc_polygon, PathPoint &collision_point,
                                           TrajectoryCost &trajectorycost,
                                           Polygon2d &col_obj_polygon);

  /**
   * @brief       安全包围盒多边形函数
   * @param[in]   &currState 车辆信息
   * @param[in]   &c_lateral_d 横向保护距离
   * @param[in]   &c_long_front_d 前向保护距离
   * @param[in]   &c_long_back_d 后向保护距离
   */
  void GetSafetyBorderPolygon(const VehicleState &vehicleState, double &c_lateral_d,
                              double &c_long_front_d, double &c_long_back_d);

  /**
   * @brief       安全包围盒多边形II函数
   * @param[in]   &point 当前位置
   * @param[in]   &c_lateral_d 横向保护距离
   * @param[in]   &c_long_front_d 前向保护距离
   * @param[in]   &c_long_back_d 后向保护距离
   * @param[out]  &polygon 包围盒多边形
   */
  void GetSafetyBorderPolygon(const PathPoint &point, const double &c_lateral_d,
                              const double &c_long_front_d, const double &c_long_back_d,
                              Polygon2d &polygon);

  /**
   * @brief       计算多个障碍物的多边形函数
   * @param[in]   &obj_list 障碍物列表
   * @param[out]  &polygon_list 障碍物的多边形列表
   */
  bool ComputeObstacleListPolygon(const std::vector<PredictionObstacleMsg> &obj_list,
                                  std::vector<Polygon2d> &polygon_list);

  /**
   * @brief       计算障碍物的多边形函数
   * @param[in]   &obj 障碍物
   * @param[out]  &polygon 障碍物的多边形
   */
  bool ComputeObstaclePolygon(const PredictionObstacleMsg &obj, Polygon2d &polygon);

  /**
   * @brief       计算碰撞代价值函数
   * @param[in]   &extract_path 轨迹
   * @param[in]   &obj_list 障碍物列表
   * @param[in]   &global_path 道路参考线
   * @param[out]  &trajectory_cost 轨迹代价值等信息
   */
  void CaluculateCollisionCosts(const std::vector<TrajectoryPoint> &extract_path,
                                const std::vector<PredictionObstacleMsg> &obj_list,
                                const int &index, const std::map<int, int> &obstacles_lane_id,
                                const legionclaw::interface::GuideRoad &current_road,
                                TrajectoryCost &trajectory_cost, Polygon2d &col_obj_polygon);

  /**
   * @brief       计算轨迹横向代价值函数
   * @param[in]   &extract_path 轨迹
   * @param[in]   &obj_list 障碍物列表
   * @param[out]  &lateral_right_cost 轨迹右侧影响代价
   * @param[out] 	&lateral_left_cost 轨迹右侧影响代价
   * @param[out]	&blocked_cost
   */
  void CalculateLateralCosts(const std::vector<TrajectoryPoint> &extract_path,
                             const std::vector<PredictionObstacleMsg> &obj_list,
                             double &lateral_right_cost, double &lateral_left_cost,
                             double &blocked_cost);

  /**
   * @brief       车辆后方障碍物过滤函数
   * @param[in]   &extract_path 轨迹
   * @param		&obj_list 障碍物列表
   * @param[in]   &params 规划common配置
   */
  void FilterObstacleBackVehicle(const std::vector<TrajectoryPoint> &extract_path,
                                 std::vector<PredictionObstacle> &obj_list);

  /**
   * @brief       轨迹映射到势场中并获取势场值的函数
   * @param[in]	&potential_field 势场类对象
   * @param[in]   &extract_path 轨迹
   * @return 		potential_cost
   */
  double CalculatePotentialCosts(const PotentialField &potential_field,
                                 const std::vector<TrajectoryPoint> &extract_path);

  /**
   * @brief
   * @param[in]
   */
  void CalculatePassageCosts(const std::map<int, bool> is_passages_safe_f,
                             const std::map<int, bool> is_passages_safe_b,
                             const std::map<int, bool> is_passages_safe_dclc);

  /**
   * @brief       计算轨迹簇代价值主函数
   * @param[in]   &obj 障碍物列表信息
   */
  std::vector<TimeConsume> ComputeCostsDynamic(Frame *frame);

  /**
   * @brief       评估器入口功能函数
   * @param		&trajectory_array 轨迹生成器产生的轨迹簇信息
   * @param[in]   &frame 评估期依赖信息：如车道匹配结果、车辆状态、障碍物等
   */
  Status EvaluateTrajectories(const vector<vector<PlanningTrajectory>> &trajectory_array,
                              Frame *frame, std::vector<TimeConsume> &time_consume_vec);

 public:
  std::vector<std::vector<PlanningTrajectory>> TrajectoryArray() { return trajectory_array_; }

 private:
  const legionclaw::planning::PlanningConf *params_;
  legionclaw::interface::VehicleParam vehicle_param_;  // 车辆参数
  TrajectoryEvaluatorConf *evaluator_params_;

  const Frame *frame_;
  PotentialField potential_field_;
  std::vector<std::vector<PlanningTrajectory>> trajectory_array_;
  std::vector<PathPoint> collision_points_;
  int station_index_global_ = -1;
  int num_col_;
  int num_row_;
};

}  // namespace planning
}  // namespace legionclaw
