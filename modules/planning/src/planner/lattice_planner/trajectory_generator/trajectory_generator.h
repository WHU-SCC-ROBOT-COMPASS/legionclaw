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
 * @brief This file provides the declaration of the class "TrajectoryGenerator".
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/reference_line/reference_line.h"
#include "modules/planning/src/proto/driving/lattice_planner_conf.pb.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/common/interface/lane_line.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/sl_boundary.hpp"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/status/status.h"
#include "modules/planning/src/common/frame.h"
#include "modules/planning/src/common/map_matcher/map_matcher.h"
#include "modules/planning/src/common/trajectory/trajectory1d_generator.h"
#include "modules/planning/src/interface/planning_trajectory.hpp"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

using namespace legionclaw::reference_line;
using namespace legionclaw::interface;
using namespace legionclaw::common::math;
using namespace legionclaw::planning::trajectory;
using legionclaw::interface::LaneLine;
using namespace legionclaw::common;

/**
 * @class TrajectoryGenerator
 * @brief
 */
class TrajectoryGenerator {
 public:
  TrajectoryGenerator() = default;

  virtual ~TrajectoryGenerator() = default;

  /**
   * @brief     初始化．
   * @param[in] void．
   * @return    void.
   */
  void Init(const legionclaw::planning::PlanningConf *planning_conf);

  /**
   * @brief 生成所有备选轨迹簇.
   * @param[in] is_need_replan　重规划标志.
   * @param[in] frame
   * 轨迹生成器所依赖的信息，如截取并平滑后的地图中心线数据、本车pose与地图的匹配信息TODO.
   * @param[in] path_density 轨迹点间隔密度.
   * @return    void.
   */
  Status GenerateTrajectories(bool &is_need_replan, Frame &frame, const double &path_density);

  /**
   * @brief 生成一个车道的备选轨迹簇.
   * @param[in] reference_line 轨迹簇生成所参考的中心线.
   * @param[in] is_need_replan　重规划标志.
   * @param[in] global_index 参考线index.
   * @param[in] path_density 轨迹点间隔密度.
   * @param     trajectories 生成的轨迹簇.
   * @return    void.
   */
  void GenerateTrajectoriesPerLane(const ReferenceLine &reference_line, bool &is_need_replan,
                                   const int &global_index, const double &path_density,
                                   std::vector<PlanningTrajectory> &trajectories);

  /**
   * @brief 为了保证轨迹更新时的平顺性，保留一段上一次的规划轨迹.
   * @param .
   * @param .
   * @return .
   */
  std::vector<PathPoint> ComputeStitchingPath(const double &stitching_length, bool &is_need_replan,
                                              const int &global_index);

  /**
   * @brief .
   * @param .
   * @param .
   * @return .
   */
  void ComputeInitFrenetState(const ReferencePoint &matched_point, const PathPoint &cartesian_state,
                              std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d);

  /**
   * @brief .
   * @param .
   * @param .
   * @return .
   */
  bool GeneratePaths(
      const Trajectory1DBundle &lat_trajectory_bundle,
      const std::vector<ReferencePoint> &path_points, const double &planning_length,
      const double &path_density, const double &start_mileage, const double &init_s,
      // std::vector<std::vector<PathPoint>> &generated_paths);
      const int &lon_id, const std::vector<int> &lat_ids,
      std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> &generated_paths);

  /**
   * @brief 根据参考线生成规划轨迹簇.
   * @param .
   * @param .
   * @return .
   */
  int PlanOnReferenceLine(
      const PathPoint &init_point, const ReferenceLine &reference_line, const double &speed,
      const double &safe_margin, const double &path_density,
      const double &trajectory_bundles_density, const double &min_lon_step,
      const double &base_length, const double &min_time, const double &max_time,
      const int &global_index,
      std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> &generated_paths);

  /**
   * @brief .
   * @param .
   * @param .
   * @return .
   */
  std::vector<PathPoint> ComputeExtendPath(const PathPoint &start_pos,  // const int &gid,
                                           const std::vector<ReferencePoint> &reference_points,
                                           ReferencePoint &matched_pos,
                                           const double &trajectory_length,
                                           const double &safe_margin, bool &is_narrow);

  /**
   * @brief .
   * @param .
   * @param .
   * @return .
   */
  std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint> &ref_points);
  /**
   * @brief 通过里程信息查询参考线上的点的序号.
   * @param[in] reference_points 参考线上的点.
   * @param[in] start_index 起始点序号.
   * @param[in] mileages 里程信息.
   * @return    int 当前里程对应的序号.
   */
  int get_matched_point_index(const std::vector<ReferencePoint> &reference_points,
                              const int start_index, const double mileages);

  std::vector<std::vector<PlanningTrajectory>> TrajectoryArray() { return trajectory_array_; }

  bool GetSLBoundary(const std::vector<ReferencePoint> &reference_points,
                     const std::vector<common::math::Vec2d> &corners,
                     SLBoundary *const sl_boundary) const;

 private:
  Frame frame_;
  const legionclaw::planning::PlanningConf *planning_conf_;
  TrajectoryGeneratorConf trajectory_generator_conf_;
  std::vector<std::vector<PlanningTrajectory>> trajectory_array_;
  legionclaw::interface::VehicleParam vehicle_param_;  // 车辆参数
  double end_offset_;                             // 采样终点偏移 左边为正右边为负
};

}  // namespace planning
}  // namespace legionclaw
