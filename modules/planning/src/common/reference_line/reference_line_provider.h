/******************************************************************************
 * Copyright 2017 The LegionClaw Authors. All Rights Reserved.
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
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <float.h>
#include <thread>

// #include "cyber/cyber.h"
#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/common/interface/vehicle_state.hpp"
// #include "modules/common/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common/interface/routing_response.hpp"
// #include "modules/common/proto/routing_response.pb.h"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/planning/src/common/math/include/smoothing_spline/spline_2d_solver.h"
// //#include "modules/planning/proto/planning_config.pb.h"
// #include "modules/planning/reference_line/discrete_points_reference_line_smoother.h"
#include "modules/planning/src/common/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/src/common/reference_line/reference_line.h"
#include "modules/planning/src/interface/route_segments.hpp"
#include "modules/planning/src/common/enum.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/curve_math.h"
#include "modules/planning/src/common/math/include/spline/spline.h"
#include "modules/planning/src/common/map_match_info.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
using namespace legionclaw::common::math;
using namespace legionclaw::reference_line;

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;
  ReferenceLineProvider(const legionclaw::planning::PlanningConf &planning_conf);

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  /**
   * @brief 初始化.
   */
  void Init();

  /**
   * @brief 初始化全局环境变量.
   */
  void VariableInit();

  /**
   * @brief
   */
  bool UpdateRoutingResponse(const interface::RoutingResponse& routing);

  /**
   * @brief
   */
  void UpdateVehicleState(const legionclaw::interface::VehicleState& vehicle_state);
  
  /**
   * @brief
   */  
  bool Start();
  
  /**
   * @brief
   */
  void Stop();
  
  /**
   * @brief
   */
  bool GetReferenceLines(std::list<ReferenceLine> *reference_lines);
  
  /**
   * @brief
   */
  double LastTimeDelay();

 private:
  /**
   * @brief
   */
  void GenerateThread();

  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLine(
      std::list<reference_line::ReferenceLine> *reference_lines,
      std::list<legionclaw::planning::RouteSegments> *segments,
      std::vector<int>& passage_match_index);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  void UpdateReferenceLine(
      const std::list<reference_line::ReferenceLine> &reference_lines,
      const std::list<legionclaw::planning::RouteSegments> &route_segments);
  
  /**
   * @brief
   */
  void SortReferenceLines(
    std::list<reference_line::ReferenceLine> *reference_lines,
    const std::vector<std::pair<std::vector<legionclaw::planning::LanePointInner>,
                                int>> &passage_smooth_lines,
    const std::list<reference_line::ReferenceLine> &latest_reference_lines);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  bool ExtendReferenceLine(const legionclaw::interface::VehicleState &vehicle_state,
                           legionclaw::planning::RouteSegments *segments,
                           reference_line::ReferenceLine *reference_line);
  
  /**
   * @brief
   */
  void GetAnchorPoints(const reference_line::ReferenceLine &reference_line,
                       std::vector<ReferencePoint> *anchor_points) const;
  
  /**
   * @brief
   */
  bool SmoothRouteSegment(const legionclaw::planning::RouteSegments &segments,
                          reference_line::ReferenceLine *reference_line);
  
  /**
   * @brief
   */
  bool SmoothPrefixedReferenceLine(
    const reference_line::ReferenceLine &prefix_ref, const reference_line::ReferenceLine &raw_ref,
    reference_line::ReferenceLine *reference_line, int &match_index);
  
  /**
   * @brief
   */
  bool ChangeAnchorPoints(const reference_line::ReferenceLine &prefix_ref,
                          std::vector<ReferencePoint> *anchor_points,
                          int &match_index);
  
  /**
   * @brief
   */
  bool SmoothReferenceLine(const reference_line::ReferenceLine &raw_reference_line,
                           reference_line::ReferenceLine *reference_line);
  
  /**
   * @brief
   */
  bool ExtendSegments(
      const std::vector<legionclaw::planning::LanePointInner> &segments,
      int start_index, int end_index,
      std::vector<legionclaw::planning::LanePointInner> *const truncated_segments)
      const;
  
  /**
   * @brief
   */
  bool CreateRouteSegments(
      std::list<legionclaw::planning::RouteSegments> *route_segments,
      const std::vector<std::pair<std::vector<legionclaw::planning::LanePointInner>,
                                  int>> &lane_line,
      const std::vector<int> &match_index_vec);

  /**
   * @brief 对各车道的中心线进行三次样条平滑并内插。
   * @param center_line 输入量：各车道的中心线。
   * @param smooth_line 输出量：平滑内插后的中心线。
   * @return true表示平滑成功，false表示失败
   */
  bool SmoothCenterLines(std::vector<legionclaw::interface::LanePoint> center_line,
                        double delta_mileage,
                        std::vector<legionclaw::planning::LanePointInner> &smooth_line);
  
  /**
   * @brief
   */
  bool Shrink(const legionclaw::interface::VehicleState &vehicle_state,
              reference_line::ReferenceLine *reference_line);
  
  /**
   * @brief
   */
  double LookForwardDistance(double velocity);
  
  /**
   * @brief
   */
  bool UpdateSmoothLines(const legionclaw::interface::RoutingResponse &routing);
  
  /**
   * @brief
   */
  bool GetPassageMatchIndex(
      const legionclaw::interface::VehicleState &vehicle_state,
      std::vector<int> &passage_match_index);

  /**
   * @brief
   */
  void GetEgoLaneType(
      const legionclaw::interface::VehicleState &vehicle_state,
      const legionclaw::interface::RoutingResponse &routing);

  /**
   * @brief 计算match_index、lat_offset、lon_index
   */
  void CalculateMapMatchInfo();

 public:
  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }

  legionclaw::planning::MapMatchInfo GetMapMatchInfo() const {
    return map_match_info_;
  }

  std::vector<std::vector<legionclaw::planning::LanePointInner>> GetSmoothLines()
      /*const*/ {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return smooth_lines_;
  }

  // bool IsReferenceLineUpdated() const { return is_reference_line_successd_; }

 private:
  planning::PlanningConf planning_conf_;

  std::unique_ptr<std::thread> task_future_;

  std::mutex vehicle_state_mutex_;
  std::mutex routing_mutex_;
  std::mutex reference_lines_mutex_;

  std::shared_ptr<legionclaw::reference_line::QpSplineReferenceLineSmoother> smoother_;
  interface::RoutingResponse routing_;
  // 平滑并插值后的中心线跟roting的lane_list 数量相等
  std::vector<std::vector<legionclaw::planning::LanePointInner>> smooth_lines_;
  // 只有三条,并且按固定结构存储，顺序依次为：左、自车道、右
  std::vector<std::pair<std::vector<legionclaw::planning::LanePointInner>, int>> passage_smooth_lines_;
  //针对可行驶的相邻车道实时产生的route_segments，数量为0～3，用于extend
  std::list<legionclaw::planning::RouteSegments> route_segments_cache_;
  //针对可行驶的相邻车道生成的平滑的车道中心线，数量和route_segments_保持一致，数量为0～3
  std::list<reference_line::ReferenceLine> reference_lines_cache_;
  // 经过从左到右排序/刷选后的平滑的车道中心线，顺序依次为：左、自车道、右
  std::list<reference_line::ReferenceLine> reference_lines_;
  //备份顺序依次为：左、自车道、右
  // std::queue<std::list<reference_line::ReferenceLine>> reference_line_history_;

  legionclaw::interface::VehicleState vehicle_state_;
  planning::MapMatchInfo map_match_info_;  //本车pose与地图的匹配信息
  double dis_to_route_center_line_ = 0.0;
  const int max_lane_num_ = 3;
  std::atomic<bool> is_stop_{false};
  std::atomic<bool> is_reference_line_updated_{true};
  bool has_routing_ = false;
  bool is_new_routing_ = false;
  // bool is_reference_line_successd_ = false;
  bool is_initialized_ = false;
};

}  // namespace planning
}  // namespace legionclaw