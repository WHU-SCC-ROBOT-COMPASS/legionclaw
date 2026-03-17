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
 * @file
 **/

#pragma once

#include "omp.h"
#include <float.h>
#include <fstream>
#include <iomanip>

#include <cfloat>
#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>

#include "modules/common/util/util.h"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/drivable_region.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/guide_info.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/traffic_light.hpp"
#include "modules/common/data/vehicle_param/proto/vehicle_param.pb.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/curve_math.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/interface/prediction_obstacles.hpp"
#include "modules/planning/src/common/local_view.h"
#include "modules/planning/src/common/math/include/spline/spline.h"
#include "modules/planning/src/common/reference_line/reference_line.h"
#include "modules/planning/src/common/struct.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "modules/planning/src/interface/planning_trajectory.hpp"
#include "modules/planning/src/interface/prediction_obstacle_msg.hpp"
#include "modules/planning/src/interface/lane_point_inner.hpp"
#include "modules/planning/src/interface/route_segments.hpp"
#include "modules/planning/src/common/enum.h"
#include "modules/planning/src/common/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/src/common/map_match_info.h"
#include "modules/planning/src/common/math/include/mat3.hpp"

namespace legionclaw {
namespace planning {
using namespace legionclaw::common::math;
using namespace legionclaw::reference_line;

struct VehicleState {
  legionclaw::interface::PathPoint pose;
  legionclaw::common::math::Polygon2d vehicle_polygon2d;
  double speed;
  double acceleration;
  //The angle between vehicle front wheel and vehicle longitudinal axis
  double steer;
  legionclaw::common::MovingStatus moving_status;
  legionclaw::common::GearPosition gear;
};

struct TrajectoryMatchInfo {
  int lon_index = -1;           //轨迹上第几个点
  double lat_offset = DBL_MAX;  //左+右-
};

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */
class Frame {
 public:
  Frame() = default;
  explicit Frame(uint32_t sequence_num);

  // Frame(uint32_t sequence_num, const LocalView &local_view,
  //       const common::TrajectoryPoint &planning_start_point,
  //       const common::VehicleState &vehicle_state,
  //       ReferenceLineProvider *reference_line_provider);

  // Frame(uint32_t sequence_num, const LocalView &local_view,
  //       const common::TrajectoryPoint &planning_start_point,
  //       const common::VehicleState &vehicle_state);

  virtual ~Frame() = default;

  // const common::TrajectoryPoint &PlanningStartPoint() const;

  void Init(const legionclaw::planning::PlanningConf *planning_conf);

  void VariableInit();

  interface::PathPoint GetStartPose();

  Lane_Changing_Mode LaneChangingMode() const { return lane_changing_mode_; }

  /**
   * @brief
   */
  void UpdateLaneChangeCmd(const interface::ObuCmdMsg &obu_cmd_msg);

  void SetLaneChangeState(const Lane_Change_State lane_change_state) {
    lane_change_state_ = lane_change_state;
  }
  Lane_Change_State LaneChangeState() const { return lane_change_state_; }

  void SetLaneChangeDirection(const common::Direction lane_change_direction) {
    lane_change_direction_ = lane_change_direction;
  }

  common::Direction LaneChangeDirection() const {
    return lane_change_direction_;
  }

  bool ExtractReferenceLinesInfo(const std::list<ReferenceLine> &reference_lines);

  bool ExtractGuideInfo(const interface::GuideInfo &guide_info);

  bool ExtractTrafficEvents(const interface::TrafficEvents &traffic_events);

  bool ExtractVehicleState(const interface::Location &location_,
                           const interface::Chassis &chassis_,
                           const legionclaw::interface::VehicleParam vehicle_param);

  bool ExtractTrafficLightInfo(
      const interface::TrafficLightMsg &traffic_light_msg);

  bool ExtractObstacleList(
      const interface::PredictionObstacles &prediction_obstacles);

  bool CopyObstacleMsg(
      const interface::PredictionObstacles &prediction_obstacles);

  void SortObstacles();

  void TrafficLightDecision();

  void GreenbeltOcclusionDecision();

  const PredictionObstacleMsg *CreateStopObstacle(
      const reference_line::ReferenceLine &reference_line,
      const std::string &obstacle_id, const common::math::Vec2d &stop_point,
      const double lane_s);

  const PredictionObstacleMsg *CreateStaticVirtualObstacle(
      const std::string &id, const Box2d &box, const int &dest_lane_index);

  /**
   * @brief 大车决策信息.
   * @param .
   * @param .
   * @return void.
   */
  void CreateDecisionOfTruckAvoidance();

  bool CalTrajectoryMatchInfo();

  void ExtractParkingInfo(const interface::ParkingInfo &parking_info);

  void ExtractParkingOutInfo(const interface::ParkingOutInfo &parking_out_info);

  void ExtractDrivableRegion(
      const legionclaw::interface::DrivableRegion &drivable_region);

  void ExtractStopInfo(const interface::StopInfo &stop_info);

  /************************
   * interface functions
   ************************/
  legionclaw::planning::VehicleState VehicleState() const { return vehicle_state_; }

  inline void set_map_match_info(
      const legionclaw::planning::MapMatchInfo &map_match_info) {
    map_match_info_ = map_match_info;
    }

    inline const legionclaw::planning::MapMatchInfo &MapMatchInfo() const {
      return map_match_info_;
    }

    inline void set_smooth_lines(
        const std::vector<std::vector<legionclaw::planning::LanePointInner>>
            &smooth_lines) {
      smooth_lines_ = smooth_lines;
    }

    inline const std::vector<std::vector<legionclaw::planning::LanePointInner>>
    SmoothLines() const {
      return smooth_lines_;
    }

    legionclaw::planning::TrajectoryMatchInfo TrajectoryMatchInfo() const {
      return trajectory_match_info_;
    }

    void SetLastPlanningTrajectory(
        const PlanningTrajectory &last_planning_trajectory) {
      last_planning_trajectory_ = last_planning_trajectory;
    }

    inline const PlanningTrajectory &LastPlanningTrajectory() const {
      return last_planning_trajectory_;
    }

    inline const std::vector<interface::TrajectoryPoint> &
    LastTrajectoryPoints() {
      return last_planning_trajectory_.trajectory_points();
    }

    inline const interface::TrajectoryPoint LastTrajectoryPoints(const int &i)
        const {
      // if (i > -1 && i <
      // (int)last_planning_trajectory_.trajectory_points_size())
      return last_planning_trajectory_.trajectory_points(i);
    }

    inline uint32_t LastPlanningTrajectorySize() const {
      return last_planning_trajectory_.trajectory_points_size();
    }

    inline const planning::BehaviourState LastBehaviour() const {
      return last_behaviour_;
    }

    void SetLastBehaviour(const BehaviourState &last_behaviour) {
      last_behaviour_ = last_behaviour;
    }

    std::vector<reference_line::ReferenceLine> ReferenceLines() const {
      return reference_lines_;
    }

    reference_line::ReferenceLine ReferenceLine(const int &i) {
      // if (i > -1 && i < (int)reference_lines_.size())
      return reference_lines_.at(i);
    }

    std::vector<interface::TrafficLight> TrafficLights() {
      return traffic_lights_;
    }

    interface::TrafficLight TrafficLights(const int &i) {
      if (i > -1 && i < (int)traffic_lights_.size())
        return traffic_lights_.at(i);
    }

    inline const std::vector<PredictionObstacleMsg> &ObstacleList() const {
      return obstacle_list_;
    }

    PredictionObstacleMsg ObstacleList(const int &i) {
      if (i > -1 && i < (int)obstacle_list_.size()) return obstacle_list_.at(i);
    }

    legionclaw::interface::ParkingInfo GetParkingInfo() { return parking_info_; }

    legionclaw::interface::DrivableRegion GetDrivableRegion() {
      return drivable_region_;
    }

    inline const legionclaw::interface::TrafficEvents &GetTrafficEvents() const {
      return traffic_events_;
    }

    inline const legionclaw::interface::GuideInfo &GetGuideInfo() const {
      return guide_info_;
    }

    inline void set_dis_to_route_center_line(
        const double &dis_to_route_center_line) {
      dis_to_route_center_line_ = dis_to_route_center_line;
    }

    inline const double &dis_to_route_center_line() const {
      return dis_to_route_center_line_;
    }

    inline void SetAheadMapLimitSpeed(const double &ahead_map_limit_speed) {
      ahead_map_limit_speed_ = ahead_map_limit_speed;
    }
    inline const double &AheadMapLimitSpeed() const {
      return ahead_map_limit_speed_;
    }

    inline const YieldAction &yield_action() const { return yield_action_; }

    inline const double &yield_distance() const { return yield_distance_; }

    inline void SetTerminalType(const int8_t &type) {
      terminal_stop_point_.set_type(type);
    }

    inline const StopPoint &terminal_stop_point() const {
      return terminal_stop_point_;
    }

    inline void set_adc_polygon(const Polygon2d &adc_polygon) {
      adc_polygon_ = adc_polygon;
    }

    inline const Polygon2d &adc_polygon() const { return adc_polygon_; }

    inline void set_col_obj_polygon(const Polygon2d &col_obj_polygon) {
      col_obj_polygon_ = col_obj_polygon;
    }

    inline const Polygon2d &col_obj_polygon() const { return col_obj_polygon_; }

    void Log();

   private:
    const legionclaw::planning::PlanningConf *planning_params_;  // 参数

    legionclaw::planning::VehicleState vehicle_state_;  // TODO:车辆状态，未赋值
    std::vector<PredictionObstacleMsg> obstacle_list_;
    std::vector<interface::TrafficLight>
        traffic_lights_;  // TODO:红绿灯，未赋值
    legionclaw::interface::GuideInfo guide_info_;
    legionclaw::interface::TrafficEvents traffic_events_;
    legionclaw::interface::DrivableRegion drivable_region_;
    legionclaw::interface::ParkingInfo parking_info_;
    legionclaw::interface::ParkingOutInfo parking_out_info_;
    interface::StopPoint terminal_stop_point_;
    // 截取并平滑后的地图中心线数据
    std::vector<reference_line::ReferenceLine> reference_lines_;
    // 平滑并插值后的中心线跟roting的lane_list 数量相等
    std::vector<std::vector<legionclaw::planning::LanePointInner>> smooth_lines_;
    planning::MapMatchInfo map_match_info_;  // 本车pose与地图的匹配信息
    // 本车pose与规划轨迹的匹配信息
    planning::TrajectoryMatchInfo trajectory_match_info_;
    PlanningTrajectory last_planning_trajectory_;  // 上一次规划的轨迹
    planning::BehaviourState last_behaviour_;      // 上一次决策的行为

    uint32_t sequence_num_ = 0;
    double dis_to_route_center_line_ = 0.0;
    double ahead_map_limit_speed_ = 10.0;
    Lane_Changing_Mode lane_changing_mode_ = Lane_Changing_Mode::Invalid;
    common::Direction lane_change_direction_ = common::Direction::DIR_INVALID;
    Lane_Change_State lane_change_state_ =
        Lane_Change_State::LANE_CHANGE_INVALID;
    int manual_lc_plus_ = 0;
    std::vector<ObstacleMsg> truck_list_;  // 左右相邻车道的大车列表
    YieldAction yield_action_;             // 避让动作
    double yield_distance_;                // 避让距离
    Polygon2d adc_polygon_;
    Polygon2d col_obj_polygon_;
    bool is_stopped_;
    bool is_stopped_first_hit_;
    int64_t last_time_vehicle_is_stopped_or_not_;
    bool greenbelt_occlusion_done_;
  };

// class FrameHistory : public IndexedQueue<uint32_t, Frame> {
//  public:
//   FrameHistory();
// };

}  // namespace planning
}  // namespace legionclaw
