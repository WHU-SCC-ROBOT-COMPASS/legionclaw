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
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/src/common/reference_line/reference_line_provider.h"

#include <algorithm>
#include <limits>
#include <utility>

// #include "cyber/common/file.h"
// #include "cyber/task/task.h"
// #include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
// #include "modules/map/hdmap/hdmap_util.h"
// #include "modules/planning/src/common/pnc_map/path.h"
// #include "modules/planning/src/common/planning_context.h"
#include "modules/planning/src/common/planning_gflags.h"
// #include "modules/routing/common/routing_gflags.h"
#include "modules/planning/src/common/map_matcher/map_matcher.h"

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

constexpr double kSegmentationEpsilon = 0.2;

// using legionclaw::common::VehicleConfigHelper;
using legionclaw::interface::VehicleState;
using legionclaw::common::math::AngleDiff;
using legionclaw::common::math::Vec2d;
// using legionclaw::cyber::Clock;
// using legionclaw::hdmap::HDMapUtil;
// using legionclaw::hdmap::LaneWaypoint;
// using legionclaw::hdmap::MapPathPoint;
// using legionclaw::hdmap::PncMap;
// using legionclaw::hdmap::RouteSegments;

ReferenceLineProvider::~ReferenceLineProvider() {}

ReferenceLineProvider::ReferenceLineProvider(const PlanningConf &planning_conf) {
  planning_conf_ = planning_conf;
  smoother_.reset(new QpSplineReferenceLineSmoother(
      planning_conf_.anchor_sampling_step(),
      planning_conf_.knots_sampling_step(),
      planning_conf_.lateral_bound_limit()));
  is_initialized_ = true;
  Init();
}

void ReferenceLineProvider::Init() {
  VariableInit();
}

void ReferenceLineProvider::VariableInit() {
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  reference_lines_.clear();
  std::list<reference_line::ReferenceLine> ().swap(reference_lines_);
  reference_lines_.resize(max_lane_num_);
  map_match_info_.lon_index = -1;
}

bool ReferenceLineProvider::UpdateRoutingResponse(
    const interface::RoutingResponse &routing) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  routing_ = routing;
  has_routing_ = true;
  is_new_routing_ = true;
  return true;
}

void ReferenceLineProvider::UpdateVehicleState(
    const legionclaw::interface::VehicleState& vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

bool ReferenceLineProvider::Start() {
  Init();
  is_stop_ = false;
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_.reset(new std::thread([this] { GenerateThread(); }));
    if (task_future_ == nullptr) {
      AERROR << "Unable to create can task_future_ thread.";
      return false;
    }
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_reference_line_provider_thread) {
    if (task_future_ != nullptr && task_future_->joinable()) {
      task_future_->join();
      task_future_.reset();
      AINFO << "task_future_ stopped [ok].";
    }
  }
}

void ReferenceLineProvider::GenerateThread() {
  int64_t sleep_interval = 0;
  int64_t start_time = TimeTool::Now2Ms();
  int64_t end_time = TimeTool::Now2Ms();
  while (!is_stop_) {
    is_reference_line_updated_ = true;
    static constexpr int32_t delta_period = 50;  // milliseconds
    sleep_interval = delta_period - (end_time - start_time);
    if (sleep_interval > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval));
    } else {
      stringstream sstream;
      // do not sleep
      sstream << "Too much time for calculation: " << end_time - start_time
              << "ms is more than minimum period: " << delta_period << "ms";
      // AWARN << sstream.str();
    }
    start_time = TimeTool::Now2Ms();
    if (!has_routing_) {
      // AERROR << "Routing is not ready.";
      end_time = TimeTool::Now2Ms();
      continue;
    }
    std::list<reference_line::ReferenceLine> reference_lines;
    std::list<legionclaw::planning::RouteSegments> segments;
    std::vector<int> passage_match_index;
    if (!CreateReferenceLine(&reference_lines, &segments, passage_match_index)) {
      is_reference_line_updated_ = false;
      AERROR << "Fail to get reference line";
      end_time = TimeTool::Now2Ms();
      continue;
    }
    //缓存最新的参考线用于下个周期进行extend 拼接
    UpdateReferenceLine(reference_lines, segments);
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    //最新的reference_lines按一定规则存放在reference_lines_
    SortReferenceLines(&reference_lines_, passage_smooth_lines_, reference_lines);
    //更新 match_index、lat_offset、lon_index
    CalculateMapMatchInfo();
    if (map_match_info_.lon_index < 0) {
      AERROR << "map_match_info_.lon_index is error";
    }
    end_time = TimeTool::Now2Ms();
    // last_calculation_time_ = end_time - start_time;
  }
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines) {
  CHECK_NOTNULL(reference_lines);
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  if (FLAGS_enable_reference_line_provider_thread) {
    for (auto iter = reference_lines_.begin(); iter != reference_lines_.end();
         ++iter) {
      if (iter->ReferencePointSize() > 0) {
        reference_lines->assign(reference_lines_.begin(),
                                reference_lines_.end());
        return true;
      }
    }
  } else {
    std::list<RouteSegments> *segments;
    std::vector<int> passage_match_index;
    CHECK_NOTNULL(segments);
    if (CreateReferenceLine(reference_lines, segments, passage_match_index)) {
      UpdateReferenceLine(*reference_lines, *segments);
      SortReferenceLines(&reference_lines_, passage_smooth_lines_,*reference_lines);
      return true;
    }
  }


  // // AWARN << "Reference line is NOT ready.";
  // if (reference_line_history_.empty()) {
  //   // AERROR << "Failed to use reference line latest history";
  //   return false;
  // }

  // reference_lines->assign(reference_line_history_.back().begin(),
  //                         reference_line_history_.back().end());
  // AWARN << "Use reference line from history!";
  return true;
}

double ReferenceLineProvider::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * planning_conf_.look_forward_time_sec();
  if(forward_distance > planning_conf_.look_forward_long_distance()){
    forward_distance = planning_conf_.look_forward_long_distance();
  }else if(forward_distance < planning_conf_.look_forward_short_distance()){
    forward_distance = planning_conf_.look_forward_short_distance();
  }
  return forward_distance;
}

bool ReferenceLineProvider::SmoothRouteSegment(const legionclaw::planning::RouteSegments &segments,
                               reference_line::ReferenceLine *reference_line) {
  reference_line::ReferenceLine new_ref(segments.lane_points());
  new_ref.ComputeAllMileage(0.0);
  return SmoothReferenceLine(new_ref, reference_line);
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const reference_line::ReferenceLine &prefix_ref, const reference_line::ReferenceLine &raw_ref,
    reference_line::ReferenceLine *reference_line, int &match_index) {
  // generate anchor points:
  std::vector<ReferencePoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);
  // modify anchor points based on prefix_ref
  //找到raw_ref的anchor_points在上一次平滑参考线内的第一个anchor_point，
  //并将上一次平滑参考线距离该anchor_point最近的一个点的位置信息和强约束重新赋值给该anchor_point
  if (!ChangeAnchorPoints(prefix_ref, &anchor_points, match_index)) {
    AWARN << "Failed to get forward shifted anchor_points";
    return false;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line,
                         planning_conf_.path_density(),
                         planning_conf_.map_spline_order())) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    *reference_line = prefix_ref;
    return false;
  }
  return true;
}

bool ReferenceLineProvider::ChangeAnchorPoints(const reference_line::ReferenceLine &prefix_ref,
                               std::vector<ReferencePoint> *anchor_points,
                               int &match_index) {
  double dist_min = DBL_MAX;
  auto anchor_point = anchor_points->begin();
  //筛选的anchor_point需满足能投影到prefix_ref上，且sl_l较小。anchor_points至少有两个
  while (anchor_points->size() > 2) {
    match_index = MapMatcher::QueryNearestPointWithBuffer(
        prefix_ref.ReferencePoints(),
        {anchor_point->x(), anchor_point->y(), anchor_point->z()},
        anchor_point->theta(), 1.0e-6, dist_min);
    auto prefix_ref_point = prefix_ref.ReferencePoints(match_index);
    anchor_point->set_x(prefix_ref_point.x());
    anchor_point->set_y(prefix_ref_point.y());
    anchor_point->set_z(prefix_ref_point.z());
    anchor_point->set_theta(prefix_ref_point.theta());
    anchor_point->set_longitudinal_bound(1e-6);
    anchor_point->set_lateral_bound(1e-6);
    anchor_point->set_enforced(true);
    return true;
  }
  return false;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const reference_line::ReferenceLine &raw_reference_line, reference_line::ReferenceLine *reference_line) {
  // generate anchor points:
  std::vector<ReferencePoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);
  if (anchor_points.size() < 2)
    return false;
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_reference_line, reference_line,
                         planning_conf_.path_density(),
                         planning_conf_.map_spline_order())) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  return true;
}

bool ReferenceLineProvider::ExtendSegments(
    const std::vector<legionclaw::planning::LanePointInner> &segments,
    int start_index, int end_index,
    std::vector<legionclaw::planning::LanePointInner> *const truncated_segments)
    const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }

  if (start_index >= end_index) {
    AERROR << "start_index(" << start_index << " >= end_index(" << end_index << ")";
    return false;
  }

  truncated_segments->clear();
  for (int i = start_index; i <= end_index; i++) {
    truncated_segments->emplace_back(segments.at(i));
  }
  return true;
}

bool ReferenceLineProvider::CreateRouteSegments(
    std::list<legionclaw::planning::RouteSegments> *route_segments,
    const std::vector<std::pair<std::vector<legionclaw::planning::LanePointInner>,
                                int>> &lane_line,
    const std::vector<int> &match_index_vec) {
  // GetRouteSegments
  int future_start_index = -1;
  int future_end_index = -1;
  for (unsigned int i = 0; i < lane_line.size(); i++) {
    if (lane_line.at(i).first.size() < 2) continue;
    auto current_lane_line = lane_line.at(i).first;

    route_segments->emplace_back();
    double future_start_s = current_lane_line.at(match_index_vec[i]).mileage() -
                            planning_conf_.look_backward_distance();
    future_start_index = match_index_vec[i];
    while (current_lane_line.at(future_start_index).mileage() > future_start_s) {
      if (future_start_index < 1) break;
      future_start_index--;
    }
    // 将look_forward_required_distance换成look_forward_extend_distance分几次，避免一次计算的长度过长，造成耗时
    //此处截取的长度还是需要和速度相关，避免速度过快的时候，生成的轨迹过短，导致换道轨迹曲率变化较大
    double look_forward_distance = LookForwardDistance(vehicle_state_.linear_velocity());
    double future_end_s =
        current_lane_line.at(match_index_vec[i]).mileage() + look_forward_distance;
    future_end_index = match_index_vec[i];
    while (current_lane_line.at(future_end_index).mileage() < future_end_s) {
      if ((unsigned int)future_end_index >= current_lane_line.size() - 1) break;
      future_end_index++;
    }
    std::vector<legionclaw::planning::LanePointInner> lane_points;
    if (!ExtendSegments(current_lane_line, future_start_index, future_end_index,
                        &lane_points)) {
      //新的参考线剩余的长度太短，舍弃这条参考线
      AWARN << "Could not further extend lane line";
      return false;
    }
    double remain_mileage = current_lane_line.back().mileage() -
                            current_lane_line.at(match_index_vec[i]).mileage();
    route_segments->back().set_start_index(future_start_index);
    route_segments->back().set_end_index(future_end_index);
    route_segments->back().set_remain_mileage(remain_mileage);
    route_segments->back().set_vehicle_close_index(match_index_vec[i]);
    route_segments->back().set_global_id(lane_line.at(i).second);
    route_segments->back().set_lane_points(lane_points);
  }
  return !route_segments->empty();
}

bool ReferenceLineProvider::UpdateSmoothLines(const interface::RoutingResponse &routing) {
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  smooth_lines_.clear();
  smooth_lines_.shrink_to_fit();
  std::vector<legionclaw::planning::LanePointInner> smooth_line;
  std::vector<std::vector<legionclaw::interface::LanePoint>> lane_list_points;
  std::vector<int> id;
  for(unsigned int i = 0; i < routing.lane_list_size(); i++){
    int global_id = routing.lane_list(i).global_id();
    int predecessor_id = 0;
    int successor_id = 0;
    int temp_pre_id = 0;
    int temp_suc_id = 0;

    std::vector<int>::iterator it = find(id.begin(),id.end(),global_id);
    if(it != id.end()){
      continue;
    }

    if(routing.lane_list(i).lane_points().size() >0){
      lane_list_points.push_back(routing.lane_list(i).lane_points());
    }
    id.push_back(global_id);
    predecessor_id = routing.lane_list(i).predecessor_id();
    successor_id = routing.lane_list(i).successor_id();
    temp_pre_id = routing.lane_list(i).global_id();
    temp_suc_id = routing.lane_list(i).global_id();

    for(unsigned int j = 0;//j < routing.lane_list_size() &&
          !(successor_id == 0 && predecessor_id ==0); j++){
      if(j > routing.lane_list_size() - 1){
       break;
      }
      global_id = routing.lane_list(j).global_id();
      std::vector<int>::iterator it = find(id.begin(),id.end(),global_id);
      if(it != id.end()){
        continue;
      }
    //先查找后面的轨迹
      if((routing.lane_list(j).global_id() == successor_id) && (routing.lane_list(j).predecessor_id() == temp_pre_id)){
        if(routing.lane_list(j).lane_points().size() >0){
          lane_list_points.back().insert(lane_list_points.back().end(), routing.lane_list(j).lane_points().begin(), routing.lane_list(j).lane_points().end());
        }
        successor_id = routing.lane_list(j).successor_id();   //更新下一个id
        temp_pre_id = routing.lane_list(j).global_id();
        id.push_back(routing.lane_list(j).global_id());
      }

      if((routing.lane_list(j).global_id() == predecessor_id) && (routing.lane_list(j).successor_id() == temp_suc_id)){
        if(routing.lane_list(j).lane_points().size() >0){
          lane_list_points.back().insert(lane_list_points.back().begin(), routing.lane_list(j).lane_points().begin(), routing.lane_list(j).lane_points().end());
        }
        predecessor_id = routing.lane_list(j).predecessor_id();   //更新上个id
        temp_suc_id = routing.lane_list(j).global_id();
        id.push_back(routing.lane_list(j).global_id());
      }
    }
  }
  double delta_mileage = 1.0;
  for (unsigned int i = 0; i < lane_list_points.size(); i++) {
    SmoothCenterLines(lane_list_points.at(i),
                      delta_mileage, smooth_line);
    smooth_lines_.push_back(smooth_line);
  }
  return true;
}

bool ReferenceLineProvider::GetPassageMatchIndex(
    const legionclaw::interface::VehicleState &vehicle_state,
    std::vector<int> &passage_match_index) {
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  std::vector<int> match_lanes_id;
  std::vector<double> match_lanes_dist;
  vector<int> match_lanes_index;
  double min_match_dis = DBL_MAX;
  double min_match_remain_s = 0.0;
  if (smooth_lines_.size() < 1) {
    AERROR << "smooth_lines_.size() < 1";
    return false;
  }
  // RoadScenes road_scenes = RoadScenes::UNKNOWN;
  for (unsigned int i = 0; i < smooth_lines_.size(); i++) {
    double match_dis = DBL_MAX;

    int match_index = MapMatcher::QueryVerticalDistanceWithBuffer(
        smooth_lines_.at(i), {vehicle_state.x(), vehicle_state.y(), vehicle_state.z()},
        vehicle_state.heading(), 1.0e-6, match_dis);
    dis_to_route_center_line_ = match_dis;
    //TODO 自车所在车道 1.距离最近 2.距离相差不大选择旧的车道 3.剩余距离过短的过滤
    if (fabs(match_dis) > planning_conf_.map_match_error() ||
        match_index < 0 || smooth_lines_.at(i).back().mileage()<13)
      continue;
    double match_index_mileage = smooth_lines_.at(i).at(match_index).mileage();
    double remanin_s = smooth_lines_.at(i).back().mileage()-match_index_mileage;
    if(remanin_s < planning_conf_.emergency_lower_limit_length())
      continue;
    //新增最近车道，延续以前的车道，多对一汇出，选择路由通的
    if ((fabs(match_dis) < min_match_dis) ||(fabs(min_match_dis - fabs(match_dis)) < 0.2)) {
      if(min_match_dis - fabs(match_dis) < 0.2 && min_match_dis < 1.0){
        //一对多、多对一场景
        //TODO 全局导航针对多对一可以重合多一点(重合多少routing会给出配置文件可配置>30m)，但是针对一对多，尽量重合的越少越好
        if (match_index_mileage > 10 && remanin_s > min_match_remain_s + 10) {
          int min_index = -1;
          auto it = std::find(match_lanes_dist.begin(), match_lanes_dist.end(),
                              min_match_dis);
          if (it != match_lanes_dist.end()) {
            min_index = std::distance(match_lanes_dist.begin(), it);
          } else {
            auto it_1 = std::find(match_lanes_dist.begin(),
                                  match_lanes_dist.end(), -min_match_dis);
            if (it_1 != match_lanes_dist.end()) {
              min_index = std::distance(match_lanes_dist.begin(), it_1);
            }
          }
          if (min_index != -1) {
            match_lanes_id.erase(match_lanes_id.begin() + min_index);
            match_lanes_index.erase(match_lanes_index.begin() + min_index);
            match_lanes_dist.erase(match_lanes_dist.begin() + min_index);
          }
          min_match_dis = fabs(match_dis);
          min_match_remain_s = remanin_s;
        } else {
          continue;
        }
      }else {
        min_match_dis = fabs(match_dis);
        min_match_remain_s = remanin_s;
      }
    }
    match_lanes_id.push_back(i);   //为smooth_line 顺序id
    match_lanes_index.push_back(match_index);
    match_lanes_dist.push_back(match_dis);
  }

  if (match_lanes_id.size() == 0) {
    return false;
  }
  //匹配上当前车道后，只取相邻三个车道的中心线smooth_lines，从左至右依次排列
  passage_smooth_lines_.clear();
  passage_smooth_lines_.shrink_to_fit();
  passage_smooth_lines_.resize(
      max_lane_num_);  // TODO:显式初始化pre_smooth_lines_中的second的值
  //初始化为 -1
  passage_smooth_lines_.at(0).second = -1;
  passage_smooth_lines_.at(1).second = -1;
  passage_smooth_lines_.at(2).second = -1;
  double left_lane_dist = 0.0;
  double right_lane_dist = 0.0;
  for (unsigned int i = 0; i < match_lanes_id.size(); i++) {
    double lane_dist = match_lanes_dist.at(i);
    if (smooth_lines_.at(match_lanes_id.at(i)).size() > 0) {
      unsigned int smooth_index_id = 1;
      if (fabs(lane_dist) == min_match_dis) {
        smooth_index_id = 1;
      } else if (lane_dist > 0 &&
                 lane_dist < planning_conf_.lane_match_tolerance()) {
        smooth_index_id = 2;
      } else if (lane_dist < 0 &&
                 lane_dist > -planning_conf_.lane_match_tolerance()) {
        smooth_index_id = 0;
      } else {
        continue;
      }
      //如果匹配到两条相同车道关系的，则取较近的那条
      if (passage_smooth_lines_.at(smooth_index_id).first.size() > 0){
        // if(fabs(lane_dist) > fabs(match_lanes_dist.at(i-1))){
        //   continue;
        // }
        if((smooth_index_id == 2 && fabs(lane_dist) > fabs(right_lane_dist)) ||
           (smooth_index_id == 0 && fabs(lane_dist) > fabs(left_lane_dist))) {
          continue;
        }
      }
      passage_smooth_lines_.at(smooth_index_id).first =
          smooth_lines_.at(match_lanes_id.at(i));
      passage_smooth_lines_.at(smooth_index_id).second = match_lanes_id.at(i);
      passage_match_index.at(smooth_index_id) = match_lanes_index.at(i);

      if(smooth_index_id == 2) {
        right_lane_dist = match_lanes_dist.at(i);
      } else if(smooth_index_id == 0) {
        left_lane_dist = match_lanes_dist.at(i);
      }
    }
  }
  return true;
}

void ReferenceLineProvider::GetEgoLaneType(
    const legionclaw::interface::VehicleState &vehicle_state,
    const legionclaw::interface::RoutingResponse &routing) {
  map_match_info_.priority[0] = -1;
  map_match_info_.priority[1] = -1;
  map_match_info_.priority[2] = -1;
  map_match_info_.ego_lane_type[0] = LaneInfoType::LANE_TYPE_UNKNOWN;
  map_match_info_.ego_lane_type[1] = LaneInfoType::LANE_TYPE_UNKNOWN;
  map_match_info_.ego_lane_type[2] = LaneInfoType::LANE_TYPE_UNKNOWN;
  if (routing.lane_list_size() < 1) {
    return;
  }
  double min_match_dis = DBL_MAX;
  int ego_lane_id = -1;
  for (unsigned int i = 0; i < routing.lane_list_size(); i++) {
    double match_dis = DBL_MAX;
    if (routing.lane_list(i).lane_points_size() > 2) {
      int match_index = MapMatcher::QueryVerticalDistanceWithBuffer(
          routing.lane_list(i).lane_points(),
          {vehicle_state.x(), vehicle_state.y(), vehicle_state.z()},
          vehicle_state.heading(), 1.0e-6, match_dis);
      if (match_index < 0 ||
          fabs(match_dis) > planning_conf_.map_match_error()) {
        continue;
      }
      if ((fabs(match_dis) < min_match_dis) ||
          (fabs(min_match_dis - fabs(match_dis)) < 0.1)) {
        if (min_match_dis - fabs(match_dis) < 0.2) {
          // 车道新增，选择既有前继又有后继关系的，
          if (routing.lane_list(i).predecessor_id() > 0 &&
              routing.lane_list(i).successor_id() > 0) {
            min_match_dis = fabs(match_dis);
            ego_lane_id = i;
            continue;
          }
          // 减少车道多对一,选择有后继的
          if (routing.lane_list(ego_lane_id).successor_id() == 0 &&
              routing.lane_list(i).successor_id() > 0) {
            min_match_dis = fabs(match_dis);
            ego_lane_id = i;
          }
        } else {
          min_match_dis = fabs(match_dis);
          ego_lane_id = i;
        }
      }
    }
  }
  if (ego_lane_id == -1) {
    return;
  }
  // 获取相邻左右车道global_id
  int8_t left_neighbor_id = routing.lane_list(ego_lane_id).left_neighbor_id();
  int8_t right_neighbor_id = routing.lane_list(ego_lane_id).right_neighbor_id();
  // 获取 global_id 对应的 lane_id
  int8_t left_lane_id = 0;
  int8_t rifht_lane_id = 0;
  for (unsigned int i = 0; i < routing.lane_list_size(); i++) {
    if (routing.lane_list(i).global_id() == left_neighbor_id) {
      left_lane_id = i + 1;
    } else if (routing.lane_list(i).global_id() == right_neighbor_id) {
      rifht_lane_id = i + 1;
    }
  }
  // 获取推荐路径
  if (left_lane_id != 0) {
    map_match_info_.priority[0] = routing.lane_list(left_lane_id - 1).priority();
    map_match_info_.ego_lane_type[0] =
        LaneInfoType(routing.lane_list(left_lane_id - 1).type());
  }
  map_match_info_.priority[1] = routing.lane_list(ego_lane_id).priority();
  map_match_info_.ego_lane_type[1] = LaneInfoType(routing.lane_list(ego_lane_id).type());
  if (rifht_lane_id != 0) {
    map_match_info_.priority[2] = routing.lane_list(rifht_lane_id - 1).priority();
    map_match_info_.ego_lane_type[2] =
        LaneInfoType(routing.lane_list(rifht_lane_id - 1).type());
  }
  // 获取绿化带停车点
  if (map_match_info_.ego_lane_type[1] == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND) {
    map_match_info_.greenbelt_occlusion_point = {
        routing.lane_list(ego_lane_id).lane_points().back().point().x(),
        routing.lane_list(ego_lane_id).lane_points().back().point().y()};
  }
  return;
}

void ReferenceLineProvider::CalculateMapMatchInfo() {
  // map_match_info_.match_index 计算，此处平滑后参考线的匹配点
  map_match_info_.current_lane_index = 1;
  map_match_info_.current_lane_id =
      passage_smooth_lines_.at(map_match_info_.current_lane_index).second;
  auto map_internal_iter = reference_lines_.begin();
  for (int i = 0; map_internal_iter != reference_lines_.end();
       ++map_internal_iter, ++i) {
    if (map_internal_iter->ReferencePointSize() > 0) {
      double dist_min = DBL_MAX;
      map_match_info_.match_index[i] = MapMatcher::QueryVerticalDistanceWithBuffer(
          map_internal_iter->ReferencePoints(),
          {vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z()},
          vehicle_state_.heading(), 1.0e-6, dist_min);
      if (fabs(dist_min) > planning_conf_.map_match_error()) {
        map_match_info_.match_index[i] = -1;
        AERROR << "Need replane due to lat_offset = " << dist_min
               << ",is too big";
      }
      if (i == map_match_info_.current_lane_index) {
        map_match_info_.lat_offset = dist_min;
        map_match_info_.lon_index = map_match_info_.match_index[i];
      }
    } else {
      map_match_info_.match_index[i] = -1;
    }
  }
}

// 根据ReferenceLine是否进行更新（new/extend/shrink）,来更新对应的route_segments_和last_reference_lines_
void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<reference_line::ReferenceLine> &reference_lines,
    const std::list<legionclaw::planning::RouteSegments> &route_segments) {
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  if (reference_lines_cache_.size() != reference_lines.size()) {
    reference_lines_cache_ = reference_lines;
    route_segments_cache_ = route_segments;
  } else {
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_cache_.begin();
    auto internal_segment_iter = route_segments_cache_.begin();
    for (auto iter = reference_lines.begin();
         iter != reference_lines.end() &&
         segment_iter != route_segments.end() &&
         internal_iter != reference_lines_cache_.end() &&
         internal_segment_iter != route_segments_cache_.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
      if (iter->ReferencePoints().empty()) {
        *internal_iter = *iter;
        *internal_segment_iter = *segment_iter;
        continue;
      }
      //如果reference_lines_某一条没有更新，则不需要更新
      if (common::util::SamePointXY(
              iter->ReferencePoints().front(),
              internal_iter->ReferencePoints().front()) &&
          common::util::SamePointXY(iter->ReferencePoints().back(),
                                    internal_iter->ReferencePoints().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
    }
  }
}

void ReferenceLineProvider::SortReferenceLines(
    std::list<reference_line::ReferenceLine> *reference_lines,
    const std::vector<std::pair<std::vector<legionclaw::planning::LanePointInner>,
                                int>> &passage_smooth_lines,
    const std::list<reference_line::ReferenceLine> &latest_reference_lines) {
#if 0
  //考虑拨杆换道请求，处于实线车道行驶时情况
  int ego_index = passage_match_index.at(map_match_info_.current_lane_index);
  if (ego_index < 0) return;
  auto ego_lane_point = passage_smooth_lines.at(map_match_info_.current_lane_index)
                            .first.at(ego_index);
  double forward_check_distance = LookForwardDistance(vehicle_state_.linear_velocity());
  double query_mileage = ego_lane_point.mileage() + forward_check_distance;
  auto predict_lane_point = MapMatcher::QueryNearestPoint(
    passage_smooth_lines.at(map_match_info_.current_lane_index).first, query_mileage);
  if (lane_change_direction_ == common::Direction::LEFT) {
    // 左侧实线驳回
    if (!((ego_lane_point.left_line_type() ==
              legionclaw::common::LaneLineType::WHITE_DASHED ||
          ego_lane_point.left_line_type() ==
              legionclaw::common::LaneLineType::YELLOW_DASHED) /*&&
              (predict_lane_point.left_line_type() ==
              legionclaw::common::LaneLineType::WHITE_DASHED ||
          predict_lane_point.left_line_type() ==
              legionclaw::common::LaneLineType::YELLOW_DASHED)*/)) {
      lane_change_state_ = Lane_Change_State::LANE_CHANGE_INVALID;
    }
    // 左侧无参考线驳回
    if (!(passage_smooth_lines.at(map_match_info_.current_lane_index -1).second > 0)) {
      lane_change_state_ = Lane_Change_State::LANE_CHANGE_INVALID;
    }
  } else if (lane_change_direction_ == common::Direction::RIGHT) {
    // 右侧实线驳回
    if (!((ego_lane_point.right_line_type() ==
              legionclaw::common::LaneLineType::WHITE_DASHED ||
          ego_lane_point.right_line_type() ==
              legionclaw::common::LaneLineType::YELLOW_DASHED) /*&&
              (predict_lane_point.right_line_type() ==
              legionclaw::common::LaneLineType::WHITE_DASHED ||
          predict_lane_point.right_line_type() ==
              legionclaw::common::LaneLineType::YELLOW_DASHED)*/)) {
      lane_change_state_ = Lane_Change_State::LANE_CHANGE_INVALID;
    }
    // 右侧无参考线驳回
    if (!(passage_smooth_lines.at(map_match_info_.current_lane_index +1).second > 0)) {
      lane_change_state_ = Lane_Change_State::LANE_CHANGE_INVALID;
    }
  }
  // TODO:判断目标拨杆车道剩余参考线长度是否够长，不够长就驳回

  //如果在未完成换道的情况下，routing进行了更新，则 lc_global_id_cache_需更新
  //TODO:routing更新时，车辆正在执行换道，且位于匹配到原来车道和换道车道的临界位置
  // if (is_new_routing_) {
  //   if (lane_change_direction_ != common::Direction::DIR_INVALID) {
  //     lc_global_id_cache_ = map_match_info_.current_lane_id;
  //   }
  // }

  int select_global_id = map_match_info_.current_lane_id;

  if (select_global_id == lc_global_id_cache_ + manual_lc_plus_) {
    lane_change_direction_ = common::Direction::DIR_INVALID;
    lane_change_state_ = Lane_Change_State::LANE_CHANGE_COMPLETE;
  }

  //拨杆换道未成功，则始终根据车当前车道去选择目标车道
  if (lane_change_state_ == Lane_Change_State::LANE_CHANGING) {
    select_global_id += manual_lc_plus_;
  }

  //根据换道模式区分：1.拨杆换道只发出一条参考线并放在当前车道
  if (lane_changing_mode_ == Lane_Changing_Mode::Manual || lane_changing_mode_ == Lane_Changing_Mode::Invalid) {
    if (select_global_id < 1) return;
    auto reference_line_iter = latest_reference_lines.begin();
    // 找到需要的参考线
    while (reference_line_iter != latest_reference_lines.end()) {
      if (select_global_id == reference_line_iter->GlobalID()) {
        reference_lines->at(map_match_info_.current_lane_index) =
            *reference_line_iter;
        break;
      }
      ++reference_line_iter;
    }
    // 需换道的车道不存在/或着还没匹配上，继续下发当前车道
    // TODO:拨杆车道在没更新route时参考线不够长，驳回
    if (reference_line_iter == latest_reference_lines.end()) {
      return;
    }
    reference_lines->front().Clear();
    reference_lines->back().Clear();
    return;
  }
#endif

  // 2.alc模式下按照固定相邻车道结构存放
  auto sort_internal_iter = reference_lines->begin();
  for (auto iter = passage_smooth_lines.begin(); iter != passage_smooth_lines.end();
       ++iter, ++sort_internal_iter) {
    if (iter->first.empty()) {
      sort_internal_iter->Clear();
      continue;
    }
    for (auto reference_line_iter = latest_reference_lines.begin();
         reference_line_iter != latest_reference_lines.end();
         ++reference_line_iter) {
      if (iter->second == reference_line_iter->GlobalID()) {
        *sort_internal_iter = *reference_line_iter;
      }
    }
  }

  // // update history
  // reference_line_history_.push(*reference_lines);
  // static constexpr int kMaxHistoryNum = 3;
  // if (reference_line_history_.size() > kMaxHistoryNum) {
  //   reference_line_history_.pop();
  // }
}

bool ReferenceLineProvider::CreateReferenceLine(
    std::list<reference_line::ReferenceLine> *reference_lines,
    std::list<legionclaw::planning::RouteSegments> *segments,
    std::vector<int>& passage_match_index) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  legionclaw::interface::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  interface::RoutingResponse routing;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing = routing_;
  }

  {
    // Update smoothlines and passage(vehicle match_index)
    if (is_new_routing_) {
      if (!UpdateSmoothLines(routing)) {
        AERROR << "Failed to update smooth_lines on routing";
        return false;
      }
    }
    GetEgoLaneType(vehicle_state,routing);
    passage_match_index.resize(max_lane_num_);
    GetPassageMatchIndex(vehicle_state, passage_match_index);
  }

  //根据车当前位置进行实时 find all neighboring passages
  if (!CreateRouteSegments(segments, passage_smooth_lines_, passage_match_index)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }

  if (is_new_routing_) {
    is_new_routing_ = false;
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      double remain_mileage = iter->remain_mileage();
      //新的segments直接平滑
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      }else {
        reference_lines->back().SetGlobalID(iter->global_id());
        reference_lines->back().set_remain_mileage(remain_mileage);
        iter->set_remain_mileage(remain_mileage);
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      double remain_mileage = iter->remain_mileage();
      iter->set_id(std::to_string(iter->global_id()));
      if (iter->global_id() == passage_smooth_lines_.at(1).second) {
        iter->set_is_on_segment(true);
      }
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        reference_lines->back().SetGlobalID(iter->global_id());
        reference_lines->back().set_remain_mileage(remain_mileage);
        iter->set_remain_mileage(remain_mileage);
        ++iter;
      }
    }
    return true;
  }
}

bool ReferenceLineProvider::ExtendReferenceLine(
    const legionclaw::interface::VehicleState &vehicle_state,
    legionclaw::planning::RouteSegments *segments,
    reference_line::ReferenceLine *reference_line) {
  // case A
  // 根据历史缓存信息，查询当前RouteSegments是否在某条(Smoothed)ReferenceLine上，如果不是就直接进行平滑参考线操作
  auto prev_segment = route_segments_cache_.begin();
  auto prev_ref = reference_lines_cache_.begin();
  // 1. 查找和segments连接的segment
  while (prev_segment != route_segments_cache_.end()) {
    if (prev_segment->global_id() == segments->global_id() &&
        prev_segment->lane_points().front().mileage() - kSegmentationEpsilon <=
            segments->lane_points().front().mileage() &&
        prev_segment->lane_points().back().mileage() + kSegmentationEpsilon >=
            segments->lane_points().front().mileage()) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  // TODO 没有相连的segments直接平滑 TODO shrink 类型转换
  if (prev_segment == route_segments_cache_.end()) {
    // AWARN << "新的segments直接平滑"<<endl;
    return SmoothRouteSegment(*segments, reference_line);
  }

  // case B
  // 如果在同一个平滑参考线(历史平滑参考线)上，计算车辆当前位置和历史平滑参考线终点的距离，如果距离超过了阈值，则可以复用这条历史参考线；否则长度不够需要拼接。
  std::pair<std::vector<legionclaw::planning::LanePointInner>, int> lane_line;
  for (unsigned int i = 0; i < passage_smooth_lines_.size(); i++) {
    if (segments->global_id() == passage_smooth_lines_.at(i).second &&
    passage_smooth_lines_.at(i).first.size() > 0) {
      lane_line = passage_smooth_lines_.at(i);
      break;
    }
  }

  double dist_min = DBL_MAX;
  int vehicle_close_index = MapMatcher::QueryNearestPointWithBuffer(
      prev_ref->ReferencePoints(),
      {vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z()},
      vehicle_state_.heading(), 1.0e-6, dist_min);
  if (vehicle_close_index < 0) {
    AWARN << "当前参考线和自车航向相反，匹配失败" << endl;
    return false;
  }

  double sl = prev_ref->ReferencePoints().at(vehicle_close_index).s();
  // double sl = lane_line.first.at(segments->vehicle_close_index()).mileage();
  const double look_forward_required_distance =
      LookForwardDistance(vehicle_state_.linear_velocity());
  const double remain_s = prev_ref->ReferencePoints().back().s() - sl;
  // const double remain_s = prev_segment->lane_points().back().mileage() - sl;
  //长度够长或者过短，不用再次截取并平滑，直接采用上一次的
  if (remain_s > look_forward_required_distance || remain_s < 1) {
    *segments = *prev_segment;
    *reference_line = *prev_ref;
    // AWARN << "长度够长直接采用上一次的reference_line"<<endl;
    return true;
  }

  // case C
  double future_start_s =
      prev_segment->lane_points().back().mileage() -
      planning_conf_.reference_line_stitch_overlap_distance();
  int future_start_index = std::min(prev_segment->end_index(),int(lane_line.first.size()-1));
  while (lane_line.first.at(future_start_index).mileage() > future_start_s) {
    if (future_start_index < 1) break;
    future_start_index--;
  }
  future_start_index =
      std::max(segments->vehicle_close_index(), future_start_index);

  if (prev_segment->end_index()+1 > (int)lane_line.first.size()) {
    AERROR << "<ExtendReferenceLine>:prev segment lane points size out of range of laneline's !";
    return false;
  }
  double future_end_s =
      lane_line.first.at(prev_segment->end_index()).mileage() +
      planning_conf_.look_forward_extend_distance();
  int future_end_index = std::min(prev_segment->end_index(),int(lane_line.first.size()-1));
  while (lane_line.first.at(future_end_index).mileage() < future_end_s) {
    if ((unsigned int)future_end_index >= lane_line.first.size() - 1) break;
    future_end_index++;
  }
  if (future_end_index == prev_segment->end_index()) {
    if(remain_s < 6){
      AWARN << "Extend lane line has been smoothed-remain_s: " <<remain_s;
      return SmoothRouteSegment(*segments, reference_line);
    }
    *segments = *prev_segment;
    *reference_line = *prev_ref;
    return true;
  }

  std::vector<legionclaw::planning::LanePointInner> shifted_segments;
  // shifted_segments.clear();
  if (!ExtendSegments(lane_line.first, future_start_index, future_end_index,
                      &shifted_segments)) {
    AERROR << "Failed to shift route segments forward";
    //如果扩展失败直接进行平滑操作,返回上一帧
    *segments = *prev_segment;
    *reference_line = *prev_ref;
    return true;
  }
  //扩展操作成功，但是扩招以后长度没有太大变化，死路，直接使用历史平滑参考线
  double extend_dis =
      shifted_segments.back().mileage() - shifted_segments.front().mileage();
  if (extend_dis < planning_conf_.anchor_sampling_step() + 0.5) {
    AWARN << "Could not further extend reference line";
    *segments = *prev_segment;
    *reference_line = *prev_ref;
    return true;
  }

  // case D
  // 扩展成功，并且额外增加了一定长度,得到了新的Path,对新的路径进行平滑然后与历史平滑参考线进行拼接，就可以得到一条更长的平滑参考线
  reference_line::ReferenceLine new_ref(shifted_segments);
  new_ref.ComputeAllMileage(0.0);
  reference_line->SetGlobalID(lane_line.second);
  int match_index = -1;
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line,
                                   match_index)) {
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }

  if (!reference_line->Stitch(*prev_ref, match_index)) {
    AWARN << "Failed to stitch reference line";
    auto min_forward_distance = max(vehicle_state_.linear_velocity() * planning_conf_.look_forward_time_sec(),30.0);
    if (remain_s > min_forward_distance) {
      *segments = *prev_segment;
      *reference_line = *prev_ref;
      return true;
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  reference_line->ComputeAllMileage(0);
  if (!ExtendSegments(lane_line.first, prev_segment->start_index(),
                      future_end_index, &shifted_segments)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  segments->set_start_index(prev_segment->start_index());
  segments->set_end_index(future_end_index);
  segments->set_lane_points(shifted_segments);
  return Shrink(vehicle_state, reference_line);
}

bool ReferenceLineProvider::Shrink(
    const legionclaw::interface::VehicleState &vehicle_state,
    reference_line::ReferenceLine *reference_line) {
  static constexpr double kMaxHeadingDiff = M_PI * 5.0 / 6.0;
  // shrink reference line
  double dist_min = DBL_MAX;
  int close_index = MapMatcher::QueryNearestPointWithBuffer(
      reference_line->ReferencePoints(),
      {vehicle_state.x(), vehicle_state.y(), vehicle_state.z()},
      vehicle_state.heading(), 1.0e-6, dist_min);

  double sl_s = reference_line->ReferencePoints(close_index).s();
  double new_backward_distance = sl_s;
  // double new_forward_distance = reference_line->Length() - sl_s;
  bool need_shrink = false;
  if (sl_s > planning_conf_.look_backward_distance() * 1.5) {
    // AWARN << "reference line current s is " << sl_s
    //        << ", shrink reference line: origin length: "
    //        << reference_line->Length();
    new_backward_distance = planning_conf_.look_backward_distance();
    need_shrink = true;
  }
  // check heading
  const auto &ref_points = reference_line->ReferencePoints();
  // const auto &lane_points = segments->lane_points();
  const double cur_heading = ref_points[close_index].theta();
  auto last_index = close_index;
  while ((unsigned int)last_index < ref_points.size() &&
         AngleDiff(cur_heading, ref_points[last_index].theta()) <
             kMaxHeadingDiff) {
    ++last_index;
  }
  --last_index;
  if ((unsigned int)last_index != ref_points.size() - 1) {
    need_shrink = true;
    // new_forward_distance = reference_line->ReferencePoints(last_index).s() - sl_s;
  }

  if (need_shrink) {
    int start_index = 0;
    if (sl_s - new_backward_distance > 0) {
      double backward_shrink_distance = sl_s - new_backward_distance;
      // shrink reference_line
      while (ref_points[start_index].s() < backward_shrink_distance) {
        start_index++;
      }
      if (!reference_line->Segment(start_index, last_index)) {
        AWARN << "Failed to shrink reference line";
      }
      reference_line->ComputeAllMileage(0);
    }
  }
  return true;
}

void ReferenceLineProvider::GetAnchorPoints(
    const reference_line::ReferenceLine &reference_line,
    std::vector<ReferencePoint> *anchor_points) const {
  // CHECK_NOTNULL(anchor_points);
  if (reference_line.reference_points_.size() <= 0) {
    return;
  }
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> headings;
  std::vector<double> RWs;
  std::vector<double> LWs;
  std::vector<legionclaw::common::LaneLineType> RWTypes;
  std::vector<legionclaw::common::LaneLineType> LWTypes;
  std::vector<double> limit_speed;
  // std::vector<double> RevWs;
  s.push_back(reference_line.reference_points_[0].s());
  x.push_back(reference_line.reference_points_[0].x());
  y.push_back(reference_line.reference_points_[0].y());
  z.push_back(reference_line.reference_points_[0].z());
  headings.push_back(reference_line.reference_points_[0].theta());
  RWs.push_back(reference_line.reference_points_[0].right_road_width());
  LWs.push_back(reference_line.reference_points_[0].left_road_width());
  RWTypes.push_back(reference_line.reference_points_[0].right_line_type());
  LWTypes.push_back(reference_line.reference_points_[0].left_line_type());
  limit_speed.push_back(reference_line.reference_points_[0].limit_speed());

  const double interval = planning_conf_.anchor_sampling_step();
  const double lateral_bound_limit = planning_conf_.lateral_bound_limit();
  for (size_t i = 1; i < reference_line.reference_points_.size(); i++) {
    ///******GetAnchorPoints : modify the way to get the anchor points
    if (reference_line.reference_points_[i].s() >= s.back() + interval)
    //    if ( delta_l > threshold_length || delta_h  > threshold_heading )
    {
      s.push_back(reference_line.reference_points_[i].s());
      x.push_back(reference_line.reference_points_[i].x());
      y.push_back(reference_line.reference_points_[i].y());
      z.push_back(reference_line.reference_points_[i].z());
      headings.push_back(reference_line.reference_points_[i].theta());
      RWs.push_back(reference_line.reference_points_[i].right_road_width());
      LWs.push_back(reference_line.reference_points_[i].left_road_width());
      RWTypes.push_back(reference_line.reference_points_[i].right_line_type());
      LWTypes.push_back(reference_line.reference_points_[i].left_line_type());
      limit_speed.push_back(reference_line.reference_points_[i].limit_speed());
      // RevWs.push_back(reference_line.reference_points_[i].reverseLaneWidth);
    }
  }

  ReferencePoint reference_point;
  for (unsigned int j = 0; j < s.size(); j++) {
    reference_point.set_s(s[j]);  // - s[0];
    reference_point.set_x(x[j]);
    reference_point.set_y(y[j]);
    reference_point.set_z(z[j]);
    reference_point.set_theta(headings[j]);
    reference_point.set_longitudinal_bound(2.0);
    reference_point.set_lateral_bound(lateral_bound_limit);
    reference_point.set_enforced(false);
    reference_point.set_right_road_width(RWs[j]);
    reference_point.set_left_road_width(LWs[j]);
    reference_point.set_right_line_type(RWTypes[j]);
    reference_point.set_left_line_type(LWTypes[j]);
    reference_point.set_limit_speed(limit_speed[j]);
    // reference_point.reverseLaneWidth = RevWs[j];
    anchor_points->emplace_back(reference_point);
  }

  anchor_points->front().set_longitudinal_bound(1e-6);
  anchor_points->front().set_lateral_bound(1e-6);
  anchor_points->front().set_enforced(true);
  anchor_points->back().set_longitudinal_bound(1e-6);
  anchor_points->back().set_lateral_bound(1e-6);
  anchor_points->back().set_enforced(true);
}

bool ReferenceLineProvider::SmoothCenterLines(
    std::vector<legionclaw::interface::LanePoint> center_line, double delta_mileage,
    std::vector<legionclaw::planning::LanePointInner> &smooth_line) {
  //里程计算
  int start_pos = 0;
  int end_pos = center_line.size();

  if (end_pos <= 0) return false;

  double x_line = center_line[start_pos].point().x();
  double y_line = center_line[start_pos].point().y();
  double xx, yy, ss;

  double s_line = 0.0;
  center_line[start_pos].set_mileage(s_line);
  //#pragma omp parallel for
  for (int i = start_pos + 1; i < end_pos; i++) {
    xx = center_line[i].point().x();
    yy = center_line[i].point().y();

    ss = sqrt((xx - x_line) * (xx - x_line) + (yy - y_line) * (yy - y_line));
    s_line += ss;

    center_line[i].set_mileage(s_line);

    x_line = xx;
    y_line = yy;
  }

  //平滑并内插
  std::vector<double> x, y, s;
  x.clear();
  y.clear();
  s.clear();
  legionclaw::planning::spline::spline spline_s_x, spline_s_y;

  double last_mileage = center_line[start_pos].mileage();
  double last_theta = center_line[start_pos].theta();
  for (size_t i = 0; i < center_line.size(); i++) {
    //对于中心线相邻之间两点重合的点进行过滤，避免进行平滑处理时，求解失败
    //过滤相邻两点之间航向相差很大的点(45度)
    if(i > 0){
      double diff_theta = fabs(AngleDiff(center_line[i].theta(), center_line[i - 1].theta()));
      if (center_line[i].mileage() - center_line[i - 1].mileage() <= 0 || diff_theta > 0.79)
        continue;
    }
    if(i == 0 || center_line[i].mileage() - last_mileage > 5.0
    || fabs(AngleDiff(center_line[i].theta(),last_theta)) > math::D2R(20.0))
    {
      last_mileage = center_line[i].mileage();
      last_theta = center_line[i].theta();

      s.push_back(center_line[i].mileage());
      x.push_back(center_line[i].point().x());
      y.push_back(center_line[i].point().y());
    }
  }

  if (s.size() <= 2) return false;

  spline_s_x.set_points(s, x);
  spline_s_y.set_points(s, y);
  smooth_line.clear();
  // legionclaw::planning::LanePointInner smooth_point;
  // legionclaw::interface::Point3D point_3d;

  double heading = 0;
  double x_s, y_s;
  double first_deriv_x, second_deriv_x, first_deriv_y, second_deriv_y;
  double cs = s[0];
  legionclaw::planning::LanePointInner smooth_point;
  smooth_point.set_point(center_line[0].point());
  smooth_point.set_limit_speed(center_line[0].limit_speed());
  smooth_point.set_right_road_width(center_line[0].right_road_width());
  smooth_point.set_left_road_width(center_line[0].left_road_width());
  smooth_point.set_right_line_type(center_line[0].right_line_type());
  smooth_point.set_left_line_type(center_line[0].left_line_type());

  // legionclaw::interface::LanePoint smooth_point = center_line[0];
  legionclaw::interface::Point3D point_3d;
  double ks = DBL_MIN;

  for (size_t i = 1; i < center_line.size();) {
    spline_s_x.compute_spline(cs, x_s, first_deriv_x,
                              second_deriv_x);  // x(s)一阶段,二阶段
    spline_s_y.compute_spline(cs, y_s, first_deriv_y,
                              second_deriv_y);  // y(s)一阶段,二阶段
    heading = GetHeadingRadian(first_deriv_x, first_deriv_y);
    // ks = compute_ks_from_spline(first_deriv_x, second_deriv_x, first_deriv_y,
    // second_deriv_y);
    ks = CurveMath::ComputeCurvature(first_deriv_x, second_deriv_x,
                                     first_deriv_y, second_deriv_y);
    smooth_point.set_mileage(cs);
    smooth_point.set_theta(NormalizeAngle(heading));
    smooth_point.set_kappa(ks);
    point_3d.set_x(x_s);
    point_3d.set_y(y_s);
    if (!planning_conf_.enable_elevation()) {
      point_3d.set_z(0.0);
    } else {
      point_3d.set_z(center_line[i].point().z());
    }

    smooth_point.set_point(point_3d);
    smooth_line.push_back(smooth_point);

    if (center_line[i].mileage() - center_line[i - 1].mileage() <=
        delta_mileage) {
      // smooth_point = center_line[i];
      smooth_point.set_limit_speed(center_line[i].limit_speed());
      smooth_point.set_right_road_width(center_line[i].right_road_width());
      smooth_point.set_left_road_width(center_line[i].left_road_width());
      smooth_point.set_right_line_type(center_line[i].right_line_type());
      smooth_point.set_left_line_type(center_line[i].left_line_type());
      cs = center_line[i].mileage();
      i++;
    } else {
      // smooth_point = center_line[i - 1];  //
      // smooth_point采用上一次的值为初始值
      cs += delta_mileage;
      if (cs >= center_line[i].mileage()) {
        // smooth_point = center_line[i];
        smooth_point.set_limit_speed(center_line[i].limit_speed());
        smooth_point.set_right_road_width(center_line[i].right_road_width());
        smooth_point.set_left_road_width(center_line[i].left_road_width());
        smooth_point.set_right_line_type(center_line[i].right_line_type());
        smooth_point.set_left_line_type(center_line[i].left_line_type());
        cs = center_line[i].mileage();
        i++;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace legionclaw
