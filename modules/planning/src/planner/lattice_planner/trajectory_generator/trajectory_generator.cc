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
 * @brief This file provides the implementation of the class
 * "TrajectoryGenerator".
 */

#include "trajectory_generator.h"

#include <math.h>

#include <memory>
#include <utility>

/**
 * @namespace legionclaw::planning
 * @brief legionclaw::planning
 */
namespace legionclaw {
namespace planning {

// using legionclaw::common::ErrorCode;
// using legionclaw::common::SLPoint;
// using legionclaw::common::SpeedPoint;
// using legionclaw::common::Status;
// using legionclaw::common::TrajectoryPoint;
// using legionclaw::common::math::Vec2d;
// using legionclaw::common::util::PointFactory;
// using legionclaw::cyber::Clock;

// namespace {
// constexpr uint32_t KDestLanePriority = 0;
// constexpr double kPathOptimizationFallbackClost = 2e4;
// constexpr double kSpeedOptimizationFallbackClost = 2e4;
// constexpr double kStraightForwardLineCost = 10.0;
// }  // namespace
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;

void TrajectoryGenerator::Init(const legionclaw::planning::PlanningConf *planning_conf)
{
  if (planning_conf == nullptr) return;
  vehicle_param_ = planning_conf->vehicle_param();
  planning_conf_ = planning_conf;
  trajectory_generator_conf_ = planning_conf->lattice_planner_conf().trajectory_generator_conf();
  end_offset_ = 0.0;
  AINFO << "TrajectoryGenerator Init is OK ." << std::endl;
}

Status TrajectoryGenerator::GenerateTrajectories(bool &is_need_replan, Frame &frame,
                                                 const double &path_density)
{
  // 截取并平滑后的地图中心线数据、本车pose与地图的匹配信息判断是否有效
  if (frame.ReferenceLines().size() == 0)
    return Status(Status::ErrorCode::PLANNING_ERROR, "ReferenceLines is Null.");
  if (frame.MapMatchInfo().current_lane_id < 0 || frame.MapMatchInfo().lon_index < 0)
    return Status(Status::ErrorCode::PLANNING_ERROR, "MapMatchInfo error.");

  frame_ = frame;
  trajectory_array_.clear();
  trajectory_array_.shrink_to_fit();
  int valid = 0;

  for (unsigned int i = 0; i < frame_.ReferenceLines().size(); ++i) {
    std::vector<PlanningTrajectory> trajectories;
    trajectories.clear();
    // 中心线的长度不能小于1m
    if (frame_.ReferenceLines().at(i).ReferencePointSize() > (int)(1.0 / path_density)) {
      GenerateTrajectoriesPerLane(frame_.ReferenceLines().at(i), is_need_replan, i, path_density,
                                  trajectories);
      if (trajectories.size() > 0) valid++;
    }

    trajectory_array_.push_back(trajectories);
  }

  if (valid > 0)
    return Status(Status::ErrorCode::OK);
  else {
    AERROR << "No trajectories have been generated.";
    return Status(Status::ErrorCode::PLANNING_ERROR, "No trajectories have been generated.");
  }
}

void TrajectoryGenerator::GenerateTrajectoriesPerLane(const ReferenceLine &reference_line,
                                                      bool &is_need_replan, const int &global_index,
                                                      const double &path_density,
                                                      std::vector<PlanningTrajectory> &trajectories)
{
  // 1.  Compute stitching path from previous planning trajectory("/final_ways")
  double stitching_length =
      trajectory_generator_conf_.stitching_path_base_length() +
      trajectory_generator_conf_.stitching_path_time() * fabs(frame_.VehicleState().speed);
  std::vector<PathPoint> stitching_path =
      ComputeStitchingPath(stitching_length, is_need_replan, global_index);

  // 2.  Generate paths based on a reference line
  // std::vector<std::vector<PathPoint>> generated_paths;
  std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> generated_paths;
  double safe_margin = 0.5 * vehicle_param_.width() + trajectory_generator_conf_.out_side_margin();
  // std::cout << "stitching_path.x : " << *stitching_path.back().mutable_x()
  //           << " stitching_path.y : " << *stitching_path.back().mutable_y()
  //           << " stitching_path.back.s :" << *stitching_path.back().mutable_s()
  //           << std::endl;
  end_offset_ = 0.0;
  if (global_index == 1) {
    end_offset_ = frame_.yield_distance();
  } else if (global_index == 0 && planning_conf_->enable_local_map_topic()) {
    end_offset_ = -trajectory_generator_conf_.traj_lc_offset();
  } else if (global_index == 2 && planning_conf_->enable_local_map_topic()) {
    end_offset_ = trajectory_generator_conf_.traj_lc_offset();
  }
  PlanOnReferenceLine(
      stitching_path.back(), reference_line, frame_.VehicleState().speed, safe_margin, path_density,
      trajectory_generator_conf_.trajectory_bundles_density(),
      trajectory_generator_conf_.traj_sampling_min_lon_step(),
      trajectory_generator_conf_.traj_sampling_base_length(),
      trajectory_generator_conf_.traj_sampling_min_time(),
      trajectory_generator_conf_.traj_sampling_max_time(), global_index, generated_paths);

  // 轨迹长度限制
  double trajectory_length =
      (frame_.VehicleState().speed * trajectory_generator_conf_.traj_gen_time());
  trajectory_length = (trajectory_length > trajectory_generator_conf_.traj_max_length()
                           ? trajectory_generator_conf_.traj_max_length()
                           : trajectory_length);
  trajectory_length = (trajectory_length < trajectory_generator_conf_.traj_min_length()
                           ? trajectory_generator_conf_.traj_min_length()
                           : trajectory_length);

  // 3. Compute extend paths and put out rollInPaths
  std::vector<PathPoint> combined_path;
  for (auto generated_path : generated_paths) {
    // compute extend paths
    ReferencePoint reference_matched_pos;
    bool is_narrow = false;
    std::vector<PathPoint> extend_path =
        ComputeExtendPath(generated_path.first.back(), reference_line.ReferencePoints(),
                          reference_matched_pos, trajectory_length, safe_margin, is_narrow);

    // Combine stitching, plan and extend paths
    combined_path.clear();
    if (stitching_path.size() > 0)
      combined_path.insert(combined_path.end(), stitching_path.begin(), stitching_path.end());
    if (generated_path.first.size() > 0)
      combined_path.insert(combined_path.end(), generated_path.first.begin() + 1,
                           generated_path.first.end());
    if (extend_path.size() > 0)
      combined_path.insert(combined_path.end(), extend_path.begin() + 1, extend_path.end());

    // 将path加上纵向信息，转成trajectory
    PlanningTrajectory combined_trajectory;
    MapMatcher::PredictConstantTimeCostForTrajectory(
        combined_path, reference_matched_pos.limit_speed(), 0.1, vehicle_param_.speed_limit(),
        combined_trajectory);
    if (is_narrow) {
      combined_trajectory.set_name("The road is too narrow, please slow down");
    }
    combined_trajectory.set_global_index(global_index);
    combined_trajectory.set_lon_id(generated_path.second.back());
    combined_trajectory.set_lat_id(generated_path.second.front());

    trajectories.push_back(combined_trajectory);
  }
}

std::vector<PathPoint> TrajectoryGenerator::ComputeStitchingPath(const double &stitching_length,
                                                                 bool &is_need_replan,
                                                                 const int &global_index)
{
  std::vector<PathPoint> stitching_path;
  is_need_replan = false;
  // 上一帧没有轨迹则进行本地重规划
  if (frame_.LastPlanningTrajectorySize() == 0) {
    is_need_replan = true;
  }

  // 针对自车本车道内换道撤回重规划
  if (global_index == 1) {
    if (frame_.LastBehaviour().lat_state ==
        legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE) {
      // 计算侵占程度
      auto reference_point = frame_.ReferenceLine(frame_.MapMatchInfo().current_lane_index)
                                 .ReferencePoints(frame_.MapMatchInfo().lon_index);
      double angle_diff =
          NormalizeAngle(frame_.VehicleState().pose.theta() - reference_point.theta());
      double lane_occupancy = frame_.MapMatchInfo().lat_offset +
                              vehicle_param_.front_edge_to_center() * sin(angle_diff) +
                              0.5 * vehicle_param_.width() * cos(angle_diff);
      if (lane_occupancy < reference_point.left_road_width() - 0.1 && lane_occupancy > 0.1) {
        is_need_replan = true;
      }
    } else if (frame_.LastBehaviour().lat_state ==
               legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
      // 计算侵占程度
      auto reference_point = frame_.ReferenceLine(frame_.MapMatchInfo().current_lane_index)
                                 .ReferencePoints(frame_.MapMatchInfo().lon_index);
      double angle_diff =
          NormalizeAngle(reference_point.theta() - frame_.VehicleState().pose.theta());
      double lane_occupancy = -frame_.MapMatchInfo().lat_offset +
                              vehicle_param_.front_edge_to_center() * sin(angle_diff) +
                              0.5 * vehicle_param_.width() * cos(angle_diff);
      if (lane_occupancy < reference_point.right_road_width() - 0.1 && lane_occupancy > 0.1) {
        is_need_replan = true;
      }
    }
  }

  // 不需要重规划，基于上一次规划轨迹生成新的轨迹
  if (!is_need_replan) {
    // 左+右-当前位置与上一次规划轨迹匹配点两点直线距离
    double matched_distance = frame_.TrajectoryMatchInfo().lat_offset;
    // 上一次规划轨迹上的点距离当前位置最短的点的序号
    int position_matched_index = frame_.TrajectoryMatchInfo().lon_index;
    // std::cout <<"===========position_matched_index: " <<position_matched_index <<endl;
    // 从匹配点往前截取上次轨迹stitching length长度
    if (fabs(matched_distance) < trajectory_generator_conf_.trajectory_match_error() &&
        position_matched_index >= 0) {
      double dlength = 0.0;
      for (size_t i = position_matched_index; i < frame_.LastPlanningTrajectorySize(); ++i) {
        dlength = frame_.LastTrajectoryPoints(i).path_point().s() -
                  frame_.LastTrajectoryPoints(position_matched_index).path_point().s();
        if (dlength <= stitching_length)
          stitching_path.push_back(frame_.LastTrajectoryPoints(i).path_point());
        else
          break;
      }
    } else  // 当前位置匹配上一次规划轨迹失败，重规划
    {
      is_need_replan = true;
    }
  }

  // 重规划，根据本车pose与地图的匹配信息
  if (is_need_replan) {
    // double dist_min;
    // 地图参考线上的点距离当前位置最短的点的序号
    // map_match_info_.current_lane_index = 1,为当前参考线
    int position_matched_index = frame_.MapMatchInfo().lon_index;
    // 如果本车pose无法匹配上地图中心线，stitching_path存储当前位置点
    if (position_matched_index < 0) {
      // 曲率赋值为0，则表示起步方向盘回正
      PathPoint start_point = frame_.VehicleState().pose;
      start_point.set_kappa(0.0);  //(1.0/(1.0/rkappa+dist_min)));
      stitching_path.push_back(start_point);
    } else  // 取车辆位置信息和匹配点的航向信息作为stitching_path
    {
      PathPoint start_point = frame_.VehicleState().pose;
      // start_point.set_kappa(0.0);  //(1.0/(1.0/rkappa+dist_min)));
      // 航向信息给匹配点的航向，避免由于车辆当前姿态很歪导致生成的轨迹不合理
      auto current_referenceline = frame_.ReferenceLine(frame_.MapMatchInfo().current_lane_index);
      if (!current_referenceline.ReferencePoints().empty()) {
        auto reference_point = current_referenceline.ReferencePoints(position_matched_index);
        // //判断参考线是否在自车较远处，若较远，则以参考线起点为规划起点
        // if (position_matched_index == 0) {
        //   math::Vec2d curr_point(start_point.x(), start_point.y());
        //   double dist_sqr = curr_point.DistanceSquareTo(
        //       {reference_point.x(), reference_point.y()});
        //   if (dist_sqr > 1.0) {
        //     start_point.set_x(reference_point.x());
        //     start_point.set_y(reference_point.y());
        //     start_point.set_z(reference_point.z());
        //   }
        // }
        start_point.set_theta(reference_point.theta());
        start_point.set_kappa(reference_point.kappa());
        start_point.set_dkappa(reference_point.dkappa());
      }
      stitching_path.push_back(start_point);
    }
  }

  return stitching_path;
}

void TrajectoryGenerator::ComputeInitFrenetState(const ReferencePoint &matched_point,
                                                 const PathPoint &cartesian_state,
                                                 std::array<double, 3> *ptr_s,
                                                 std::array<double, 3> *ptr_d)
{
  math::CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(), matched_point.theta(),
      matched_point.kappa(), matched_point.dkappa(), cartesian_state.x(), cartesian_state.y(), 0.0,
      0.0,  // cartesian_state.v, cartesian_state.acc,
      cartesian_state.theta(), cartesian_state.kappa(), ptr_s, ptr_d);
}

bool TrajectoryGenerator::GeneratePaths(
    const Trajectory1DBundle &lat_trajectory_bundle,
    const std::vector<ReferencePoint> &reference_points, const double &planning_length,
    const double &path_density, const double &start_mileage, const double &init_s,
    const int &lon_id, const std::vector<int> &lat_ids,
    std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> &generated_paths)
{
  if (lat_ids.size() != lat_trajectory_bundle.size()) return false;

  generated_paths.clear();
  PathPoint generated_path_point;
  std::pair<std::vector<PathPoint>, std::array<int, 2>> generated_path;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double v = 0.0;
  double a = 0.0;
  double relative_s = 0.0;
  int lat_index = 0;
  for (auto lat_trajectory : lat_trajectory_bundle) {
    relative_s = 0.0;
    generated_path.first.clear();
    for (; relative_s <= planning_length; relative_s += path_density) {
      if (init_s + relative_s > reference_points.back().s()) {
        break;
      }
      // linear extrapolation is handled internally in LatticeTrajectory1d;
      // no worry about s_param > lat_trajectory.ParamLength() situation
      double d = lat_trajectory->Evaluate(0, relative_s);
      double d_prime = lat_trajectory->Evaluate(1, relative_s);
      double d_pprime = lat_trajectory->Evaluate(2, relative_s);

      ReferencePoint matched_ref_point =
          MapMatcher::MatchToPath(reference_points, init_s + relative_s);

      // TODO 关于道路与中心线的偏移量，后续再进行优化
      // 判断生成的轨迹是否会和实线有交叉点，根据左右路宽进行判断，d <
      // 0,表示在当前参考线的右边，否则在左边 轨迹和右边道路线交叉点是否为虚线
      if (d < 0 && fabs(d + matched_ref_point.right_road_width()) <= path_density &&
          !(matched_ref_point.right_line_type() == legionclaw::common::LaneLineType::WHITE_DASHED ||
            matched_ref_point.right_line_type() == legionclaw::common::LaneLineType::YELLOW_DASHED)) {
        AWARN << "轨迹和右边道路线交叉点为实线，无法生成轨迹.";
        generated_path.first.clear();
        break;
      }
      // 轨迹和左边道路线交叉点是否为虚线
      if (d > 0 && fabs(d - matched_ref_point.left_road_width()) <= path_density &&
          !(matched_ref_point.left_line_type() == legionclaw::common::LaneLineType::WHITE_DASHED ||
            matched_ref_point.left_line_type() == legionclaw::common::LaneLineType::YELLOW_DASHED)) {
        AWARN << "轨迹和左边道路线交叉点为实线，无法生成轨迹.";
        generated_path.first.clear();
        break;
      }

      const double rs = matched_ref_point.s();
      const double rx = matched_ref_point.x();
      const double ry = matched_ref_point.y();
      const double rtheta = matched_ref_point.theta();
      const double rkappa = matched_ref_point.kappa();
      const double rdkappa = matched_ref_point.dkappa();

      std::array<double, 3> s_conditions = {rs, 0.0, 0.0};
      std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
      math::CartesianFrenetConverter::frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa,
                                                          s_conditions, d_conditions, &x, &y,
                                                          &theta, &kappa, &v, &a);

      generated_path_point.set_x(x);
      generated_path_point.set_y(y);
      generated_path_point.set_z(matched_ref_point.z());
      generated_path_point.set_theta(theta);
      generated_path_point.set_kappa(kappa);
      generated_path_point.set_s(relative_s + start_mileage);
      // generated_path_point.set_lat_offset(d_conditions[0]);
      // generated_path_point.v = v;
      // generated_path_point.acc = a;
      // generated_path_point.gid = gid;
      // generated_path_point.set_lon_id(lon_id);
      // generated_path_point.set_lat_id(lat_ids[lat_index]);
      // generated_path.push_back(generated_path_point);
      generated_path.first.push_back(generated_path_point);
    }
    if (generated_path.first.size() > 1) {
      generated_path.second = {lat_ids[lat_index], lon_id};
      generated_paths.push_back(generated_path);
    }
    lat_index++;
  }

  if (generated_paths.size() < 1) {
    return false;
  }
  return true;
}

bool TrajectoryGenerator::GetSLBoundary(const std::vector<ReferencePoint> &reference_points,
                                        const std::vector<common::math::Vec2d> &corners,
                                        SLBoundary *const sl_boundary) const
{
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto &point : corners) {
    double min_vertical_distance = DBL_MAX;
    int index = MapMatcher::QueryVerticalDistanceWithBuffer(
        reference_points, point, frame_.VehicleState().pose.theta(), 1.0e-6, min_vertical_distance);
    if (index == -1) {
      AERROR << "Failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    SLPoint sl_point;
    sl_point.set_s(reference_points.at(index).s());
    sl_point.set_l(min_vertical_distance);
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto &p0 = corners[index0];
    const auto &p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    double min_vertical_distance = DBL_MAX;
    int index = MapMatcher::QueryVerticalDistanceWithBuffer(
        reference_points, p_mid, frame_.VehicleState().pose.theta(), 1.0e-6, min_vertical_distance);
    if (index == -1) {
      AERROR << "Failed to get projection for point: " << p_mid.DebugString()
             << " on reference line.";
      return false;
    }
    SLPoint sl_point_mid;
    sl_point_mid.set_s(reference_points.at(index).s());
    sl_point_mid.set_l(min_vertical_distance);

    Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
             sl_corners[index1].l() - sl_corners[index0].l());

    Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(), sl_point_mid.l() - sl_corners[index0].l());

    // *sl_boundary->add_boundary_point(sl_corners[index0]);
    sl_boundary->add_boundary_point(sl_corners[index0]);

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0) {
      // *sl_boundary->add_boundary_point(sl_point_mid);
      sl_boundary->add_boundary_point(sl_point_mid);
    }
  }

  for (const auto &sl_point : sl_boundary->boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());  // 最小的最右边
    end_l = std::fmax(end_l, sl_point.l());      // 最大的最左边
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

int TrajectoryGenerator::PlanOnReferenceLine(
    const PathPoint &init_point, const ReferenceLine &reference_line, const double &speed,
    const double &safe_margin, const double &path_density, const double &trajectory_bundles_density,
    const double &min_lon_step, const double &base_length, const double &min_time,
    const double &max_time, const int &global_index,
    std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> &generated_paths)

{
  generated_paths.clear();
  // int gid = reference_line->GlobalID();

  // 1. obtain a reference points.
  std::vector<ReferencePoint> reference_points = reference_line.ReferencePoints();

  // 2. compute the matched point of the init planning point on the reference
  // line. 将stitching_path.back()最后一个点作为
  // init_point，获取离该点最近的点的序号matched_point_index，
  int matched_point_index = 0;
  // dist_min为init_point 到 matched_point_index和后一个点构成的直线的最短距离
  ReferencePoint matched_ref_point =
      MapMatcher::MatchToPath(reference_points, {init_point.x(), init_point.y(), init_point.z()},
                              init_point.theta(), matched_point_index);
  // // 获取车辆当前位置在参考线上的匹配点
  // int vehicle_matched_point_index = 0;
  // ReferencePoint vehicle_matched_ref_point =
  //     MapMatcher::MatchToPath(reference_points,
  //                             {frame_.VehicleState().pose.x(), frame_.VehicleState().pose.y(),
  //                              frame_.VehicleState().pose.z()},
  //                             frame_.VehicleState().pose.theta(), vehicle_matched_point_index);

  // 3. according to the matched point, compute the init state in Frenet frame_.
  // 此处与autoware的相比，差异处：autoware里会取距离init_point最近的点前后两个点之间进行插值得到匹配点，此处直接取距离最近的点为匹配点
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  // cartesian_to_frenet 获得起始点init_s、init_d，其中 init_s[0] =  matched_point.s()
  //  init_point.set_theta(matched_ref_point.theta());
  ComputeInitFrenetState(matched_ref_point, init_point, &init_s, &init_d);

  if (init_s[0] < -1.0 && !planning_conf_->enable_local_map_topic()) {
    return 0;
  } else if (init_s[0] < -planning_conf_->virtual_length_max()) {
    return 0;
  }
  // std::cout << "------- theta1 = "
  //           << R2D(reference_points.at(matched_point_index).theta() -
  //                  init_point.theta())
  //           << "  theta2 = " << R2D(matched_point.theta() - init_point.theta())
  //           << std::endl;
  // std::cout << "*********init_d: " << init_d.at(0)
  //           << "  init_d-1: " << init_d.at(1) << "  init_d-2: " << init_d.at(2)
  //           << std::endl;
  // 4.  generate 1d trajectory bundle for lateral respectively.
  std::vector<Condition> target_conditions;
  std::vector<std::vector<Condition>> target_conditions_matrix;
  // int centralTrajectoryIndex = rollOutNumber/2;
  double remain_length = reference_points.back().s() - init_s[0];
  if (remain_length <= 2) {
    AERROR << "remain_length is error ";
    return 0;
  }

  double lon_step = std::max(min_lon_step, speed);
  double start_margin = base_length + min_time * lon_step;  // 后边界
  double end_margin = base_length + max_time * lon_step;
  // 如果剩下的参考线不够长，只生成一排轨迹（以参考线最后一个点，生成一排轨迹）
  if (start_margin >= remain_length) {
    start_margin = remain_length;
    end_margin = remain_length;
  } else if (end_margin > remain_length) {
    end_margin = remain_length;
  }
  // TODO终点选取待优化，终点选取可考虑障碍物
  // 通过里程信息获取终点
  int matched_end_point_index =
      get_matched_point_index(reference_points, matched_point_index, init_s[0] + end_margin);

  // 对选取的终点进行校验
  int min_index = matched_point_index + base_length / path_density;
  while (matched_end_point_index > min_index) {
    // 若前方道路选取的终点太窄无法生成轨迹，往前搜索合适的终点，
    if (reference_points.at(matched_end_point_index).right_road_width() < safe_margin ||
        reference_points.at(matched_end_point_index).left_road_width() < safe_margin) {
      matched_end_point_index--;
    } else {
      break;
    }
  }

  ReferencePoint matched_end_point = reference_points.at(matched_end_point_index);
  double left_road_width = matched_end_point.left_road_width();
  double right_road_width = matched_end_point.right_road_width();
  double left_margin = left_road_width;     // left_road_width - safe_margin;
  double right_margin = -right_road_width;  //-right_road_width + safe_margin;
  // left_margin +
  // if (matched_end_point.left_line_type() ==
  //         legionclaw::common::LaneLineType::CURB ||
  //     matched_end_point.left_line_type() ==
  //         legionclaw::common::LaneLineType::WHITE_SOLID ||
  //     matched_end_point.left_line_type() ==
  //         legionclaw::common::LaneLineType::YELLOW_SOLID) {
  //   left_margin = left_road_width - trajectory_generator_conf_.out_side_margin();
  // } else if (matched_end_point.left_line_type() ==
  //                legionclaw::common::LaneLineType::WHITE_DASHED ||
  //            matched_end_point.left_line_type() ==
  //                legionclaw::common::LaneLineType::YELLOW_DASHED ||
  //            matched_end_point.left_line_type() ==
  //                legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN)
  //   left_margin = left_road_width;

  // right_margin -
  // if (matched_end_point.right_line_type() ==
  //         legionclaw::common::LaneLineType::CURB ||
  //     matched_end_point.right_line_type() ==
  //         legionclaw::common::LaneLineType::WHITE_SOLID ||
  //     matched_end_point.right_line_type() ==
  //         legionclaw::common::LaneLineType::YELLOW_SOLID) {
  //   right_margin = -right_road_width + trajectory_generator_conf_.out_side_margin();
  // } else if (matched_end_point.right_line_type() ==
  //                legionclaw::common::LaneLineType::WHITE_DASHED ||
  //            matched_end_point.right_line_type() ==
  //                legionclaw::common::LaneLineType::YELLOW_DASHED ||
  //            matched_end_point.right_line_type() ==
  //                legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN)
  //   right_margin = -right_road_width;
  // 本车道避让功能，可行驶边界计算,在道路够宽可通行的情况下进行偏移。
  // 获取道路宽度，获取静止障碍物在车道的关系
  bool enable_candidate_trajectory = false;
  double target_end_l = 0.0;
  if (global_index == 1) {
    // 获取车辆前方一定区域内的最近静止障碍物
    int max_horizon_index = max(
        matched_point_index + (int)(planning_conf_->look_forward_short_distance() / path_density),
        matched_end_point_index);
    int min_horizon_index = matched_point_index;

    // int max_horizon_index =
    //     max(vehicle_matched_point_index +
    //             (int)((vehicle_param_.front_edge_to_center() + 3.0 * frame_.VehicleState().speed) /
    //                   path_density),
    //         matched_end_point_index);
    // int min_horizon_index =
    //     vehicle_matched_point_index - (int)(vehicle_param_.back_edge_to_center() / path_density);

    auto last_behaviour_lat = frame_.LastBehaviour().lat_state;
    for (auto iter = frame_.ObstacleList().begin(); iter != frame_.ObstacleList().end(); iter++) {
      auto obs = iter->perception_obstacle();
      int obs_lon_index = obs.obj_lon_index() + (int)(obs.length() / path_density);
      double obs_speed = Norm(obs.velocity().x(), obs.velocity().y());
      if (last_behaviour_lat ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_STATE ||
          last_behaviour_lat ==
              legionclaw::interface::ADCTrajectory::BehaviourLatState::AVOIDANCE_BACK_PRE_STATE) {
        min_horizon_index =
            matched_point_index -
            (int)((obs.length() + planning_conf_->safe_length_lower_limit()) / path_density);
      }

      // if (obs_speed < planning_conf_->threshold_static_speed() &&
      //     obs_lon_index >
      //         min_horizon_index + frame_.VehicleState().speed / path_density &&
      //     obs_lon_index <= max_horizon_index /*&&
      //     (obs.lane_position() == legionclaw::common::LanePosition::EGO_LANE)*/)
      if (obs_speed < planning_conf_->threshold_static_speed() &&
          obs_lon_index > min_horizon_index && obs_lon_index <= max_horizon_index) {
        SLBoundary sl_boundary;
        if (!GetSLBoundary(reference_points, obs.polygon().points(), &sl_boundary)) {
          break;
        }
        // 非侵占本车道障碍物过滤
        if (sl_boundary.end_l() < right_margin || sl_boundary.start_l() > left_margin) continue;
        // 由障碍物划分左右两个凸空间，构建可行驶区域，并取最优值
        //  右边凸空间 [right_margin,convex_space_r]
        double convex_space_r = sl_boundary.start_l() - planning_conf_->safe_width_lower_limit();
        // 左边凸空间 [convex_space_l,left_margin]
        double convex_space_l = sl_boundary.end_l() + planning_conf_->safe_width_lower_limit();
        double driving_area_r = convex_space_r - right_margin;
        double driving_area_l = left_margin - convex_space_l;

        // 过滤可借道车道的障碍物
        // 向左借道
        if (left_road_width > right_road_width) {
          if (driving_area_r < driving_area_l) {
            if (driving_area_l < vehicle_param_.width() /*||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::CURB ||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID*/) {
              AINFO << " The area is impassable.";
              break;
            } else {
              // target_end_l = convex_space_l + 0.5 * (driving_area_l);
              target_end_l = std::min(convex_space_l + 0.5 * (driving_area_l),
                                      convex_space_l + 0.5 * vehicle_param_.width() +
                                          planning_conf_->safe_width_lower_limit());
              target_end_l = std::max(target_end_l, 0.0);
            }
          } else {
            if (driving_area_r < vehicle_param_.width() /*||
                /*matched_end_point.right_line_type() == legionclaw::common::LaneLineType::CURB ||
                matched_end_point.right_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
                matched_end_point.right_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID*/) {
              AINFO << " The area is impassable.";
              break;
            } else {
              // target_end_l = convex_space_r - 0.5 * (driving_area_r);
              // target_end_l = std::max(convex_space_r - 0.5 * (driving_area_r),
              //                         convex_space_r - 0.5 * vehicle_param_.width() -
              //                             planning_conf_->safe_width_lower_limit());
              target_end_l = std::min(convex_space_r - 0.5 * (driving_area_r), 0.0);
            }
          }
        } else {
          if (driving_area_r < driving_area_l) {
            if (driving_area_l < vehicle_param_.width() /*||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::CURB ||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
                matched_end_point.left_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID*/) {
              AINFO << " The area is impassable.";
              break;
            } else {
              // target_end_l = convex_space_l + 0.5 * (driving_area_l);
              // target_end_l = std::min(convex_space_l + 0.5 * (driving_area_l),
              //                         convex_space_l + 0.5 * vehicle_param_.width() +
              //                             planning_conf_->safe_width_lower_limit());
              target_end_l = std::max(convex_space_l + 0.5 * (driving_area_l), 0.0);
            }
          } else {
            if (driving_area_r < vehicle_param_.width() /*||
                matched_end_point.right_line_type() == legionclaw::common::LaneLineType::CURB ||
                matched_end_point.right_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
                matched_end_point.right_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID*/) {
              AINFO << " The area is impassable.";
              break;
            } else {
              // target_end_l = convex_space_r - 0.5 * (driving_area_r);
              target_end_l = std::max(convex_space_r - 0.5 * (driving_area_r),
                                      convex_space_r - 0.5 * vehicle_param_.width() -
                                          planning_conf_->safe_width_lower_limit());
              target_end_l = std::min(target_end_l, 0.0);
            }
          }
        }

        // // 取空间较大者
        // if (driving_area_r < driving_area_l) {
        //   if (driving_area_l < vehicle_param_.width() ||
        //       matched_end_point.left_line_type() == legionclaw::common::LaneLineType::CURB ||
        //       matched_end_point.left_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
        //       matched_end_point.left_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID) {
        //     AINFO << " The area is impassable.";
        //     break;
        //   } else {
        //     // target_end_l = convex_space_l + 0.5 * (driving_area_l);
        //     target_end_l = std::min(convex_space_l + 0.5 * (driving_area_l),
        //                             convex_space_l + 0.5 * vehicle_param_.width() +
        //                                 planning_conf_->safe_width_lower_limit());
        //   }
        // } else {
        //   if (driving_area_r < vehicle_param_.width() ||
        //       matched_end_point.right_line_type() == legionclaw::common::LaneLineType::CURB ||
        //       matched_end_point.right_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
        //       matched_end_point.right_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID) {
        //     AINFO << " The area is impassable.";
        //     break;
        //   } else {
        //     // target_end_l = convex_space_r - 0.5 * (driving_area_r);
        //     target_end_l = std::max(convex_space_r - 0.5 * (driving_area_r),
        //                             convex_space_r - 0.5 * vehicle_param_.width() -
        //                                 planning_conf_->safe_width_lower_limit());
        //     // target_end_l = std::min(convex_space_r - 0.5 * (driving_area_r), 0.0);
        //   }
        // }

        // TODO 暂时屏蔽掉避让轨迹生成
        enable_candidate_trajectory = true;
        // 终点更新
        if (obs.obj_lon_index() > matched_point_index + base_length / path_density) {
          matched_end_point = reference_points.at(obs.obj_lon_index());
        }
        break;
      }
    }
  }

  // 后边界更新
  end_margin = matched_end_point.s() - init_s[0];
  if (start_margin > end_margin) {
    start_margin = end_margin;
  }
  if (planning_conf_->enable_local_map_topic()) {
    start_margin = max(start_margin,
                       base_length + trajectory_generator_conf_.traj_lc_sampling_time() * lon_step);
    end_margin = max(end_margin, start_margin);
  }

  target_conditions.clear();
  // 道路中心线轨迹
  State end_d_state = {end_offset_, 0.0, 0.0};
  // 备选轨迹
  if (enable_candidate_trajectory) {
    end_margin = std::max(end_margin, base_length + frame_.VehicleState().speed * 2);
    if (frame_.yield_action() == YieldAction::YIELD_TO_LEFT && target_end_l < end_offset_ &&
        target_end_l > 0) {
      // end_d_state = {end_offset_, 0.0, 0.0};
    } else if (frame_.yield_action() == YieldAction::YIELD_TO_RIGHT && target_end_l > end_offset_ &&
               target_end_l < 0) {
      // end_d_state = {end_offset_, 0.0, 0.0};
    } else {
      end_d_state = {target_end_l, 0.0, 0.0};
    }

    // target_conditions.emplace_back(end_d_state, end_margin);
    // if (target_end_l > 0) {
    //   std::reverse(target_conditions.begin(), target_conditions.end());
    // }
  }
  target_conditions.emplace_back(end_d_state, end_margin);
  target_conditions_matrix.emplace_back(target_conditions);

  // 5. Use the common function to generate trajectory bundles.
  if (target_conditions_matrix.size() <= 0) {
    return 0;
  }
  std::vector<Trajectory1DBundle> trajectory_bundles;
  Trajectory1DBundle lat_trajectory_bundle;
  trajectory::Trajectory1dGenerator trajectory_1d_generator(init_s, init_d);
  for (auto end_d_conditions : target_conditions_matrix) {
    if (end_d_conditions.size() <= 0) continue;
    lat_trajectory_bundle.clear();
    trajectory_1d_generator.GenerateTrajectory1DBundle<5>(init_d, end_d_conditions,
                                                          &lat_trajectory_bundle);
    trajectory_bundles.emplace_back(lat_trajectory_bundle);
  }

  // 6. Generate paths
  std::vector<int> lat_ids;
  for (auto target_condition : target_conditions_matrix.front()) {
    if (target_conditions.size() == 1) {
      lat_ids.emplace_back(0);
      break;
    }
    if (target_condition.first[0] == end_offset_) {
      lat_ids.emplace_back(0);
      continue;
    }
    if (target_condition.first[0] > 0) {
      lat_ids.emplace_back(1);
    } else {
      lat_ids.emplace_back(-1);
    }
    // } else {
    //   lat_ids.emplace_back(0);
    // }
  }
  // std::vector<std::vector<PathPoint>> lat_generated_paths;
  std::vector<std::pair<std::vector<PathPoint>, std::array<int, 2>>> lat_generated_paths_and_id;

  for (unsigned int i = 0; i < trajectory_bundles.size(); ++i) {
    if (GeneratePaths(trajectory_bundles[i], reference_points,
                      target_conditions_matrix[i].front().second, path_density, init_point.s(),
                      init_s[0], i, lat_ids, lat_generated_paths_and_id))
      generated_paths.insert(generated_paths.end(), lat_generated_paths_and_id.begin(),
                             lat_generated_paths_and_id.end());
    // TODO:lat_id,lon_id,gid
  }

  return 1;
}

std::vector<PathPoint> TrajectoryGenerator::ComputeExtendPath(
    const PathPoint &start_pos,  // const int &gid,
    const std::vector<ReferencePoint> &reference_points, ReferencePoint &matched_pos,
    const double &trajectory_length, const double &safe_margin, bool &is_narrow)
{
  PathPoint tmp_extend_point;
  std::vector<PathPoint> extend_path;
  extend_path.clear();

  double dist_min;  // 左正右负
  size_t matched_index = MapMatcher::QueryNearestPointWithBuffer(
      reference_points, {start_pos.x(), start_pos.y(), start_pos.z()}, start_pos.theta(), 1.0e-6,
      dist_min);
  if (matched_index >= 0) {
    if (matched_index < reference_points.size())
      matched_pos = reference_points.at(matched_index);
    else
      matched_pos = reference_points.back();
    for (unsigned int i = matched_index + 1; i < reference_points.size(); ++i) {
      // 对extend的点进行道路是否安全较验
      if ((reference_points[i].right_road_width() + dist_min < safe_margin &&
           (reference_points[i].right_line_type() == legionclaw::common::LaneLineType::CURB ||
            reference_points[i].right_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
            reference_points[i].right_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID)) ||
          (reference_points[i].left_road_width() - dist_min < safe_margin &&
           (reference_points[i].left_line_type() == legionclaw::common::LaneLineType::CURB ||
            reference_points[i].left_line_type() == legionclaw::common::LaneLineType::WHITE_SOLID ||
            reference_points[i].left_line_type() == legionclaw::common::LaneLineType::YELLOW_SOLID))) {
        is_narrow = true;
        break;
      }
      tmp_extend_point.set_x(reference_points[i].x() - dist_min * sin(reference_points[i].theta()));
      tmp_extend_point.set_y(reference_points[i].y() + dist_min * cos(reference_points[i].theta()));
      tmp_extend_point.set_z(reference_points[i].z());
      tmp_extend_point.set_theta(reference_points[i].theta());
      // tmp_extend_point.v = reference_points[i].v;
      tmp_extend_point.set_kappa(1.0 / (1.0 / reference_points[i].kappa() + dist_min));
      double delta_mileage = reference_points[i].s() - reference_points[matched_index].s();
      tmp_extend_point.set_s(start_pos.s() + delta_mileage);
      // tmp_extend_point.set_lat_offset(dist_min);
      extend_path.push_back(tmp_extend_point);
      if (tmp_extend_point.s() >= trajectory_length) break;
    }
  }
  return extend_path;
}

int TrajectoryGenerator::get_matched_point_index(
    const std::vector<ReferencePoint> &reference_points, const int start_index,
    const double mileages)
{
  size_t match_point_index = 0;
  for (match_point_index = start_index; match_point_index < reference_points.size() - 1;
       ++match_point_index) {
    if (reference_points.at(match_point_index).s() >= mileages) break;
  }
  return match_point_index;
}

std::vector<PathPoint> TrajectoryGenerator::ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint> &ref_points)
{
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto &ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_z(ref_point.z());
    path_point.set_theta(ref_point.theta());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());
    // path_point.set_lw(ref_point.LW);
    // path_point.set_rw(ref_point.RW);
    // path_point.set_reverse_width(ref_point.reverseLaneWidth);

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

}  // namespace planning
}  // namespace legionclaw
