
/// \file TrajectoryEvaluator.cpp
/// \brief Calculate collision costs for roll out trajectory for free trajectory
/// evaluation for OpenPlanner local planner version 1.5+ \author Hatem Darweesh
/// \date Jan 14, 2018

#include "trajectory_evaluator.h"

#include "omp.h"

namespace legionclaw {
namespace planning {

void TrajectoryEvaluator::Init(const legionclaw::planning::PlanningConf *planning_conf)
{
  EvaluatorParamsInit(planning_conf);
  EvaluatorVariableInit();
}

void TrajectoryEvaluator::EvaluatorParamsInit(const legionclaw::planning::PlanningConf *planning_conf)
{
  if (planning_conf == nullptr) return;
  vehicle_param_ = planning_conf->vehicle_param();
  params_ = planning_conf;
  evaluator_params_ = planning_conf->lattice_planner_conf().mutable_trajectory_evaluator_conf();
}

void TrajectoryEvaluator::EvaluatorVariableInit()
{
  num_col_ = 0;
  num_row_ = 0;
}

void TrajectoryEvaluator::UpdateKeyParams()
{
  // 根据速度更新实时保护距离
  double safe_width_real_time = 0.0;
  double safe_length_real_time = 0.0;
  FindLinearCFromAToB(params_->safe_width_lower_limit(), params_->safe_width(),
                      params_->safe_dis_speed_lower_limit(), params_->safe_dis_speed_upper_limit(),
                      frame_->VehicleState().speed, safe_width_real_time);
  evaluator_params_->set_safe_width_real_time(safe_width_real_time);
  FindLinearCFromAToB(params_->safe_length_lower_limit(), params_->safe_length(),
                      params_->safe_dis_speed_lower_limit(), params_->safe_dis_speed_upper_limit(),
                      frame_->VehicleState().speed, safe_length_real_time);
  evaluator_params_->set_safe_length_real_time(safe_length_real_time);
  // 根据车速更新障碍物的影响范围
  double object_impact_real_time_range = 0.0;
  FindLinearCFromAToB(
      evaluator_params_->object_impact_lower_range(), evaluator_params_->object_impact_upper_range(),
      evaluator_params_->object_impact_lower_speed(), evaluator_params_->object_impact_upper_speed(),
      frame_->VehicleState().speed, object_impact_real_time_range);
  evaluator_params_->set_object_impact_real_time_range(object_impact_real_time_range);
  // AINFO<< evaluator_params_->object_impact_real_time_range()
  // << ", "
  // // << evaluator_params_->safe_length_real_time()
  // << ", speed: "
  // << frame_->VehicleState().speed
  // <<"\n";
}

void TrajectoryEvaluator::FindLinearCFromAToB(const double &a, const double &b, const double &x,
                                              const double &y, const double input, double &c)
{
  double ratio_relative_to_speed = (b - a) / (y - x);
  c = a + ratio_relative_to_speed * (input - x);
  if (c > b) c = b;
  if (c < a) c = a;
}

void TrajectoryEvaluator::InitTrajectoryCost(TrajectoryCost &tc)
{
  tc.rightmost_cost = 0;
  tc.priority_cost = 0;
  tc.transition_cost = 0;
  tc.closest_obj_cost = 0;
  tc.blocked_cost = 0;
  tc.cost = 0;
  tc.closest_obj_velocity = 0;
  tc.closest_obj_acceleration = 0;
  tc.closest_obj_distance = 99999;
  tc.closest_obj_lateral_distance = 99999;
  tc.closest_obj_collision_dis = 99999;
  tc.closest_obj_collisiontime = 99999;
  tc.closest_obj_theta_abs = 0;
  tc.closest_obj_theta_veh = 0;
  tc.closest_obj_id = -1;
  tc.curvature_cost = 0;
  tc.horizon_cost = 0;
  tc.junction_close_cost = 0;
  tc.lane_change_cost = 0;
  tc.potential_cost = 0;
  tc.lateral_left_cost = 0;
  tc.lateral_right_cost = 0;
  tc.longitudinal_cost = 0;
  tc.is_fully_block = false;
  tc.is_passage_safe_f = true;
  tc.is_passage_safe_b = true;
  tc.is_passage_safe_dclc = true;
  tc.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_UNKNOWN;
  tc.id = "";
}

Status TrajectoryEvaluator::EvaluateTrajectories(
    const vector<vector<PlanningTrajectory>> &trajectory_array, Frame *frame,
    std::vector<TimeConsume> &time_consume_vec)
{
  // double t1 = TimeTool::NowToSeconds();
  if (frame->ReferenceLines().size() == 0 || frame->MapMatchInfo().current_lane_id < 0 ||
      frame->MapMatchInfo().lon_index < 0)
    return Status(Status::ErrorCode::PLANNING_ERROR, "Input error.");

  trajectory_array_ = trajectory_array;
  frame_ = frame;
  UpdateKeyParams();
  // std::vector<Polygon2d> obstacle_polygon_list;
  // double t1 = TimeTool::NowToSeconds();
  // ComputeObstacleListPolygon(frame_->ObstacleList(), obstacle_polygon_list);
  double t2 = TimeTool::NowToSeconds();
  // create potential field
  if (evaluator_params_->weight_pot()) {
    std::vector<double> boundary_val;
    potential_field_.InitPotentialFieldFromFrenet(frame_, params_, evaluator_params_, num_row_,
                                                  num_col_, boundary_val);
  }
  double t3 = TimeTool::NowToSeconds();
  TimeConsume delta_init_potential;
  delta_init_potential.set_time_consume(t3 - t2);
  delta_init_potential.set_name("init potential");
  time_consume_vec.emplace_back(delta_init_potential);

  if (evaluator_params_->weight_pot()) {
    potential_field_.CalculatePotentialField(frame_->ObstacleList(), frame_->VehicleState(),
                                             evaluator_params_, params_->threshold_static_speed());
  }
  double t4 = TimeTool::NowToSeconds();
  TimeConsume delta_calculate_potential;
  delta_calculate_potential.set_time_consume(t4 - t3);
  delta_calculate_potential.set_name("calculate potential");
  time_consume_vec.emplace_back(delta_calculate_potential);

  // calculate per extract_path cost
  std::vector<TimeConsume> time_consume_vec_temp = ComputeCostsDynamic(frame);
  time_consume_vec.insert(time_consume_vec.end(), time_consume_vec_temp.begin(),
                          time_consume_vec_temp.end());
  double t5 = TimeTool::NowToSeconds();
  TimeConsume delta_calculate_dynamic_cost;
  delta_calculate_dynamic_cost.set_time_consume(t5 - t4);
  delta_calculate_dynamic_cost.set_name("calculate dynamic cost");
  time_consume_vec.emplace_back(delta_calculate_dynamic_cost);
  NormalizeCosts();
  double t6 = TimeTool::NowToSeconds();
  TimeConsume delta_normalize;
  delta_normalize.set_time_consume(t6 - t5);
  delta_normalize.set_name("calculate normalize cost");
  time_consume_vec.emplace_back(delta_normalize);

  if (evaluator_params_->b_cost_value_dynamic_log()) {
    ofstream outfile("./log/select_value_dynamic_log.log", std::ofstream::app);
    outfile.precision(6);
    if (trajectory_array_.size() > 0) {
      int index = 0;
      for (size_t i = 0; i < trajectory_array_.size(); ++i) {
        for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
          outfile << "Index: " << index << ", lat_id: " << trajectory_array_.at(i).at(j).lat_id()
                  << ", lon_id: " << trajectory_array_.at(i).at(j).lon_id() << ", Priority: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().priority_cost *
                         evaluator_params_->weight_priority()
                  << ", Transition: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().transition_cost *
                         evaluator_params_->weight_transition()
                  << ", Lat_l: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().lateral_left_cost *
                         evaluator_params_->weight_lat()
                  << ", Lat_r: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().lateral_right_cost *
                         evaluator_params_->weight_lat()
                  << ", Pot: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().potential_cost *
                         evaluator_params_->weight_pot()
                  << ", Long: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().longitudinal_cost *
                         evaluator_params_->weight_lon()
                  << ", Change: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().lane_change_cost *
                         evaluator_params_->weight_lane_change()
                  << ", Horizon: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().horizon_cost *
                         evaluator_params_->weight_horizon()
                  << ", Junction: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().junction_close_cost
                  << ", Curvature: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().curvature_cost *
                         evaluator_params_->weight_curvature()
                  << ", IsCollision: "
                  << trajectory_array_.at(i).at(j).trajectory_cost().is_fully_block
                  << ", block: " << trajectory_array_.at(i).at(j).trajectory_cost().blocked_cost
                  << ", Avg: " << trajectory_array_.at(i).at(j).trajectory_cost().cost << endl;
          index++;
        }
      }
    }
    outfile << "\n";
    outfile.close();
  }

  return Status(Status::ErrorCode::OK);
}

int TrajectoryEvaluator::InitializeTrajectoryCosts()
{
  TrajectoryCost tc;
  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
      InitTrajectoryCost(tc);
      trajectory_array_.at(i).at(j).set_trajectory_cost(tc);
    }
  }
  return 1;
}

int TrajectoryEvaluator::CalculatePriorityCosts(const double &biasPreferCurrentlane)
{
  double priority_cost = 0.0;
  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
      priority_cost = pow(trajectory_array_.at(i).at(j).lat_id(), 2) +
                      1.0 / (trajectory_array_.at(i).at(j).lon_id() + 1.0);
      if (trajectory_array_.at(i).at(j).global_index() != frame_->MapMatchInfo().current_lane_index)
        priority_cost += biasPreferCurrentlane;
      trajectory_array_.at(i).at(j).set_priority_cost(priority_cost);
    }
  }
  return 1;
}

bool TrajectoryEvaluator::CalculateRightmostCosts()
{
  // 环岛和交叉路口不适用该规则
  if (station_index_global_ > -1 ||
      frame_->GetTrafficEvents().junction_info().distance_to_junction() < 15 ||
      frame_->GetGuideInfo().current_road().road_type() == 1 ||
      frame_->GetGuideInfo().current_road().road_type() == 2 ||
      (frame_->MapMatchInfo().priority[1] == 1 &&
       frame_->GetTrafficEvents().junction_info().direction_flag() &&
       frame_->GetTrafficEvents().junction_info().direction() != legionclaw::common::Direction::LEFT)) {
    return true;
  }
  for (int i = frame_->ReferenceLines().size() - 1; i > 0; --i) {
    if (frame_->ReferenceLines().at(i).ReferencePointSize() > 0) {
      if (i == 2) {  // 车当前不在最右边
        return true;
      }
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        trajectory_array_.at(i).at(j).set_rightmost_cost(evaluator_params_->weight_rightmost());
      }
    }
  }
  return true;
}

int TrajectoryEvaluator::CalculateHorizonCosts(
    const std::vector<reference_line::ReferenceLine> &reference_lines)
{
  // 找到最长的切片信息
  double remain_length_max = 0;
  unsigned int remain_length_max_id = 0;
  for (unsigned int i = 0; i < reference_lines.size(); ++i) {
    if (reference_lines.at(i).ReferencePointSize() > 0) {
      if (remain_length_max < reference_lines.at(i).remain_mileage()) {
        remain_length_max_id = i;
        remain_length_max = reference_lines.at(i).remain_mileage();
      }
    }
  }

  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    // 最长不考虑
    if (i == remain_length_max_id) continue;
    // 推荐车道不考虑
    if (frame_->MapMatchInfo().priority[i] == 1) {
      continue;
    }
    double horizon_cost = 0.0;
    double remain_mileage = reference_lines.at(i).remain_mileage();
    if (remain_length_max - remain_mileage >= evaluator_params_->horizon_dist_diff()) {
      // if (remain_mileage < evaluator_params_->horizon_dist_diff()) {
      // 剩余距离和当前静止障碍物的距离进行比较，若小于静止障碍物距离，则对应代价值需大于1000
      if (remain_mileage <= 15 || remain_mileage <= 4 * frame_->VehicleState().speed ||
          (trajectory_array_.at(1).front().trajectory_cost().blocked_cost == 1000 &&
           trajectory_array_.at(1).front().trajectory_cost().closest_obj_distance >
               remain_mileage)) {  // 大于静止障碍物代价值
        horizon_cost = 1100;
      } else {  // 代价值需小于静止障碍物
        horizon_cost = std::fmin(
            evaluator_params_->weight_horizon() * (10000.0 / (1 + remain_mileage) + 500), 700.0);
      }
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        // 道路类型的junction方向一致，则不考虑断头路代价值(多对一目前做的断头路处理)
        double junction_cost = trajectory_array_.at(i).at(j).trajectory_cost().junction_close_cost;
        // TODO routing优化多对一场景后，该条件可以去掉。暂时使用该条件
        if (junction_cost == 0 && frame_->GetGuideInfo().current_road().road_type() != 1 &&
            frame_->GetTrafficEvents().junction_info().direction_flag() && horizon_cost < 1000) {
          continue;
        }
        if (horizon_cost < 1000 &&
            frame_->GetTrafficEvents().junction_info().distance_to_junction() >
                200) {  // 代价值小于静止障碍物
          horizon_cost = 900 - junction_cost;
        }
        trajectory_array_.at(i).at(j).set_horizon_cost(horizon_cost);
      }
    }
  }
  potential_field_.ClearVecLengthRemain();
  return 1;
}

int TrajectoryEvaluator::CalculateJunctionCloseCosts(
    const legionclaw::interface::JunctionInfo &junction_info)
{
  // case1：当前道路类型是环岛并且规划路线还是在环岛主路中行驶
  if (frame_->GetGuideInfo().current_road().road_type() == 1 &&
      frame_->GetGuideInfo().next_road().road_type() == 1) {
    return 0;
  }
  // TODO case2：当前道路类型是环岛并且规划路线即将驶出环岛主路
  LaneInfoType ego_lane_type = frame_->MapMatchInfo().ego_lane_type[1];
  // TODO 非环岛状态才允许响应左转指令，地图引擎的bug引起的，先暂时加条件规避该bug
  if (frame_->GetGuideInfo().round_status() == 0 &&
      junction_info.direction() == legionclaw::common::Direction::LEFT &&
      junction_info.distance_to_junction() < evaluator_params_->distance_to_junction() &&
      junction_info.distance_to_junction() > 15) {
    if (!(ego_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
          ego_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND)) {
      for (size_t i = 1; i < trajectory_array_.size(); ++i) {
        if (trajectory_array_.at(i).size() > 0) {
          if (frame_->MapMatchInfo().ego_lane_type[i] == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
              frame_->MapMatchInfo().ego_lane_type[i] == LaneInfoType::LEFT_TURN_AND_TURN_AROUND) {
            continue;
          } else {
            for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
              trajectory_array_.at(i).at(j).set_junction_close_cost(
                  i * evaluator_params_->weight_junction() + 500);
            }
          }
        }
      }
    }
    // if (ego_lane_type == LaneInfoType::STRAIGHT || ego_lane_type == LaneInfoType::RIGHT_TURN ||
    //     ego_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN ||
    //     ego_lane_type == LaneInfoType::LANE_TYPE_UNKNOWN) {
    //   for (size_t i = 1; i < trajectory_array_.size(); ++i) {
    //     for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
    //       // 当前进行判断的车道不是最左边车道，则进行junction代价计算
    //       if (trajectory_array_.at(i - 1).size() > 0) {
    //         trajectory_array_.at(i).at(j).set_junction_close_cost(
    //             i * evaluator_params_->weight_junction() + 500);
    //       }
    //     }
    //   }
    // }
    else {  // 自车在左转车道
      // 判断右边车道类型为非左转道
      auto right_lane_type = frame_->MapMatchInfo().ego_lane_type[2];
      if (trajectory_array_.at(2).size() > 0 &&
          (right_lane_type == LaneInfoType::STRAIGHT ||
           right_lane_type == LaneInfoType::RIGHT_TURN ||
           right_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN ||
           right_lane_type == LaneInfoType::LANE_TYPE_UNKNOWN)) {
        for (size_t j = 0; j < trajectory_array_.at(2).size(); ++j) {
          trajectory_array_.at(2).at(j).set_junction_close_cost(600);
        }
      }
    }
  } else if (junction_info.direction() == legionclaw::common::Direction::UP &&
             junction_info.distance_to_junction() < evaluator_params_->distance_to_junction()) {
    // 自车在可左转车道
    if (ego_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
        ego_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND ||
        ego_lane_type == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND) {
      for (size_t i = 0; i < trajectory_array_.size() - 1; ++i) {
        for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
          // 当前进行判断的车道不是最右边车道，则进行junction代价计算
          if (trajectory_array_.at(i + 1).size() > 0) {
            trajectory_array_.at(i).at(j).set_junction_close_cost(
                (trajectory_array_.size() - i - 1) * evaluator_params_->weight_junction() + 500);
          }
        }
      }
    } else if (ego_lane_type == LaneInfoType::RIGHT_TURN) {  // 自车在可右转车道
      for (size_t i = 1; i < trajectory_array_.size(); ++i) {
        for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
          // 当前进行判断的车道不是最左边车道，则进行junction代价计算
          if (trajectory_array_.at(i - 1).size() > 0) {
            trajectory_array_.at(i).at(j).set_junction_close_cost(
                i * evaluator_params_->weight_junction() + 500);
          }
        }
      }
    } else {  // 自车在直行道
      // 判断左边、右边为非直行车道
      auto right_lane_type = frame_->MapMatchInfo().ego_lane_type[2];
      if (trajectory_array_.at(2).size() > 0 &&
          (right_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
           right_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND ||
           right_lane_type == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND ||
           right_lane_type == LaneInfoType::RIGHT_TURN)) {
        for (size_t j = 0; j < trajectory_array_.at(2).size(); ++j) {
          trajectory_array_.at(2).at(j).set_junction_close_cost(600);
        }
      }
      auto left_lane_type = frame_->MapMatchInfo().ego_lane_type[0];
      if (trajectory_array_.at(0).size() > 0 &&
          (left_lane_type == LaneInfoType::LEFT_TURN_NO_TURN_AROUND ||
           left_lane_type == LaneInfoType::LEFT_TURN_AND_TURN_AROUND ||
           left_lane_type == LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND ||
           left_lane_type == LaneInfoType::RIGHT_TURN)) {
        for (size_t j = 0; j < trajectory_array_.at(0).size(); ++j) {
          trajectory_array_.at(0).at(j).set_junction_close_cost(600);
        }
      }
    }
  } else if (junction_info.direction() == legionclaw::common::Direction::RIGHT &&
             junction_info.distance_to_junction() < evaluator_params_->distance_to_junction()) {
    if (!(ego_lane_type == LaneInfoType::RIGHT_TURN ||
          ego_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN)) {
      for (size_t i = 0; i < trajectory_array_.size() - 1; ++i) {
        if (trajectory_array_.at(i).size() > 0) {
          // TODO 可能存在bug，ego_lane_type和轨迹不一定完全对应
          if (frame_->MapMatchInfo().ego_lane_type[i] == LaneInfoType::RIGHT_TURN ||
              frame_->MapMatchInfo().ego_lane_type[i] == LaneInfoType::STRAIGHT_AND_RIGHT_TURN) {
            continue;
          } else {
            for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
              // // 当前进行判断的车道不是最右边车道，则进行junction代价计算
              // if (trajectory_array_.at(i + 1).size() > 0 ||
              //     frame_->MapMatchInfo().ego_lane_type[i] != LaneInfoType::LANE_TYPE_UNKNOWN) {
              //   trajectory_array_.at(i).at(j).set_junction_close_cost(
              //       (trajectory_array_.size() - i - 1) * evaluator_params_->weight_junction()
              //       + 500);
              // }
              trajectory_array_.at(i).at(j).set_junction_close_cost(
                  (trajectory_array_.size() - i - 1) * evaluator_params_->weight_junction() +
                  500);
            }
          }
        }
      }
    } else {  // 自车在右转车道
      // 判断左边车道类型为非右转道
      auto left_lane_type = frame_->MapMatchInfo().ego_lane_type[0];
      if (trajectory_array_.at(0).size() > 0 &&
          (!(left_lane_type == LaneInfoType::RIGHT_TURN ||
             left_lane_type == LaneInfoType::STRAIGHT_AND_RIGHT_TURN))) {
        for (size_t j = 0; j < trajectory_array_.at(0).size(); ++j) {
          trajectory_array_.at(0).at(j).set_junction_close_cost(600);
        }
      }
    }

  } else {
    ;
  }
  return 1;
}

int TrajectoryEvaluator::CalculateCurvatureCosts(const double &turning_radius)
{
  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
      double sum_curvature = 0;
      size_t num_waypoints = trajectory_array_.at(i).at(j).trajectory_points_size();
      if (num_waypoints > 0) {
        for (size_t iw = 0; iw < num_waypoints; ++iw) {
          if (fabs(trajectory_array_.at(i).at(j).trajectory_points(iw).path_point().kappa()) >
              2.0 / turning_radius)  // 有两倍的冗余
          {
            sum_curvature = 10e9 * num_waypoints;
            break;
          }
          sum_curvature +=
              fabs(trajectory_array_.at(i).at(j).trajectory_points(iw).path_point().kappa());
        }
        trajectory_array_.at(i).at(j).set_curvature_cost(sum_curvature / num_waypoints);
      }
    }
  }
  return 1;
}

int TrajectoryEvaluator::CalculateTransitionCosts()
{
  double match_distance = DBL_MAX;  // 左正右负
  station_index_global_ = -1;
  double goal_distance = DBL_MAX;
  int car_index = frame_->MapMatchInfo().lon_index;
  ReferenceLine current_reference_line =
      frame_->ReferenceLine(frame_->MapMatchInfo().current_lane_index);

  auto terminal_stop_point = frame_->terminal_stop_point();
  if (terminal_stop_point.type() != -1) {
    station_index_global_ = MapMatcher::QueryNearestPointWithBuffer(
        current_reference_line.ReferencePoints(),
        {terminal_stop_point.point().x(),
         terminal_stop_point.point().y()},  // TODO:暂时认为只有一个停车点
        terminal_stop_point.theta(), 1.0e-6,
        match_distance);  // 站点与参考线匹配
  }
  if (station_index_global_ > -1 && fabs(match_distance) < 8) {
    goal_distance = current_reference_line.ReferencePoints(station_index_global_).s() -
                    current_reference_line.ReferencePoints(car_index).s();
  }
  // 新增靠边停车代价
  if ((match_distance < -0.8 || frame_->MapMatchInfo().lat_offset >= 0.2) && goal_distance < 65) {
    for (size_t i = 0; i < trajectory_array_.size(); ++i) {
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        trajectory_array_.at(i).at(j).set_transition_cost(2 - i);
      }
    }
  }
  return 1;
}

int TrajectoryEvaluator::NormalizeCosts()
{
  // Normalize costs
  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
      double cost = trajectory_array_.at(i).at(j).trajectory_cost().blocked_cost +
                    trajectory_array_.at(i).at(j).trajectory_cost().rightmost_cost +
                    evaluator_params_->weight_priority() *
                        trajectory_array_.at(i).at(j).trajectory_cost().priority_cost +
                    evaluator_params_->weight_transition() *
                        trajectory_array_.at(i).at(j).trajectory_cost().transition_cost +
                    evaluator_params_->weight_lat() *
                        trajectory_array_.at(i).at(j).trajectory_cost().lateral_left_cost +
                    evaluator_params_->weight_lat() *
                        trajectory_array_.at(i).at(j).trajectory_cost().lateral_right_cost +
                    evaluator_params_->weight_lon() *
                        trajectory_array_.at(i).at(j).trajectory_cost().longitudinal_cost +
                    evaluator_params_->weight_curvature() *
                        trajectory_array_.at(i).at(j).trajectory_cost().curvature_cost +
                    trajectory_array_.at(i).at(j).trajectory_cost().horizon_cost +
                    evaluator_params_->weight_pot() *
                        trajectory_array_.at(i).at(j).trajectory_cost().potential_cost +
                    trajectory_array_.at(i).at(j).trajectory_cost().junction_close_cost;
      trajectory_array_.at(i).at(j).set_cost(cost);
    }
  }
  return 1;
}

bool TrajectoryEvaluator::CalculateDynamicObjPolygonCollision(const PredictionObstacleMsg &obj,
                                                              const TrajectoryPoint &path_pose,
                                                              const Polygon2d &adc_polygon,
                                                              PathPoint &collision_point,
                                                              TrajectoryCost &trajectorycost,
                                                              Polygon2d &col_obj_polygon)
{
  // int64_t t0 = TimeTool::Now2Us();

  // int iw_step = obj.perception_obstacle().width() / params_->path_density();
  // #pragma omp parallel for
  for (size_t it = 0; it < obj.trajectory_size(); ++it) {
    if (obj.trajectory(it).probability() > 0) {
      int polygon_size = obj.polygon_lists(it).size();
      // int count_loop = 0;
      // int64_t t1 = TimeTool::Now2Us();

      for (int j = 0; j < polygon_size; j++) {
        // int count_loop = j;
        // int64_t tt1 = TimeTool::Now2Us();
        if (obj.polygon_lists(it).at(j).HasOverlap(adc_polygon)) {
          // int index_point = j * iw_step;
          // if ((size_t)index_point >= obj.trajectory(it).trajectory_points_size()) {
          //   index_point = obj.trajectory(it).trajectory_points_size() - 1;
          // }
          double collision_time_diff =
              path_pose.relative_time() - obj.trajectory(it).trajectory_points(j).relative_time();
          if (fabs(collision_time_diff) < evaluator_params_->collision_time_diff()) {
            collision_point = path_pose.path_point();
            double diff_theta_prediction =
                AngleDiff(collision_point.theta(),
                          obj.trajectory(it).trajectory_points(j).path_point().theta());
            double diff_theta_obj =
                AngleDiff(collision_point.theta(), obj.perception_obstacle().theta());
            if (fabs(diff_theta_prediction) > 0.75 * M_PI &&
                frame_->GetGuideInfo().current_road().road_type() != 2 &&
                (obj.perception_obstacle().lane_position() ==
                     legionclaw::common::LanePosition::NEXT_LEFT_LANE ||
                 obj.perception_obstacle().lane_position() ==
                     legionclaw::common::LanePosition::NEXT_RIGHT_LANE)) {
              if (fabs(diff_theta_obj) <= 0.75 * M_PI) {
                return false;
              }
            }
            // 过滤朝自车方向crossing卡车类障碍物
            if (fabs(diff_theta_prediction) >= M_PI / 6.0 &&
                frame_->GetGuideInfo().current_road().road_type() != 2 &&
                (j > 0 && obj.trajectory(it).trajectory_points(j).v() < 0.28) &&
                (obj.perception_obstacle().sub_type() == legionclaw::common::ObstacleSubType::ST_TRUCK ||
                 obj.perception_obstacle().sub_type() == legionclaw::common::ObstacleSubType::ST_VAN)) {
              if (fabs(diff_theta_obj) < M_PI / 6.0) {
                return false;
              }
            }
            // 横穿场景，自车需先障碍物车先到达，后达到的时间差允许超过1s，则判定为无碰撞风险
            if (fabs(diff_theta_prediction) >= M_PI / 6.0 &&
                fabs(diff_theta_prediction) <= 0.75 * M_PI) {
              if (collision_time_diff > 1.5) {
                return false;
              }
            }

            trajectorycost.is_fully_block = true;
            trajectorycost.closest_obj_collisiontime = collision_time_diff;
            trajectorycost.closest_obj_theta_abs = diff_theta_prediction;
            trajectorycost.closest_obj_velocity = Norm(obj.perception_obstacle().velocity().x(),
                                                       obj.perception_obstacle().velocity().y());
            trajectorycost.closest_obj_acceleration = obj.perception_obstacle().acceleration().x();

            col_obj_polygon = obj.polygon_lists(it).at(j);
            return true;
          }
          // AINFO << "  ///////////    j : " << j << endl;
          break;
        }
        // int64_t tt2 = TimeTool::Now2Us();
        // AINFO << "    ----  tt2-tt1: " << tt2 - tt1 << endl;
      }
      // AINFO << "      polygon size : " << polygon_size
      //       << " , count loop : " << count_loop << endl;
      // int64_t t2 = TimeTool::Now2Us();
      // AINFO << "      t2-t1: " << t2 - t1 << endl;
    }
  }
  // int64_t t3 = TimeTool::Now2Us();
  // AINFO << "      t1-t0: " << t1 - t0 << endl;
  // AINFO << "@@@@@@@@      t3-t0: " << t3 - t0 << endl;
  return false;
}

void TrajectoryEvaluator::GetSafetyBorderPolygon(const PathPoint &point, const double &c_lateral_d,
                                                 const double &c_long_front_d,
                                                 const double &c_long_back_d, Polygon2d &polygon)
{
  // double c_lateral_d = 0.5 * vehicle_param_.width + safe_width;
  // double c_long_front_d = 0.5 * vehicle_param_.wheelbase + 0.5 *
  // vehicle_param_.length + safe_length; double c_long_back_d = 0.5 *
  // vehicle_param_.length + safe_length - 0.5 * vehicle_param_.wheelbase;

  Mat3 invRotationMat(point.theta() - M_PI_2);
  Mat3 invTranslationMat(point.x(), point.y());

  PathPoint bottom_left, bottom_right, top_right, top_left;  // TODO:pathpoint
  bottom_left.set_x(-c_lateral_d);
  bottom_left.set_y(-c_long_back_d);
  bottom_left.set_z(point.z());

  bottom_right.set_x(c_lateral_d);
  bottom_right.set_y(-c_long_back_d);
  bottom_right.set_z(point.z());

  top_right.set_x(c_lateral_d);
  top_right.set_y(c_long_front_d);
  top_right.set_z(point.z());

  top_left.set_x(-c_lateral_d);
  top_left.set_y(c_long_front_d);
  top_left.set_z(point.z());

  bottom_left = invRotationMat * bottom_left;
  bottom_left = invTranslationMat * bottom_left;

  top_right = invRotationMat * top_right;
  top_right = invTranslationMat * top_right;

  bottom_right = invRotationMat * bottom_right;
  bottom_right = invTranslationMat * bottom_right;

  top_left = invRotationMat * top_left;
  top_left = invTranslationMat * top_left;

  vector<Vec2d> poly_vertices;
  // polygon.ClearPoints();
  poly_vertices.push_back(Vec2d(bottom_left.x(), bottom_left.y()));
  poly_vertices.push_back(Vec2d(bottom_right.x(), bottom_right.y()));
  poly_vertices.push_back(Vec2d(top_right.x(), top_right.y()));
  poly_vertices.push_back(Vec2d(top_left.x(), top_left.y()));
  polygon = Polygon2d(poly_vertices);
}

bool TrajectoryEvaluator::ComputeObstacleListPolygon(
    const std::vector<PredictionObstacleMsg> &obj_list, std::vector<Polygon2d> &polygon_list)
{
  if (obj_list.size() < 1) {
    return false;
  }
  Polygon2d polygon;
  Vec2d p;
  std::vector<Vec2d> obstacle_polygon;
  polygon_list.clear();
  for (auto obj : obj_list) {
    if (ComputeObstaclePolygon(obj, polygon)) polygon_list.push_back(polygon);
  }
  return true;
}

bool TrajectoryEvaluator::ComputeObstaclePolygon(const PredictionObstacleMsg &obj,
                                                 Polygon2d &polygon)
{
  Vec2d p;
  std::vector<Vec2d> obstacle_polygon;
  int contour_num = obj.perception_obstacle().polygon().num_points();
  if (contour_num < 3) return false;
  obstacle_polygon.clear();
  for (int icon = 0; icon < contour_num; icon++) {
    p.set_x(obj.perception_obstacle().polygon().points().at(icon).x());
    p.set_y(obj.perception_obstacle().polygon().points().at(icon).y());
    obstacle_polygon.push_back(p);
  }
  polygon = Polygon2d(obstacle_polygon);
  return true;
}

void TrajectoryEvaluator::CaluculateCollisionCosts(
    const std::vector<TrajectoryPoint> &extract_path,
    const std::vector<PredictionObstacleMsg> &obj_list, const int &index,
    const std::map<int, int> &obstacles_lane_id, const legionclaw::interface::GuideRoad &current_road,
    TrajectoryCost &trajectory_cost, Polygon2d &col_obj_polygon)
{
  // Vec2d p;
  // int64_t t0 = TimeTool::Now2Ms();
  double dis_min = DBL_MAX;
  // 粗搜再细搜
  int imatched_adc = MapMatcher::QueryNearestPointWithBuffer(
      extract_path, Vec2d(frame_->VehicleState().pose.x(), frame_->VehicleState().pose.y()),
      frame_->VehicleState().pose.theta(), 1.0e-6, dis_min);
  if (imatched_adc < 0) return;
  collision_points_.clear();
  size_t ie_step = vehicle_param_.width() / params_->path_density();
  Polygon2d adc_polygon;
  double critical_long_front_distance =
      vehicle_param_.front_edge_to_center() + evaluator_params_->safe_length_real_time();
  double critical_long_back_distance =
      vehicle_param_.back_edge_to_center() + evaluator_params_->safe_length_real_time();
  double critical_lateral_distance_static =
      0.5 * vehicle_param_.width() + params_->safe_width_lower_limit();
  double critical_lateral_distance_dynamic =
      0.5 * vehicle_param_.width() + evaluator_params_->safe_width_real_time();
  // 优先处理前方障碍物按照从近到远依次处理
  for (size_t io = 0; io < obj_list.size(); ++io) {
    ObstacleMsg perception_obs = obj_list.at(io).perception_obstacle();
    // case1 过滤没匹配上车道的障碍物
    if (perception_obs.obj_lon_index() < 0) {
      continue;
    }
    // case2 匹配不上轨迹过滤
    Vec2d obj_position = {perception_obs.position().x(), perception_obs.position().y()};
    double longitudinal_dist = DBL_MAX;
    double dis_min_so = DBL_MAX;
    int imatched_obj =
        MapMatcher::QueryNearestPointWithBuffer(extract_path, obj_position, 1.0e-6, dis_min_so);
    if (imatched_obj < 0) continue;
    // case3 后方障碍物不考虑
    double dis_min_obj_vertical_distance = DBL_MAX;
    MapMatcher::QueryVerticalDistanceWithBuffer(frame_->ReferenceLine(1).ReferencePoints(),
                                                obj_position, frame_->VehicleState().pose.theta(),
                                                1.0e-6, dis_min_obj_vertical_distance);
    if (frame_->MapMatchInfo().lon_index > -1) {
      // 后方障碍物定义为：障碍车车头在自车定位后方
      // TODO 待新增优化障碍物的sl
      int obj_lon_index =
          perception_obs.obj_lon_index() + (int)(perception_obs.length() / params_->path_density());
      double rel_dis = frame_->MapMatchInfo().lat_offset - dis_min_obj_vertical_distance;
      if (frame_->MapMatchInfo().lon_index <= obj_lon_index) {
        if (rel_dis > 0) {
          if (2 * fabs(rel_dis) > vehicle_param_.width()) {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_RIGHTUP;
          } else {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_FORWARD;
          }
        } else {
          if (2 * fabs(rel_dis) > vehicle_param_.width()) {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_LEFTUP;
          } else {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_FORWARD;
          }
        }
      } else {
        if (rel_dis > 0) {
          if (2 * fabs(rel_dis) > vehicle_param_.width()) {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_RIGHTDOWN;
          } else {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_BACK;
          }
        } else {
          if (2 * fabs(rel_dis) > vehicle_param_.width()) {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_LEFTDOWN;
          } else {
            trajectory_cost.dynamic_objects_pose = DynamicObjectsPose::D_OBJ_BACK;
          }
        }
        // 路口右转，左边障碍物不进行过滤，让直行车辆
        // 路口左转，右边障碍物不进行过滤，让直行车辆
        if (current_road.road_type() == 2 &&
            (frame_->MapMatchInfo().lon_index <=
             obj_lon_index + (int)(3 * perception_obs.width() / params_->path_density())) &&
            ((current_road.turn_type() == 3 &&
              trajectory_cost.dynamic_objects_pose == DynamicObjectsPose::D_OBJ_LEFTDOWN) ||
             (current_road.turn_type() == 1 &&
              trajectory_cost.dynamic_objects_pose == DynamicObjectsPose::D_OBJ_RIGHTDOWN))) {
        } else {
          continue;
        }
      }
    }
    double obj_speed = Norm(perception_obs.velocity().x(), perception_obs.velocity().y());
    // case4 过滤站点前方且距离自车大于安全保护距离的静止障碍物/同向运动的障碍物
    int safe_index_station_front = -1;
    if (station_index_global_ > -1) {
      double diff_theta_obj =
          (obj_speed <= params_->threshold_static_speed())
              ? 0.0
              : AngleDiff(frame_->terminal_stop_point().theta(), perception_obs.theta());
      if (fabs(diff_theta_obj) <= 0.5 * M_PI) {
        safe_index_station_front =
            (vehicle_param_.front_edge_to_center() + evaluator_params_->object_impact_lower_range() -
             0.5 * pow(1.4, 2) / vehicle_param_.standstill_acceleration()) /
            params_->path_density();

        if (std::max(station_index_global_, frame_->MapMatchInfo().lon_index) +
                safe_index_station_front <
            perception_obs.obj_lon_index())
          continue;
      }
    }
    if (obj_speed <= params_->threshold_static_speed() ||
        obj_list.at(io).trajectory_size() == 0) {  // 静态障碍物
      Polygon2d obj_polygon = perception_obs.polygon();
      // 修复长度较小的近处虚拟墙障碍物会被过滤掉的bug
      for (size_t ie = 0; ie < extract_path.size(); ie += ie_step)  // rollout waypoints loop
      {
        // vehicle safety border // distance repeated calculation
        GetSafetyBorderPolygon(extract_path.at(ie).path_point(), critical_lateral_distance_static,
                               critical_long_front_distance, critical_long_back_distance,
                               adc_polygon);
        if (obj_polygon.HasOverlap(adc_polygon))  // 有碰撞
        {
          trajectory_cost.id = obj_list.at(io).Id();
          trajectory_cost.is_fully_block = true;
          // 红绿灯虚拟障碍物/站点前方存在可停车空间的障碍物不计算碰撞代价值
          if (trajectory_cost.id == "traffic_light_stop_wall" ||
              (safe_index_station_front > -1 &&
               station_index_global_ + safe_index_station_front < perception_obs.obj_lon_index())) {
            trajectory_cost.blocked_cost = 0;
          } else {
            trajectory_cost.blocked_cost = 1000;
          }
          // 自车位置距障碍物最近边的距离
          if (trajectory_cost.id == "greenbelt_occlusion_stop_wall") {
            if (ie < ie_step) {
              trajectory_cost.closest_obj_collision_dis = obj_polygon.DistanceToBoundary(
                  {extract_path.at(ie).path_point().x(), extract_path.at(ie).path_point().y()});
            } else {
              trajectory_cost.closest_obj_collision_dis =
                  (ie - ie_step) * params_->path_density() +
                  obj_polygon.DistanceToBoundary({extract_path.at(ie - ie_step).path_point().x(),
                                                  extract_path.at(ie - ie_step).path_point().y()});
            }

          } else {  // 车头距障碍物最近边的距离
            if (ie < ie_step) {
              trajectory_cost.closest_obj_collision_dis =
                  obj_polygon.DistanceToBoundary({extract_path.at(ie).path_point().x(),
                                                  extract_path.at(ie).path_point().y()}) -
                  vehicle_param_.front_edge_to_center();
            } else {
              trajectory_cost.closest_obj_collision_dis =
                  (ie - ie_step) * params_->path_density() - vehicle_param_.front_edge_to_center() +
                  obj_polygon.DistanceToBoundary({extract_path.at(ie - ie_step).path_point().x(),
                                                  extract_path.at(ie - ie_step).path_point().y()});
            }

            double temp_collision_dis =
                (perception_obs.obj_lon_index() - frame_->MapMatchInfo().lon_index) *
                    params_->path_density() -
                vehicle_param_.front_edge_to_center();

            trajectory_cost.closest_obj_collision_dis =
                std::min(trajectory_cost.closest_obj_collision_dis, temp_collision_dis);
          }
          if (trajectory_cost.closest_obj_collision_dis < 0.0)
            trajectory_cost.closest_obj_collision_dis = 0.0;
          trajectory_cost.lateral_right_cost = 10.0;
          trajectory_cost.lateral_left_cost = 10.0;

          trajectory_cost.closest_obj_id = perception_obs.id();
          // 条件2
          longitudinal_dist = trajectory_cost.closest_obj_collision_dis;
          if (longitudinal_dist != 0) {
            trajectory_cost.longitudinal_cost += 1.0 / fabs(longitudinal_dist);
          } else {
            trajectory_cost.longitudinal_cost = 1000;
          }
          trajectory_cost.closest_obj_distance = longitudinal_dist;
          trajectory_cost.closest_obj_lateral_distance = 0;
          trajectory_cost.closest_obj_velocity = 0;
          trajectory_cost.closest_obj_acceleration = 0;
          return;
        }
      }
    } else {  // 动态障碍物
      for (size_t ie = vehicle_param_.length() / params_->path_density(); ie < extract_path.size();
           ie += ie_step) {
        // double ttt2 = TimeTool::NowToSeconds();
        GetSafetyBorderPolygon(extract_path.at(ie).path_point(), critical_lateral_distance_dynamic,
                               critical_long_front_distance, critical_long_back_distance,
                               adc_polygon);
        PathPoint collision_point;

        // double tt3 = TimeTool::NowToSeconds();
        // AINFO << "------------ tt3-ttt2: " << tt3 - ttt2 << endl;
        if (CalculateDynamicObjPolygonCollision(obj_list.at(io), extract_path.at(ie), adc_polygon,
                                                collision_point, trajectory_cost,
                                                col_obj_polygon)) {
          // 路口直行，若其它车辆左转，且速度低于本车速度并晚于本车到达碰撞点，则判定为无碰撞风险
          if (current_road.road_type() == 2 &&
              obj_list.at(io).intent().type() ==
                  legionclaw::interface::ObstacleIntent::Type::INTENT_TURN_LEFT_STATE) {
            if (frame_->VehicleState().speed > obj_speed &&
                trajectory_cost.closest_obj_collisiontime <= -0.5) {
              InitTrajectoryCost(trajectory_cost);
              break;
            }
          }
          // 最近距离
          for (int ipoint = 0; ipoint < col_obj_polygon.num_points(); ++ipoint) {
            double distance;
            Vec2d x_y = col_obj_polygon.points().at(ipoint);
            int ipm =
                MapMatcher::QueryVerticalDistanceWithBuffer(extract_path, x_y, 1.0e-6, distance);
            if (ipm < 0) {
              continue;
            }
            if (fabs(distance) < fabs(dis_min_so)) {
              dis_min_so = distance;
            }
          }
          trajectory_cost.closest_obj_id = perception_obs.id();
          trajectory_cost.closest_obj_collision_dis =
              (ie - ie_step) * params_->path_density() - vehicle_param_.front_edge_to_center() +
              col_obj_polygon.DistanceToBoundary(
                  {extract_path.at(ie - ie_step).path_point().x(),
                   extract_path.at(ie - ie_step).path_point().y()});  // 车头距障碍物最近边的距离
          if (trajectory_cost.closest_obj_collision_dis < 0.0)
            trajectory_cost.closest_obj_collision_dis = 0.0;

          // double tt4 = TimeTool::NowToSeconds();
          // AINFO << "tt4-tt3: " << tt4 - tt3 << endl;
          if (trajectory_cost.closest_obj_collision_dis > 0) {
            longitudinal_dist =
                MapMatcher::GetExactDistanceOnTrajectory(extract_path, imatched_adc, imatched_obj);
            // TODO 待优化
            longitudinal_dist = longitudinal_dist - vehicle_param_.front_edge_to_center() -
                                perception_obs.length() / 2;
          } else {
            longitudinal_dist = 0;
          }
          if (fabs(trajectory_cost.closest_obj_theta_abs) > 0.75 * M_PI) {
            trajectory_cost.closest_obj_distance = longitudinal_dist;
          } else {
            trajectory_cost.closest_obj_distance =
                min(longitudinal_dist, trajectory_cost.closest_obj_collision_dis);
          }
          if ((imatched_obj != int(extract_path.size() - 1)) /*&&
              imatched_obj != 0*/) {
            trajectory_cost.closest_obj_lateral_distance = std::fabs(dis_min_so);
          }
          // 路口动态障碍物暂不考虑换道
          if (current_road.road_type() == 2) {
            return;
          }
          // collision point for visualize
          collision_points_.push_back(collision_point);
          auto iter = obstacles_lane_id.find(perception_obs.id());
          int obs_lane_id = -1;
          // 障碍物速度较慢且距离较近（跟车距离配置待优化），则增加代价
          // 细分障碍物速度，
          double final_length = 5 + frame_->AheadMapLimitSpeed() * 2.5;
          if (iter != obstacles_lane_id.end()) {
            obs_lane_id = iter->second;
            double follow_v = obj_speed;
            if (follow_v <
                    frame_->AheadMapLimitSpeed() * evaluator_params_->forward_obs_speed_ratio() &&
                obs_lane_id == index) {
              // case1 TODO 距离较近 ,换道跟车距离条件优化
              // TODO 待细化
              // case2 急刹换道场景，障碍物速度慢，且与本车速度相差较大
              if (trajectory_cost.closest_obj_distance < final_length ||
                  (follow_v < 8 && (follow_v - frame_->VehicleState().speed) < -2)) {
                trajectory_cost.blocked_cost = 500;
              }
            }
          }
          if (index == 1) {
            if (trajectory_cost.dynamic_objects_pose == DynamicObjectsPose::D_OBJ_FORWARD ||
                trajectory_cost.dynamic_objects_pose == DynamicObjectsPose::D_OBJ_LEFTUP ||
                trajectory_cost.dynamic_objects_pose == DynamicObjectsPose::D_OBJ_RIGHTUP) {
              return;
            }
          } else if (obs_lane_id == index) {
            return;
          }
          if (frame_->VehicleState().speed > obj_speed) {
            return;
          }
          break;
        }
      }
    }
  }
}

void TrajectoryEvaluator::CalculateLateralCosts(const std::vector<TrajectoryPoint> &extract_path,
                                                const std::vector<PredictionObstacleMsg> &obj_list,
                                                double &lateral_right_cost,
                                                double &lateral_left_cost, double &blocked_cost)
{
  double lateralDist_l = DBL_MAX;
  double lateralDist_r = -DBL_MAX;
  for (size_t io = 0; io < obj_list.size(); ++io)  // objects list loop
  {
    double vx = obj_list.at(io).perception_obstacle().velocity().x();
    double vy = obj_list.at(io).perception_obstacle().velocity().y();
    double obj_speed = Norm(vx, vy);
    if (obj_speed > params_->threshold_static_speed()) {
      continue;
    } else {
      Vec2d contour_pose;
      // #pragma omp parallel for num_threads(4)
      int contour_num = obj_list.at(io).perception_obstacle().polygon().num_points();
      if (contour_num < 3) continue;
      for (int ic = 0; ic < contour_num; ++ic) {
        double delta_lat_dis = DBL_MAX;
        // TODO:提前计算？
        contour_pose = Vec2d(obj_list.at(io).perception_obstacle().polygon().points().at(ic).x(),
                             obj_list.at(io).perception_obstacle().polygon().points().at(ic).y());
        // double theta = obj_list.at(io).perception_obstacle().theta();
        // match the closest point of object to rollout,
        // and calculate the distance
        if (MapMatcher::QueryNearestPointWithBuffer(extract_path, contour_pose, 1.0e-6,
                                                    delta_lat_dis) < 0) {
          continue;
        }
        if (delta_lat_dis < lateralDist_l && delta_lat_dis > 0) {
          lateralDist_l = delta_lat_dis;
        }
        if (delta_lat_dis > lateralDist_r && delta_lat_dis < 0) {
          lateralDist_r = delta_lat_dis;
        }
      }
    }
  }
  lateral_right_cost = 1.0 / -lateralDist_r;
  lateral_left_cost = 1.0 / lateralDist_l;
  blocked_cost = 0;
}

void TrajectoryEvaluator::FilterObstacleBackVehicle(
    const std::vector<TrajectoryPoint> &extract_path, std::vector<PredictionObstacle> &obj_list)
{
  double dis_min = DBL_MAX;
  int imatched_adc = MapMatcher::QueryNearestPointWithBuffer(
      extract_path, Vec2d(frame_->VehicleState().pose.x(), frame_->VehicleState().pose.y()),
      frame_->VehicleState().pose.theta(), 1.0e-6, dis_min);
  if (imatched_adc < 0) return;

  Polygon2d vehicle_polygon;
  double critical_lateral_distance =
      0.5 * vehicle_param_.width() + evaluator_params_->safe_width_real_time();
  // 定位输出点到车头的距离,定位一般定义在后轴中心
  double critical_long_front_distance =
      vehicle_param_.front_edge_to_center() + evaluator_params_->safe_length_real_time();
  // 定位输出点到车尾的距离
  double critical_long_back_distance =
      vehicle_param_.back_edge_to_center() + evaluator_params_->safe_length_real_time();
  GetSafetyBorderPolygon(frame_->VehicleState().pose, critical_lateral_distance,
                         critical_long_front_distance, critical_long_back_distance,
                         vehicle_polygon);
  double boundary_min, boundary_max;
  vector<double> boundary_dis;
  boundary_dis.clear();
  // PathPoint vehicle_point;
  for (size_t i = 0; i < vehicle_polygon.points().size(); ++i) {
    double distance = DBL_MAX;
    int imatched = MapMatcher::QueryNearestPointWithBuffer(
        extract_path, Vec2d(vehicle_polygon.points().at(i).x(), vehicle_polygon.points().at(i).y()),
        frame_->VehicleState().pose.theta(), 1.0e-6, distance);
    if (imatched >= 0) boundary_dis.push_back(distance);
  }

  sort(boundary_dis.begin(), boundary_dis.end());
  boundary_min = boundary_dis.front();
  boundary_max = boundary_dis.back();
  // std::cout<< "min distace : " << boundary_min << " , max distace : " <<
  // boundary_max <<"\n";

  std::vector<PredictionObstacle> temp_obj_list;
  temp_obj_list.clear();
  for (size_t k = 0; k < obj_list.size(); ++k) {
    double distance = DBL_MAX;
    int imatched = MapMatcher::QueryNearestPointWithBuffer(
        extract_path,
        Vec2d(obj_list.at(k).perception_obstacle().position().x(),
              obj_list.at(k).perception_obstacle().position().y()),
        1.0e-6, distance);
    if (imatched < imatched_adc && distance > boundary_min && distance < boundary_max)
      continue;
    else
      temp_obj_list.push_back(obj_list.at(k));
  }
  obj_list.swap(temp_obj_list);
  // std::cout << "After Filter, Predicted Objects size = " << obj_list.size()
  // << "\n";
}

double TrajectoryEvaluator::CalculatePotentialCosts(
    const PotentialField &potential_field, const std::vector<TrajectoryPoint> &extract_path)
{
  if (potential_field.GridsSize() < 1) {
    return 0;
  }
  double potential_cost = 0.0;
  vector<double> lats, lons;
  for (int ilat = num_col_ - 1; ilat >= 0; --ilat) {
    lats.push_back(potential_field.Grid(ilat).d);
  }
  for (int ilon = 0; ilon < num_row_; ++ilon) {
    lons.push_back(potential_field.Grid(ilon * num_col_).s);
  }
  int ir_step = vehicle_param_.width() / params_->path_density();

  int index_start = 0;
  for (size_t ir = 0; ir < extract_path.size(); ir += ir_step) {
    // transfer x/y to s/d
    double dis_min = DBL_MAX;
    Vec2d point = Vec2d(extract_path.at(ir).path_point().x(), extract_path.at(ir).path_point().y());
    // 需要考虑没有匹配上的情况吗？轨迹会超过参考线吗？
    std::vector<ReferencePoint> trajectory;
    int imatched = MapMatcher::QueryNearestPointWithBuffer(potential_field.RefGlobalPath(), point,
                                                           extract_path.at(ir).path_point().theta(),
                                                           1.0e-6, dis_min, index_start);
    index_start = imatched;
    // AINFO << " INDEX START : " << index_start << "\n";

    if (imatched < 0) {
      // std::cout << "-------- trajectory point project potential field,
      // matched failed! -------- imatched: " << imatched << "\n";
      break;
    }
    reference_line::ReferencePoint matched_point = potential_field.RefGlobalPathPoint(imatched);
    double dx = extract_path.at(ir).path_point().x() - matched_point.x();
    double dy = extract_path.at(ir).path_point().y() - matched_point.y();
    double cos_theta_r = std::cos(matched_point.theta());
    double sin_theta_r = std::sin(matched_point.theta());
    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    double d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    double s = matched_point.s();
    auto d_addr = std::lower_bound(lats.begin(), lats.end(), d);
    size_t d_index = std::max(int(d_addr - lats.begin()) - 1, 0);
    if (d_index + 1 < lats.size() && fabs(d - lats.at(d_index + 1)) < fabs(d - lats.at(d_index)))
      d_index = d_index + 1;
    d_index = lats.size() - 1 - d_index;
    auto s_addr = std::lower_bound(lons.begin(), lons.end(), s);
    size_t s_index = std::max(int(s_addr - lons.begin()) - 1, 0);
    if (s_index + 1 < lons.size() && fabs(s - lons.at(s_index + 1)) < fabs(s - lons.at(s_index)))
      s_index = s_index + 1;
    int final_index = (num_col_ * s_index + d_index >= (size_t)potential_field.GridsSize())
                          ? (potential_field.GridsSize() - 1)
                          : (num_col_ * s_index + d_index);
    potential_cost += potential_field.Grid(final_index).potential_val;
  }
  return potential_cost;
}

void TrajectoryEvaluator::CalculatePassageCosts(std::map<int, bool> is_passages_safe_f,
                                                std::map<int, bool> is_passages_safe_b,
                                                std::map<int, bool> is_passages_safe_dclc)
{
  // passages loop, reject trajectories in the passage unsafe
  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    if (trajectory_array_.at(i).size() == 0) continue;
    int global_index = trajectory_array_.at(i).at(0).global_index();
    if (is_passages_safe_dclc[global_index] == false) {
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        // TrajectoryCost trajectory_cost;
        // InitTrajectoryCost(trajectory_cost);
        trajectory_array_.at(i).at(j).set_is_passage_safe_dclc(false);
      }
      // continue;
    }
    if (is_passages_safe_f[global_index] == false) {
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        trajectory_array_.at(i).at(j).set_is_passage_safe_f(false);
      }
    }
    if (is_passages_safe_b[global_index] == false) {
      for (size_t j = 0; j < trajectory_array_.at(i).size(); ++j) {
        trajectory_array_.at(i).at(j).set_is_passage_safe_b(false);
      }
    }
  }
}

std::vector<TimeConsume> TrajectoryEvaluator::ComputeCostsDynamic(Frame *frame)
{
  std::vector<TimeConsume> time_consume_vec;
  // double t1 = TimeTool::NowToSeconds();
  InitializeTrajectoryCosts();

  CalculateRightmostCosts();

  CalculatePriorityCosts(evaluator_params_->bias_prefer_current_lane());

  potential_field_.CalcRefLineRemainLen(frame);
  if (frame->GetTrafficEvents().junction_info().direction_flag()) {
    CalculateJunctionCloseCosts(frame->GetTrafficEvents().junction_info());
  }

  CalculateCurvatureCosts(vehicle_param_.min_turning_radius());

  CalculateTransitionCosts();

  std::map<int, bool> is_passages_safe_f;
  std::map<int, bool> is_passages_safe_b;
  std::map<int, bool> is_passages_safe_dclc;
  std::map<int, int> obstacles_lane_id;
  potential_field_.EvaluateSidePassageSafety(frame, params_, evaluator_params_, is_passages_safe_f,
                                             is_passages_safe_b, is_passages_safe_dclc,
                                             obstacles_lane_id);

  CalculatePassageCosts(is_passages_safe_f, is_passages_safe_b, is_passages_safe_dclc);

  double t2 = TimeTool::NowToSeconds();
  // AINFO << "t2-t1: " << t2 - t1 << endl;

  for (size_t i = 0; i < trajectory_array_.size(); ++i) {
    if (trajectory_array_.at(i).size() == 0) continue;

    for (size_t k = 0; k < trajectory_array_.at(i).size(); ++k) {
      if (trajectory_array_.at(i).at(k).trajectory_points_size() == 0) {
        continue;
      }
      std::vector<TrajectoryPoint> extract_path;
      extract_path = trajectory_array_.at(i).at(k).trajectory_points();

      // double t3 = TimeTool::NowToSeconds();
      // 对每条轨迹投影到势场图赋值代价值, m_WeightPot > 0 则开启横向cost计算
      if (evaluator_params_->weight_pot() && potential_field_.GridsSize() > 0) {
        double potential_cost = CalculatePotentialCosts(potential_field_, extract_path);
        trajectory_array_.at(i).at(k).set_potential_cost(potential_cost);
      }

      // double t4 = TimeTool::NowToSeconds();
      // AINFO << "each trajectory calculate potential costs : " << t4 - t3
      //       << endl;
      // 对每条轨迹进行碰撞检测
      TrajectoryCost trajectory_cost;
      InitTrajectoryCost(trajectory_cost);
      Polygon2d col_obj_polygon_temp;
      // double tt5 = TimeTool::NowToSeconds();
      CaluculateCollisionCosts(extract_path, frame->ObstacleList(), i, obstacles_lane_id,
                               frame_->GetGuideInfo().current_road(), trajectory_cost,
                               col_obj_polygon_temp);
      if (i == 1) {
        frame->set_col_obj_polygon(col_obj_polygon_temp);
      }
      // double t5 = TimeTool::NowToSeconds();
      // AINFO << "each trajectory calculate collision : " << t5 - tt5 << endl;
      trajectory_array_.at(i).at(k).set_id(trajectory_cost.id);
      trajectory_array_.at(i).at(k).set_closest_obj_collisiontime(
          trajectory_cost.closest_obj_collisiontime);
      trajectory_array_.at(i).at(k).set_is_fully_block(trajectory_cost.is_fully_block);
      trajectory_array_.at(i).at(k).set_blocked_cost(trajectory_cost.blocked_cost);
      trajectory_array_.at(i).at(k).set_lateral_right_cost(trajectory_cost.lateral_right_cost);
      trajectory_array_.at(i).at(k).set_lateral_left_cost(trajectory_cost.lateral_left_cost);
      trajectory_array_.at(i).at(k).set_closest_obj_collision_dis(
          trajectory_cost.closest_obj_collision_dis);
      trajectory_array_.at(i).at(k).set_closest_obj_distance(trajectory_cost.closest_obj_distance);
      trajectory_array_.at(i).at(k).set_closest_obj_lateral_distance(
          trajectory_cost.closest_obj_lateral_distance);
      trajectory_array_.at(i).at(k).set_closest_obj_velocity(trajectory_cost.closest_obj_velocity);
      trajectory_array_.at(i).at(k).set_closest_obj_acceleration(
          trajectory_cost.closest_obj_acceleration);
      trajectory_array_.at(i).at(k).set_closest_obj_theta_abs(
          trajectory_cost.closest_obj_theta_abs);
      trajectory_array_.at(i).at(k).set_dynamic_objects_pose(trajectory_cost.dynamic_objects_pose);
      if (trajectory_cost.closest_obj_id != -1)
        trajectory_array_.at(i).at(k).set_closest_obj_id(trajectory_cost.closest_obj_id);

      // int64_t t6 = TimeTool::Now2Ms();

      if (trajectory_array_.at(i).at(k).trajectory_cost().is_fully_block &&
          trajectory_array_.at(i).at(k).trajectory_cost().blocked_cost > 0) {
        // trajectory_costs_.at(ir).lateral_right_cost = 1000.0;
        // trajectory_costs_.at(ir).lateral_left_cost 	= 1000.0;
        // trajectory_costs_.at(index_rollout).blocked_cost = 1000;
        continue;
      } else {
        // 如果轨迹没有碰撞，计算周围障碍物对轨迹的影响lat_cost, m_WeightLat > 0
        // 则开启横向cost计算
        if (evaluator_params_->weight_lat()) {
          double lateral_right_cost, lateral_left_cost, blocked_cost;
          CalculateLateralCosts(extract_path, frame->ObstacleList(), lateral_right_cost,
                                lateral_left_cost, blocked_cost);
          trajectory_array_.at(i).at(k).set_lateral_right_cost(lateral_right_cost);
          trajectory_array_.at(i).at(k).set_lateral_left_cost(lateral_left_cost);
          trajectory_array_.at(i).at(k).set_blocked_cost(blocked_cost);
        }
      }
    }
  }
  CalculateHorizonCosts(frame->ReferenceLines());
  double t7 = TimeTool::NowToSeconds();
  TimeConsume delta_t3;
  delta_t3.set_time_consume(t7 - t2);
  delta_t3.set_name("calculate obstalce cost");
  time_consume_vec.emplace_back(delta_t3);
  return time_consume_vec;
}

}  // namespace planning
}  // namespace legionclaw
