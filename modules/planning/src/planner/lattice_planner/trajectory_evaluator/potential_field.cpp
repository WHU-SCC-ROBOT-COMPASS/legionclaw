#include "potential_field.h"

namespace legionclaw {
namespace planning {

void PotentialField::InitPotentialFieldFromFrenet(
    const Frame* frame, const PlanningConf* params,
    const TrajectoryEvaluatorConf* evaluator_params, int& num_row, int& num_col,
    std::vector<double>& min_max_x_y) {
  std::vector<reference_line::ReferenceLine> reference_lines;
  reference_lines = frame->ReferenceLines();
  if (reference_lines.size() <= 0) return;
  // 地图参考线（最左边道）匹配, 势场范围选取（长度参考裁剪后碰撞检测轨迹）
  ref_global_path_.clear();
  ref_global_path_.shrink_to_fit();
  length_front_ =
      params->baselen_collison_check() +
      frame->VehicleState().speed * params->speedcoeff_collision_check();
  // 势场前方不小于50m
  double len_num = length_front_ > 50 ? length_front_ : 50;

  int i_match_index = 0, i_longest = -1;
  double longest_remain = 0;
  for (auto reference_line : reference_lines) {
    // TODO:MapMatchInfo匹配误差可能较大，需要优化
    double length_remain;
    int match_index_temp = frame->MapMatchInfo().match_index[i_match_index];
    if (match_index_temp < 0) {
      length_remain = 0;
    } else {
      length_remain = reference_line.ReferencePoints().back().s() -
                      reference_line.ReferencePoints().at(match_index_temp).s();
    }
    vec_length_remain_.push_back(length_remain);
    if (length_remain > longest_remain) {
      longest_remain = length_remain;
      i_longest = i_match_index;
    }
    i_match_index++;
  }
  if (longest_remain > len_num) {
    ref_global_path_ = reference_lines.at(i_longest).ReferencePoints();
  } else {
    // AERROR << "ALL REFERENCE LINES ARE TOO SHORT TO CREATE POTENTIAL FIELD !"
    //        << endl;
    return;
  }

  // double close_distance = DBL_MAX;
  int close_index = frame->MapMatchInfo().match_index[i_longest];
  int len_front = len_num;
  int len_back =
      evaluator_params->length_back_vehicle_potential() / params->path_density();
  ReferencePoint matched_pos_vehicle = ref_global_path_.at(close_index);
  ReferencePoint back_pos = close_index - len_back > 0
                                ? ref_global_path_.at(close_index - len_back)
                                : ref_global_path_.at(0);
  ReferencePoint front_pos =
      (size_t)(close_index + len_front) < ref_global_path_.size()
          ? ref_global_path_.at(close_index + len_front)
          : ref_global_path_.back();

  // sampling
  std::vector<std::pair<double, double>> sample_points;
  std::vector<std::vector<std::pair<double, double>>> sample_matrix;
  std::vector<PathPoint> matched_points;
  std::vector<double> vec_x, vec_y;
  double l_x, l_y, r_x, r_y;
  double boundary_left = fabs(matched_pos_vehicle.left_road_width());
  double boundary_right = -fabs(matched_pos_vehicle.right_road_width());
  for (int i = i_longest - 1; i > -1; --i) {
    if (reference_lines.at(i).ReferencePointSize() > 0)
      boundary_left +=
          (fabs(reference_lines.at(i).ReferencePoints(0).left_road_width()) +
           fabs(reference_lines.at(i).ReferencePoints(0).right_road_width()));
  }
  for (size_t i = i_longest + 1; i < reference_lines.size(); ++i) {
    if (reference_lines.at(i).ReferencePointSize() > 0)
      boundary_right +=
          (-fabs(reference_lines.at(i).ReferencePoints(0).left_road_width()) -
           fabs(reference_lines.at(i).ReferencePoints(0).right_road_width()));
  }
  double step_lon = evaluator_params->scale_grid_gampling();
  double step_lat = evaluator_params->scale_grid_gampling();
  for (double lon = back_pos.s(); lon <= front_pos.s(); lon += step_lon) {
    PathPoint matched_point =
        MapMatcher::QueryNearestPoint(ref_global_path_, lon);
    matched_points.push_back(matched_point);
    sample_points.clear();
    for (double left_offset = step_lat; left_offset <= boundary_left;
         left_offset += step_lat)  // same longitudinal, sampling left lateral
    {
      sample_points.push_back(make_pair(lon, left_offset));
    }
    std::reverse(sample_points.begin(), sample_points.end());
    for (double right_offset = 0; right_offset >= boundary_right;
         right_offset -= step_lat)  // same longitudinal, sampling right lateral
    {
      sample_points.push_back(make_pair(lon, right_offset));
    }
    sample_matrix.push_back(sample_points);
    // get boundary points
    l_x = matched_point.x() - sin(matched_point.theta()) * boundary_left;
    l_y = matched_point.y() + cos(matched_point.theta()) * boundary_left;
    vec_x.push_back(l_x);
    vec_y.push_back(l_y);
    r_x = matched_point.x() - sin(matched_point.theta()) * boundary_right;
    r_y = matched_point.y() + cos(matched_point.theta()) * boundary_right;
    vec_x.push_back(r_x);
    vec_y.push_back(r_y);
  }
  min_max_x_y.clear();
  std::sort(vec_x.begin(), vec_x.end());
  std::sort(vec_y.begin(), vec_y.end());
  min_max_x_y.push_back(vec_x.front());
  min_max_x_y.push_back(vec_x.back());
  min_max_x_y.push_back(vec_y.front());
  min_max_x_y.push_back(vec_y.back());
  // std::cout<< "reference point yaw : " << matched_points.at(0).theta()
  // <<"\n";

  // Frenet to Cartesian
  grids_.clear();
  grids_.shrink_to_fit();
  num_row = sample_matrix.size();
  num_col = sample_matrix.at(0).size();
  for (size_t irow = 0; irow < sample_matrix.size(); ++irow) {
    // int size_col = sample_matrix.at(irow).size();
    double rx = matched_points.at(irow).x();
    double ry = matched_points.at(irow).y();
    double rtheta = matched_points.at(irow).theta();
    for (size_t icol = 0; icol < sample_matrix.at(irow).size(); ++icol) {
      SingleGrid single_grid;
      single_grid.s = sample_matrix.at(irow).at(icol).first;
      // std::cout<< "single_grid.s : " << single_grid.s <<"\n";
      single_grid.d = sample_matrix.at(irow).at(icol).second;
      // std::cout<< "single_grid.d : " << single_grid.d <<"\n";
      single_grid.x = rx - sin(rtheta) * single_grid.d;
      single_grid.y = ry + cos(rtheta) * single_grid.d;

      single_grid.obj_potential_val = 0;
      grids_.push_back(single_grid);
    }
  }
}

void PotentialField::CalculatePotentialField(
    const std::vector<PredictionObstacleMsg>& obj_list,
    const VehicleState& vehicle_state,
    const TrajectoryEvaluatorConf* evaluator_params,
    const double& threshold_static_speed) {
  double limit_distance = 0.5;
  double max_potential_val = 10 * evaluator_params->coeff_object_potential();
  for (size_t i = 0; i < grids_.size(); ++i) {
    // int num_valid_objects = 0;
    // filter zone out of road
    if (grids_.at(i).potential_val < 0) {
      continue;
    }
    // calculate potential field caused by obstacle
    double coeff_object_potential = evaluator_params->coeff_object_potential();
    double range_object_potential =
        evaluator_params->object_impact_real_time_range();  // unit meter
    for (size_t io = 0; io < obj_list.size(); ++io) {
      double obj_speed =
          Norm(obj_list.at(io).perception_obstacle().velocity().x(),
               obj_list.at(io).perception_obstacle().velocity().y());
      if (obj_speed > threshold_static_speed) continue;
      Vec2d cell_center(grids_.at(i).x, grids_.at(i).y);
      double distance_min = 0.0;
      if (obj_list.at(io)
              .perception_obstacle()
              .polygon()
              .line_segments()
              .size() == 0) {
        continue;
      } else {
        distance_min =
            obj_list.at(io).perception_obstacle().polygon().DistanceTo(
                cell_center);  // TODO:maybe use overlap replace above check
                               // step
      }
      if (distance_min < limit_distance) {
        distance_min = limit_distance;
      }

      double val = coeff_object_potential *
                   (1 / distance_min - 1 / range_object_potential) /
                   pow(distance_min, 2);
      if (val < 0) {
        val = 0;
      }

      grids_.at(i).obj_potential_val += val;
      // num_valid_objects++;
    }
    if (grids_.at(i).obj_potential_val >
        max_potential_val)  // TODO:需要处理的更合理些
    {
      grids_.at(i).obj_potential_val = max_potential_val;
    }
    // 将栅格距离车辆的远近纳入评估条件
    double w = 1;
    Vec2d grid_2d(grids_.at(i).x, grids_.at(i).y);
    double distance_grid_vehicle =
        Norm(vehicle_state.pose.x() - grids_.at(i).x,
             vehicle_state.pose.y() - grids_.at(i).y);
    // if (distance_grid_vehicle >= range_object_potential) {
    //   w = 1;
    // } else if (distance_grid_vehicle < range_object_potential &&
    //             distance_grid_vehicle >=
    //             evaluator_params->safe_width_real_time()) {
    //   w = 1 + pow((1 / distance_grid_vehicle - 1 / range_object_potential),
    //   2);
    // } else if (distance_grid_vehicle <
    // evaluator_params->safe_width_real_time()) {
    //   w = 250;
    // }
    if (distance_grid_vehicle > length_front_)
      distance_grid_vehicle = length_front_;
    if (distance_grid_vehicle > 5.0) {
      w = 1.0 - (distance_grid_vehicle - 5.0) / (length_front_ + 0.0);
    } else {
      w = 1.0;
    }
    // AINFO<< "w --   --  " << w << ",  "<< distance_grid_vehicle << ",  "<<
    // length_front_ << "\n";
    grids_.at(i).potential_val = grids_.at(i).obj_potential_val * w;
  }
  // std::cout<< "Finish -- Potential -- Field" <<"\n";
}

void PotentialField::CreateBoundingFiledandProjectPotentialField(
    std::vector<double>& min_max_x_y, int& num_row, int& num_col,
    std::vector<SingleGrid>& final_grids) {
  // create bounding filed
  final_grids.clear();
  double x_min = min_max_x_y.at(0);
  double x_max = min_max_x_y.at(1);
  double y_min = min_max_x_y.at(2);
  double y_max = min_max_x_y.at(3);
  double step_lon = 0.5, step_lat = 0.5;
  std::vector<double> lons, lats;
  for (double lon = y_min; lon < y_max; lon += step_lon) {
    lons.push_back(lon);
    for (double lat = x_min; lat < x_max; lat += step_lat) {
      SingleGrid single_grid;
      single_grid.x = lat;
      single_grid.y = lon;
      single_grid.obj_potential_val = 0;
      single_grid.potential_val = 0;
      final_grids.push_back(single_grid);
    }
  }
  num_row = lons.size();
  num_col = final_grids.size() / lons.size();
  for (int i = 0; i < num_col; ++i) {
    lats.push_back(final_grids.at(i).x);
  }
#if 0
// project potential filed to bonding field
for (size_t j = 0; j < grids_.size(); ++j)  //grids_ loop
{
	// match in x direction, find column index
	double x = grids_.at(j).x;
	// auto x_addr = std::upper_bound(lats.begin(), lats.end(), x);
	auto x_addr = std::lower_bound(lats.begin(), lats.end(), x);
	int x_index = x_addr - lats.begin();
	// std::cout<< "x_index : " << x_index << ", value : "<< *x_addr <<"\n";
	// match in y direction, find row index
	double y = grids_.at(j).y;
	// auto y_addr = std::upper_bound(lons.begin(), lons.end(), y);
	auto y_addr = std::lower_bound(lons.begin(), lons.end(), y);
	int y_index = y_addr - lons.begin();
	// std::cout<< "y_index : " << y_index << ", value : "<< *y_addr <<"\n";

	int final_index = num_col*(y_index-1) + x_index;
	final_grids.at(final_index).potential_val += grids_.at(j).potential_val;
	// std::cout<< "j: " << j << ", value : "<< grids_.at(j).potential_val <<"\n";
	// std::cout<< "final index : " << final_index << ", value : "<< final_grids.at(final_index).potential_val <<"\n";
}
#endif
}

void PotentialField::CalcRefLineRemainLen(const Frame *frame) {
  for (auto reference_line : frame->ReferenceLines()) {
    // TODO:MapMatchInfo匹配误差可能较大，需要优化
    double length_remain = reference_line.remain_mileage();
    if (length_remain < 0) {
      length_remain = 0;
    }
    vec_length_remain_.push_back(length_remain);
  }
}

void PotentialField::EvaluateSidePassageSafety(
    const Frame* frame, const PlanningConf* params,
    const TrajectoryEvaluatorConf* evaluator_params,
    std::map<int, bool>& is_passages_safe_f,
    std::map<int, bool>& is_passages_safe_b,
    std::map<int, bool>& is_passages_safe_dclc,
    std::map<int, int>& obstacles_lane_id) {
  double ego_theta = frame->VehicleState().pose.theta();
  double vehicle_v = frame->VehicleState().speed;
  double k1 = evaluator_params->forward_obs_max_time_dclc();
  double k2 = evaluator_params->forward_obs_min_time_dclc();
  // range C
  double c_upper_bound_dclc =
      evaluator_params->c_upper_length_dclc() + frame->VehicleState().speed * k2;
  double c_lower_bound_dclc = -evaluator_params->c_lower_length_dclc();
  double c_upper_bound =
      evaluator_params->c_upper_length() + frame->VehicleState().speed * k2;
  double c_lower_bound = -evaluator_params->c_lower_length();
  // range B
  double b_length = frame->VehicleState().speed * k1;
  double b_upper_bound_dclc = c_upper_bound_dclc + b_length;
  double b_upper_bound = c_upper_bound + b_length;
  // range D
  double d_length_dclc = evaluator_params->d_length_dclc();
  double d_lower_bound_dclc = c_lower_bound_dclc - d_length_dclc;
  double d_length = evaluator_params->d_length();
  double d_lower_bound = c_lower_bound - d_length;
  // lines loop, choose left and right; [0,1,2] size 3
  for (unsigned int i = 0; i < frame->ReferenceLines().size(); ++i) {
    // check lines validity
    reference_line::ReferenceLine ref_line = frame->ReferenceLines().at(i);
    if (ref_line.ReferencePointSize() < 1) {
      is_passages_safe_f.insert(std::make_pair(i, false));
      is_passages_safe_b.insert(std::make_pair(i, false));
      is_passages_safe_dclc.insert(std::make_pair(i, false));
      continue;
    }

    bool is_safe_dclc = true;
    bool is_safe_f = true;
    bool is_safe_b = true;
    bool is_front_static_block = false;
    // vehicle match line
    int vehicle_index = frame->MapMatchInfo().match_index[i];
    if (vehicle_index == -1) continue;
    double vehicle_s = ref_line.ReferencePoints().at(vehicle_index).s();
    double b_upper_s_dclc = vehicle_s + b_upper_bound_dclc;
    double c_upper_s_dclc = vehicle_s + c_upper_bound_dclc;
    double c_lower_s_dclc =
        vehicle_s + c_lower_bound_dclc > 0 ? vehicle_s + c_lower_bound_dclc : 0;
    double d_lower_s_dclc =
        vehicle_s + d_lower_bound_dclc > 0 ? vehicle_s + d_lower_bound_dclc : 0;
    double b_upper_s = vehicle_s + b_upper_bound;
    double c_upper_s = vehicle_s + c_upper_bound;
    double c_lower_s =
        vehicle_s + c_lower_bound > 0 ? vehicle_s + c_lower_bound : 0;
    double d_lower_s =
        vehicle_s + d_lower_bound > 0 ? vehicle_s + d_lower_bound : 0;
    // dynamic obstacles loop frame->ObstacleList()
    ObstacleMsg obs_f, obs_b;
    double obs_f_s = DBL_MAX, obs_f_r_s = DBL_MAX, obs_b_s = DBL_MAX,
           obs_b_r_s = DBL_MAX;
    for (unsigned int j = 0; j < frame->ObstacleList().size(); ++j) {
      // ignore static obstacle
      ObstacleMsg obs = frame->ObstacleList().at(j).perception_obstacle();
      double obs_v = Norm(obs.velocity().x(), obs.velocity().y());
      if (obs_v < params->threshold_static_speed()) {
        continue;
      }
      double dis_min = DBL_MAX;
      Vec2d obs_pos(obs.position().x(), obs.position().y());
      // TODO 障碍物航向有问题，暂时用自车航向
      int obs_index = MapMatcher::QueryNearestPointWithBuffer(
          ref_line.ReferencePoints(), obs_pos, ego_theta, 1.0e-6, dis_min);
      for (size_t ipoint = 0; ipoint < obs.polygon().num_points(); ++ipoint) {
        double distance;
        Vec2d x_y = obs.polygon().points().at(ipoint);
        int ipm = MapMatcher::QueryNearestPointWithBuffer(
            ref_line.ReferencePoints(), x_y, ego_theta, 1.0e-6, distance);
        if (ipm < 0) {
          continue;
        }
        if (fabs(distance) < fabs(dis_min)) {
          dis_min = distance;
        }
      }
      if (obs_index < 0 ||
          fabs(dis_min) > evaluator_params->passage_safe_lateral_buffer()/*||
          obs.lane_position() == legionclaw::common::LanePosition::EGO_LANE*/)
        continue;  // TODO:后向参考线不够长时，障碍物会被忽视；同时匹配到两条路问题
      // 存放障碍物所在车道序列
      // TODO 由于障碍物的车道线位置会出现异常，增加异常情况车线位置处理
      if (((obs.lane_position() == legionclaw::common::LanePosition::LEFT_LANE ||
            obs.lane_position() ==
                legionclaw::common::LanePosition::LANE_POSITION_UNKNOWN) &&
           i == 0) ||
          ((obs.lane_position() == legionclaw::common::LanePosition::EGO_LANE ||
            obs.lane_position() ==
                legionclaw::common::LanePosition::LANE_POSITION_UNKNOWN) &&
           i == 1) ||
          ((obs.lane_position() == legionclaw::common::LanePosition::RIGHT_LANE ||
            obs.lane_position() ==
                legionclaw::common::LanePosition::LANE_POSITION_UNKNOWN) &&
           i == 2)) {
        obstacles_lane_id.insert(std::make_pair(obs.id(), i));
      }

      // ignore current road
      if (i == 1) {
        is_passages_safe_f.insert(std::make_pair(i, true));
        is_passages_safe_b.insert(std::make_pair(i, true));
        is_passages_safe_dclc.insert(std::make_pair(i, true));
        continue;
      }
      double obs_s = ref_line.ReferencePoints().at(obs_index).s();
      // most interest obstacle
      double relative_s = obs_s - vehicle_s;
      if (relative_s >= 0) {
        if (fabs(relative_s) < obs_f_r_s) {
          obs_f_r_s = fabs(relative_s);
          obs_f_s = obs_s;
          obs_f = obs;
        }
      } else {
        if (fabs(relative_s) < obs_b_r_s) {
          obs_b_r_s = fabs(relative_s);
          obs_b_s = obs_s;
          obs_b = obs;
        }
      }
      // // TODO:第三个条件可能需要紧一些
      // if (obs_v <= params->threshold_static_speed() &&
      //     relative_s >= 0 &&
      //     obs_s <= b_upper_s_dclc) {
      //   is_front_static_block = true;
      //   break;
      // }
    }
    // match obstacle in which range
    double obs_f_v = Norm(obs_f.velocity().x(), obs_f.velocity().y());
    double obs_b_v = Norm(obs_b.velocity().x(), obs_b.velocity().y());
    if (obs_f_s > b_upper_s_dclc) {
      ;
    } else if (obs_f_s > c_upper_s_dclc && obs_f_s <= b_upper_s_dclc) {
      // TODO: 细化ttc
      if (obs_f_v <
          evaluator_params->forward_obs_speed_ratio_dclc() * vehicle_v) {
        is_safe_dclc = false;
      }
    } else if (obs_f_s <= c_upper_s_dclc) {
      if (obs_f_v > params->threshold_static_speed()) is_safe_dclc = false;
    }
    if (obs_b_s > c_lower_s_dclc &&
        obs_b_s <= c_lower_s_dclc + evaluator_params->c_lower_length()) {
      if (obs_b_v > params->threshold_static_speed()) is_safe_dclc = false;
    } else if (obs_b_s > d_lower_s_dclc && obs_b_s <= c_lower_s_dclc) {
      // TODO: 细化ttc
      if (obs_b_v >
          evaluator_params->backward_obs_speed_ratio_dclc() * vehicle_v) {
        is_safe_dclc = false;
      }
    } else if (obs_b_s <= d_lower_s_dclc) {
      ;
    }

    if (obs_f_s > b_upper_s) {
      ;
    } else if (obs_f_s > c_upper_s && obs_f_s <= b_upper_s) {
      // TODO: 细化ttc
      if (obs_f_v < evaluator_params->forward_obs_speed_ratio() * vehicle_v) {
        is_safe_f = false;
      }
    } else if (obs_f_s <= c_upper_s) {
      if (obs_f_v > params->threshold_static_speed()) is_safe_f = false;
    }
    if (obs_b_s > c_lower_s &&
        obs_b_s <= c_lower_s + evaluator_params->c_lower_length()) {
      if (obs_b_v > params->threshold_static_speed()) is_safe_b = false;
    } else if (obs_b_s > d_lower_s && obs_b_s <= c_lower_s) {
      // TODO: 细化ttc
      if (obs_b_v > evaluator_params->backward_obs_speed_ratio() * vehicle_v) {
        is_safe_b = false;
      }
    } else if (obs_b_s <= d_lower_s) {
      double diff_v = obs_b_v - vehicle_v;
      if (diff_v > (vehicle_s - obs_b_s) / 6.0) {
        is_safe_b = false;
      }
      if (obs_b_v > 1.2 * vehicle_v && obs_b_v > 14) {
        is_safe_b = false;
      }
    }
    // set result with road global index
    is_passages_safe_dclc.insert(make_pair(i, is_safe_dclc));
    if (is_front_static_block == true) {
      is_passages_safe_f.insert(make_pair(i, false));
      is_passages_safe_b.insert(make_pair(i, false));
    } else {
      is_passages_safe_f.insert(make_pair(i, is_safe_f));
      is_passages_safe_b.insert(make_pair(i, is_safe_b));
    }
  }
}

}  // namespace planning
}  // namespace legionclaw
