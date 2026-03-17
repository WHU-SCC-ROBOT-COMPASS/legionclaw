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
 * @brief This file provides the implementation of the class "VectorMapPredictor".
 */

#include <memory>
#include <utility>

#include "modules/prediction/src/proto/prediction_conf.pb.h"
#include "modules/prediction/src/predictor/vector_map_predictor.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/time/time_tool.h"
#include "modules/prediction/src/common/spline/spline.h"
#include "modules/common/math/curve_math.h"

using namespace legionclaw::common;

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */
namespace legionclaw
{
  namespace prediction
  {
    using namespace legionclaw::interface;

    bool VectorMapPredictor::SmoothCenterLines(
      std::vector<legionclaw::interface::LanePoint> center_line, double delta_mileage,
      std::vector<legionclaw::prediction::LanePointInner> &smooth_line) {
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
    legionclaw::prediction::spline::spline spline_s_x, spline_s_y;

    double last_mileage = center_line[start_pos].mileage();
    double last_theta = center_line[start_pos].theta();
    for (size_t i = 0; i < center_line.size(); i++) {
      //对于中心线相邻之间两点重合的点进行过滤，避免进行平滑处理时，求解失败
      if (i > 0 && center_line[i].mileage() - center_line[i - 1].mileage() <= 0)
        continue;
      if(i == 0 || center_line[i].mileage() - last_mileage > 5.0
      || fabs(center_line[i].theta() - last_theta) > math::D2R(20.0))
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

    double heading = 0;
    double x_s, y_s;
    double first_deriv_x, second_deriv_x, first_deriv_y, second_deriv_y;
    double cs = s[0];
    legionclaw::prediction::LanePointInner smooth_point;
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
      heading = math::GetHeadingRadian(first_deriv_x, first_deriv_y);
      // ks = compute_ks_from_spline(first_deriv_x, second_deriv_x, first_deriv_y,
      // second_deriv_y);
      ks = math::CurveMath::ComputeCurvature(first_deriv_x, second_deriv_x,
                                      first_deriv_y, second_deriv_y);
      smooth_point.set_mileage(cs);
      smooth_point.set_theta(math::NormalizeAngle(heading));
      smooth_point.set_kappa(ks);
      point_3d.set_x(x_s);
      point_3d.set_y(y_s);
      point_3d.set_z(center_line[i].point().z());

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
    
    void VectorMapPredictor::PredictLocalMap(const legionclaw::interface::RoutingResponse &routing)
    {
      // process routing
      // 平滑并插值后的中心线跟roting的lane_list 数量相等
      std::vector<std::vector<legionclaw::prediction::LanePointInner>> smooth_lines;
      std::vector<legionclaw::prediction::LanePointInner> smooth_line;
      double delta_mileage = 1.0;
      for (unsigned int i = 0; i < routing.lane_list_size(); i++) {
        SmoothCenterLines(routing.lane_list(i).lane_points(),
                          delta_mileage, smooth_line);
        smooth_lines.push_back(smooth_line);
  }

      PredictVehicleLocalMap(smooth_lines);
      PredictPedestrianLocalMap(smooth_lines);
      PredictOtherObstacleLocalMap();
    }

    bool VectorMapPredictor::PredictVehicleLocalMap(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines)
    {
      std::vector<legionclaw::prediction::PredictionTrajectory> trajectories;
      std::vector<legionclaw::interface::TrajectoryInPrediction> predict_trajectories;
      legionclaw::interface::TrajectoryInPrediction predict_trajectory;
      legionclaw::interface::ObstacleIntent intent;
      // int ped_obstacles_size = vehicle_obstacle_ids_.size();

      for (auto iter = vehicle_obstacle_ids_.begin(); iter != vehicle_obstacle_ids_.end(); iter++)
      {
        auto iter_obstacle = GetObstacleWithLRUUpdate(*iter);
        if (iter_obstacle == nullptr || !iter_obstacle->size())
          continue;
        if (filter_mode_by_lane_ == FILTER_LL_RR && 
             (iter_obstacle->front().lane_position() == legionclaw::common::NEXT_LEFT_LANE ||
               iter_obstacle->front().lane_position() == legionclaw::common::NEXT_RIGHT_LANE))  //filter based on LanePosition
          continue;
        if (vector_map_prediction_.PredictVehicle(smooth_lines, *iter_obstacle, prediction_conf_, filter_mode_, trajectories))
        {

          intent_prediction_.PredictIntentForVehicle(trajectories, prediction_conf_.prediction_horizon() * prediction_conf_.predict_time_interval(), intent);

          predict_trajectories.clear();
          for(auto trajectory : trajectories)
          {
            predict_trajectory = trajectory.trajectory_in_prediction();
            if(predict_trajectory.probability() > 0.5)
              predict_trajectories.push_back(predict_trajectory);
          }
          auto prediction_obstacle = GetPredictionResult(iter_obstacle->front(), predict_trajectories, intent);

          prediction_obstacles_.add_prediction_obstacles(prediction_obstacle);
        }
      }

      return true;
    }

    bool VectorMapPredictor::PredictPedestrianLocalMap(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines)
    {
      std::vector<legionclaw::prediction::PredictionTrajectory> trajectories;
      std::vector<legionclaw::interface::TrajectoryInPrediction> predict_trajectories;
      legionclaw::interface::TrajectoryInPrediction predict_trajectory;
      legionclaw::interface::ObstacleIntent intent;
      // int ped_obstacles_size = pedestrian_obstacle_ids_.size();

      for (auto iter = pedestrian_obstacle_ids_.begin(); iter != pedestrian_obstacle_ids_.end(); iter++)
      {
        auto iter_obstacle = GetObstacleWithLRUUpdate(*iter);
        if (iter_obstacle == nullptr || !iter_obstacle->size())
          continue;
        if (filter_mode_by_lane_ == FILTER_LL_RR && 
             (iter_obstacle->front().lane_position() == legionclaw::common::NEXT_LEFT_LANE ||
               iter_obstacle->front().lane_position() == legionclaw::common::NEXT_RIGHT_LANE))  //filter based on LanePosition
          continue;
        if (vector_map_prediction_.PredictPedestrian(smooth_lines, *iter_obstacle, prediction_conf_, filter_mode_, trajectories))
        {
          // intent_prediction_.PredictIntentForPedestrian(local_map, trajectories, prediction_conf_.prediction_horizon() * prediction_conf_.predict_time_interval(), intent);

          predict_trajectories.clear();
          for(auto trajectory : trajectories)
          {
            predict_trajectory = trajectory.trajectory_in_prediction();
            predict_trajectories.push_back(predict_trajectory);
          }
          auto prediction_obstacle = GetPredictionResult(iter_obstacle->front(), predict_trajectories, intent);

          prediction_obstacles_.add_prediction_obstacles(prediction_obstacle);
        }
      }

      return true;
    }

    bool VectorMapPredictor::PredictOtherObstacleLocalMap()
    {
      return true;
    }

  } // namespace prediction
} // namespace legionclaw
