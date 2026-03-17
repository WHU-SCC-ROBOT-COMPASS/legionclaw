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

    void VectorMapPredictor::PredictOSM(const legionclaw::interface::Location &location)
    {
      // double time1=legionclaw::common::TimeTool::NowToSeconds();

      double local_x=location.utm_position().x();
      double local_y=location.utm_position().y();
      if (localiztion_mode_ == 1 && map_mode_ == 0)  // odometry模式，预测信息从gps坐标系转回odometry坐标系
      {
        Eigen::Vector3d pos_in(local_x, local_y, location.utm_position().z());
        Eigen::Vector3d pos_out = transformer_.trans_inverse(pos_in);
        local_x = pos_out(0);
        local_y = pos_out(1);
      }
      lanelet::Lanelets inRegion = hdmap_->laneletLayer.search(lanelet::BoundingBox2d(lanelet::BasicPoint2d(local_x-prediction_conf_.local_map_horizon(),local_y-prediction_conf_.local_map_horizon()), lanelet::BasicPoint2d(local_x+prediction_conf_.local_map_horizon(), local_y+prediction_conf_.local_map_horizon())));
      // double time2=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"111111111111111111111 pred_time  :   "<<time2-time1<<"     seconds "<<std::endl<<std::endl;
      
      lanelet::LaneletMapUPtr local_map=lanelet::utils::createMap(inRegion);

      // double time3=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"222222222222222222222 pred_time  :   "<<time3-time2<<"     seconds "<<std::endl<<std::endl;

      lanelet::routing::RoutingGraphUPtr local_route_map=lanelet::routing::RoutingGraph::build(*local_map, *traffic_rule_);

      // double time4=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"333333333333333333333 pred_time  :   "<<time4-time3<<"     seconds "<<std::endl<<std::endl;

      PredictVehicleOSM(local_map,local_route_map);
      PredictPedestrianOSM(local_map,local_route_map);
      PredictOtherObstacleOSM();

      // double time5=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"444444444444444444444 pred_time  :   "<<time5-time4<<"     seconds "<<std::endl<<std::endl;
    }

    bool VectorMapPredictor::PredictVehicleOSM(const lanelet::LaneletMapUPtr &local_map,const lanelet::routing::RoutingGraphUPtr &local_routing_map)
    {
      std::vector<legionclaw::prediction::PredictionTrajectory> trajectories;
      std::vector<legionclaw::interface::TrajectoryInPrediction> predict_trajectories;
      legionclaw::interface::TrajectoryInPrediction predict_trajectory;
      legionclaw::interface::ObstacleIntent intent;
      // int ped_obstacles_size = vehicle_obstacle_ids_.size();

      // double time1=legionclaw::common::TimeTool::NowToSeconds();

      for (auto iter = vehicle_obstacle_ids_.begin(); iter != vehicle_obstacle_ids_.end(); iter++)
      {

        auto iter_obstacle = GetObstacleWithLRUUpdate(*iter);
        if (iter_obstacle == nullptr || !iter_obstacle->size())
          continue;
        if (filter_mode_by_lane_ == FILTER_LL_RR && 
             (iter_obstacle->front().lane_position() == legionclaw::common::NEXT_LEFT_LANE ||
               iter_obstacle->front().lane_position() == legionclaw::common::NEXT_RIGHT_LANE))  //filter based on LanePosition
          continue;
        // std::set<lanelet::Ids> besides_lanelet;
        if (vector_map_prediction_.PredictVehicle(local_map, local_routing_map, *iter_obstacle, prediction_conf_, filter_mode_, trajectories))
        {

          intent_prediction_.PredictIntentForVehicle(trajectories, prediction_conf_.prediction_horizon() * prediction_conf_.predict_time_interval(), intent);

          predict_trajectories.clear();
          double max_probability = -1;
          int index_max = -1;
          for(unsigned int i = 0; i < trajectories.size(); i++)
          {
            if(trajectories[i].trajectory_in_prediction().probability() > max_probability)
            {
              index_max = i;
              max_probability = trajectories[i].trajectory_in_prediction().probability();
            }
            // predict_trajectories.push_back(trajectories[i].trajectory_in_prediction());
          }
          if (index_max >= 0)
            predict_trajectories.push_back(trajectories[index_max].trajectory_in_prediction());
          auto prediction_obstacle = GetPredictionResult(iter_obstacle->front(), predict_trajectories, intent);

          prediction_obstacles_.add_prediction_obstacles(prediction_obstacle);

          //prediction_obstacles_.set_self_intent(prediction_obstacle);
        }
      }
      // double time2=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"222222222222222222222pred_time  :   "<<time2-time1<<"     seconds "<<std::endl<<std::endl;

      return true;
    }

    bool VectorMapPredictor::PredictPedestrianOSM(const lanelet::LaneletMapUPtr &local_map,const lanelet::routing::RoutingGraphUPtr &local_routing_map)
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
        // std::set<lanelet::Ids> besides_lanelet;
        // int64_t t1 = TimeTool::Now2Ms();  //for test
        if (vector_map_prediction_.PredictPedestrian(local_map, local_routing_map, *iter_obstacle, prediction_conf_, filter_mode_, trajectories))
        {
          // int64_t t2 = TimeTool::Now2Ms();  //for test
          intent_prediction_.PredictIntentForPedestrian(local_map, trajectories, prediction_conf_.prediction_horizon() * prediction_conf_.predict_time_interval(), intent);
          // int64_t t3 = TimeTool::Now2Ms();  //for test

          predict_trajectories.clear();
          for(auto trajectory : trajectories)
          {
            predict_trajectory = trajectory.trajectory_in_prediction();
            predict_trajectories.push_back(predict_trajectory);
          }
          auto prediction_obstacle = GetPredictionResult(iter_obstacle->front(), predict_trajectories, intent);
          // int64_t t4 = TimeTool::Now2Ms();  //for test

          prediction_obstacles_.add_prediction_obstacles(prediction_obstacle);
          // int64_t t5 = TimeTool::Now2Ms();  //for test
          // std::cout << "t12 = " << t2-t1 << std::endl;
          // std::cout << "t23 = " << t3-t2 << std::endl;
          // std::cout << "t34 = " << t4-t3 << std::endl;
          // std::cout << "t45 = " << t5-t4 << std::endl;
        }
      }

      return true;
    }

    bool VectorMapPredictor::PredictOtherObstacleOSM()
    {
      return true;
    }

  } // namespace prediction
} // namespace legionclaw
