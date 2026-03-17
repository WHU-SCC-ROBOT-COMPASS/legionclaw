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
 * @brief This file provides the declaration of the class "VectorMapPredictor".
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/interface/perception_obstacle.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/interface/trajectory_in_prediction.hpp"
#include "modules/common/interface/prediction_obstacles.hpp"
#include "modules/prediction/src/interface/lane_point_inner.hpp"

#include "modules/prediction/src/predictor/predictor.h"
#include "modules/prediction/src/common/local_view.h"
#include "modules/common/interface/obstacle_intent.hpp"
#include "modules/prediction/src/predictor/vector_map_prediction.h"
#include "modules/prediction/src/predictor/intent_prediction.h"
#include "modules/prediction/src/common/transformer.hpp"

#include "modules/prediction/src/common/lru_cache.h"

//ROS加载地图所需头文件
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// #include <lanelet2_visualization.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include "lanelet2_io/Io.h"
#include <UTM.h>

#define NO_MAP -1
#define FILTER_NONE 0
#define FILTER_BY_CENTER_POINT 1
#define FILTER_BY_POLYGON 2

#define FILTER_BY_LANE_NONE 0
#define FILTER_LL_RR 1

#define USE_MAP_THETA 0
#define USE_VELOCITY_THETA 1
#define USE_BOX_THETA 2

// using legionclaw::prediction::Prediction;
/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */
namespace legionclaw
{
  namespace prediction
  {
    using namespace legionclaw::common;
    // class Prediction;

    /**
 * @class VectorMapPredictor
 * @brief VectorMapPredictor is a predictor based on 
 */
    class VectorMapPredictor : public Predictor
    {
    public:
      VectorMapPredictor() = default;

      virtual ~VectorMapPredictor() = default;

      std::string Name() const override { return "VectorMapPredictor"; }

      common::Status Init(const PredictionConf &prediction_conf) override;

      /**
       * @brief Override function Plan in parent class Predictor.
       * @param prediction_init_point The trajectory point where prediction starts.
       * @param frame Current prediction frame.
       * @return OK if prediction succeeds; error otherwise.
       */
      common::Status Predict(const LocalView &local_view) override;

      /**
       * @brief     障碍物数据处理：根据障碍物id将障碍物信息存储到ptr_obstacles_.
       * @param[in] object 某个障碍物信息.
       * @return    void.
       */
      void InsertObstacleToHistory(const legionclaw::interface::Obstacle &object);

      /**
       * @brief     加载地图信息，在地图信息上显示障碍物信息.
       * @param[in] osm_map_path、origin_lat、origin_lon 地图文件位置、原始点经纬度信息.
       * @return    void.
       */
      void LoadLanelet2Map(const double &origin_lat, const double &origin_lon);

      bool TransformObstacleOdom2GPS(const legionclaw::interface::Obstacle &obs_in, legionclaw::interface::Obstacle &obs_out);

      bool TransformPredictionPerceptionObstacleGPS2Odom(const legionclaw::interface::PerceptionObstacle &obs_in, 
                                                                                                                             legionclaw::interface::PerceptionObstacle &obs_out);

      bool TransformTrajectoriesObstacleGPS2Odom(const double &z,  const std::vector<legionclaw::interface::TrajectoryInPrediction> &in, 
                                                                                                         std::vector<legionclaw::interface::TrajectoryInPrediction> &out);

      /**
       * @brief     根据障碍物id从ptr_obstacles_中获取相应的障碍物队列指针.
       * @param[in] obstacle_id 障碍物id.
       * @return    相应的障碍物队列指针.
       */
      std::deque<legionclaw::interface::Obstacle> *GetObstacleWithLRUUpdate(const int obstacle_id);

      /**
       * @brief     新建一个障碍物队列，并将障碍物信息插入到该队列中.
       * @param[in] object 某个障碍物信息.
       * @return    新建的障碍物队列.
       */
      static std::unique_ptr<std::deque<legionclaw::interface::Obstacle>> Create(const legionclaw::interface::Obstacle &object);

      /**
       * @brief     构建一个矩形.
       * @param[in] x 矩形中心点x坐标.
       * @param[in] y 矩形中心点y坐标.
       * @param[in] yaw 矩形方向角（长度方向与北方向夹角，单位：°）.
       * @param[in] width 矩形宽.
       * @param[in] length 矩形长.
       * @return    矩形的四个顶点.
       */
      std::vector<legionclaw::interface::Point3D> ConstructRect(const double &x, const double &y, const double &yaw,
                                                           const double &width, const double &length);

      /**
       * @brief     将FusedObject转换成PerceptionObstacle格式.
       * @param[in] fused_object FusedObject类型的障碍物信息.
       * @return    PerceptionObstacle类型的障碍物信息.
       */
      legionclaw::interface::PerceptionObstacle ConvertFusedObject2PerceptionObstacle(const legionclaw::interface::Obstacle &fused_object);

      /**
       * @brief     根据障碍物信息和预测的所有轨迹，得到PredictionObstacle.
       * @param[in] object 某个障碍物信息.
       * @param[in] trajectories 轨迹预测输出的某个障碍物的所有轨迹.
       * @return    PredictionObstacle.
       */
      legionclaw::interface::PredictionObstacle GetPredictionResult(const legionclaw::interface::Obstacle &object,
                            const std::vector<legionclaw::interface::TrajectoryInPrediction> &trajectories,
                            const legionclaw::interface::ObstacleIntent &intent);
      /**
       * @brief predict with the traditional method with vector map.
       * @return    void.
       */
      void PredictOSM(const legionclaw::interface::Location &location);
      bool PredictVehicleOSM(const lanelet::LaneletMapUPtr &local_map,const lanelet::routing::RoutingGraphUPtr &local_routing_map);
      bool PredictPedestrianOSM(const lanelet::LaneletMapUPtr &local_map,const lanelet::routing::RoutingGraphUPtr &local_routing_map);
      bool PredictOtherObstacleOSM();

      bool SmoothCenterLines(std::vector<legionclaw::interface::LanePoint> center_line, double delta_mileage,
                                                           std::vector<legionclaw::prediction::LanePointInner> &smooth_line);
                                  
      void PredictLocalMap(const legionclaw::interface::RoutingResponse &routing);
      bool PredictVehicleLocalMap(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines);
      bool PredictPedestrianLocalMap(const std::vector<std::vector<legionclaw::prediction::LanePointInner>> &smooth_lines);
      bool PredictOtherObstacleLocalMap();

      void Stop() override {}

      legionclaw::interface::PredictionObstacles PredictionObstacles()  const
      {
        return prediction_obstacles_;
      }

    private:
      prediction::PredictionConf prediction_conf_; //参数

      //map
      bool map_loaded_ = false;
      // map_convert::Converter map_c_;
      lanelet::LaneletMapUPtr hdmap_;
      lanelet::traffic_rules::TrafficRulesPtr traffic_rule_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
      lanelet::routing::RoutingGraphUPtr routing_map_;
      // lanelet::projection::UtmProjector projector;
      // lanelet::LaneletMapUPtr map_;
      // lanelet::routing::RoutingGraphUPtr routing_map_;

      //ros发布routing_map地图信息
      // ros::NodeHandle node_handle_;
      // ros::Publisher map_pub;
      // ros::Publisher prediction_result_pub_;
      // ros::Subscriber current_pose_sub;
      // ros::Subscriber get_locaton_;

      transform::Transformer transformer_;
      int localiztion_mode_ = 0;  //0:ins, 1:odometry, 2:lane
      int map_mode_ = 0;  //0=use hdmap, 1=use localmap

      //数据处理后障碍物按此格式存储
      std::vector<int> pedestrian_obstacle_ids_;                                                                    //行人障碍物id列表
      std::vector<int> vehicle_obstacle_ids_;                                                                       //车辆障碍物id列表
      std::vector<int> other_obstacle_ids_;                                                                         //其他障碍物id列表
      legionclaw::common::util::LRUCache<int, std::unique_ptr<std::deque<legionclaw::interface::Obstacle>>> ptr_obstacles_; //id，障碍物队列
  
      // legionclaw::interface::PredictionOutArray prediction_out_array_;
      legionclaw::interface::PredictionObstacles prediction_obstacles_;
      int filter_mode_by_lane_ = FILTER_BY_LANE_NONE;
      int filter_mode_ = FILTER_NONE;

      //处理方法
      VectorMapPrediction vector_map_prediction_; //基于矢量地图的轨迹预测
      IntentPrediction intent_prediction_;        //意图预测
    };

  } // namespace prediction
} // namespace legionclaw
