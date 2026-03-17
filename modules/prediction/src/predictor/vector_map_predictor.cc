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

    Status VectorMapPredictor::Init(const PredictionConf &prediction_conf)
    {
      prediction_conf_ = prediction_conf;
      // LoadLanelet2Map();

      return Status::Ok();
    }

    common::Status VectorMapPredictor::Predict(const LocalView &local_view)
    {
      // double time1=legionclaw::common::TimeTool::NowToSeconds();
      localiztion_mode_ = local_view.localiztion_mode_;
      map_mode_ = local_view.map_mode_;
      // odometry模式（要使用高精度地图），需要计算RT矩阵
      if (localiztion_mode_ == 1 && map_mode_ == 0)
      {
        Eigen::Quaterniond quaternion(local_view.odometry_.orientation().qw(),  local_view.odometry_.orientation().qx(), 
                                                                          local_view.odometry_.orientation().qy(), local_view.odometry_.orientation().qz());
        transformer_.set_quaternion(quaternion);
        Eigen::Vector3d translation(local_view.odometry_.position().x(), local_view.odometry_.position().y(), local_view.odometry_.position().z());
        transformer_.set_translation(translation);
      }

      // Update history
      pedestrian_obstacle_ids_.clear();
      vehicle_obstacle_ids_.clear();
      other_obstacle_ids_.clear();
      pedestrian_obstacle_ids_.shrink_to_fit();
      vehicle_obstacle_ids_.shrink_to_fit();
      other_obstacle_ids_.shrink_to_fit();
      
      std::vector<legionclaw::interface::Obstacle> obstacle_list;
      local_view.obstacle_list_.obstacle(obstacle_list);
      for (auto obstacle : obstacle_list)
      { 
        double area = obstacle.width() * obstacle.length();
        // cout << "id: " << obstacle.id() << ",w: " << obstacle.width() << ", l: " << obstacle.length() << endl;
        if(/*obstacle.fusion_type() == legionclaw::interface::Obstacle::FusionType::LIDAR && */area <= 0.03)
          continue;
        // if(obstacle.id() != 282 && obstacle.id() != 6193 && obstacle.id() != 6186 && obstacle.id() != 660 && obstacle.id() != 656)
        // if(obstacle.id() != 1837 && obstacle.id() != 6179 && obstacle.id() != 632 && obstacle.id() != 660 && obstacle.id() != 656)
        //   continue;
        if (obstacle.id() != -1)
        {
          if (localiztion_mode_ == 1 && map_mode_ == 0)  // odometry模式，障碍物从odometry坐标系转到gps坐标系（与osm地图匹配）
          {
            legionclaw::interface::Obstacle obstacle_gps;
            TransformObstacleOdom2GPS(obstacle, obstacle_gps);
            InsertObstacleToHistory(obstacle_gps);
          }
          else
            InsertObstacleToHistory(obstacle);
        }
      }

      // double time2=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"11111111111111111111 pred_time  :   "<<time2-time1<<"     seconds "<<std::endl<<std::endl;

      prediction_obstacles_.clear_prediction_obstacles();
      //是否需要check
      //predict with the traditional method with vector map.
      if (localiztion_mode_ == 2)  //lane模式，暂时按照匀速直线运动预测（TODO: 按车道线预测）
        filter_mode_ = NO_MAP;
      else if ( local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_INITIAL_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARKING_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FORWARD_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_BACKWARD_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::PARALLEL_PARKING_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FORWARD_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_BEGIN_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_BACKWARD_FINISH_STATE ||
           local_view.adc_trajectory_.behaviour_lat_state() == legionclaw::interface::ADCTrajectory::BehaviourLatState::VERTICAL_PARKING_FINISH_STATE )
        filter_mode_ = FILTER_NONE;
      else
        filter_mode_ = prediction_conf_.filter_mode();
      filter_mode_by_lane_ = prediction_conf_.filter_mode_by_lane();

      // double time2=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"11111111111111111111 pred_time  :   "<<time2-time1<<"     seconds "<<std::endl<<std::endl;
      if (map_mode_ == 0)
        PredictOSM(local_view.location_);
      else
        PredictLocalMap(local_view.routing_response_);
      // double time3=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"222222222222222222222 pred_time  :   "<<time3-time2<<"     seconds "<<std::endl<<std::endl;
      // std::cout<<"---------------------------------- totL_time  :   "<<time3-time1<<"     seconds "<<std::endl<<std::endl;

      return Status(Status::ErrorCode::OK);
    }

    legionclaw::interface::Point3D Vector3dToPoint3d(Eigen::Vector3d pos_in)
    {
      legionclaw::interface::Point3D pos_out;
      pos_out.set_x(pos_in(0));
      pos_out.set_y(pos_in(1));
      pos_out.set_z(pos_in(2));

      return pos_out;
    }
    bool VectorMapPredictor::TransformObstacleOdom2GPS(const legionclaw::interface::Obstacle &obs_in, legionclaw::interface::Obstacle &obs_out)
    {
      obs_out = obs_in;

      Eigen::Vector3d pos_in(obs_in.center_pos_abs().x(), obs_in.center_pos_abs().y(), obs_in.center_pos_abs().z());
      Eigen::Vector3d pos_out = transformer_.trans_inverse(pos_in);
      obs_out.set_center_pos_abs(Vector3dToPoint3d(pos_out));
      obs_out.set_theta_abs(math::NormalizeAngle(obs_in.theta_abs() - transformer_.get_yaw()));

      // 速度加速度只旋转不平移
      Eigen::Vector3d vel_in(obs_in.velocity_abs().x(), obs_in.velocity_abs().y(), obs_in.velocity_abs().z());
      Eigen::Vector3d vel_out = transformer_.rot_inverse(vel_in);
      obs_out.set_velocity_abs(Vector3dToPoint3d(vel_out));

      Eigen::Vector3d acc_in(obs_in.acceleration_abs().x(), obs_in.acceleration_abs().y(), obs_in.acceleration_abs().z());
      Eigen::Vector3d acc_out = transformer_.rot_inverse(acc_in);
      obs_out.set_acceleration_abs(Vector3dToPoint3d(acc_out));

      return true;
    }

    bool VectorMapPredictor::TransformPredictionPerceptionObstacleGPS2Odom(const legionclaw::interface::PerceptionObstacle &obs_in, 
                                                                            legionclaw::interface::PerceptionObstacle &obs_out)
    {
      obs_out = obs_in;

      Eigen::Vector3d pos_in(obs_in.position().x(), obs_in.position().y(), obs_in.position().z());
      Eigen::Vector3d pos_out = transformer_.trans(pos_in);
      obs_out.set_position(Vector3dToPoint3d(pos_out));
      obs_out.set_theta(math::NormalizeAngle(obs_in.theta() + transformer_.get_yaw()));

      // 速度加速度只旋转不平移
      Eigen::Vector3d vel_in(obs_in.velocity().x(), obs_in.velocity().y(), obs_in.velocity().z());
      Eigen::Vector3d vel_out = transformer_.rot(vel_in);
      obs_out.set_velocity(Vector3dToPoint3d(vel_out));

      Eigen::Vector3d acc_in(obs_in.acceleration().x(), obs_in.acceleration().y(), obs_in.acceleration().z());
      Eigen::Vector3d acc_out = transformer_.rot(acc_in);
      obs_out.set_acceleration(Vector3dToPoint3d(acc_out));

      return true;
    }

    bool VectorMapPredictor::TransformTrajectoriesObstacleGPS2Odom(const double &z,  const std::vector<legionclaw::interface::TrajectoryInPrediction> &in, 
                                                                    std::vector<legionclaw::interface::TrajectoryInPrediction> &out)
    {
      out = in;
      for (unsigned int i = 0; i < out.size(); i++)
      // for (auto trajectory : out)
      {
        std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
        auto points = out[i].trajectory_points();
        for (auto point : points)
        {
          legionclaw::interface::TrajectoryPoint trajectory_point;
          Eigen::Vector3d pos_in(point.path_point().x(), point.path_point().y(), z);
          Eigen::Vector3d pos_trans = transformer_.trans(pos_in);
          legionclaw::interface::PathPoint path_point;
          path_point.set_x(pos_trans(0));
          path_point.set_y(pos_trans(1));
          path_point.set_z(pos_trans(2));
          path_point.set_s(point.path_point().s());
          path_point.set_theta(math::NormalizeAngle(point.path_point().theta() + transformer_.get_yaw()));
          trajectory_point.set_path_point(path_point);
          trajectory_point.set_v(point.v());
          trajectory_point.set_a(point.a());
          trajectory_point.set_relative_time(point.relative_time());
          trajectory_points.push_back(trajectory_point);
        }
        out[i].set_trajectory_points(trajectory_points);
      }


      return true;
    }

    void VectorMapPredictor::LoadLanelet2Map(const double &origin_lat, const double &origin_lon)
    {
      std::cout << "Loaded Lanelet2 OSM Map" << std::endl;
      if (!map_loaded_)
      {
        // double origin_lat = param_.origin_lat();
        // double origin_lon = param_.origin_lon();
        // double origin_lat = 30.425457029885372151;  //TODO:add to config file
        // double origin_lon = 114.09623523096009023;
        // std::string osm_map_path = param_.map_file_path();
        std::string osm_map_path = "../../common/data/map/nad.osm";
        std::cout << osm_map_path << std::endl;
        lanelet::projection::UtmProjector projector(lanelet::Origin({origin_lat, origin_lon}));
        hdmap_ = lanelet::load(osm_map_path, projector);
        lanelet::traffic_rules::TrafficRulesPtr traffic_rule_ = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        routing_map_ = lanelet::routing::RoutingGraph::build(*hdmap_, *traffic_rule_);
        map_loaded_ = true;
        std::cout << "Loaded Lanelet2 OSM Map Succeeded" << std::endl;
      }
    }

    void VectorMapPredictor::InsertObstacleToHistory(const legionclaw::interface::Obstacle &object)
    {
      // Insert the obstacle and update the LRUCache
      int id = (int)object.id();
      auto obstacle_ptr = GetObstacleWithLRUUpdate(id);

      if (obstacle_ptr != nullptr)
      {
        // Insert current info
        obstacle_ptr->emplace_front(object);

        // Trim history
        // auto num_of_frames = obstacle_ptr->size();
        double latest_ts = obstacle_ptr->front().timestamp().sec();
        while (obstacle_ptr->size() > 0 &&
               latest_ts - obstacle_ptr->back().timestamp().sec() >= prediction_conf_.max_history_time())
        {
          obstacle_ptr->pop_back();
        }

        // auto num_of_discarded_frames = num_of_frames - obstacle_ptr->size();
        // if (num_of_discarded_frames > 0) {
        //     // std::cout << "Obstacle [" << id << "] discards " << num_of_discarded_frames
        //     //           << " histories" << std::endl;
        // }
      }
      else
      {
        auto ptr_obstacle = VectorMapPredictor::Create(object);
        if (ptr_obstacle == nullptr)
        {
          std::cout << "Failed to insert obstacle into history" << std::endl;
          return;
        }
        ptr_obstacles_.Put(id, std::move(ptr_obstacle));
        // std::cout << "Insert obstacle [" << id << "]" << std::endl;
      }

      // type name pedestrian6 or vehicle1
      if (object.type() == legionclaw::common::OBSTACLE_PEDESTRIAN ||
          object.sub_type() == legionclaw::common::ST_PEDESTRIAN ||
          object.sub_type() == legionclaw::common::ST_CYCLIST) {
        pedestrian_obstacle_ids_.push_back(id);
      } else if (object.type() == legionclaw::common::OBSTACLE_UNKNOWN ||
                 object.type() == legionclaw::common::OBSTACLE_UNKNOWN_MOVABLE ||
                 object.type() == legionclaw::common::OBSTACLE_UNKNOWN_UNMOVABLE ||
                 object.type() == legionclaw::common::OBSTACLE_VEHICLE ||
                 object.sub_type() == legionclaw::common::ST_CAR ||
                 object.sub_type() == legionclaw::common::ST_VAN ||
                 object.sub_type() == legionclaw::common::ST_TRUCK ||
                 object.sub_type() == legionclaw::common::ST_BUS) {
        vehicle_obstacle_ids_.push_back(id);
      } else {
        other_obstacle_ids_.push_back(id);
      }
    }

    std::deque<legionclaw::interface::Obstacle> *VectorMapPredictor::GetObstacleWithLRUUpdate(const int obstacle_id)
    {
      auto ptr_obstacle = ptr_obstacles_.Get(obstacle_id);
      if (ptr_obstacle != nullptr)
      {
        return ptr_obstacle->get();
      }
      return nullptr;
    }

    std::unique_ptr<std::deque<legionclaw::interface::Obstacle>> VectorMapPredictor::Create(const legionclaw::interface::Obstacle &object)
    {
      std::unique_ptr<std::deque<legionclaw::interface::Obstacle>> ptr_obstacle(new std::deque<legionclaw::interface::Obstacle>());
      ptr_obstacle->emplace_front(object);
      return ptr_obstacle;
    }

    std::vector<legionclaw::interface::Point3D> VectorMapPredictor::ConstructRect(const double &x, const double &y, const double &yaw_rad,
                                                                     const double &width, const double &length)
    {
      std::vector<legionclaw::interface::Point3D> box;
      legionclaw::interface::Point3D p_front_center, p_back_center, p1, p2, p3, p4;
      // double yaw_rad = yaw * M_PI / 180.0;

      p_front_center.set_x(x + 0.5 * length * cos(yaw_rad));
      p_front_center.set_y(y + 0.5 * length * sin(yaw_rad));

      p_back_center.set_x(x - 0.5 * length * cos(yaw_rad));
      p_back_center.set_y(y - 0.5 * length * sin(yaw_rad));

      //front left
      p1.set_x(p_front_center.x() + 0.5 * width * sin(yaw_rad));
      p1.set_y(p_front_center.y() - 0.5 * width * cos(yaw_rad));

      //front right
      p2.set_x(p_front_center.x() - 0.5 * width * sin(yaw_rad));
      p2.set_y(p_front_center.y() + 0.5 * width * cos(yaw_rad));

      //back right
      p3.set_x(p_back_center.x() - 0.5 * width * sin(yaw_rad));
      p3.set_y(p_back_center.y() + 0.5 * width * cos(yaw_rad));

      //back left
      p4.set_x(p_back_center.x() + 0.5 * width * sin(yaw_rad));
      p4.set_y(p_back_center.y() - 0.5 * width * cos(yaw_rad));

      box.push_back(p1);
      box.push_back(p2);
      box.push_back(p3);
      box.push_back(p4);

      return box;
    }

    legionclaw::interface::PerceptionObstacle VectorMapPredictor::ConvertFusedObject2PerceptionObstacle(const legionclaw::interface::Obstacle &fused_object)
    {
      legionclaw::interface::PerceptionObstacle perception_obstacle;

      //#感知障碍物ID
      perception_obstacle.set_id(fused_object.id());
      //#地图坐标系下障碍物的Point位置
      legionclaw::interface::Point3D position;
      position.set_x(fused_object.center_pos_abs().x());
      position.set_y(fused_object.center_pos_abs().y());
      position.set_z(fused_object.center_pos_abs().z());
      perception_obstacle.set_position(position);
      //#地图坐标系下障碍物姿态角，长边与x轴的夹角(rad)
      // perception_obstacle.set_theta(90.0 - common::math::R2D(fused_object.theta_abs()));
      perception_obstacle.set_theta(fused_object.theta_abs());
      //#速度，障碍物在地图坐标系下的速度分量
      legionclaw::interface::Point3D velocity;
      velocity.set_x(fused_object.velocity_abs().x());
      velocity.set_y(fused_object.velocity_abs().y());
      velocity.set_z(fused_object.velocity_abs().z());
      perception_obstacle.set_velocity(velocity);
      // //#是否有速度，默认值为false
      // perception_obstacle.set_has_velocity(false);
      // if (velocity.x() > 0 || velocity.y() > 0)
      //   perception_obstacle.set_has_velocity(true);
      //#障碍物长
      perception_obstacle.set_length(fused_object.length());
      //#障碍物宽
      perception_obstacle.set_width(fused_object.width());
      //#障碍物高
      perception_obstacle.set_height(fused_object.height());
      //#地图坐标系下多边形点集合 //#障碍物bounding_box　按顺时针顺序排列
      std::vector<legionclaw::interface::Point3D> polygon_points;
      fused_object.polygon_point_abs(polygon_points);
      // polygon_points = ConstructRect(position.x(), position.y(), perception_obstacle.theta(),
      //                                perception_obstacle.width(), perception_obstacle.length());
      perception_obstacle.set_polygon_point(&polygon_points);
      perception_obstacle.set_bounding_box(&polygon_points);
      //#踪时间，保留位
      perception_obstacle.set_tracking_time(fused_object.last_updated_time().sec());
      //#障碍物类型
      perception_obstacle.set_type(fused_object.type());
      //车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE
      //2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN
      perception_obstacle.set_lane_position(fused_object.lane_position());
      //#置信度，范围0-1
      perception_obstacle.set_confidence(fused_object.existence_prob());
      //#时间戳，记录GPS时间，以秒为单位
      perception_obstacle.set_timestamp(fused_object.timestamp().sec());
      //#置信度类型:0-CONFIDENCE_UNKNOWN,1-CONFIDENCE_CN,2-CONFIDENCE_RAD
      perception_obstacle.set_confidence_type(0);
      //#障碍物轨迹
      perception_obstacle.set_drops(position);
      //#障碍物加速度
      legionclaw::interface::Point3D acceleration;
      acceleration.set_x(fused_object.acceleration_abs().x());
      acceleration.set_y(fused_object.acceleration_abs().y());
      acceleration.set_z(fused_object.acceleration_abs().z());
      perception_obstacle.set_acceleration(acceleration);
      //#障碍物的锚点
      legionclaw::interface::Point3D anchor_point;
      anchor_point.set_x(fused_object.anchor_point_abs().x());
      anchor_point.set_y(fused_object.anchor_point_abs().y());
      anchor_point.set_z(fused_object.anchor_point_abs().z());
      perception_obstacle.set_anchor_point(anchor_point);
      //#障碍物子类别
      perception_obstacle.set_sub_type(fused_object.sub_type());
      //#障碍物最低点离地垂直高度
      perception_obstacle.set_height_above_ground(position.z() - perception_obstacle.height() * 0.5);
      //#位置协方差，行优先矩阵
      std::vector<double> covariance;
      fused_object.position_abs_covariance(covariance);
      perception_obstacle.set_position_covariance(&covariance);
      //#速度协方差，行优先矩阵
      covariance.clear();
      fused_object.velocity_abs_covariance(covariance);
      perception_obstacle.set_velocity_covariance(&covariance);
      //#加速度协方差，行优先矩阵
      covariance.clear();
      fused_object.acceleration_abs_covariance(covariance);
      perception_obstacle.set_acceleration_covariance(&covariance);
      //#障碍物车辆信号灯状态：0-OFF,1-LEFT_TURN_VISIBLE,2-LEFT_TURN_ON,3-RIGHT_TURN_VISIBLE,4-RIGHT_TURN_ON,5-BRAKE_VISIBLE,6-BRAKE_ON,7-UNKNOWN
      perception_obstacle.set_light_status(fused_object.blinker_flag());

      return perception_obstacle;
    }

    legionclaw::interface::PredictionObstacle VectorMapPredictor::GetPredictionResult(const legionclaw::interface::Obstacle &object,
                            const std::vector<legionclaw::interface::TrajectoryInPrediction> &trajectories,
                            const legionclaw::interface::ObstacleIntent &intent)
    {
      legionclaw::interface::PredictionObstacle prediction_obstacle; // ------ one predicted obstacle
      prediction_obstacle.clear_trajectory();

      //Convert FusedObject to PerceptionObstacle
      legionclaw::interface::PerceptionObstacle perception_obstacle;
      perception_obstacle = ConvertFusedObject2PerceptionObstacle(object);
      std::vector<legionclaw::interface::TrajectoryInPrediction> tmp_trajectories;

      if (localiztion_mode_ == 1 && map_mode_ == 0)  // odometry模式，预测信息从gps坐标系转回odometry坐标系
      {
        legionclaw::interface::PerceptionObstacle perception_obstacle_odom;
        TransformPredictionPerceptionObstacleGPS2Odom(perception_obstacle, perception_obstacle_odom);
        TransformTrajectoriesObstacleGPS2Odom(object.center_pos_abs().z(), trajectories, tmp_trajectories);

        prediction_obstacle.set_perception_obstacle(perception_obstacle_odom);
      }
      else
      {
        tmp_trajectories = trajectories;
        prediction_obstacle.set_perception_obstacle(perception_obstacle);
      }
      prediction_obstacle.set_trajectory(&tmp_trajectories);
      prediction_obstacle.set_timestamp(object.timestamp().nsec());
      prediction_obstacle.set_predicted_period(prediction_conf_.prediction_horizon() * prediction_conf_.predict_time_interval());
      prediction_obstacle.set_intent(intent);
      //差uintent_、priority_、is_static_、feature_;

      return prediction_obstacle;
    }

  } // namespace prediction
} // namespace legionclaw
