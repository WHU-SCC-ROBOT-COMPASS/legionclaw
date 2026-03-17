/**
 * @file    lcm_message_manager.hpp
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "lcm_message_manager.h"
#include "modules/common/macros/macros.h"
#include "modules/common/logging/logging.h"
#include "modules/common/base_message/message.h"
#include "modules/common/math/euler_angles_zxy.h"

#if LCM_ENABLE
/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {
using namespace legionclaw::common;
template <typename T> void LcmMessageManager<T>::Init(T* t) {
  is_init_ = false;
  is_active_ = false;
  instance_ = t;
  std::map<std::string, legionclaw::common::Message> messages =
      instance_->GetConf()->messages();
  lcm_ = std::make_shared<lcm::LCM>(messages["LCM"].url);

  if (!lcm_->good()) {
    AERROR << "lcm init error!";
    return;
  }
  lcm_->subscribe("/vui_client/ObuCmdMsg",
                &LcmMessageManager::HandleObuCmdMsgMessage, this);

  //线程执行开始
  handle_message_thread_.reset(new std::thread([this] { Run(); }));
  if (handle_message_thread_ == nullptr) {
    AERROR << "Unable to create handle_message_thread thread.";
    return;
  }
  is_init_ = true;
}

template <typename T>
bool LcmMessageManager<T>::Activate()
{
    if(is_active_)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    location_sub_ = lcm_->subscribe("/localization/global_fusion/Location",
                    &LcmMessageManager::HandleLocationMessage, this);

    adc_trajectory_sub_ = lcm_->subscribe("/planning/ADCTrajectory",
                    &LcmMessageManager::HandleADCTrajectoryMessage, this);

    obstacle_list_sub_ = lcm_->subscribe("/perception/fusion/motion_manager/MMObstacleList",
                    &LcmMessageManager::HandleObstacleListMessage, this);

    odometry_sub_ = lcm_->subscribe("/localization/local_map_location/Map2LocalTF",
                    &LcmMessageManager::HandleOdometryMessage, this);

    traffic_events_sub_ = lcm_->subscribe("/routing/TrafficEventsLocal",
                    &LcmMessageManager::HandleTrafficEventsMessage, this);

    routing_reponse_input_sub_ = lcm_->subscribe("/local_map/LocalRoutingResponse",
                    &LcmMessageManager::HandleRoutingResponseMessage, this);
    routing_reponse_input_sub_ =
        lcm_->subscribe("/routing/RoutingResponse",
                        &LcmMessageManager::HandleRoutingResponseMessage, this);

    lanelist_input_sub_ = lcm_->subscribe("MFLaneList", &LcmMessageManager::HandleLaneListMessage,
                    this);
    is_active_ = true;
    return true;
}

template <typename T>
bool LcmMessageManager<T>::DeActivate()
{
    if(is_active_==false)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    lcm_->unsubscribe(routing_reponse_input_sub_);
    lcm_->unsubscribe(location_sub_);
    lcm_->unsubscribe(traffic_events_sub_);
    lcm_->unsubscribe(adc_trajectory_sub_);
    lcm_->unsubscribe(lanelist_input_sub_);
    lcm_->unsubscribe(obstacle_list_sub_);
    lcm_->unsubscribe(odometry_sub_);
    is_active_ = false;
    std::cout<<"lcm deactivate"<<std::endl;
    return true;
}

template <typename T>
void LcmMessageManager<T>::PublishPredictionObstacles(
    legionclaw::interface::PredictionObstacles msg) {
  if (is_init_ == false)
    return;
  lcm_interface::PredictionObstacles prediction_obstacles;
  MESSAGE_HEADER_ASSIGN(lcm_interface, prediction_obstacles)
  std::vector<lcm_interface::PredictionObstacle>
      lcm_prediction_obstacles_vector;
  std::vector<legionclaw::interface::PredictionObstacle>
      legion_prediction_obstacles_vector;
  msg.prediction_obstacles(legion_prediction_obstacles_vector);
  for (auto it_prediction_obstacles_vector :
       legion_prediction_obstacles_vector) {
    lcm_interface::PredictionObstacle prediction_obstacles_prediction_obstacle;
    lcm_interface::PerceptionObstacle
        prediction_obstacles_prediction_obstacle_perception_obstacle;
    prediction_obstacles_prediction_obstacle_perception_obstacle.id =
        it_prediction_obstacles_vector.perception_obstacle().id();
    lcm_interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_position;
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.x =
        it_prediction_obstacles_vector.perception_obstacle().position().x();
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.y =
        it_prediction_obstacles_vector.perception_obstacle().position().y();
    prediction_obstacles_prediction_obstacle_perception_obstacle_position.z =
        it_prediction_obstacles_vector.perception_obstacle().position().z();
    prediction_obstacles_prediction_obstacle_perception_obstacle.position =
        prediction_obstacles_prediction_obstacle_perception_obstacle_position;
    prediction_obstacles_prediction_obstacle_perception_obstacle.theta =
        it_prediction_obstacles_vector.perception_obstacle().theta();
    lcm_interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_velocity;
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.x =
        it_prediction_obstacles_vector.perception_obstacle().velocity().x();
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.y =
        it_prediction_obstacles_vector.perception_obstacle().velocity().y();
    prediction_obstacles_prediction_obstacle_perception_obstacle_velocity.z =
        it_prediction_obstacles_vector.perception_obstacle().velocity().z();
    prediction_obstacles_prediction_obstacle_perception_obstacle.velocity =
        prediction_obstacles_prediction_obstacle_perception_obstacle_velocity;
    prediction_obstacles_prediction_obstacle_perception_obstacle.length =
        it_prediction_obstacles_vector.perception_obstacle().length();
    prediction_obstacles_prediction_obstacle_perception_obstacle.width =
        it_prediction_obstacles_vector.perception_obstacle().width();
    prediction_obstacles_prediction_obstacle_perception_obstacle.height =
        it_prediction_obstacles_vector.perception_obstacle().height();
    std::vector<lcm_interface::Point3D> lcm_polygon_point;
    std::vector<legionclaw::interface::Point3D> legion_polygon_point;
    it_prediction_obstacles_vector.perception_obstacle().polygon_point(
        legion_polygon_point);
    for (auto it_polygon_point : legion_polygon_point) {
      lcm_interface::Point3D
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d;
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.x =
          it_polygon_point.x();
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.y =
          it_polygon_point.y();
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.z =
          it_polygon_point.z();
      lcm_polygon_point.emplace_back(
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .polygon_point_size = lcm_polygon_point.size();
    prediction_obstacles_prediction_obstacle_perception_obstacle.polygon_point =
        lcm_polygon_point;
    prediction_obstacles_prediction_obstacle_perception_obstacle.tracking_time =
        it_prediction_obstacles_vector.perception_obstacle().tracking_time();
    prediction_obstacles_prediction_obstacle_perception_obstacle.type =
        it_prediction_obstacles_vector.perception_obstacle().type();
    prediction_obstacles_prediction_obstacle_perception_obstacle.lane_position =
        it_prediction_obstacles_vector.perception_obstacle().lane_position();
    prediction_obstacles_prediction_obstacle_perception_obstacle.confidence =
        it_prediction_obstacles_vector.perception_obstacle().confidence();
    prediction_obstacles_prediction_obstacle_perception_obstacle.timestamp =
        it_prediction_obstacles_vector.perception_obstacle().timestamp();
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .confidence_type =
        it_prediction_obstacles_vector.perception_obstacle().confidence_type();
    lcm_interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_drops;
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.x =
        it_prediction_obstacles_vector.perception_obstacle().drops().x();
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.y =
        it_prediction_obstacles_vector.perception_obstacle().drops().y();
    prediction_obstacles_prediction_obstacle_perception_obstacle_drops.z =
        it_prediction_obstacles_vector.perception_obstacle().drops().z();
    prediction_obstacles_prediction_obstacle_perception_obstacle.drops =
        prediction_obstacles_prediction_obstacle_perception_obstacle_drops;
    lcm_interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration;
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .x =
        it_prediction_obstacles_vector.perception_obstacle().acceleration().x();
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .y =
        it_prediction_obstacles_vector.perception_obstacle().acceleration().y();
    prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration
        .z =
        it_prediction_obstacles_vector.perception_obstacle().acceleration().z();
    prediction_obstacles_prediction_obstacle_perception_obstacle.acceleration =
        prediction_obstacles_prediction_obstacle_perception_obstacle_acceleration;
    lcm_interface::Point3D
        prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point;
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .x =
        it_prediction_obstacles_vector.perception_obstacle().anchor_point().x();
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .y =
        it_prediction_obstacles_vector.perception_obstacle().anchor_point().y();
    prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point
        .z =
        it_prediction_obstacles_vector.perception_obstacle().anchor_point().z();
    prediction_obstacles_prediction_obstacle_perception_obstacle.anchor_point =
        prediction_obstacles_prediction_obstacle_perception_obstacle_anchor_point;
    std::vector<lcm_interface::Point3D> lcm_bounding_box;
    std::vector<legionclaw::interface::Point3D> legion_bounding_box;
    it_prediction_obstacles_vector.perception_obstacle().bounding_box(
        legion_bounding_box);
    for (auto it_bounding_box : legion_bounding_box) {
      lcm_interface::Point3D
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d;
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.x =
          it_bounding_box.x();
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.y =
          it_bounding_box.y();
      prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d.z =
          it_bounding_box.z();
      lcm_bounding_box.emplace_back(
          prediction_obstacles_prediction_obstacle_perception_obstacle_point_3d);
    }
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .bounding_box_size = lcm_bounding_box.size();
    prediction_obstacles_prediction_obstacle_perception_obstacle.bounding_box =
        lcm_bounding_box;
    prediction_obstacles_prediction_obstacle_perception_obstacle.sub_type =
        it_prediction_obstacles_vector.perception_obstacle().sub_type();
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .height_above_ground =
        it_prediction_obstacles_vector.perception_obstacle()
            .height_above_ground();
    std::vector<double> position_covariance;
    it_prediction_obstacles_vector.perception_obstacle().position_covariance(
        position_covariance);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .position_covariance_size = position_covariance.size();
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .position_covariance = position_covariance;
    std::vector<double> velocity_covariance;
    it_prediction_obstacles_vector.perception_obstacle().velocity_covariance(
        velocity_covariance);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .velocity_covariance_size = velocity_covariance.size();
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .velocity_covariance = velocity_covariance;
    std::vector<double> acceleration_covariance;
    it_prediction_obstacles_vector.perception_obstacle()
        .acceleration_covariance(acceleration_covariance);
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .acceleration_covariance_size = acceleration_covariance.size();
    prediction_obstacles_prediction_obstacle_perception_obstacle
        .acceleration_covariance = acceleration_covariance;
    prediction_obstacles_prediction_obstacle_perception_obstacle.light_status =
        it_prediction_obstacles_vector.perception_obstacle().light_status();
    prediction_obstacles_prediction_obstacle.perception_obstacle =
        prediction_obstacles_prediction_obstacle_perception_obstacle;
    prediction_obstacles_prediction_obstacle.timestamp =
        it_prediction_obstacles_vector.timestamp();
    prediction_obstacles_prediction_obstacle.predicted_period =
        it_prediction_obstacles_vector.predicted_period();
    std::vector<lcm_interface::TrajectoryInPrediction> lcm_trajectory;
    std::vector<legionclaw::interface::TrajectoryInPrediction> legion_trajectory;
    it_prediction_obstacles_vector.trajectory(legion_trajectory);
    for (auto it_trajectory : legion_trajectory) {
      lcm_interface::TrajectoryInPrediction
          prediction_obstacles_prediction_obstacle_trajectory_in_prediction;
      prediction_obstacles_prediction_obstacle_trajectory_in_prediction
          .probability = it_trajectory.probability();
      std::vector<lcm_interface::TrajectoryPoint> lcm_trajectory_points;
      std::vector<legionclaw::interface::TrajectoryPoint> legion_trajectory_points;
      it_trajectory.trajectory_points(legion_trajectory_points);
      for (auto it_trajectory_points : legion_trajectory_points) {
        lcm_interface::TrajectoryPoint
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point;
        lcm_interface::PathPoint
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point;
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .x = it_trajectory_points.path_point().x();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .y = it_trajectory_points.path_point().y();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .z = it_trajectory_points.path_point().z();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .theta = it_trajectory_points.path_point().theta();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .kappa = it_trajectory_points.path_point().kappa();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .s = it_trajectory_points.path_point().s();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .dkappa = it_trajectory_points.path_point().dkappa();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .ddkappa = it_trajectory_points.path_point().ddkappa();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .lane_id = it_trajectory_points.path_point().lane_id();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .x_derivative = it_trajectory_points.path_point().x_derivative();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point
            .y_derivative = it_trajectory_points.path_point().y_derivative();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .path_point =
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point_path_point;
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .v = it_trajectory_points.v();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .a = it_trajectory_points.a();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .relative_time = it_trajectory_points.relative_time();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .da = it_trajectory_points.da();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .is_steer_valid = it_trajectory_points.is_steer_valid();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .front_steer = it_trajectory_points.front_steer();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .rear_steer = it_trajectory_points.rear_steer();
        prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point
            .gear = it_trajectory_points.gear();
        lcm_trajectory_points.emplace_back(
            prediction_obstacles_prediction_obstacle_trajectory_in_prediction_trajectory_point);
      }
      prediction_obstacles_prediction_obstacle_trajectory_in_prediction
          .trajectory_points_size = lcm_trajectory_points.size();
      prediction_obstacles_prediction_obstacle_trajectory_in_prediction
          .trajectory_points = lcm_trajectory_points;
      lcm_trajectory.emplace_back(
          prediction_obstacles_prediction_obstacle_trajectory_in_prediction);
    }
    prediction_obstacles_prediction_obstacle.trajectory_size =
        lcm_trajectory.size();
    prediction_obstacles_prediction_obstacle.trajectory = lcm_trajectory;
    lcm_interface::ObstacleIntent
        prediction_obstacles_prediction_obstacle_intent;
    prediction_obstacles_prediction_obstacle_intent.type =
        it_prediction_obstacles_vector.intent().type();
    prediction_obstacles_prediction_obstacle.intent =
        prediction_obstacles_prediction_obstacle_intent;
    lcm_interface::ObstaclePriority
        prediction_obstacles_prediction_obstacle_priority;
    prediction_obstacles_prediction_obstacle_priority.priority =
        it_prediction_obstacles_vector.priority().priority();
    prediction_obstacles_prediction_obstacle.priority =
        prediction_obstacles_prediction_obstacle_priority;
    lcm_interface::ObstacleInteractiveTag
        prediction_obstacles_prediction_obstacle_interactive_tag;
    prediction_obstacles_prediction_obstacle_interactive_tag.interactive_tag =
        it_prediction_obstacles_vector.interactive_tag().interactive_tag();
    prediction_obstacles_prediction_obstacle.interactive_tag =
        prediction_obstacles_prediction_obstacle_interactive_tag;
    prediction_obstacles_prediction_obstacle.is_static =
        it_prediction_obstacles_vector.is_static();
    lcm_prediction_obstacles_vector.emplace_back(
        prediction_obstacles_prediction_obstacle);
  }
  prediction_obstacles.prediction_obstacles_size =
      lcm_prediction_obstacles_vector.size();
  prediction_obstacles.prediction_obstacles = lcm_prediction_obstacles_vector;
  prediction_obstacles.change_origin_flag = msg.change_origin_flag();
  prediction_obstacles.start_timestamp = msg.start_timestamp();
  prediction_obstacles.end_timestamp = msg.end_timestamp();
  prediction_obstacles.self_intent = msg.self_intent();
  prediction_obstacles.scenario = msg.scenario();

  lcm_->publish("/prediction/PredictionObstacles", &prediction_obstacles);
}

template <typename T>
void LcmMessageManager<T>::PublishFaults(legionclaw::interface::Faults msg) {
  if (is_init_ == false)
    return;
  lcm_interface::Faults faults;
  MESSAGE_HEADER_ASSIGN(lcm_interface, faults)
  FAULTS_PARSER(lcm, faults)
  faults.faults_size = faults.faults.size();

  lcm_->publish("/prediction/Faults", &faults);
}

template <typename T>
void LcmMessageManager<T>::HandleLocationMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::Location* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::Location location;
  MESSAGE_HEADER_PARSER(location)
  legionclaw::interface::PointLLH location_position;
  location_position.set_lon(msg->position.lon);
  location_position.set_lat(msg->position.lat);
  location_position.set_height(msg->position.height);
  location.set_position(location_position);
  location.set_pitch(msg->pitch);
  location.set_roll(msg->roll);
  location.set_heading(msg->heading);
  legionclaw::interface::Point3D location_linear_velocity;
  location_linear_velocity.set_x(msg->linear_velocity.x);
  location_linear_velocity.set_y(msg->linear_velocity.y);
  location_linear_velocity.set_z(msg->linear_velocity.z);
  location.set_linear_velocity(location_linear_velocity);
  legionclaw::interface::Point3D location_linear_acceleration;
  location_linear_acceleration.set_x(msg->linear_acceleration.x);
  location_linear_acceleration.set_y(msg->linear_acceleration.y);
  location_linear_acceleration.set_z(msg->linear_acceleration.z);
  location.set_linear_acceleration(location_linear_acceleration);
  legionclaw::interface::Point3D location_angular_velocity;
  location_angular_velocity.set_x(msg->angular_velocity.x);
  location_angular_velocity.set_y(msg->angular_velocity.y);
  location_angular_velocity.set_z(msg->angular_velocity.z);
  location.set_angular_velocity(location_angular_velocity);
  location.set_rtk_flag((legionclaw::interface::Location::RTKFlag)msg->rtk_flag);
  location.set_odom_type((legionclaw::interface::Location::OdomType)msg->odom_type);
  location.set_auxiliary_type(
      (legionclaw::interface::Location::AuxiliaryType)msg->auxiliary_type);
  location.set_location_valid_flag(
      (legionclaw::common::IsValid)msg->location_valid_flag);
  location.set_origin_lat(msg->origin_lat);
  location.set_origin_lon(msg->origin_lon);
  legionclaw::interface::PointENU location_utm_position;
  location_utm_position.set_x(msg->utm_position.x);
  location_utm_position.set_y(msg->utm_position.y);
  location_utm_position.set_z(msg->utm_position.z);
  location.set_utm_position(location_utm_position);
  location.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  legionclaw::interface::PointENU location_utm_position_next;
  location_utm_position_next.set_x(msg->utm_position_next.x);
  location_utm_position_next.set_y(msg->utm_position_next.y);
  location_utm_position_next.set_z(msg->utm_position_next.z);
  location.set_utm_position_next(location_utm_position_next);
  legionclaw::interface::Point3D location_position_std_dev;
  location_position_std_dev.set_x(msg->position_std_dev.x);
  location_position_std_dev.set_y(msg->position_std_dev.y);
  location_position_std_dev.set_z(msg->position_std_dev.z);
  location.set_position_std_dev(location_position_std_dev);
  legionclaw::interface::Point3D location_orientation_std_dev;
  location_orientation_std_dev.set_x(msg->orientation_std_dev.x);
  location_orientation_std_dev.set_y(msg->orientation_std_dev.y);
  location_orientation_std_dev.set_z(msg->orientation_std_dev.z);
  location.set_orientation_std_dev(location_orientation_std_dev);
  legionclaw::interface::Point3D location_linear_velocity_std_dev;
  location_linear_velocity_std_dev.set_x(msg->linear_velocity_std_dev.x);
  location_linear_velocity_std_dev.set_y(msg->linear_velocity_std_dev.y);
  location_linear_velocity_std_dev.set_z(msg->linear_velocity_std_dev.z);
  location.set_linear_velocity_std_dev(location_linear_velocity_std_dev);
  legionclaw::interface::Point3D location_linear_acceleration_std_dev;
  location_linear_acceleration_std_dev.set_x(
      msg->linear_acceleration_std_dev.x);
  location_linear_acceleration_std_dev.set_y(
      msg->linear_acceleration_std_dev.y);
  location_linear_acceleration_std_dev.set_z(
      msg->linear_acceleration_std_dev.z);
  location.set_linear_acceleration_std_dev(
      location_linear_acceleration_std_dev);
  legionclaw::interface::Point3D location_angular_velocity_std_dev;
  location_angular_velocity_std_dev.set_x(msg->angular_velocity_std_dev.x);
  location_angular_velocity_std_dev.set_y(msg->angular_velocity_std_dev.y);
  location_angular_velocity_std_dev.set_z(msg->angular_velocity_std_dev.z);
  location.set_angular_velocity_std_dev(location_angular_velocity_std_dev);
  instance_->HandleLocation(location);
}

template <typename T>
void LcmMessageManager<T>::HandleADCTrajectoryMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ADCTrajectory* msg) {
  if (is_active_ == false)
    return;
  legionclaw::interface::ADCTrajectory adc_trajectory;
  MESSAGE_HEADER_PARSER(adc_trajectory)
  adc_trajectory.set_total_path_length(msg->total_path_length);
  adc_trajectory.set_total_path_time(msg->total_path_time);
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  for (auto it_trajectory_points : msg->trajectory_points) {
    legionclaw::interface::TrajectoryPoint adc_trajectory_trajectory_point;
    legionclaw::interface::PathPoint adc_trajectory_trajectory_point_path_point;
    adc_trajectory_trajectory_point_path_point.set_x(
        it_trajectory_points.path_point.x);
    adc_trajectory_trajectory_point_path_point.set_y(
        it_trajectory_points.path_point.y);
    adc_trajectory_trajectory_point_path_point.set_z(
        it_trajectory_points.path_point.z);
    adc_trajectory_trajectory_point_path_point.set_theta(
        it_trajectory_points.path_point.theta);
    adc_trajectory_trajectory_point_path_point.set_kappa(
        it_trajectory_points.path_point.kappa);
    adc_trajectory_trajectory_point_path_point.set_s(
        it_trajectory_points.path_point.s);
    adc_trajectory_trajectory_point_path_point.set_dkappa(
        it_trajectory_points.path_point.dkappa);
    adc_trajectory_trajectory_point_path_point.set_ddkappa(
        it_trajectory_points.path_point.ddkappa);
    adc_trajectory_trajectory_point_path_point.set_lane_id(
        it_trajectory_points.path_point.lane_id);
    adc_trajectory_trajectory_point_path_point.set_x_derivative(
        it_trajectory_points.path_point.x_derivative);
    adc_trajectory_trajectory_point_path_point.set_y_derivative(
        it_trajectory_points.path_point.y_derivative);
    adc_trajectory_trajectory_point.set_path_point(
        adc_trajectory_trajectory_point_path_point);
    adc_trajectory_trajectory_point.set_v(it_trajectory_points.v);
    adc_trajectory_trajectory_point.set_a(it_trajectory_points.a);
    adc_trajectory_trajectory_point.set_relative_time(
        it_trajectory_points.relative_time);
    adc_trajectory_trajectory_point.set_da(it_trajectory_points.da);
    adc_trajectory_trajectory_point.set_is_steer_valid(
        it_trajectory_points.is_steer_valid);
    adc_trajectory_trajectory_point.set_front_steer(it_trajectory_points.front_steer);
    adc_trajectory_trajectory_point.set_rear_steer(it_trajectory_points.rear_steer);
    adc_trajectory_trajectory_point.set_gear(
        (legionclaw::common::GearPosition)it_trajectory_points.gear);
    trajectory_points.emplace_back(adc_trajectory_trajectory_point);
  }
  adc_trajectory.set_trajectory_points(&trajectory_points);
  adc_trajectory.set_car_action(
      (legionclaw::interface::ADCTrajectory::CarAction)msg->car_action);
  adc_trajectory.set_behaviour_lat_state(
      (legionclaw::interface::ADCTrajectory::BehaviourLatState)
          msg->behaviour_lat_state);
  adc_trajectory.set_behaviour_lon_state(
      (legionclaw::interface::ADCTrajectory::BehaviourLonState)
          msg->behaviour_lon_state);
  adc_trajectory.set_scenario(
      (legionclaw::interface::ADCTrajectory::Scenario)msg->scenario);
  adc_trajectory.set_driving_mode(
      (legionclaw::common::DrivingMode)msg->driving_mode);
  adc_trajectory.set_adc_trajectory_type(
      (legionclaw::interface::ADCTrajectory::ADCTrajectoryType)
          msg->adc_trajectory_type);
  legionclaw::interface::EStop adc_trajectory_estop;
  adc_trajectory_estop.set_is_estop(msg->estop.is_estop);
  adc_trajectory_estop.set_reason(msg->estop.reason);
  adc_trajectory.set_estop(adc_trajectory_estop);
  adc_trajectory.set_is_replan(msg->is_replan);
  adc_trajectory.set_replan_reason(msg->replan_reason);
  adc_trajectory.set_right_of_way_status(
      (legionclaw::interface::ADCTrajectory::RightOfWayStatus)
          msg->right_of_way_status);
  legionclaw::interface::RSSInfo adc_trajectory_rss_info;
  adc_trajectory_rss_info.set_is_rss_safe(msg->rss_info.is_rss_safe);
  adc_trajectory_rss_info.set_cur_dist_lon(msg->rss_info.cur_dist_lon);
  adc_trajectory_rss_info.set_rss_safe_dist_lon(
      msg->rss_info.rss_safe_dist_lon);
  adc_trajectory_rss_info.set_acc_lon_range_minimum(
      msg->rss_info.acc_lon_range_minimum);
  adc_trajectory_rss_info.set_acc_lon_range_maximum(
      msg->rss_info.acc_lon_range_maximum);
  adc_trajectory_rss_info.set_acc_lat_left_range_minimum(
      msg->rss_info.acc_lat_left_range_minimum);
  adc_trajectory_rss_info.set_acc_lat_left_range_maximum(
      msg->rss_info.acc_lat_left_range_maximum);
  adc_trajectory_rss_info.set_acc_lat_right_range_minimum(
      msg->rss_info.acc_lat_right_range_minimum);
  adc_trajectory_rss_info.set_acc_lat_right_range_maximum(
      msg->rss_info.acc_lat_right_range_maximum);
  adc_trajectory.set_rss_info(adc_trajectory_rss_info);
  instance_->HandleADCTrajectory(adc_trajectory);
}

template <typename T>
void LcmMessageManager<T>::HandleObstacleListMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ObstacleList* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::ObstacleList obstacle_list;
  MESSAGE_HEADER_PARSER(obstacle_list)
  obstacle_list.set_sensor_id((legionclaw::common::SensorID)msg->sensor_id);
  std::vector<legionclaw::interface::Obstacle> obstacle;
  for (auto it_obstacle : msg->obstacle) {
    legionclaw::interface::Obstacle obstacle_list_obstacle;
    legionclaw::interface::Time obstacle_list_obstacle_timestamp;
    obstacle_list_obstacle_timestamp.set_sec(it_obstacle.timestamp.sec);
    obstacle_list_obstacle_timestamp.set_nsec(it_obstacle.timestamp.nsec);
    obstacle_list_obstacle.set_timestamp(obstacle_list_obstacle_timestamp);
    obstacle_list_obstacle.set_id(it_obstacle.id);
    obstacle_list_obstacle.set_existence_prob(it_obstacle.existence_prob);
    legionclaw::interface::Time obstacle_list_obstacle_create_time;
    obstacle_list_obstacle_create_time.set_sec(it_obstacle.create_time.sec);
    obstacle_list_obstacle_create_time.set_nsec(it_obstacle.create_time.nsec);
    obstacle_list_obstacle.set_create_time(obstacle_list_obstacle_create_time);
    legionclaw::interface::Time obstacle_list_obstacle_last_updated_time;
    obstacle_list_obstacle_last_updated_time.set_sec(
        it_obstacle.last_updated_time.sec);
    obstacle_list_obstacle_last_updated_time.set_nsec(
        it_obstacle.last_updated_time.nsec);
    obstacle_list_obstacle.set_last_updated_time(
        obstacle_list_obstacle_last_updated_time);
    legionclaw::interface::Point3D obstacle_list_obstacle_center_pos_vehicle;
    obstacle_list_obstacle_center_pos_vehicle.set_x(
        it_obstacle.center_pos_vehicle.x);
    obstacle_list_obstacle_center_pos_vehicle.set_y(
        it_obstacle.center_pos_vehicle.y);
    obstacle_list_obstacle_center_pos_vehicle.set_z(
        it_obstacle.center_pos_vehicle.z);
    obstacle_list_obstacle.set_center_pos_vehicle(
        obstacle_list_obstacle_center_pos_vehicle);
    legionclaw::interface::Point3D obstacle_list_obstacle_center_pos_abs;
    obstacle_list_obstacle_center_pos_abs.set_x(it_obstacle.center_pos_abs.x);
    obstacle_list_obstacle_center_pos_abs.set_y(it_obstacle.center_pos_abs.y);
    obstacle_list_obstacle_center_pos_abs.set_z(it_obstacle.center_pos_abs.z);
    obstacle_list_obstacle.set_center_pos_abs(
        obstacle_list_obstacle_center_pos_abs);
    obstacle_list_obstacle.set_theta_vehicle(it_obstacle.theta_vehicle);
    obstacle_list_obstacle.set_theta_abs(it_obstacle.theta_abs);
    legionclaw::interface::Point3D obstacle_list_obstacle_velocity_vehicle;
    obstacle_list_obstacle_velocity_vehicle.set_x(
        it_obstacle.velocity_vehicle.x);
    obstacle_list_obstacle_velocity_vehicle.set_y(
        it_obstacle.velocity_vehicle.y);
    obstacle_list_obstacle_velocity_vehicle.set_z(
        it_obstacle.velocity_vehicle.z);
    obstacle_list_obstacle.set_velocity_vehicle(
        obstacle_list_obstacle_velocity_vehicle);
    legionclaw::interface::Point3D obstacle_list_obstacle_velocity_abs;
    obstacle_list_obstacle_velocity_abs.set_x(it_obstacle.velocity_abs.x);
    obstacle_list_obstacle_velocity_abs.set_y(it_obstacle.velocity_abs.y);
    obstacle_list_obstacle_velocity_abs.set_z(it_obstacle.velocity_abs.z);
    obstacle_list_obstacle.set_velocity_abs(
        obstacle_list_obstacle_velocity_abs);
    obstacle_list_obstacle.set_length(it_obstacle.length);
    obstacle_list_obstacle.set_width(it_obstacle.width);
    obstacle_list_obstacle.set_height(it_obstacle.height);
    std::vector<legionclaw::interface::ImageKeyPoint> image_key_points;
    for (auto it_image_key_points : it_obstacle.image_key_points) {
      legionclaw::interface::ImageKeyPoint obstacle_list_obstacle_image_key_point;
      obstacle_list_obstacle_image_key_point.set_x(it_image_key_points.x);
      obstacle_list_obstacle_image_key_point.set_y(it_image_key_points.y);
      obstacle_list_obstacle_image_key_point.set_confidence(
          it_image_key_points.confidence);
      image_key_points.emplace_back(obstacle_list_obstacle_image_key_point);
    }
    obstacle_list_obstacle.set_image_key_points(&image_key_points);
    std::vector<legionclaw::interface::Point3D> polygon_point_abs;
    for (auto it_polygon_point_abs : it_obstacle.polygon_point_abs) {
      legionclaw::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_polygon_point_abs.x);
      obstacle_list_obstacle_point_3d.set_y(it_polygon_point_abs.y);
      obstacle_list_obstacle_point_3d.set_z(it_polygon_point_abs.z);
      polygon_point_abs.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_polygon_point_abs(&polygon_point_abs);
    std::vector<legionclaw::interface::Point3D> polygon_point_vehicle;
    for (auto it_polygon_point_vehicle : it_obstacle.polygon_point_vehicle) {
      legionclaw::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_polygon_point_vehicle.x);
      obstacle_list_obstacle_point_3d.set_y(it_polygon_point_vehicle.y);
      obstacle_list_obstacle_point_3d.set_z(it_polygon_point_vehicle.z);
      polygon_point_vehicle.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_polygon_point_vehicle(&polygon_point_vehicle);
    obstacle_list_obstacle.set_tracking_time(it_obstacle.tracking_time);
    obstacle_list_obstacle.set_type(
        (legionclaw::common::ObstacleType)it_obstacle.type);
    obstacle_list_obstacle.set_confidence(it_obstacle.confidence);
    obstacle_list_obstacle.set_confidence_type(
        (legionclaw::interface::Obstacle::ConfidenceType)
            it_obstacle.confidence_type);
    std::vector<legionclaw::interface::Point3D> drops;
    for (auto it_drops : it_obstacle.drops) {
      legionclaw::interface::Point3D obstacle_list_obstacle_point_3d;
      obstacle_list_obstacle_point_3d.set_x(it_drops.x);
      obstacle_list_obstacle_point_3d.set_y(it_drops.y);
      obstacle_list_obstacle_point_3d.set_z(it_drops.z);
      drops.emplace_back(obstacle_list_obstacle_point_3d);
    }
    obstacle_list_obstacle.set_drops(&drops);
    legionclaw::interface::Point3D obstacle_list_obstacle_acceleration_vehicle;
    obstacle_list_obstacle_acceleration_vehicle.set_x(
        it_obstacle.acceleration_vehicle.x);
    obstacle_list_obstacle_acceleration_vehicle.set_y(
        it_obstacle.acceleration_vehicle.y);
    obstacle_list_obstacle_acceleration_vehicle.set_z(
        it_obstacle.acceleration_vehicle.z);
    obstacle_list_obstacle.set_acceleration_vehicle(
        obstacle_list_obstacle_acceleration_vehicle);
    legionclaw::interface::Point3D obstacle_list_obstacle_acceleration_abs;
    obstacle_list_obstacle_acceleration_abs.set_x(
        it_obstacle.acceleration_abs.x);
    obstacle_list_obstacle_acceleration_abs.set_y(
        it_obstacle.acceleration_abs.y);
    obstacle_list_obstacle_acceleration_abs.set_z(
        it_obstacle.acceleration_abs.z);
    obstacle_list_obstacle.set_acceleration_abs(
        obstacle_list_obstacle_acceleration_abs);
    legionclaw::interface::Point2D obstacle_list_obstacle_anchor_point_image;
    obstacle_list_obstacle_anchor_point_image.set_x(
        it_obstacle.anchor_point_image.x);
    obstacle_list_obstacle_anchor_point_image.set_y(
        it_obstacle.anchor_point_image.y);
    obstacle_list_obstacle.set_anchor_point_image(
        obstacle_list_obstacle_anchor_point_image);
    legionclaw::interface::Point3D obstacle_list_obstacle_anchor_point_vehicle;
    obstacle_list_obstacle_anchor_point_vehicle.set_x(
        it_obstacle.anchor_point_vehicle.x);
    obstacle_list_obstacle_anchor_point_vehicle.set_y(
        it_obstacle.anchor_point_vehicle.y);
    obstacle_list_obstacle_anchor_point_vehicle.set_z(
        it_obstacle.anchor_point_vehicle.z);
    obstacle_list_obstacle.set_anchor_point_vehicle(
        obstacle_list_obstacle_anchor_point_vehicle);
    legionclaw::interface::Point3D obstacle_list_obstacle_anchor_point_abs;
    obstacle_list_obstacle_anchor_point_abs.set_x(
        it_obstacle.anchor_point_abs.x);
    obstacle_list_obstacle_anchor_point_abs.set_y(
        it_obstacle.anchor_point_abs.y);
    obstacle_list_obstacle_anchor_point_abs.set_z(
        it_obstacle.anchor_point_abs.z);
    obstacle_list_obstacle.set_anchor_point_abs(
        obstacle_list_obstacle_anchor_point_abs);
    legionclaw::interface::BBox2D obstacle_list_obstacle_bbox2d;
    obstacle_list_obstacle_bbox2d.set_xmin(it_obstacle.bbox2d.xmin);
    obstacle_list_obstacle_bbox2d.set_ymin(it_obstacle.bbox2d.ymin);
    obstacle_list_obstacle_bbox2d.set_xmax(it_obstacle.bbox2d.xmax);
    obstacle_list_obstacle_bbox2d.set_ymax(it_obstacle.bbox2d.ymax);
    obstacle_list_obstacle.set_bbox2d(obstacle_list_obstacle_bbox2d);
    legionclaw::interface::BBox2D obstacle_list_obstacle_bbox2d_rear;
    obstacle_list_obstacle_bbox2d_rear.set_xmin(it_obstacle.bbox2d_rear.xmin);
    obstacle_list_obstacle_bbox2d_rear.set_ymin(it_obstacle.bbox2d_rear.ymin);
    obstacle_list_obstacle_bbox2d_rear.set_xmax(it_obstacle.bbox2d_rear.xmax);
    obstacle_list_obstacle_bbox2d_rear.set_ymax(it_obstacle.bbox2d_rear.ymax);
    obstacle_list_obstacle.set_bbox2d_rear(obstacle_list_obstacle_bbox2d_rear);
    obstacle_list_obstacle.set_sub_type(
        (legionclaw::common::ObstacleSubType)it_obstacle.sub_type);
    obstacle_list_obstacle.set_height_above_ground(
        it_obstacle.height_above_ground);
    std::vector<double> position_abs_covariance;
    for (auto it_position_abs_covariance :
         it_obstacle.position_abs_covariance) {
      double position_abs_covariance_item;
      position_abs_covariance_item = it_position_abs_covariance;
      position_abs_covariance.emplace_back(position_abs_covariance_item);
    }
    obstacle_list_obstacle.set_position_abs_covariance(
        &position_abs_covariance);
    std::vector<double> velocity_abs_covariance;
    for (auto it_velocity_abs_covariance :
         it_obstacle.velocity_abs_covariance) {
      double velocity_abs_covariance_item;
      velocity_abs_covariance_item = it_velocity_abs_covariance;
      velocity_abs_covariance.emplace_back(velocity_abs_covariance_item);
    }
    obstacle_list_obstacle.set_velocity_abs_covariance(
        &velocity_abs_covariance);
    std::vector<double> acceleration_abs_covariance;
    for (auto it_acceleration_abs_covariance :
         it_obstacle.acceleration_abs_covariance) {
      double acceleration_abs_covariance_item;
      acceleration_abs_covariance_item = it_acceleration_abs_covariance;
      acceleration_abs_covariance.emplace_back(
          acceleration_abs_covariance_item);
    }
    obstacle_list_obstacle.set_acceleration_abs_covariance(
        &acceleration_abs_covariance);
    obstacle_list_obstacle.set_theta_abs_covariance(
        it_obstacle.theta_abs_covariance);
    std::vector<double> position_vehicle_covariance;
    for (auto it_position_vehicle_covariance :
         it_obstacle.position_vehicle_covariance) {
      double position_vehicle_covariance_item;
      position_vehicle_covariance_item = it_position_vehicle_covariance;
      position_vehicle_covariance.emplace_back(
          position_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_position_vehicle_covariance(
        &position_vehicle_covariance);
    std::vector<double> velocity_vehicle_covariance;
    for (auto it_velocity_vehicle_covariance :
         it_obstacle.velocity_vehicle_covariance) {
      double velocity_vehicle_covariance_item;
      velocity_vehicle_covariance_item = it_velocity_vehicle_covariance;
      velocity_vehicle_covariance.emplace_back(
          velocity_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_velocity_vehicle_covariance(
        &velocity_vehicle_covariance);
    std::vector<double> acceleration_vehicle_covariance;
    for (auto it_acceleration_vehicle_covariance :
         it_obstacle.acceleration_vehicle_covariance) {
      double acceleration_vehicle_covariance_item;
      acceleration_vehicle_covariance_item = it_acceleration_vehicle_covariance;
      acceleration_vehicle_covariance.emplace_back(
          acceleration_vehicle_covariance_item);
    }
    obstacle_list_obstacle.set_acceleration_vehicle_covariance(
        &acceleration_vehicle_covariance);
    obstacle_list_obstacle.set_theta_vehicle_covariance(
        it_obstacle.theta_vehicle_covariance);
    legionclaw::interface::SensorCalibrator
        obstacle_list_obstacle_sensor_calibrator;
    legionclaw::interface::Point3D obstacle_list_obstacle_sensor_calibrator_pose;
    obstacle_list_obstacle_sensor_calibrator_pose.set_x(
        it_obstacle.sensor_calibrator.pose.x);
    obstacle_list_obstacle_sensor_calibrator_pose.set_y(
        it_obstacle.sensor_calibrator.pose.y);
    obstacle_list_obstacle_sensor_calibrator_pose.set_z(
        it_obstacle.sensor_calibrator.pose.z);
    obstacle_list_obstacle_sensor_calibrator.set_pose(
        obstacle_list_obstacle_sensor_calibrator_pose);
    legionclaw::interface::Point3D obstacle_list_obstacle_sensor_calibrator_angle;
    obstacle_list_obstacle_sensor_calibrator_angle.set_x(
        it_obstacle.sensor_calibrator.angle.x);
    obstacle_list_obstacle_sensor_calibrator_angle.set_y(
        it_obstacle.sensor_calibrator.angle.y);
    obstacle_list_obstacle_sensor_calibrator_angle.set_z(
        it_obstacle.sensor_calibrator.angle.z);
    obstacle_list_obstacle_sensor_calibrator.set_angle(
        obstacle_list_obstacle_sensor_calibrator_angle);
    obstacle_list_obstacle.set_sensor_calibrator(
        obstacle_list_obstacle_sensor_calibrator);
    obstacle_list_obstacle.set_cipv_flag(it_obstacle.cipv_flag);
    obstacle_list_obstacle.set_lane_position(
        (legionclaw::common::LanePosition)it_obstacle.lane_position);
    obstacle_list_obstacle.set_pihp_percentage(it_obstacle.pihp_percentage);
    obstacle_list_obstacle.set_blinker_flag(
        (legionclaw::common::BlinkerFlag)it_obstacle.blinker_flag);
    obstacle_list_obstacle.set_fusion_type(
        (legionclaw::interface::Obstacle::FusionType)it_obstacle.fusion_type);
    obstacle.emplace_back(obstacle_list_obstacle);
  }
  obstacle_list.set_obstacle(&obstacle);
  obstacle_list.set_error_code((legionclaw::common::ErrorCode)msg->error_code);
  obstacle_list.set_is_valid(msg->is_valid);
  obstacle_list.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);

  instance_->HandleObstacleList(obstacle_list);
}

template <typename T>
void LcmMessageManager<T>::HandleOdometryMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::Odometry* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::Odometry odometry;
  MESSAGE_HEADER_PARSER(odometry)
  legionclaw::interface::PointENU odometry_position;
  odometry_position.set_x(msg->position.x);
  odometry_position.set_y(msg->position.y);
  odometry_position.set_z(msg->position.z);
  odometry.set_position(odometry_position);
  legionclaw::interface::Quaternion odometry_orientation;
  odometry_orientation.set_qx(msg->orientation.qx);
  odometry_orientation.set_qy(msg->orientation.qy);
  odometry_orientation.set_qz(msg->orientation.qz);
  odometry_orientation.set_qw(msg->orientation.qw);
  odometry.set_orientation(odometry_orientation);
  std::vector<double> covariance;
  for (auto it_covariance : msg->covariance) {
    double covariance_item;
    covariance_item = it_covariance;
    covariance.emplace_back(covariance_item);
  }
  odometry.set_covariance(&covariance);

  instance_->HandleOdometry(odometry);
}

template <typename T>
void LcmMessageManager<T>::HandleTrafficEventsMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::TrafficEvents* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::TrafficEvents traffic_events;
  MESSAGE_HEADER_PARSER(traffic_events)
  legionclaw::interface::RouteFusionInfo traffic_events_route_fusion_info;
  traffic_events_route_fusion_info.set_fusion_flag(
      (legionclaw::common::IsValid)msg->route_fusion_info.fusion_flag);
  traffic_events_route_fusion_info.set_fusion_reason(
      msg->route_fusion_info.fusion_reason);
  traffic_events.set_route_fusion_info(traffic_events_route_fusion_info);
  legionclaw::interface::JunctionInfo traffic_events_junction_info;
  traffic_events_junction_info.set_id(msg->junction_info.id);
  traffic_events_junction_info.set_light_flag(
      (legionclaw::common::IsValid)msg->junction_info.light_flag);
  traffic_events_junction_info.set_light_color(
      (legionclaw::common::TrafficLightColor)msg->junction_info.light_color);
  traffic_events_junction_info.set_light_remain_time(
      msg->junction_info.light_remain_time);
  traffic_events_junction_info.set_distance_to_stop(
      msg->junction_info.distance_to_stop);
  traffic_events_junction_info.set_direction_flag(
      (legionclaw::common::IsValid)msg->junction_info.direction_flag);
  traffic_events_junction_info.set_direction(
      (legionclaw::common::Direction)msg->junction_info.direction);
  traffic_events_junction_info.set_distance_to_junction(
      msg->junction_info.distance_to_junction);
  std::vector<legionclaw::interface::Point3D> stop_line;
  for (auto it_stop_line : msg->junction_info.stop_line) {
    legionclaw::interface::Point3D traffic_events_junction_info_point_3d;
    traffic_events_junction_info_point_3d.set_x(it_stop_line.x);
    traffic_events_junction_info_point_3d.set_y(it_stop_line.y);
    traffic_events_junction_info_point_3d.set_z(it_stop_line.z);
    stop_line.emplace_back(traffic_events_junction_info_point_3d);
  }
  traffic_events_junction_info.set_stop_line(&stop_line);
  traffic_events.set_junction_info(traffic_events_junction_info);
  legionclaw::interface::LimitSpeedInfo traffic_events_limit_speed_info;
  traffic_events_limit_speed_info.set_limitspeed_valid_flag(
      (legionclaw::common::IsValid)msg->limit_speed_info.limitspeed_valid_flag);
  traffic_events_limit_speed_info.set_limit_speed(
      msg->limit_speed_info.limit_speed);
  traffic_events_limit_speed_info.set_limit_distance(
      msg->limit_speed_info.limit_distance);
  traffic_events.set_limit_speed_info(traffic_events_limit_speed_info);
  instance_->HandleTrafficEvents(traffic_events);
}

template <typename T>
void LcmMessageManager<T>::HandleRoutingResponseMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::RoutingResponse* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::RoutingResponse routing_response;
  MESSAGE_HEADER_PARSER(routing_response)
  routing_response.set_plan_status(
      (legionclaw::interface::RoutingResponse::PlanStatus)msg->plan_status);
  routing_response.set_replan_flag(
      (legionclaw::interface::RoutingResponse::ReplanFlag)msg->replan_flag);
  routing_response.set_route_reason(
      (legionclaw::interface::RoutingResponse::RouteReason)msg->route_reason);
  std::vector<legionclaw::interface::LaneInfo> lane_list;
  for (auto it_lane_list : msg->lane_list) {
    legionclaw::interface::LaneInfo routing_response_lane_info;
    routing_response_lane_info.set_priority(it_lane_list.priority);
    routing_response_lane_info.set_global_id(it_lane_list.global_id);
    routing_response_lane_info.set_type(it_lane_list.type);
    std::vector<legionclaw::interface::LanePoint> lane_points;
    for (auto it_lane_points : it_lane_list.lane_points) {
      legionclaw::interface::LanePoint routing_response_lane_info_lane_point;
      legionclaw::interface::Point3D routing_response_lane_info_lane_point_point;
      routing_response_lane_info_lane_point_point.set_x(it_lane_points.point.x);
      routing_response_lane_info_lane_point_point.set_y(it_lane_points.point.y);
      routing_response_lane_info_lane_point_point.set_z(it_lane_points.point.z);
      routing_response_lane_info_lane_point.set_point(
          routing_response_lane_info_lane_point_point);
      routing_response_lane_info_lane_point.set_theta(it_lane_points.theta);
      routing_response_lane_info_lane_point.set_mileage(it_lane_points.mileage);
      routing_response_lane_info_lane_point.set_limit_speed(
          it_lane_points.limit_speed);
      routing_response_lane_info_lane_point.set_left_road_width(
          it_lane_points.left_road_width);
      routing_response_lane_info_lane_point.set_right_road_width(
          it_lane_points.right_road_width);
      routing_response_lane_info_lane_point.set_left_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.left_line_type);
      routing_response_lane_info_lane_point.set_right_line_type(
          (legionclaw::common::LaneLineType)it_lane_points.right_line_type);
      lane_points.emplace_back(routing_response_lane_info_lane_point);
    }
    routing_response_lane_info.set_lane_points(&lane_points);
    lane_list.emplace_back(routing_response_lane_info);
  }
  routing_response.set_lane_list(&lane_list);

  instance_->HandleRoutingResponse(routing_response);
}

template <typename T>
void LcmMessageManager<T>::HandleLaneListMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::LaneList* msg) {
  if (is_active_ == false)
    return;

  legionclaw::interface::LaneList lane_list;
  MESSAGE_HEADER_PARSER(lane_list)
  lane_list.set_sensor_id((legionclaw::common::SensorID)msg->sensor_id);
  lane_list.set_error_code(msg->error_code);
  lane_list.set_sensor_status(msg->sensor_status);
  lane_list.set_change_origin_flag(
      (legionclaw::interface::Location::ChangeOriginFlag)msg->change_origin_flag);
  lane_list.set_is_valid(msg->is_valid);
  legionclaw::interface::SensorCalibrator lane_list_sensor_calibrator;
  legionclaw::interface::Point3D lane_list_sensor_calibrator_pose;
  lane_list_sensor_calibrator_pose.set_x(msg->sensor_calibrator.pose.x);
  lane_list_sensor_calibrator_pose.set_y(msg->sensor_calibrator.pose.y);
  lane_list_sensor_calibrator_pose.set_z(msg->sensor_calibrator.pose.z);
  lane_list_sensor_calibrator.set_pose(lane_list_sensor_calibrator_pose);
  legionclaw::interface::Point3D lane_list_sensor_calibrator_angle;
  lane_list_sensor_calibrator_angle.set_x(msg->sensor_calibrator.angle.x);
  lane_list_sensor_calibrator_angle.set_y(msg->sensor_calibrator.angle.y);
  lane_list_sensor_calibrator_angle.set_z(msg->sensor_calibrator.angle.z);
  lane_list_sensor_calibrator.set_angle(lane_list_sensor_calibrator_angle);
  lane_list.set_sensor_calibrator(lane_list_sensor_calibrator);
  std::vector<legionclaw::interface::LaneLine> camera_laneline;
  for (auto it_camera_laneline : msg->camera_laneline) {
    legionclaw::interface::LaneLine lane_list_lane_line;
    lane_list_lane_line.set_lane_type(
        (legionclaw::interface::LaneLine::LaneType)it_camera_laneline.lane_type);
    lane_list_lane_line.set_lane_color(
        (legionclaw::interface::LaneLine::LaneColor)it_camera_laneline.lane_color);
    lane_list_lane_line.set_pos_type(
        (legionclaw::interface::LaneLine::PosType)it_camera_laneline.pos_type);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_vehicle;
    lane_list_lane_line_curve_vehicle.set_start_x(
        it_camera_laneline.curve_vehicle.start_x);
    lane_list_lane_line_curve_vehicle.set_end_x(
        it_camera_laneline.curve_vehicle.end_x);
    lane_list_lane_line_curve_vehicle.set_a(it_camera_laneline.curve_vehicle.a);
    lane_list_lane_line_curve_vehicle.set_b(it_camera_laneline.curve_vehicle.b);
    lane_list_lane_line_curve_vehicle.set_c(it_camera_laneline.curve_vehicle.c);
    lane_list_lane_line_curve_vehicle.set_d(it_camera_laneline.curve_vehicle.d);
    lane_list_lane_line.set_curve_vehicle(lane_list_lane_line_curve_vehicle);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_image;
    lane_list_lane_line_curve_image.set_start_x(
        it_camera_laneline.curve_image.start_x);
    lane_list_lane_line_curve_image.set_end_x(
        it_camera_laneline.curve_image.end_x);
    lane_list_lane_line_curve_image.set_a(it_camera_laneline.curve_image.a);
    lane_list_lane_line_curve_image.set_b(it_camera_laneline.curve_image.b);
    lane_list_lane_line_curve_image.set_c(it_camera_laneline.curve_image.c);
    lane_list_lane_line_curve_image.set_d(it_camera_laneline.curve_image.d);
    lane_list_lane_line.set_curve_image(lane_list_lane_line_curve_image);
    legionclaw::interface::LaneLineCubicCurve lane_list_lane_line_curve_abs;
    lane_list_lane_line_curve_abs.set_start_x(
        it_camera_laneline.curve_abs.start_x);
    lane_list_lane_line_curve_abs.set_end_x(it_camera_laneline.curve_abs.end_x);
    lane_list_lane_line_curve_abs.set_a(it_camera_laneline.curve_abs.a);
    lane_list_lane_line_curve_abs.set_b(it_camera_laneline.curve_abs.b);
    lane_list_lane_line_curve_abs.set_c(it_camera_laneline.curve_abs.c);
    lane_list_lane_line_curve_abs.set_d(it_camera_laneline.curve_abs.d);
    lane_list_lane_line.set_curve_abs(lane_list_lane_line_curve_abs);
    std::vector<legionclaw::interface::Point3D> pts_vehicle;
    for (auto it_pts_vehicle : it_camera_laneline.pts_vehicle) {
      legionclaw::interface::Point3D lane_list_lane_line_point_3d;
      lane_list_lane_line_point_3d.set_x(it_pts_vehicle.x);
      lane_list_lane_line_point_3d.set_y(it_pts_vehicle.y);
      lane_list_lane_line_point_3d.set_z(it_pts_vehicle.z);
      pts_vehicle.emplace_back(lane_list_lane_line_point_3d);
    }
    lane_list_lane_line.set_pts_vehicle(&pts_vehicle);
    std::vector<legionclaw::interface::Point2D> pts_image;
    for (auto it_pts_image : it_camera_laneline.pts_image) {
      legionclaw::interface::Point2D lane_list_lane_line_point_2d;
      lane_list_lane_line_point_2d.set_x(it_pts_image.x);
      lane_list_lane_line_point_2d.set_y(it_pts_image.y);
      pts_image.emplace_back(lane_list_lane_line_point_2d);
    }
    lane_list_lane_line.set_pts_image(&pts_image);
    std::vector<legionclaw::interface::Point3D> pts_abs;
    for (auto it_pts_abs : it_camera_laneline.pts_abs) {
      legionclaw::interface::Point3D lane_list_lane_line_point_3d;
      lane_list_lane_line_point_3d.set_x(it_pts_abs.x);
      lane_list_lane_line_point_3d.set_y(it_pts_abs.y);
      lane_list_lane_line_point_3d.set_z(it_pts_abs.z);
      pts_abs.emplace_back(lane_list_lane_line_point_3d);
    }
    lane_list_lane_line.set_pts_abs(&pts_abs);
    legionclaw::interface::EndPoints lane_list_lane_line_image_end_point;
    legionclaw::interface::Point2D lane_list_lane_line_image_end_point_start;
    lane_list_lane_line_image_end_point_start.set_x(
        it_camera_laneline.image_end_point.start.x);
    lane_list_lane_line_image_end_point_start.set_y(
        it_camera_laneline.image_end_point.start.y);
    lane_list_lane_line_image_end_point.set_start(
        lane_list_lane_line_image_end_point_start);
    legionclaw::interface::Point2D lane_list_lane_line_image_end_point_end;
    lane_list_lane_line_image_end_point_end.set_x(
        it_camera_laneline.image_end_point.end.x);
    lane_list_lane_line_image_end_point_end.set_y(
        it_camera_laneline.image_end_point.end.y);
    lane_list_lane_line_image_end_point.set_end(
        lane_list_lane_line_image_end_point_end);
    lane_list_lane_line.set_image_end_point(
        lane_list_lane_line_image_end_point);
    std::vector<legionclaw::interface::Point2D> pts_key;
    for (auto it_pts_key : it_camera_laneline.pts_key) {
      legionclaw::interface::Point2D lane_list_lane_line_point_2d;
      lane_list_lane_line_point_2d.set_x(it_pts_key.x);
      lane_list_lane_line_point_2d.set_y(it_pts_key.y);
      pts_key.emplace_back(lane_list_lane_line_point_2d);
    }
    lane_list_lane_line.set_pts_key(&pts_key);
    lane_list_lane_line.set_hd_lane_id(it_camera_laneline.hd_lane_id);
    lane_list_lane_line.set_confidence(it_camera_laneline.confidence);
    lane_list_lane_line.set_lane_quality(
        (legionclaw::interface::LaneLine::LaneQuality)
            it_camera_laneline.lane_quality);
    lane_list_lane_line.set_fused_lane_type(
        (legionclaw::interface::LaneLine::FusedLaneType)
            it_camera_laneline.fused_lane_type);
    std::vector<double> homography_mat;
    for (auto it_homography_mat : it_camera_laneline.homography_mat) {
      double homography_mat_item;
      homography_mat_item = it_homography_mat;
      homography_mat.emplace_back(homography_mat_item);
    }
    lane_list_lane_line.set_homography_mat(&homography_mat);
    std::vector<double> homography_mat_inv;
    for (auto it_homography_mat_inv : it_camera_laneline.homography_mat_inv) {
      double homography_mat_inv_item;
      homography_mat_inv_item = it_homography_mat_inv;
      homography_mat_inv.emplace_back(homography_mat_inv_item);
    }
    lane_list_lane_line.set_homography_mat_inv(&homography_mat_inv);
    lane_list_lane_line.set_lane_coordinate_type(
        (legionclaw::interface::LaneLine::LaneCoordinateType)
            it_camera_laneline.lane_coordinate_type);
    lane_list_lane_line.set_use_type(
        (legionclaw::interface::LaneLine::UseType)it_camera_laneline.use_type);
    legionclaw::interface::Time lane_list_lane_line_create_time;
    lane_list_lane_line_create_time.set_sec(it_camera_laneline.create_time.sec);
    lane_list_lane_line_create_time.set_nsec(
        it_camera_laneline.create_time.nsec);
    lane_list_lane_line.set_create_time(lane_list_lane_line_create_time);
    camera_laneline.emplace_back(lane_list_lane_line);
  }
  lane_list.set_camera_laneline(&camera_laneline);
  legionclaw::interface::HolisticPathPrediction lane_list_hpp;
  legionclaw::interface::LaneLineCubicCurve lane_list_hpp_hpp;
  lane_list_hpp_hpp.set_start_x(msg->hpp.hpp.start_x);
  lane_list_hpp_hpp.set_end_x(msg->hpp.hpp.end_x);
  lane_list_hpp_hpp.set_a(msg->hpp.hpp.a);
  lane_list_hpp_hpp.set_b(msg->hpp.hpp.b);
  lane_list_hpp_hpp.set_c(msg->hpp.hpp.c);
  lane_list_hpp_hpp.set_d(msg->hpp.hpp.d);
  lane_list_hpp.set_hpp(lane_list_hpp_hpp);
  lane_list_hpp.set_planning_source(
      (legionclaw::interface::HolisticPathPrediction::PlanningSource)
          msg->hpp.planning_source);
  lane_list_hpp.set_ego_lane_width(msg->hpp.ego_lane_width);
  lane_list_hpp.set_confidence(msg->hpp.confidence);
  lane_list.set_hpp(lane_list_hpp);
  std::vector<legionclaw::interface::RoadMark> road_marks;
  for (auto it_road_marks : msg->road_marks) {
    legionclaw::interface::RoadMark lane_list_road_mark;
    lane_list_road_mark.set_longitude_dist(it_road_marks.longitude_dist);
    lane_list_road_mark.set_lateral_dist(it_road_marks.lateral_dist);
    lane_list_road_mark.set_x(it_road_marks.x);
    lane_list_road_mark.set_y(it_road_marks.y);
    lane_list_road_mark.set_z(it_road_marks.z);
    lane_list_road_mark.set_confidence(it_road_marks.confidence);
    lane_list_road_mark.set_type(
        (legionclaw::common::RoadMarkType)it_road_marks.type);
    road_marks.emplace_back(lane_list_road_mark);
  }
  lane_list.set_road_marks(&road_marks);

  instance_->HandleLaneList(lane_list);
}

template <typename T>
void LcmMessageManager<T>::HandleObuCmdMsgMessage(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const lcm_interface::ObuCmdMsg* msg) {
  if (is_init_ == false)
    return;

  legionclaw::interface::ObuCmdMsg obu_cmd_msg;
  MESSAGE_HEADER_PARSER(obu_cmd_msg)
  obu_cmd_msg.set_id(msg->id);
  obu_cmd_msg.set_name(msg->name);
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  for (auto it_obu_cmd_list : msg->obu_cmd_list) {
    legionclaw::interface::ObuCmd obu_cmd_msg_obu_cmd;
    obu_cmd_msg_obu_cmd.set_code(it_obu_cmd_list.code);
    obu_cmd_msg_obu_cmd.set_val(it_obu_cmd_list.val);
    obu_cmd_list.emplace_back(obu_cmd_msg_obu_cmd);
  }
  obu_cmd_msg.set_obu_cmd_list(&obu_cmd_list);

  instance_->HandleObuCmdMsg(obu_cmd_msg);
}

template <typename T> void LcmMessageManager<T>::Run() {
  while (0 == lcm_->handle())
    ;
}

template <typename T> void LcmMessageManager<T>::Stop() {
  if (handle_message_thread_ != nullptr && handle_message_thread_->joinable()) {
    handle_message_thread_->join();
    handle_message_thread_.reset();
    AINFO << "handle_message_thread stopped [ok].";
  }
}
} // namespace prediction
} // namespace legionclaw
#endif
