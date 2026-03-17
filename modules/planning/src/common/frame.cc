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
 * @file frame.cc
 **/
#include "frame.h"

#include <algorithm>
#include <limits>

#include "modules/common/math/math_tools.h"
#include "modules/planning/src/common/map_matcher/map_matcher.h"

namespace legionclaw {
namespace planning {
using namespace legionclaw::common;
constexpr double kSegmentationEpsilon = 0.2;

void Frame::Init(const legionclaw::planning::PlanningConf *planning_conf) {
  planning_params_ = planning_conf;
  VariableInit();
}

void Frame::VariableInit(){
  last_planning_trajectory_ = PlanningTrajectory();
  //改成默认自主换道
  lane_changing_mode_ = Lane_Changing_Mode::Autonomous;
  lane_change_direction_ = common::Direction::DIR_INVALID;
  lane_change_state_ = Lane_Change_State::LANE_CHANGE_INVALID;
  manual_lc_plus_ = 0;
  yield_action_ = YieldAction::NO_ACTION;
  yield_distance_ = 0.0;
  greenbelt_occlusion_done_ = false;
}

interface::PathPoint Frame::GetStartPose() {
  interface::PathPoint start_pose;
  if (last_planning_trajectory_.trajectory_points_size() > 0) {
    start_pose.set_x(last_planning_trajectory_.trajectory_points()
                         .front()
                         .path_point()
                         .x());
    start_pose.set_y(last_planning_trajectory_.trajectory_points()
                         .front()
                         .path_point()
                         .y());
    start_pose.set_theta(last_planning_trajectory_.trajectory_points()
                             .front()
                             .path_point()
                             .theta());
  } else {
    start_pose.set_x(vehicle_state_.pose.x());
    start_pose.set_y(vehicle_state_.pose.y());
    start_pose.set_z(vehicle_state_.pose.z());
    start_pose.set_theta(vehicle_state_.pose.theta());
  }

  return start_pose;
}

void Frame::UpdateLaneChangeCmd(
    const interface::ObuCmdMsg &obu_cmd_msg){
  static legionclaw::interface::Time last_obu_cmd_msg_time;
  if (fabs(TimeTool::GetTimeDiff(last_obu_cmd_msg_time,
      obu_cmd_msg.header().stamp())) == 0.0) {
    return;
  }
  last_obu_cmd_msg_time = obu_cmd_msg.header().stamp();
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  obu_cmd_msg.obu_cmd_list(obu_cmd_list);
  for (auto obu_cmd : obu_cmd_list) {
    switch (obu_cmd.code()) {
      case 10022:  // 左右换道选择：0：invalid； 1：left；2：right；3：xleft；4：xright；
        if (obu_cmd.val() == 0) {
          lane_change_direction_ = common::Direction::DIR_INVALID; //invalid
        } else if (obu_cmd.val() == 1) {
          lane_change_direction_ = common::Direction::LEFT; //left change lane
          manual_lc_plus_ = -1;
          lane_change_state_ = Lane_Change_State::LANE_CHANGING;
        } else if (obu_cmd.val() == 2) {
          lane_change_direction_ = common::Direction::RIGHT; //right change lane;
          manual_lc_plus_ = 1;
          lane_change_state_ = Lane_Change_State::LANE_CHANGING;
        }
        break;
      case 10023:  // 换道功能：0：invalid；1：auto自主；2：手动；
        if (obu_cmd.val() == 0) {
          lane_changing_mode_ = Lane_Changing_Mode::Invalid;
        } else if (obu_cmd.val() == 1) {
          lane_changing_mode_ = Lane_Changing_Mode::Autonomous; //auto
        } else if (obu_cmd.val() == 2) {
          lane_changing_mode_ = Lane_Changing_Mode::Manual; //manual
        }
        break;
      default:
        break;
    }
  }
  return;
}

bool Frame::ExtractReferenceLinesInfo(
    const std::list<reference_line::ReferenceLine> &reference_lines) {
  reference_lines_.clear();
  reference_lines_.shrink_to_fit();
  auto ref_line_iter = reference_lines.begin();
  while (ref_line_iter != reference_lines.end()) {
    // frame
    // 里面也将reference_lines_进行赋值，为了以前的lattice直接使用，不用通过访问reference_line_info来获取
    reference_lines_.emplace_back(*ref_line_iter);
    ++ref_line_iter;
  }
  return true;
}

bool Frame::CalTrajectoryMatchInfo() {
  double dist_min = DBL_MAX;
  if (last_planning_trajectory_.trajectory_points_size() > 0) {
    trajectory_match_info_.lon_index = MapMatcher::QueryNearestPointWithBuffer(
        last_planning_trajectory_.trajectory_points(),
        {vehicle_state_.pose.x(), vehicle_state_.pose.y(),
         vehicle_state_.pose.z()},
        vehicle_state_.pose.theta(), 1.0e-6, dist_min);
    trajectory_match_info_.lat_offset = dist_min;
  }

  return true;
}

bool Frame::ExtractGuideInfo(const interface::GuideInfo &guide_info) {
  guide_info_ =  guide_info;
  return true;
}

bool Frame::ExtractTrafficEvents(const interface::TrafficEvents &traffic_events) {
  traffic_events_ =  traffic_events;
  return true;
}

bool Frame::ExtractVehicleState(
    const interface::Location &location_, const interface::Chassis &chassis_,
    const legionclaw::interface::VehicleParam vehicle_param) {
  legionclaw::interface::PathPoint pose;
  pose.set_x((double)location_.utm_position().x());
  pose.set_y((double)location_.utm_position().y());
  pose.set_z((double)location_.utm_position().z());
  pose.set_theta(NormalizeAngle((double)location_.heading()));
  // pose.set_theta(NormalizeAngle(D2R(90.0 - (double)location_.heading())));
  // pose.set_theta(D2R(NormalizeAngle(location_.heading())));
  // pose.set_kappa((double)local_view.location().utm_position().z());
  // pose.set_ddkappa((double)local_view.location().utm_position().z());
  // pose.set_lane_id((double)local_view.location().utm_position().z());
  vehicle_state_.pose = pose;
  vehicle_state_.speed = chassis_.speed_mps();
  vehicle_state_.moving_status = chassis_.moving_status();

  //将定位转换到几何中心
  double preview_x = vehicle_state_.pose.x() +
                     0.5 * planning_params_->vehicle_param().wheelbase() *
                         std::cos(vehicle_state_.pose.theta());
  double preview_y = vehicle_state_.pose.y() +
                     0.5 * planning_params_->vehicle_param().wheelbase() *
                         std::sin(vehicle_state_.pose.theta());

  double preview_theta = vehicle_state_.pose.theta();

  math::Polygon2d car_safety_border(
      preview_x, preview_y, preview_theta,
      planning_params_->vehicle_param().width() + planning_params_->safe_width(),
      planning_params_->vehicle_param().length() +
          planning_params_->safe_length());
  vehicle_state_.vehicle_polygon2d = car_safety_border;

  //TODO采用前方向盘
  vehicle_state_.steer =
      D2R(chassis_.front_steering_value() / vehicle_param.steer_ratio());
  vehicle_state_.gear = chassis_.gear_location();
  // frame_.vehicle_state_.acceleration = local_view.chassis_.accel_value_;
  return true;
}

bool Frame::ExtractTrafficLightInfo(
    const interface::TrafficLightMsg &traffic_light_msg) {
  traffic_light_msg.traffic_light(traffic_lights_);

  return true;
}

void Frame::ExtractParkingInfo(const interface::ParkingInfo &parking_info) {
    static legionclaw::interface::Time last_parking_info_time;
  if (fabs(TimeTool::GetTimeDiff(last_parking_info_time,
      parking_info.header().stamp())) == 0.0) {
    return;
  }
  last_parking_info_time = parking_info.header().stamp();
  parking_info_ = parking_info;
  parking_info_.set_theta(NormalizeAngle(parking_info_.theta()));
  // parking_info_.set_theta(NormalizeAngle(parking_info_.theta()));
  return;
}

void Frame::ExtractParkingOutInfo(
    const interface::ParkingOutInfo &parking_out_info) {
    static legionclaw::interface::Time last_parking_out_info_time;
  if (fabs(TimeTool::GetTimeDiff(last_parking_out_info_time,
      parking_out_info.header().stamp())) == 0.0) {
    return;
  }
  last_parking_out_info_time = parking_out_info.header().stamp();
  parking_out_info_ = parking_out_info;
  parking_out_info_.set_theta(NormalizeAngle(parking_out_info_.theta()));
  // parking_info_.set_theta(NormalizeAngle(parking_info_.theta()));
  return;
}

void Frame::ExtractDrivableRegion(
    const legionclaw::interface::DrivableRegion &drivable_region) {
  drivable_region_ = drivable_region;
}

void Frame::ExtractStopInfo(const interface::StopInfo &stop_info) {
  static legionclaw::interface::Time last_stop_info_time;
  if (fabs(TimeTool::GetTimeDiff(last_stop_info_time,
      stop_info.header().stamp())) == 0.0) {
    return;
  }
  last_stop_info_time = stop_info.header().stamp();
  if (stop_info.stop_points_size() == 0) {
    terminal_stop_point_.set_type(-1);
    return;
  }
  for (auto stop_point : *stop_info.mutable_stop_points()) {
    if (stop_point.type() == STOP_TYPE_TERMINAL) {
      stop_point.set_theta(NormalizeAngle(stop_point.theta()));
      terminal_stop_point_ = stop_point;
    }
  }
  return;
}

bool Frame::ExtractObstacleList(
    const interface::PredictionObstacles &prediction_obstacles) {
  //   prediction_obstacles.prediction_obstacles(obstacle_list_);
  if (CopyObstacleMsg(prediction_obstacles)) {
    //添加交通规则：红绿灯处添加虚拟静止障碍物
    TrafficLightDecision();
    //添加绿化带决策，绿化带处添加一定条件下的生命周期虚拟静止障碍物
    GreenbeltOcclusionDecision();
    //TODO 障碍物排序决策
    SortObstacles();
    //创建大车避让决策信息
    CreateDecisionOfTruckAvoidance();
    return true;
  };
  AERROR << "ExtractObstacleList() failed";
  return false;
}

void Frame::Log() {
  if (1) {
    ofstream outfile("./log/smooth_line.csv", std::ofstream::app);
    outfile << std::fixed;
    outfile << std::setprecision(6);
    // for (auto &lane_point : smooth_line_) {
    for (auto &reference_point : reference_lines_.at(1).reference_points_) {
      stringstream sstream;
      sstream << reference_point.x() << "," << reference_point.y() << ","
              << reference_point.kappa() << "," << reference_point.dkappa() << ",";
      outfile << sstream.str() << std::endl;
    }
    outfile.close();
  }
}

void Frame::SortObstacles() {
  //从近到远进行排序
  if (!obstacle_list_.empty()) {
    std::sort(
        obstacle_list_.begin(), obstacle_list_.end(),
        [](const PredictionObstacleMsg &a, const PredictionObstacleMsg &b) {
          return a.perception_obstacle().obj_lon_index() <
                 b.perception_obstacle().obj_lon_index();
        });
  }
}

void Frame::CreateDecisionOfTruckAvoidance() {
  yield_action_ = YieldAction::NO_ACTION;
  yield_distance_ = 0.0;
  if (truck_list_.empty() || (map_match_info_.lon_index == -1)) {
    return;
  }

  // 分三种情况：1、左边 2、右边 3、左右两边
  int left_truck_num = 0;
  int right_truck_num = 0;
  double dis_min = DBL_MAX;
  if (reference_lines_.size() < 3) return;
  auto current_referencepoints =
      reference_lines_.at(map_match_info_.current_lane_index).ReferencePoints();
  for (auto it : truck_list_) {
    int match_index = MapMatcher::QueryNearestPointWithBuffer(
        current_referencepoints, {it.position().x(), it.position().y()},
        it.theta(), 1.0e-6, dis_min);
    if (match_index == -1) continue;
    //只针对前方障碍物处于0～20m内的障碍物进行决策
    //TODO 根据测试情况距离后续跟场景挂钩
    double decision_distance = planning_params_->path_density() *
                               (match_index - map_match_info_.lon_index);
    if (decision_distance >= 0 &&
        decision_distance <
            planning_params_->decision_yield_distance() + 0.5 * it.length()) {
      if (it.lane_position() == legionclaw::common::LanePosition::LEFT_LANE) {
        ++left_truck_num;
        yield_action_ = YieldAction::YIELD_TO_RIGHT;
        yield_distance_ = -planning_params_->yield_offset();
      } else {
        ++right_truck_num;
        yield_action_ = YieldAction::YIELD_TO_LEFT;
        yield_distance_ = planning_params_->yield_offset();
      }
      if (left_truck_num > 0 && right_truck_num > 0) {
        yield_action_ = YieldAction::KEEP_IN_MIDDLE;
        yield_distance_ = 0.0;
        return;
      }
    }
  }
  return;
}

bool Frame::CopyObstacleMsg(
    const interface::PredictionObstacles &prediction_obstacles) {
  obstacle_list_.clear();
  obstacle_list_.shrink_to_fit();
  truck_list_.clear();
  truck_list_.shrink_to_fit();
  std::vector<legionclaw::interface::PredictionObstacle> prediction_obstacles_list;
  prediction_obstacles.prediction_obstacles(prediction_obstacles_list);
  auto current_road = guide_info_.current_road();
  for (auto it_prediction_obstacles : prediction_obstacles_list) {
    PredictionObstacleMsg prediction_out_array_prediction_obstacle;
    ObstacleMsg prediction_obstacle_perception_obstacle;
    prediction_obstacle_perception_obstacle.set_id(
        it_prediction_obstacles.perception_obstacle().id());

    legionclaw::interface::Point3D
        prediction_out_array_prediction_obstacle_perception_obstacle_position;
    prediction_out_array_prediction_obstacle_perception_obstacle_position.set_x(
        it_prediction_obstacles.perception_obstacle().position().x());
    prediction_out_array_prediction_obstacle_perception_obstacle_position.set_y(
        it_prediction_obstacles.perception_obstacle().position().y());
    prediction_out_array_prediction_obstacle_perception_obstacle_position.set_z(
        it_prediction_obstacles.perception_obstacle().position().z());
    prediction_obstacle_perception_obstacle.set_position(
        prediction_out_array_prediction_obstacle_perception_obstacle_position);
    prediction_obstacle_perception_obstacle.set_theta(
        it_prediction_obstacles.perception_obstacle().theta());
    legionclaw::interface::Point3D
        prediction_out_array_prediction_obstacle_perception_obstacle_velocity;
    prediction_out_array_prediction_obstacle_perception_obstacle_velocity.set_x(
        it_prediction_obstacles.perception_obstacle().velocity().x());
    prediction_out_array_prediction_obstacle_perception_obstacle_velocity.set_y(
        it_prediction_obstacles.perception_obstacle().velocity().y());
    prediction_out_array_prediction_obstacle_perception_obstacle_velocity.set_z(
        it_prediction_obstacles.perception_obstacle().velocity().z());
    prediction_obstacle_perception_obstacle.set_velocity(
        prediction_out_array_prediction_obstacle_perception_obstacle_velocity);
    prediction_obstacle_perception_obstacle
        .set_has_velocity(
            it_prediction_obstacles.perception_obstacle().has_velocity());
    prediction_obstacle_perception_obstacle.set_length(
        it_prediction_obstacles.perception_obstacle().length());
    prediction_obstacle_perception_obstacle.set_width(
        it_prediction_obstacles.perception_obstacle().width());
    prediction_obstacle_perception_obstacle.set_height(
        it_prediction_obstacles.perception_obstacle().height());
    // 当前位置的polygon计算与顶点的相对位置计算:静态障碍物
    std::vector<Vec2d> points;
    vector<Vec2d> raletive_positions;
    legionclaw::interface::PerceptionObstacle perception_obstacle =
        it_prediction_obstacles.perception_obstacle();
    size_t contour_num = perception_obstacle.polygon_point_size();
    if (contour_num < 3) continue;
    Vec2d position(perception_obstacle.position().x(),
                   perception_obstacle.position().y());
    for (auto it_polygon_point :
         *(perception_obstacle.mutable_polygon_point())) {
      Vec2d point;
      point.set_x(it_polygon_point.x());
      point.set_y(it_polygon_point.y());
      Vec2d raletive_position = point - position;
      raletive_positions.emplace_back(raletive_position);
      points.emplace_back(point);
    }
    // AERROR << "+++++++++++++++++++: " << points[0].x();
    // AERROR << "+++++++++++++++++++: " << points[1].x();
    Polygon2d polygon(points);
    prediction_obstacle_perception_obstacle.set_polygon(
        polygon);

    prediction_obstacle_perception_obstacle
        .set_tracking_time(
            it_prediction_obstacles.perception_obstacle().tracking_time());
    prediction_obstacle_perception_obstacle.set_type(
        (legionclaw::common::ObstacleType)it_prediction_obstacles
            .perception_obstacle()
            .type());
    prediction_obstacle_perception_obstacle.set_confidence(
        it_prediction_obstacles.perception_obstacle().confidence());
    prediction_obstacle_perception_obstacle.set_timestamp(
        it_prediction_obstacles.perception_obstacle().timestamp());
    prediction_obstacle_perception_obstacle
        .set_confidence_type(
            it_prediction_obstacles.perception_obstacle().confidence_type());

    // TODO oncoming待优化，位置属性
    if (map_match_info_.lon_index != -1 && !ReferenceLines().empty()) {
      int obj_lon_index = -1;
      double dis_min_obj_nearest = DBL_MAX;
      math::Vec2d obj_position;
      double x = prediction_obstacle_perception_obstacle.position().x();
      double y = prediction_obstacle_perception_obstacle.position().y();
      obj_position = {x, y};
      // TODO排序按障碍物尾部进行排序
      // 交叉路口障碍物处理
      if (current_road.road_type() == 2) {
        if (prediction_obstacle_perception_obstacle.length() > 20) {
          int index_min = INT_MAX;
          for (const auto &point :
               prediction_obstacle_perception_obstacle.polygon().points()) {
            int corners_index = MapMatcher::QueryVerticalDistanceWithBuffer(
                ReferenceLine(1).ReferencePoints(), point,
                VehicleState().pose.theta(), 1.0e-6, dis_min_obj_nearest);
            index_min = std::min(index_min, corners_index);
            obj_lon_index = index_min;
            if (corners_index == -1) break;
          }
        } else {
          obj_lon_index = MapMatcher::QueryNearestPointWithBuffer(
              ReferenceLine(1).ReferencePoints(), obj_position,
              prediction_obstacle_perception_obstacle.theta(), 1.0e-6,
              dis_min_obj_nearest, current_road.turn_type());
        }
      } else {
        // 非路口障碍物处理
        if (prediction_obstacle_perception_obstacle.length() > 20) {
          int index_min = INT_MAX;
          for (const auto &point :
               prediction_obstacle_perception_obstacle.polygon().points()) {
            int corners_index = MapMatcher::QueryVerticalDistanceWithBuffer(
                ReferenceLine(1).ReferencePoints(), point,
                VehicleState().pose.theta(), 1.0e-6, dis_min_obj_nearest);
            index_min = std::min(index_min, corners_index);
            obj_lon_index = index_min;
            if (corners_index == -1) break;
          }
        } else {
          obj_lon_index = MapMatcher::QueryNearestPointWithBuffer(
              ReferenceLine(1).ReferencePoints(), obj_position,
              VehicleState().pose.theta(), 1.0e-6, dis_min_obj_nearest);
        }
      }
      if (prediction_obstacle_perception_obstacle.length() <= 20 &&
          obj_lon_index != -1) {
        obj_lon_index -= std::max(
            int(0.5 * prediction_obstacle_perception_obstacle.length() /
                planning_params_->path_density()),
            0);
      }
      // 虚拟障碍物向后延长一定距离
      if (prediction_obstacle_perception_obstacle.id() < -1 &&
          obj_lon_index != -1) {
        double dis_min_obj = std::max(10.0, 5 + 6 * VehicleState().speed);
        obj_lon_index -=
            std::max(int(dis_min_obj / planning_params_->path_density()), 0);
      }
      prediction_obstacle_perception_obstacle.set_obj_lon_index(obj_lon_index);
    }

    legionclaw::interface::Point3D
        prediction_out_array_prediction_obstacle_perception_obstacle_acceleration;
    prediction_out_array_prediction_obstacle_perception_obstacle_acceleration
        .set_x(
            it_prediction_obstacles.perception_obstacle().acceleration().x());
    prediction_out_array_prediction_obstacle_perception_obstacle_acceleration
        .set_y(
            it_prediction_obstacles.perception_obstacle().acceleration().y());
    prediction_out_array_prediction_obstacle_perception_obstacle_acceleration
        .set_z(
            it_prediction_obstacles.perception_obstacle().acceleration().z());
    prediction_obstacle_perception_obstacle.set_acceleration(
        prediction_out_array_prediction_obstacle_perception_obstacle_acceleration);

    prediction_obstacle_perception_obstacle
        .set_lane_position((legionclaw::common::LanePosition)it_prediction_obstacles
                               .perception_obstacle()
                               .lane_position());

    prediction_obstacle_perception_obstacle.set_sub_type(
        (legionclaw::common::ObstacleSubType)it_prediction_obstacles
            .perception_obstacle()
            .sub_type());

    prediction_obstacle_perception_obstacle
        .set_height_above_ground(it_prediction_obstacles.perception_obstacle()
                                     .height_above_ground());
    std::vector<double> position_covariance;
    for (auto it_position_covariance :
         it_prediction_obstacles.perception_obstacle()
              .position_covariance()) {
      double position_covariance_item;
      position_covariance_item = it_position_covariance;
      position_covariance.emplace_back(position_covariance_item);
    }
    prediction_obstacle_perception_obstacle
        .set_position_covariance(&position_covariance);
    std::vector<double> velocity_covariance;
    for (auto it_velocity_covariance :
         it_prediction_obstacles.perception_obstacle()
              .velocity_covariance()) {
      double velocity_covariance_item;
      velocity_covariance_item = it_velocity_covariance;
      velocity_covariance.emplace_back(velocity_covariance_item);
    }
    prediction_obstacle_perception_obstacle
        .set_velocity_covariance(&velocity_covariance);
    std::vector<double> acceleration_covariance;
    for (auto it_acceleration_covariance :
         it_prediction_obstacles.perception_obstacle()
              .acceleration_covariance()) {
      double acceleration_covariance_item;
      acceleration_covariance_item = it_acceleration_covariance;
      acceleration_covariance.emplace_back(acceleration_covariance_item);
    }
    prediction_obstacle_perception_obstacle
        .set_acceleration_covariance(&acceleration_covariance);
    prediction_obstacle_perception_obstacle
        .set_light_status(
            it_prediction_obstacles.perception_obstacle().light_status());

    //挑选出左/右边车道的大货车
    if ((prediction_obstacle_perception_obstacle.sub_type() ==
             legionclaw::common::ObstacleSubType::ST_BUS ||
         prediction_obstacle_perception_obstacle.sub_type() ==
             legionclaw::common::ObstacleSubType::ST_TRUCK) &&
        (prediction_obstacle_perception_obstacle.lane_position() ==
             legionclaw::common::LanePosition::LEFT_LANE ||
         prediction_obstacle_perception_obstacle.lane_position() ==
             legionclaw::common::LanePosition::RIGHT_LANE)) {
      truck_list_.push_back(prediction_obstacle_perception_obstacle);
    }

    prediction_out_array_prediction_obstacle.set_perception_obstacle(
        prediction_obstacle_perception_obstacle);
    prediction_out_array_prediction_obstacle.set_timestamp(
        it_prediction_obstacles.timestamp());
    prediction_out_array_prediction_obstacle.set_predicted_period(
        it_prediction_obstacles.predicted_period());
    // 增加polygon的计算
    std::vector<std::vector<Polygon2d>> polygon_lists;
    std::vector<legionclaw::interface::TrajectoryInPrediction> trajectory;
    for (auto it_trajectory : *it_prediction_obstacles.mutable_trajectory()) {
      legionclaw::interface::TrajectoryInPrediction
          prediction_out_array_prediction_obstacle_trajectory_in_prediction;
      prediction_out_array_prediction_obstacle_trajectory_in_prediction
          .set_probability(it_trajectory.probability());
      // std::vector<legionclaw::interface::TrajectoryPointInPrediction>
      //     trajectory_points;
      std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
      std::vector<Polygon2d> polygon_list;
      // int step_count = 0;
      // int step = perception_obstacle.width() / planning_params_->path_density();
      for (auto it_trajectory_points : it_trajectory.trajectory_points()) {
        legionclaw::interface::TrajectoryPoint
            prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction;
        // legionclaw::interface::TrajectoryPointInPrediction
        //     prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction;
        legionclaw::interface::PathPoint
            prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point;
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_x(it_trajectory_points.path_point().x());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_y(it_trajectory_points.path_point().y());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_z(it_trajectory_points.path_point().z());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_theta(NormalizeAngle(it_trajectory_points.path_point().theta()));
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_kappa(it_trajectory_points.path_point().kappa());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_s(it_trajectory_points.path_point().s());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_dkappa(it_trajectory_points.path_point().dkappa());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_ddkappa(it_trajectory_points.path_point().ddkappa());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_lane_id(it_trajectory_points.path_point().lane_id());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_x_derivative(it_trajectory_points.path_point().x_derivative());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point
            .set_y_derivative(it_trajectory_points.path_point().y_derivative());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_path_point(
                prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction_path_point);
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_v(it_trajectory_points.v());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_a(it_trajectory_points.a());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_relative_time(it_trajectory_points.relative_time());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_da(it_trajectory_points.da());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_is_steer_valid(it_trajectory_points.is_steer_valid());

        //TODO区分前后轮
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_front_steer(it_trajectory_points.front_steer());
        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_rear_steer(it_trajectory_points.rear_steer());

        prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction
            .set_gear(it_trajectory_points.gear());
        trajectory_points.emplace_back(
            prediction_out_array_prediction_obstacle_trajectory_in_prediction_trajectory_point_in_prediction);

        // create polygon list
        if (it_trajectory.probability() > 0 /*&& (step_count > step || step_count == 0)*/) {
          // step_count = 0;
          Polygon2d a_polygon;
          std::vector<Vec2d> contour_points;
          //障碍物polygon优化
          if (it_trajectory_points.path_point().s() >=
                  0.5 * planning_params_->vehicle_param().length() &&
              contour_num == 4) {
            a_polygon =
                Polygon2d(it_trajectory_points.path_point().x(),
                          it_trajectory_points.path_point().y(),
                          it_trajectory_points.path_point().theta(),
                          prediction_obstacle_perception_obstacle.width(),
                          prediction_obstacle_perception_obstacle.length());
          } else {
            for (size_t ic = 0; ic < contour_num; ++ic) {
              Vec2d traj_position(it_trajectory_points.path_point().x(),
                                  it_trajectory_points.path_point().y());
              contour_points.push_back(raletive_positions.at(ic) +
                                       traj_position);
            }
            a_polygon = Polygon2d(contour_points);
          }
          polygon_list.emplace_back(a_polygon);
        }
        // step_count++;
      }
      polygon_lists.emplace_back(polygon_list);
      prediction_out_array_prediction_obstacle_trajectory_in_prediction
          .set_trajectory_points(&trajectory_points);
      trajectory.emplace_back(
          prediction_out_array_prediction_obstacle_trajectory_in_prediction);
    }
    prediction_out_array_prediction_obstacle.set_polygon_lists(&polygon_lists);
    prediction_out_array_prediction_obstacle.set_trajectory(&trajectory);
    // legionclaw::planning::PredictionObstacleMsg::Intent intent;
    // it_prediction_obstacles.set_intent(&intent);
    prediction_out_array_prediction_obstacle.set_intent(
        it_prediction_obstacles.intent());
    prediction_out_array_prediction_obstacle.set_is_static(
        it_prediction_obstacles.is_static());

    obstacle_list_.emplace_back(prediction_out_array_prediction_obstacle);
  }
  return true;
}

void Frame::TrafficLightDecision() {
  if (!planning_params_->enable_traffic_light_behavior()) {
    return;
  }
  // 交通信号获取
  legionclaw::interface::TrafficEvents traffic_events = traffic_events_;
  if (traffic_events.junction_info().light_flag() ==
      legionclaw::common::IsValid::VALID) {
    if (traffic_events.junction_info().light_color() ==
        legionclaw::common::TrafficLightColor::GREEN) {
      return;
    } else if (traffic_events.junction_info().light_color() ==
                   legionclaw::common::TrafficLightColor::RED ||
               traffic_events.junction_info().light_color() ==
                   legionclaw::common::TrafficLightColor::YELLOW) {
      if (reference_lines_.size() < 3) return;
      double distance_to_stop = traffic_events.junction_info().distance_to_stop() + 7.0;
      if (distance_to_stop < 0) {
        distance_to_stop = DBL_MAX;
      }
      common::math::Vec2d adc_position = {VehicleState().pose.x(), VehicleState().pose.y()};
      common::math::Vec2d stop_line_point = {
          VehicleState().pose.x() + distance_to_stop * cos(VehicleState().pose.theta()),
          VehicleState().pose.y() + distance_to_stop * sin(VehicleState().pose.theta())};
      int matched_point_index = -1;
      std::vector<ReferencePoint> reference_points =
          reference_lines_.at(map_match_info_.current_lane_index).ReferencePoints();
      double stop_line_s = MapMatcher::MatchToPath(reference_points, {stop_line_point.x(), stop_line_point.y(), 0},
                                    VehicleState().pose.theta(), matched_point_index).s();
      for (auto it : traffic_events.junction_info().stop_line()) {
        common::math::Vec2d traffic_light_point = {it.x(), it.y()};
        double distance = common::util::DistanceXY(traffic_light_point, adc_position);
        if (distance < distance_to_stop) {
          distance_to_stop = distance;
          stop_line_point = traffic_light_point;
        }
        ReferencePoint stop_line_matched_ref_point =
            MapMatcher::MatchToPath(reference_points, {it.x(), it.y(), 0},
                                    VehicleState().pose.theta(), matched_point_index);
        if (stop_line_matched_ref_point.s() > stop_line_s) {
          stop_line_s = stop_line_matched_ref_point.s();
        }
      }

      // 在frenet坐标系下精准判断是否过停止线
      ReferencePoint ego_matched_ref_point = MapMatcher::MatchToPath(
          reference_points, {VehicleState().pose.x(), VehicleState().pose.y(), 0},
          VehicleState().pose.theta(), matched_point_index);
      // 黄/红灯场景下车尾已过停止线,但是上一个状态为非红绿灯停车状态，则不响应黄/红灯
      if (stop_line_s <
              ego_matched_ref_point.s() - planning_params_->vehicle_param().back_edge_to_center() &&
          last_behaviour_.lon_state !=
              legionclaw::interface::ADCTrajectory::BehaviourLonState::TRAFFIC_LIGHT_STOP_STATE) {
        return;
      }

      // create virtual stop wall
      std::string stop_wall_id = "traffic_light_stop_wall";
      const auto obstacle =
          CreateStopObstacle(reference_lines_.at(map_match_info_.current_lane_index), stop_wall_id,
                             stop_line_point, distance_to_stop);
      if (!obstacle) {
        AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
        return;
      }
      obstacle_list_.emplace_back(*obstacle);
    }
  }
}

void Frame::GreenbeltOcclusionDecision()
{
  // 获取车当前位置到绿化带停车点的距离
  common::math::Vec2d adc_position = {VehicleState().pose.x(), VehicleState().pose.y()};
  const double distance =
      common::util::DistanceXY(map_match_info_.greenbelt_occlusion_point, adc_position);

  if (map_match_info_.lon_index == -1 || distance > 7) {
    greenbelt_occlusion_done_ = false;
    return;
  }
  // check if greenbelt-occlusion-stop already finished, set by scenario/stage
  if (fabs(vehicle_state_.speed) < 0.01) {
    if (is_stopped_first_hit_ == true) last_time_vehicle_is_stopped_or_not_ = TimeTool::Now2Ms();

    if (is_stopped_first_hit_ == true) {
      is_stopped_first_hit_ = false;
    }
    int64_t cur_time = TimeTool::Now2Ms();
    int64_t diff = cur_time - last_time_vehicle_is_stopped_or_not_;
    if (diff >= 2000) {
      is_stopped_ = true;
    } else {
      is_stopped_ = false;
    }
  } else {
    is_stopped_ = false;
    is_stopped_first_hit_ = true;
  }
  // TODO绿化带处距离虚拟墙的位置若大于1m启动自动驾驶，停车点处精准停车待优化，否则会一直处于stop状态
  if (distance < 2.0 && is_stopped_) {
    greenbelt_occlusion_done_ = true;
  }
  if (greenbelt_occlusion_done_) {
    return;
  }
  // create virtual stop wall
  std::string stop_wall_id = "greenbelt_occlusion_stop_wall";
  const auto obstacle =
      CreateStopObstacle(reference_lines_.at(map_match_info_.current_lane_index), stop_wall_id,
                         map_match_info_.greenbelt_occlusion_point, guide_info_.next_dis());
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return;
  }
  obstacle_list_.emplace_back(*obstacle);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const PredictionObstacleMsg *Frame::CreateStopObstacle(
    const reference_line::ReferenceLine &reference_line,
    const std::string &obstacle_id, const common::math::Vec2d &stop_point,
    const double lane_s) {
  if (reference_line.ReferencePointSize() == 0) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }
  // 获取虚拟障碍物位置(ego_s+distance_to_stop+0.5*width)
  int dest_lane_index = map_match_info_.lon_index +
                        (int)(lane_s + 0.1) / planning_params_->path_density();
  auto dest_point = reference_line.ReferencePoints(dest_lane_index);
  double lane_left_width = dest_point.left_road_width();
  double lane_right_width = dest_point.right_road_width();

  Box2d stop_wall_box{{stop_point.x(), stop_point.y()},
                      dest_point.theta(),
                      0.2,
                      3 * (lane_left_width + lane_right_width)};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box,
                                     dest_lane_index);
}

const PredictionObstacleMsg *Frame::CreateStaticVirtualObstacle(
    const std::string &id, const Box2d &box, const int &dest_lane_index) {
  ObstacleMsg obstacle_msg;
  legionclaw::interface::Point3D position;
  position.set_x(box.center_x());
  position.set_y(box.center_y());
  position.set_z(0.0);
  obstacle_msg.set_position(position);
  obstacle_msg.set_theta(box.heading());
  legionclaw::interface::Point3D velocity;
  velocity.set_x(0.0);
  velocity.set_y(0.0);
  velocity.set_z(0.0);
  obstacle_msg.set_velocity(velocity);
  obstacle_msg.set_length(box.length());
  obstacle_msg.set_width(box.width());
  obstacle_msg.set_obj_lon_index(dest_lane_index);

  Polygon2d box_polygon = Polygon2d(box.center_x(), box.center_y(),
                                    box.heading(), box.width(), box.length());
  obstacle_msg.set_polygon(box_polygon);
  obstacle_msg.set_lane_position(LanePosition::EGO_LANE);
  PredictionObstacleMsg *virtual_obstacle = new PredictionObstacleMsg();
  virtual_obstacle->SetId(id);
  virtual_obstacle->set_perception_obstacle(obstacle_msg);
  return virtual_obstacle;
}

}  // namespace planning
}  // namespace legionclaw
