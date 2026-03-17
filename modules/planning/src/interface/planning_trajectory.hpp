/**
 * @file              planning_trajectory.hpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-07-07 06:29:25
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#pragma once

#include <iostream>

#include "modules/planning/src/common/enum.h"
#include "modules/common/interface/trajectory.hpp"

using namespace std;

namespace legionclaw {
namespace planning {
/**
 * @struct TrajectoryCost
 * @brief TrajectoryCost
 */
struct TrajectoryCost {
 public:
  double rightmost_cost;
  double priority_cost;     // 0 to 1
  double transition_cost;   // 0 to 1
  double closest_obj_cost;  // 0 to 1
  double blocked_cost;
  double cost;
  double closest_obj_velocity;
  double closest_obj_acceleration;
  double closest_obj_distance;
  double closest_obj_lateral_distance;
  double closest_obj_collision_dis;
  double closest_obj_collisiontime;
  double closest_obj_theta_abs;
  double closest_obj_theta_veh;
  int closest_obj_id;

  double curvature_cost;
  double horizon_cost;
  double junction_close_cost;
  double lane_change_cost;
  double potential_cost;
  double lateral_left_cost;
  double lateral_right_cost;
  double longitudinal_cost;
  bool is_fully_block;
  bool is_behind_obj;
  bool is_passage_safe_f;
  bool is_passage_safe_b;
  bool is_passage_safe_dclc;
  legionclaw::planning::DynamicObjectsPose dynamic_objects_pose;
  std::string id = "";
  // std::vector<std::pair<int, double>> lateral_costs;
  // std::vector<double> lateral_left_costs;
  // std::vector<double> lateral_right_costs;
};

/**
 * @class PlanningTrajectory
 *
 * @brief 消息的描述.
 */
class PlanningTrajectory : public legionclaw::interface::Trajectory {
 public:
  // PlanningTrajectory() = delete;
  PlanningTrajectory() = default;

  virtual ~PlanningTrajectory() = default;

  int global_index() const { return global_index_; }
  int lon_id() const { return lon_id_; }
  int lat_id() const { return lat_id_; }
  inline const TrajectoryCost &trajectory_cost() const {
    return trajectory_cost_;
  }

  inline void set_global_index(const int &global_index) {
    global_index_ = global_index;
  }
  inline void set_lon_id(const int &lon_id) { lon_id_ = lon_id; }
  inline void set_lat_id(const int &lat_id) { lat_id_ = lat_id; }
  inline void set_trajectory_cost(const TrajectoryCost &trajectory_cost) {
    trajectory_cost_ = trajectory_cost;
  }
  inline void set_rightmost_cost(const double &rightmost_cost) {
    trajectory_cost_.rightmost_cost = rightmost_cost;
  }
  inline void set_priority_cost(const double &priority_cost) {
    trajectory_cost_.priority_cost = priority_cost;
  }
  inline void set_transition_cost(const double &transition_cost) {
    trajectory_cost_.transition_cost = transition_cost;
  }
  inline void set_closest_obj_cost(const double &closest_obj_cost) {
    trajectory_cost_.closest_obj_cost = closest_obj_cost;
  }
  inline void set_blocked_cost(const double &blocked_cost) {
    trajectory_cost_.blocked_cost = blocked_cost;
  }
  inline void set_cost(const double &cost) { trajectory_cost_.cost = cost; }
  inline void set_closest_obj_velocity(const double &closest_obj_velocity) {
    trajectory_cost_.closest_obj_velocity = closest_obj_velocity;
  }
    inline void set_closest_obj_acceleration(const double &closest_obj_acceleration) {
    trajectory_cost_.closest_obj_acceleration = closest_obj_acceleration;
  }
  inline void set_closest_obj_distance(const double &closest_obj_distance) {
    trajectory_cost_.closest_obj_distance = closest_obj_distance;
  }
  inline void set_closest_obj_lateral_distance(const double &closest_obj_lateral_distance) {
    trajectory_cost_.closest_obj_lateral_distance = closest_obj_lateral_distance;
  }
  inline void set_closest_obj_collision_dis(
      const double &closest_obj_collision_dis) {
    trajectory_cost_.closest_obj_collision_dis = closest_obj_collision_dis;
  }
  inline void set_closest_obj_collisiontime(
      const double &closest_obj_collisiontime) {
    trajectory_cost_.closest_obj_collisiontime = closest_obj_collisiontime;
  }
  inline void set_closest_obj_theta_abs(const double &closest_obj_theta_abs) {
    trajectory_cost_.closest_obj_theta_abs = closest_obj_theta_abs;
  }
  inline void set_closest_obj_theta_veh(const double &closest_obj_theta_veh) {
    trajectory_cost_.closest_obj_theta_veh = closest_obj_theta_veh;
  }
  inline void set_closest_obj_id(const int &closest_obj_id) {
    trajectory_cost_.closest_obj_id = closest_obj_id;
  }
  inline void set_curvature_cost(const double &curvature_cost) {
    trajectory_cost_.curvature_cost = curvature_cost;
  }
  inline void set_horizon_cost(const double &horizon_cost) {
    trajectory_cost_.horizon_cost = horizon_cost;
  }
  inline void set_junction_close_cost(const double &junction_close_cost) {
    trajectory_cost_.junction_close_cost = junction_close_cost;
  }
  inline void set_lane_change_cost(const double &lane_change_cost) {
    trajectory_cost_.lane_change_cost = lane_change_cost;
  }
  inline void set_potential_cost(const double &potential_cost) {
    trajectory_cost_.potential_cost = potential_cost;
  }
  inline void set_lateral_left_cost(const double &lateral_left_cost) {
    trajectory_cost_.lateral_left_cost = lateral_left_cost;
  }
  inline void set_lateral_right_cost(const double &lateral_right_cost) {
    trajectory_cost_.lateral_right_cost = lateral_right_cost;
  }
  inline void set_longitudinal_cost(const double &longitudinal_cost) {
    trajectory_cost_.longitudinal_cost = longitudinal_cost;
  }
  inline void set_is_fully_block(const bool &is_fully_block) {
    trajectory_cost_.is_fully_block = is_fully_block;
  }
  inline void set_is_passage_safe_f(const bool &is_passage_safe_f) {
    trajectory_cost_.is_passage_safe_f = is_passage_safe_f;
  }
  inline void set_is_passage_safe_b(const bool &is_passage_safe_b) {
    trajectory_cost_.is_passage_safe_b = is_passage_safe_b;
  }
  inline void set_is_passage_safe_dclc(const bool &is_passage_safe_dclc) {
    trajectory_cost_.is_passage_safe_dclc = is_passage_safe_dclc;
  }
  inline void set_dynamic_objects_pose(const legionclaw::planning::DynamicObjectsPose &dynamic_objects_pose_) {
    trajectory_cost_.dynamic_objects_pose = dynamic_objects_pose_;
  }

  inline void set_id(const std::string& id) { trajectory_cost_.id = id; }

 private:
  int global_index_ = 0;
  int lon_id_ = 0;
  int lat_id_ = 0;
  TrajectoryCost trajectory_cost_;
  
};
}  // namespace planning
}  // namespace legionclaw
