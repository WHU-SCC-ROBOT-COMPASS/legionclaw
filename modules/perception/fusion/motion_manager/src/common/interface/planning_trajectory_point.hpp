/**
 * @file    planning_trajectory_point.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "time.hpp"
#include "geometry_pose.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PlanningTrajectoryPoint
{
public:
    PlanningTrajectoryPoint() { 
    longitudinal_velocity_mps_ = 0.0;
    lateral_velocity_mps_ = 0.0;
    acceleration_mps2_ = 0.0;
    heading_rate_rps_ = 0.0;
    front_wheel_angle_rad_ = 0.0;
    rear_wheel_angle_rad_ = 0.0;
 }
    ~PlanningTrajectoryPoint() = default;

    inline void set_time_from_start(const motion_manager::interface::Time& time_from_start)
    {
        time_from_start_ = time_from_start;
        time_from_start_ptr_ = &time_from_start_;
    }

    inline const motion_manager::interface::Time& time_from_start() const
    {
        return time_from_start_;
    }

    inline motion_manager::interface::Time* mutable_time_from_start()
    {
        return& time_from_start_;
    }

    inline bool has_time_from_start()
    {
        return (time_from_start_ptr_ != nullptr);
    }

    inline void set_pose(const motion_manager::interface::GeometryPose& pose)
    {
        pose_ = pose;
        pose_ptr_ = &pose_;
    }

    inline const motion_manager::interface::GeometryPose& pose() const
    {
        return pose_;
    }

    inline motion_manager::interface::GeometryPose* mutable_pose()
    {
        return& pose_;
    }

    inline bool has_pose()
    {
        return (pose_ptr_ != nullptr);
    }

    inline void set_longitudinal_velocity_mps(const float& longitudinal_velocity_mps)
    {
        longitudinal_velocity_mps_ = longitudinal_velocity_mps;
        longitudinal_velocity_mps_ptr_ = &longitudinal_velocity_mps_;
    }

    inline const float& longitudinal_velocity_mps() const
    {
        return longitudinal_velocity_mps_;
    }

    inline float* mutable_longitudinal_velocity_mps()
    {
        return& longitudinal_velocity_mps_;
    }

    inline bool has_longitudinal_velocity_mps()
    {
        return (longitudinal_velocity_mps_ptr_ != nullptr);
    }

    inline void set_lateral_velocity_mps(const float& lateral_velocity_mps)
    {
        lateral_velocity_mps_ = lateral_velocity_mps;
        lateral_velocity_mps_ptr_ = &lateral_velocity_mps_;
    }

    inline const float& lateral_velocity_mps() const
    {
        return lateral_velocity_mps_;
    }

    inline float* mutable_lateral_velocity_mps()
    {
        return& lateral_velocity_mps_;
    }

    inline bool has_lateral_velocity_mps()
    {
        return (lateral_velocity_mps_ptr_ != nullptr);
    }

    inline void set_acceleration_mps2(const float& acceleration_mps2)
    {
        acceleration_mps2_ = acceleration_mps2;
        acceleration_mps2_ptr_ = &acceleration_mps2_;
    }

    inline const float& acceleration_mps2() const
    {
        return acceleration_mps2_;
    }

    inline float* mutable_acceleration_mps2()
    {
        return& acceleration_mps2_;
    }

    inline bool has_acceleration_mps2()
    {
        return (acceleration_mps2_ptr_ != nullptr);
    }

    inline void set_heading_rate_rps(const float& heading_rate_rps)
    {
        heading_rate_rps_ = heading_rate_rps;
        heading_rate_rps_ptr_ = &heading_rate_rps_;
    }

    inline const float& heading_rate_rps() const
    {
        return heading_rate_rps_;
    }

    inline float* mutable_heading_rate_rps()
    {
        return& heading_rate_rps_;
    }

    inline bool has_heading_rate_rps()
    {
        return (heading_rate_rps_ptr_ != nullptr);
    }

    inline void set_front_wheel_angle_rad(const float& front_wheel_angle_rad)
    {
        front_wheel_angle_rad_ = front_wheel_angle_rad;
        front_wheel_angle_rad_ptr_ = &front_wheel_angle_rad_;
    }

    inline const float& front_wheel_angle_rad() const
    {
        return front_wheel_angle_rad_;
    }

    inline float* mutable_front_wheel_angle_rad()
    {
        return& front_wheel_angle_rad_;
    }

    inline bool has_front_wheel_angle_rad()
    {
        return (front_wheel_angle_rad_ptr_ != nullptr);
    }

    inline void set_rear_wheel_angle_rad(const float& rear_wheel_angle_rad)
    {
        rear_wheel_angle_rad_ = rear_wheel_angle_rad;
        rear_wheel_angle_rad_ptr_ = &rear_wheel_angle_rad_;
    }

    inline const float& rear_wheel_angle_rad() const
    {
        return rear_wheel_angle_rad_;
    }

    inline float* mutable_rear_wheel_angle_rad()
    {
        return& rear_wheel_angle_rad_;
    }

    inline bool has_rear_wheel_angle_rad()
    {
        return (rear_wheel_angle_rad_ptr_ != nullptr);
    }

void operator = (const PlanningTrajectoryPoint& planning_trajectory_point){
    CopyFrom(planning_trajectory_point);
  }

  void CopyFrom(const PlanningTrajectoryPoint& planning_trajectory_point ){
    time_from_start_ = planning_trajectory_point.time_from_start();
    pose_ = planning_trajectory_point.pose();
    longitudinal_velocity_mps_ = planning_trajectory_point.longitudinal_velocity_mps();
    lateral_velocity_mps_ = planning_trajectory_point.lateral_velocity_mps();
    acceleration_mps2_ = planning_trajectory_point.acceleration_mps2();
    heading_rate_rps_ = planning_trajectory_point.heading_rate_rps();
    front_wheel_angle_rad_ = planning_trajectory_point.front_wheel_angle_rad();
    rear_wheel_angle_rad_ = planning_trajectory_point.rear_wheel_angle_rad();
  }

protected:
    motion_manager::interface::Time time_from_start_;
    motion_manager::interface::Time* time_from_start_ptr_ = nullptr;
    motion_manager::interface::GeometryPose pose_;
    motion_manager::interface::GeometryPose* pose_ptr_ = nullptr;
    float longitudinal_velocity_mps_;
    float* longitudinal_velocity_mps_ptr_ = nullptr;
    float lateral_velocity_mps_;
    float* lateral_velocity_mps_ptr_ = nullptr;
    float acceleration_mps2_;
    float* acceleration_mps2_ptr_ = nullptr;
    float heading_rate_rps_;
    float* heading_rate_rps_ptr_ = nullptr;
    float front_wheel_angle_rad_;
    float* front_wheel_angle_rad_ptr_ = nullptr;
    float rear_wheel_angle_rad_;
    float* rear_wheel_angle_rad_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
