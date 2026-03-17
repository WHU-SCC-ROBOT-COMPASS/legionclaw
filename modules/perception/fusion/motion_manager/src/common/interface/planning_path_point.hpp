/**
 * @file    planning_path_point.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "geometry_pose.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PlanningPathPoint
{
public:
    PlanningPathPoint() { 
    longitudinal_velocity_mps_ = 0.0;
    lateral_velocity_mps_ = 0.0;
    heading_rate_rps_ = 0.0;
    is_final_ = false;
 }
    ~PlanningPathPoint() = default;

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

    inline void set_is_final(const bool& is_final)
    {
        is_final_ = is_final;
        is_final_ptr_ = &is_final_;
    }

    inline const bool& is_final() const
    {
        return is_final_;
    }

    inline bool* mutable_is_final()
    {
        return& is_final_;
    }

    inline bool has_is_final()
    {
        return (is_final_ptr_ != nullptr);
    }

void operator = (const PlanningPathPoint& planning_path_point){
    CopyFrom(planning_path_point);
  }

  void CopyFrom(const PlanningPathPoint& planning_path_point ){
    pose_ = planning_path_point.pose();
    longitudinal_velocity_mps_ = planning_path_point.longitudinal_velocity_mps();
    lateral_velocity_mps_ = planning_path_point.lateral_velocity_mps();
    heading_rate_rps_ = planning_path_point.heading_rate_rps();
    is_final_ = planning_path_point.is_final();
  }

protected:
    motion_manager::interface::GeometryPose pose_;
    motion_manager::interface::GeometryPose* pose_ptr_ = nullptr;
    float longitudinal_velocity_mps_;
    float* longitudinal_velocity_mps_ptr_ = nullptr;
    float lateral_velocity_mps_;
    float* lateral_velocity_mps_ptr_ = nullptr;
    float heading_rate_rps_;
    float* heading_rate_rps_ptr_ = nullptr;
    bool is_final_;
    bool* is_final_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
