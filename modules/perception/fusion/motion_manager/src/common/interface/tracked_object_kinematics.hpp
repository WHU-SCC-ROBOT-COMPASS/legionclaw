/**
 * @file    tracked_object_kinematics.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "pose_with_covariance.hpp"
#include "accel_with_covariance.hpp"
#include "twist_with_covariance.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class TrackedObjectKinematics
{
public:
    TrackedObjectKinematics() { 
    orientation_availability_ = 0;
    is_stationary_ = false;
 }
    ~TrackedObjectKinematics() = default;

    inline void set_pose_with_covariance(const motion_manager::interface::PoseWithCovariance& pose_with_covariance)
    {
        pose_with_covariance_ = pose_with_covariance;
        pose_with_covariance_ptr_ = &pose_with_covariance_;
    }

    inline const motion_manager::interface::PoseWithCovariance& pose_with_covariance() const
    {
        return pose_with_covariance_;
    }

    inline motion_manager::interface::PoseWithCovariance* mutable_pose_with_covariance()
    {
        return& pose_with_covariance_;
    }

    inline bool has_pose_with_covariance()
    {
        return (pose_with_covariance_ptr_ != nullptr);
    }

    inline void set_pose_with_covariance_vehicle(const motion_manager::interface::PoseWithCovariance& pose_with_covariance_vehicle)
    {
        pose_with_covariance_vehicle_ = pose_with_covariance_vehicle;
        pose_with_covariance_vehicle_ptr_ = &pose_with_covariance_vehicle_;
    }

    inline const motion_manager::interface::PoseWithCovariance& pose_with_covariance_vehicle() const
    {
        return pose_with_covariance_vehicle_;
    }

    inline motion_manager::interface::PoseWithCovariance* mutable_pose_with_covariance_vehicle()
    {
        return& pose_with_covariance_vehicle_;
    }

    inline bool has_pose_with_covariance_vehicle()
    {
        return (pose_with_covariance_vehicle_ptr_ != nullptr);
    }

    inline void set_twist_with_covariance(const motion_manager::interface::TwistWithCovariance& twist_with_covariance)
    {
        twist_with_covariance_ = twist_with_covariance;
        twist_with_covariance_ptr_ = &twist_with_covariance_;
    }

    inline const motion_manager::interface::TwistWithCovariance& twist_with_covariance() const
    {
        return twist_with_covariance_;
    }

    inline motion_manager::interface::TwistWithCovariance* mutable_twist_with_covariance()
    {
        return& twist_with_covariance_;
    }

    inline bool has_twist_with_covariance()
    {
        return (twist_with_covariance_ptr_ != nullptr);
    }

    inline void set_acceleration_with_covariance(const motion_manager::interface::AccelWithCovariance& acceleration_with_covariance)
    {
        acceleration_with_covariance_ = acceleration_with_covariance;
        acceleration_with_covariance_ptr_ = &acceleration_with_covariance_;
    }

    inline const motion_manager::interface::AccelWithCovariance& acceleration_with_covariance() const
    {
        return acceleration_with_covariance_;
    }

    inline motion_manager::interface::AccelWithCovariance* mutable_acceleration_with_covariance()
    {
        return& acceleration_with_covariance_;
    }

    inline bool has_acceleration_with_covariance()
    {
        return (acceleration_with_covariance_ptr_ != nullptr);
    }

    inline void set_orientation_availability(const uint8_t& orientation_availability)
    {
        orientation_availability_ = orientation_availability;
        orientation_availability_ptr_ = &orientation_availability_;
    }

    inline const uint8_t& orientation_availability() const
    {
        return orientation_availability_;
    }

    inline uint8_t* mutable_orientation_availability()
    {
        return& orientation_availability_;
    }

    inline bool has_orientation_availability()
    {
        return (orientation_availability_ptr_ != nullptr);
    }

    inline void set_is_stationary(const bool& is_stationary)
    {
        is_stationary_ = is_stationary;
        is_stationary_ptr_ = &is_stationary_;
    }

    inline const bool& is_stationary() const
    {
        return is_stationary_;
    }

    inline bool* mutable_is_stationary()
    {
        return& is_stationary_;
    }

    inline bool has_is_stationary()
    {
        return (is_stationary_ptr_ != nullptr);
    }

void operator = (const TrackedObjectKinematics& tracked_object_kinematics){
    CopyFrom(tracked_object_kinematics);
  }

  void CopyFrom(const TrackedObjectKinematics& tracked_object_kinematics ){
    pose_with_covariance_ = tracked_object_kinematics.pose_with_covariance();
    pose_with_covariance_vehicle_ = tracked_object_kinematics.pose_with_covariance_vehicle();
    twist_with_covariance_ = tracked_object_kinematics.twist_with_covariance();
    acceleration_with_covariance_ = tracked_object_kinematics.acceleration_with_covariance();
    orientation_availability_ = tracked_object_kinematics.orientation_availability();
    is_stationary_ = tracked_object_kinematics.is_stationary();
  }

protected:
    motion_manager::interface::PoseWithCovariance pose_with_covariance_;
    motion_manager::interface::PoseWithCovariance* pose_with_covariance_ptr_ = nullptr;
    motion_manager::interface::PoseWithCovariance pose_with_covariance_vehicle_;
    motion_manager::interface::PoseWithCovariance* pose_with_covariance_vehicle_ptr_ = nullptr;
    motion_manager::interface::TwistWithCovariance twist_with_covariance_;
    motion_manager::interface::TwistWithCovariance* twist_with_covariance_ptr_ = nullptr;
    motion_manager::interface::AccelWithCovariance acceleration_with_covariance_;
    motion_manager::interface::AccelWithCovariance* acceleration_with_covariance_ptr_ = nullptr;
    uint8_t orientation_availability_;
    uint8_t* orientation_availability_ptr_ = nullptr;
    bool is_stationary_;
    bool* is_stationary_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
