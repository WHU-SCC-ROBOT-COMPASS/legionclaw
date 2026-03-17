/**
 * @file    detected_object_kinematics.hpp
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
#include "twist_with_covariance.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class DetectedObjectKinematics
{
public:
    DetectedObjectKinematics() { 
    has_position_covariance_ = false;
    orientation_availability_ = 0;
    has_twist_ = false;
    has_twist_covariance_ = false;
 }
    ~DetectedObjectKinematics() = default;

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

    inline void set_has_position_covariance(const bool& has_position_covariance)
    {
        has_position_covariance_ = has_position_covariance;
        has_position_covariance_ptr_ = &has_position_covariance_;
    }

    inline const bool& has_position_covariance() const
    {
        return has_position_covariance_;
    }

    inline bool* mutable_has_position_covariance()
    {
        return& has_position_covariance_;
    }

    inline bool has_has_position_covariance()
    {
        return (has_position_covariance_ptr_ != nullptr);
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

    inline void set_has_twist(const bool& has_twist)
    {
        has_twist_ = has_twist;
        has_twist_ptr_ = &has_twist_;
    }

    inline const bool& has_twist() const
    {
        return has_twist_;
    }

    inline bool* mutable_has_twist()
    {
        return& has_twist_;
    }

    inline bool has_has_twist()
    {
        return (has_twist_ptr_ != nullptr);
    }

    inline void set_has_twist_covariance(const bool& has_twist_covariance)
    {
        has_twist_covariance_ = has_twist_covariance;
        has_twist_covariance_ptr_ = &has_twist_covariance_;
    }

    inline const bool& has_twist_covariance() const
    {
        return has_twist_covariance_;
    }

    inline bool* mutable_has_twist_covariance()
    {
        return& has_twist_covariance_;
    }

    inline bool has_has_twist_covariance()
    {
        return (has_twist_covariance_ptr_ != nullptr);
    }

void operator = (const DetectedObjectKinematics& detected_object_kinematics){
    CopyFrom(detected_object_kinematics);
  }

  void CopyFrom(const DetectedObjectKinematics& detected_object_kinematics ){
    pose_with_covariance_ = detected_object_kinematics.pose_with_covariance();
    pose_with_covariance_vehicle_ = detected_object_kinematics.pose_with_covariance_vehicle();
    has_position_covariance_ = detected_object_kinematics.has_position_covariance();
    orientation_availability_ = detected_object_kinematics.orientation_availability();
    twist_with_covariance_ = detected_object_kinematics.twist_with_covariance();
    has_twist_ = detected_object_kinematics.has_twist();
    has_twist_covariance_ = detected_object_kinematics.has_twist_covariance();
  }

protected:
    motion_manager::interface::PoseWithCovariance pose_with_covariance_;
    motion_manager::interface::PoseWithCovariance* pose_with_covariance_ptr_ = nullptr;
    motion_manager::interface::PoseWithCovariance pose_with_covariance_vehicle_;
    motion_manager::interface::PoseWithCovariance* pose_with_covariance_vehicle_ptr_ = nullptr;
    bool has_position_covariance_;
    bool* has_position_covariance_ptr_ = nullptr;
    uint8_t orientation_availability_;
    uint8_t* orientation_availability_ptr_ = nullptr;
    motion_manager::interface::TwistWithCovariance twist_with_covariance_;
    motion_manager::interface::TwistWithCovariance* twist_with_covariance_ptr_ = nullptr;
    bool has_twist_;
    bool* has_twist_ptr_ = nullptr;
    bool has_twist_covariance_;
    bool* has_twist_covariance_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
