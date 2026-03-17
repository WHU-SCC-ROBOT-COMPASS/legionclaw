/**
 * @file    geometry_pose.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "point_3d.hpp"
#include "quaternion.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class GeometryPose
{
public:
    GeometryPose() = default;
    ~GeometryPose() = default;

    inline void set_position(const motion_manager::interface::Point3D& position)
    {
        position_ = position;
        position_ptr_ = &position_;
    }

    inline const motion_manager::interface::Point3D& position() const
    {
        return position_;
    }

    inline motion_manager::interface::Point3D* mutable_position()
    {
        return& position_;
    }

    inline bool has_position()
    {
        return (position_ptr_ != nullptr);
    }

    inline void set_orientation(const motion_manager::interface::Quaternion& orientation)
    {
        orientation_ = orientation;
        orientation_ptr_ = &orientation_;
    }

    inline const motion_manager::interface::Quaternion& orientation() const
    {
        return orientation_;
    }

    inline motion_manager::interface::Quaternion* mutable_orientation()
    {
        return& orientation_;
    }

    inline bool has_orientation()
    {
        return (orientation_ptr_ != nullptr);
    }

void operator = (const GeometryPose& geometry_pose){
    CopyFrom(geometry_pose);
  }

  void CopyFrom(const GeometryPose& geometry_pose ){
    position_ = geometry_pose.position();
    orientation_ = geometry_pose.orientation();
  }

protected:
    motion_manager::interface::Point3D position_;
    motion_manager::interface::Point3D* position_ptr_ = nullptr;
    motion_manager::interface::Quaternion orientation_;
    motion_manager::interface::Quaternion* orientation_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
