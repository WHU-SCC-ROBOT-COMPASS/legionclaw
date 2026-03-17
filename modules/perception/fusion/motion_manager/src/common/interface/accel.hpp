/**
 * @file    accel.hpp
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


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Accel
{
public:
    Accel() = default;
    ~Accel() = default;

    inline void set_linear(const motion_manager::interface::Point3D& linear)
    {
        linear_ = linear;
        linear_ptr_ = &linear_;
    }

    inline const motion_manager::interface::Point3D& linear() const
    {
        return linear_;
    }

    inline motion_manager::interface::Point3D* mutable_linear()
    {
        return& linear_;
    }

    inline bool has_linear()
    {
        return (linear_ptr_ != nullptr);
    }

    inline void set_angular(const motion_manager::interface::Point3D& angular)
    {
        angular_ = angular;
        angular_ptr_ = &angular_;
    }

    inline const motion_manager::interface::Point3D& angular() const
    {
        return angular_;
    }

    inline motion_manager::interface::Point3D* mutable_angular()
    {
        return& angular_;
    }

    inline bool has_angular()
    {
        return (angular_ptr_ != nullptr);
    }

void operator = (const Accel& accel){
    CopyFrom(accel);
  }

  void CopyFrom(const Accel& accel ){
    linear_ = accel.linear();
    angular_ = accel.angular();
  }

protected:
    motion_manager::interface::Point3D linear_;
    motion_manager::interface::Point3D* linear_ptr_ = nullptr;
    motion_manager::interface::Point3D angular_;
    motion_manager::interface::Point3D* angular_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
