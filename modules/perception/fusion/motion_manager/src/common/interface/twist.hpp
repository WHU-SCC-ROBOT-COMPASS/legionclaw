/**
 * @file    twist.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "header.hpp"
#include "point_3d.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Twist
{
public:
    Twist() = default;
    ~Twist() = default;

    inline void set_header(const motion_manager::interface::Header& header)
    {
        header_ = header;
        header_ptr_ = &header_;
    }

    inline const motion_manager::interface::Header& header() const
    {
        return header_;
    }

    inline motion_manager::interface::Header* mutable_header()
    {
        return& header_;
    }

    inline bool has_header()
    {
        return (header_ptr_ != nullptr);
    }

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

void operator = (const Twist& twist){
    CopyFrom(twist);
  }

  void CopyFrom(const Twist& twist ){
    header_ = twist.header();
    linear_ = twist.linear();
    angular_ = twist.angular();
  }

protected:
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    //线速度 m/s
    motion_manager::interface::Point3D linear_;
    motion_manager::interface::Point3D* linear_ptr_ = nullptr;
    //角速度 deg/s
    motion_manager::interface::Point3D angular_;
    motion_manager::interface::Point3D* angular_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
