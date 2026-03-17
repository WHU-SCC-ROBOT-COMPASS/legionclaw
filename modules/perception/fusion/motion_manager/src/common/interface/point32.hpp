/**
 * @file    point_32.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>



/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Point32
{
public:
    Point32() { 
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
 }
    ~Point32() = default;

    inline void set_x(const float& x)
    {
        x_ = x;
        x_ptr_ = &x_;
    }

    inline const float& x() const
    {
        return x_;
    }

    inline float* mutable_x()
    {
        return& x_;
    }

    inline bool has_x()
    {
        return (x_ptr_ != nullptr);
    }

    inline void set_y(const float& y)
    {
        y_ = y;
        y_ptr_ = &y_;
    }

    inline const float& y() const
    {
        return y_;
    }

    inline float* mutable_y()
    {
        return& y_;
    }

    inline bool has_y()
    {
        return (y_ptr_ != nullptr);
    }

    inline void set_z(const float& z)
    {
        z_ = z;
        z_ptr_ = &z_;
    }

    inline const float& z() const
    {
        return z_;
    }

    inline float* mutable_z()
    {
        return& z_;
    }

    inline bool has_z()
    {
        return (z_ptr_ != nullptr);
    }

void operator = (const Point32& point32){
    CopyFrom(point32);
  }

  void CopyFrom(const Point32& point32 ){
    x_ = point32.x();
    y_ = point32.y();
    z_ = point32.z();
  }

protected:
    float x_;
    float* x_ptr_ = nullptr;
    float y_;
    float* y_ptr_ = nullptr;
    float z_;
    float* z_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
