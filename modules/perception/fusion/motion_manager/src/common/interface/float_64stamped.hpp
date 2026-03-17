/**
 * @file    float_64stamped.hpp
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


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Float64Stamped
{
public:
    Float64Stamped() { 
    data_ = 0.0;
 }
    ~Float64Stamped() = default;

    inline void set_stamp(const motion_manager::interface::Time& stamp)
    {
        stamp_ = stamp;
        stamp_ptr_ = &stamp_;
    }

    inline const motion_manager::interface::Time& stamp() const
    {
        return stamp_;
    }

    inline motion_manager::interface::Time* mutable_stamp()
    {
        return& stamp_;
    }

    inline bool has_stamp()
    {
        return (stamp_ptr_ != nullptr);
    }

    inline void set_data(const double& data)
    {
        data_ = data;
        data_ptr_ = &data_;
    }

    inline const double& data() const
    {
        return data_;
    }

    inline double* mutable_data()
    {
        return& data_;
    }

    inline bool has_data()
    {
        return (data_ptr_ != nullptr);
    }

void operator = (const Float64Stamped& float_64stamped){
    CopyFrom(float_64stamped);
  }

  void CopyFrom(const Float64Stamped& float_64stamped ){
    stamp_ = float_64stamped.stamp();
    data_ = float_64stamped.data();
  }

protected:
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    double data_;
    double* data_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
