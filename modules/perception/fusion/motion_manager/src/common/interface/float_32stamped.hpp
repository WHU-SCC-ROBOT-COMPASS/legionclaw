/**
 * @file    float_32stamped.hpp
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
class Float32Stamped
{
public:
    Float32Stamped() { 
    data_ = 0.0;
 }
    ~Float32Stamped() = default;

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

    inline void set_data(const float& data)
    {
        data_ = data;
        data_ptr_ = &data_;
    }

    inline const float& data() const
    {
        return data_;
    }

    inline float* mutable_data()
    {
        return& data_;
    }

    inline bool has_data()
    {
        return (data_ptr_ != nullptr);
    }

void operator = (const Float32Stamped& float_32stamped){
    CopyFrom(float_32stamped);
  }

  void CopyFrom(const Float32Stamped& float_32stamped ){
    stamp_ = float_32stamped.stamp();
    data_ = float_32stamped.data();
  }

protected:
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    float data_;
    float* data_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
