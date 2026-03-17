/**
 * @file    int_32stamped.hpp
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
class Int32Stamped
{
public:
    Int32Stamped() { 
    data_ = 0;
 }
    ~Int32Stamped() = default;

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

    inline void set_data(const int32_t& data)
    {
        data_ = data;
        data_ptr_ = &data_;
    }

    inline const int32_t& data() const
    {
        return data_;
    }

    inline int32_t* mutable_data()
    {
        return& data_;
    }

    inline bool has_data()
    {
        return (data_ptr_ != nullptr);
    }

void operator = (const Int32Stamped& int_32stamped){
    CopyFrom(int_32stamped);
  }

  void CopyFrom(const Int32Stamped& int_32stamped ){
    stamp_ = int_32stamped.stamp();
    data_ = int_32stamped.data();
  }

protected:
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    int32_t data_;
    int32_t* data_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
