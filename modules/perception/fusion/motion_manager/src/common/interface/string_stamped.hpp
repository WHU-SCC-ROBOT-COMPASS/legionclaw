/**
 * @file    string_stamped.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <string>
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
class StringStamped
{
public:
    StringStamped() { 
    data_.clear();
 }
    ~StringStamped() = default;

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

    inline void set_data(const std::string& data)
    {
        data_ = data;
        data_ptr_ = &data_;
    }

    inline const std::string& data() const
    {
        return data_;
    }

    inline std::string* mutable_data()
    {
        return& data_;
    }

    inline bool has_data()
    {
        return (data_ptr_ != nullptr);
    }

void operator = (const StringStamped& string_stamped){
    CopyFrom(string_stamped);
  }

  void CopyFrom(const StringStamped& string_stamped ){
    stamp_ = string_stamped.stamp();
    data_ = string_stamped.data();
  }

protected:
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    std::string data_;
    std::string* data_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
