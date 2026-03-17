/**
 * @file    lanelet_primitive.hpp
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



/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class LaneletPrimitive
{
public:
    LaneletPrimitive() { 
    id_ = 0;
    primitive_type_.clear();
 }
    ~LaneletPrimitive() = default;

    inline void set_id(const int64_t& id)
    {
        id_ = id;
        id_ptr_ = &id_;
    }

    inline const int64_t& id() const
    {
        return id_;
    }

    inline int64_t* mutable_id()
    {
        return& id_;
    }

    inline bool has_id()
    {
        return (id_ptr_ != nullptr);
    }

    inline void set_primitive_type(const std::string& primitive_type)
    {
        primitive_type_ = primitive_type;
        primitive_type_ptr_ = &primitive_type_;
    }

    inline const std::string& primitive_type() const
    {
        return primitive_type_;
    }

    inline std::string* mutable_primitive_type()
    {
        return& primitive_type_;
    }

    inline bool has_primitive_type()
    {
        return (primitive_type_ptr_ != nullptr);
    }

void operator = (const LaneletPrimitive& lanelet_primitive){
    CopyFrom(lanelet_primitive);
  }

  void CopyFrom(const LaneletPrimitive& lanelet_primitive ){
    id_ = lanelet_primitive.id();
    primitive_type_ = lanelet_primitive.primitive_type();
  }

protected:
    int64_t id_;
    int64_t* id_ptr_ = nullptr;
    std::string primitive_type_;
    std::string* primitive_type_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
