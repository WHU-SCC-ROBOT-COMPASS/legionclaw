/**
 * @file    shape.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "polygon.hpp"
#include "point_3d.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Shape
{
public:
    Shape() { 
    type_ = 0;
 }
    ~Shape() = default;

    inline void set_type(const uint8_t& type)
    {
        type_ = type;
        type_ptr_ = &type_;
    }

    inline const uint8_t& type() const
    {
        return type_;
    }

    inline uint8_t* mutable_type()
    {
        return& type_;
    }

    inline bool has_type()
    {
        return (type_ptr_ != nullptr);
    }

    inline void set_footprint(const motion_manager::interface::Polygon& footprint)
    {
        footprint_ = footprint;
        footprint_ptr_ = &footprint_;
    }

    inline const motion_manager::interface::Polygon& footprint() const
    {
        return footprint_;
    }

    inline motion_manager::interface::Polygon* mutable_footprint()
    {
        return& footprint_;
    }

    inline bool has_footprint()
    {
        return (footprint_ptr_ != nullptr);
    }

    inline void set_dimensions(const motion_manager::interface::Point3D& dimensions)
    {
        dimensions_ = dimensions;
        dimensions_ptr_ = &dimensions_;
    }

    inline const motion_manager::interface::Point3D& dimensions() const
    {
        return dimensions_;
    }

    inline motion_manager::interface::Point3D* mutable_dimensions()
    {
        return& dimensions_;
    }

    inline bool has_dimensions()
    {
        return (dimensions_ptr_ != nullptr);
    }

void operator = (const Shape& shape){
    CopyFrom(shape);
  }

  void CopyFrom(const Shape& shape ){
    type_ = shape.type();
    footprint_ = shape.footprint();
    dimensions_ = shape.dimensions();
  }

protected:
    uint8_t type_;
    uint8_t* type_ptr_ = nullptr;
    motion_manager::interface::Polygon footprint_;
    motion_manager::interface::Polygon* footprint_ptr_ = nullptr;
    motion_manager::interface::Point3D dimensions_;
    motion_manager::interface::Point3D* dimensions_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
