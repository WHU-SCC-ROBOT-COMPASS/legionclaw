/**
 * @file    polygon.hpp
 * @author  dabai-legion
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <mutex>
#include <vector>
#include <memory>
#include <iostream>
#include <stdint.h>

#include "point32.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Polygon
{
public:
    Polygon() { 
        points_mutex_ = std::make_shared<std::mutex>();

    clear_points();
 }
    ~Polygon() = default;

    inline void set_points(std::vector<motion_manager::interface::Point32>* points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.assign(points->begin(), points->end());
    }

    inline void set_points(const std::vector<motion_manager::interface::Point32>& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_=points;
    }

    inline void set_points(const uint32_t index,motion_manager::interface::Point32& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_[index] = points;
    }

    inline void add_points(const 
    motion_manager::interface::Point32& points) {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.emplace_back(points);
    }

    inline const motion_manager::interface::Point32& points(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return points_[index];
    }

    inline std::vector<motion_manager::interface::Point32>* mutable_points()
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return& points_;
    }

    inline void points(std::vector<motion_manager::interface::Point32>& points) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points.assign(points_.begin(),points_.end());
    }

    inline const std::vector<motion_manager::interface::Point32>& points() const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return points_;
    }

    inline uint32_t points_size() const {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return points_.size();
    }

    inline void clear_points() {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.clear();
        points_.shrink_to_fit();
    }

    inline bool has_points() {
        return (points_size() != 0);
    }

void operator = (const Polygon& polygon){
    CopyFrom(polygon);
  }

  void CopyFrom(const Polygon& polygon ){
    points_ = polygon.points();
  }

protected:
    std::shared_ptr<std::mutex> points_mutex_;
    std::vector<motion_manager::interface::Point32> points_;
};
} // namespace interface
} // namespace legion
