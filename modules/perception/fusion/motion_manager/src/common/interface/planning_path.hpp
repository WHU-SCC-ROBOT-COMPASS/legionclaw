/**
 * @file    planning_path.hpp
 * @author  dabai-motion_manager
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

#include "header.hpp"
#include "point_3d.hpp"
#include "planning_path_point.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PlanningPath
{
public:
    PlanningPath() { 
        points_mutex_ = std::make_shared<std::mutex>();

    clear_points();
 }
    ~PlanningPath() = default;

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

    inline void set_points(std::vector<motion_manager::interface::PlanningPathPoint>* points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.assign(points->begin(), points->end());
    }

    inline void set_points(const std::vector<motion_manager::interface::PlanningPathPoint>& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_=points;
    }

    inline void set_points(const uint32_t index,motion_manager::interface::PlanningPathPoint& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_[index] = points;
    }

    inline void add_points(const 
    motion_manager::interface::PlanningPathPoint& points) {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.emplace_back(points);
    }

    inline const motion_manager::interface::PlanningPathPoint& points(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return points_[index];
    }

    inline std::vector<motion_manager::interface::PlanningPathPoint>* mutable_points()
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return& points_;
    }

    inline void points(std::vector<motion_manager::interface::PlanningPathPoint>& points) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points.assign(points_.begin(),points_.end());
    }

    inline const std::vector<motion_manager::interface::PlanningPathPoint>& points() const
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

    inline void set_left_bound(const motion_manager::interface::Point3D& left_bound)
    {
        left_bound_ = left_bound;
        left_bound_ptr_ = &left_bound_;
    }

    inline const motion_manager::interface::Point3D& left_bound() const
    {
        return left_bound_;
    }

    inline motion_manager::interface::Point3D* mutable_left_bound()
    {
        return& left_bound_;
    }

    inline bool has_left_bound()
    {
        return (left_bound_ptr_ != nullptr);
    }

    inline void set_right_bound(const motion_manager::interface::Point3D& right_bound)
    {
        right_bound_ = right_bound;
        right_bound_ptr_ = &right_bound_;
    }

    inline const motion_manager::interface::Point3D& right_bound() const
    {
        return right_bound_;
    }

    inline motion_manager::interface::Point3D* mutable_right_bound()
    {
        return& right_bound_;
    }

    inline bool has_right_bound()
    {
        return (right_bound_ptr_ != nullptr);
    }

void operator = (const PlanningPath& planning_path){
    CopyFrom(planning_path);
  }

  void CopyFrom(const PlanningPath& planning_path ){
    header_ = planning_path.header();
    points_ = planning_path.points();
    left_bound_ = planning_path.left_bound();
    right_bound_ = planning_path.right_bound();
  }

protected:
    std::shared_ptr<std::mutex> points_mutex_;
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    std::vector<motion_manager::interface::PlanningPathPoint> points_;
    motion_manager::interface::Point3D left_bound_;
    motion_manager::interface::Point3D* left_bound_ptr_ = nullptr;
    motion_manager::interface::Point3D right_bound_;
    motion_manager::interface::Point3D* right_bound_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
