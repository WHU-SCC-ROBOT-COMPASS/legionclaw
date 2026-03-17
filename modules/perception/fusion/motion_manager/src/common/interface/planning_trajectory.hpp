/**
 * @file    planning_trajectory.hpp
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
#include "planning_trajectory_point.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PlanningTrajectory
{
public:
    PlanningTrajectory() { 
        points_mutex_ = std::make_shared<std::mutex>();

    clear_points();
 }
    ~PlanningTrajectory() = default;

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

    inline void set_points(std::vector<motion_manager::interface::PlanningTrajectoryPoint>* points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.assign(points->begin(), points->end());
    }

    inline void set_points(const std::vector<motion_manager::interface::PlanningTrajectoryPoint>& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_=points;
    }

    inline void set_points(const uint32_t index,motion_manager::interface::PlanningTrajectoryPoint& points)
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_[index] = points;
    }

    inline void add_points(const 
    motion_manager::interface::PlanningTrajectoryPoint& points) {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points_.emplace_back(points);
    }

    inline const motion_manager::interface::PlanningTrajectoryPoint& points(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return points_[index];
    }

    inline std::vector<motion_manager::interface::PlanningTrajectoryPoint>* mutable_points()
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        return& points_;
    }

    inline void points(std::vector<motion_manager::interface::PlanningTrajectoryPoint>& points) const
    {
        std::lock_guard<std::mutex> lock(*points_mutex_);
        points.assign(points_.begin(),points_.end());
    }

    inline const std::vector<motion_manager::interface::PlanningTrajectoryPoint>& points() const
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

void operator = (const PlanningTrajectory& planning_trajectory){
    CopyFrom(planning_trajectory);
  }

  void CopyFrom(const PlanningTrajectory& planning_trajectory ){
    header_ = planning_trajectory.header();
    points_ = planning_trajectory.points();
  }

protected:
    std::shared_ptr<std::mutex> points_mutex_;
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    std::vector<motion_manager::interface::PlanningTrajectoryPoint> points_;
};
} // namespace interface
} // namespace motion_manager
