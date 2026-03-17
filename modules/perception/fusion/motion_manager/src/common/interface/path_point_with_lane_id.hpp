/**
 * @file    path_point_with_lane_id.hpp
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

#include "planning_path_point.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PathPointWithLaneId
{
public:
    PathPointWithLaneId() { 
        lane_ids_mutex_ = std::make_shared<std::mutex>();

    clear_lane_ids();
 }
    ~PathPointWithLaneId() = default;

    inline void set_point(const motion_manager::interface::PlanningPathPoint& point)
    {
        point_ = point;
        point_ptr_ = &point_;
    }

    inline const motion_manager::interface::PlanningPathPoint& point() const
    {
        return point_;
    }

    inline motion_manager::interface::PlanningPathPoint* mutable_point()
    {
        return& point_;
    }

    inline bool has_point()
    {
        return (point_ptr_ != nullptr);
    }

    inline void set_lane_ids(std::vector<int64_t>* lane_ids)
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids_.assign(lane_ids->begin(), lane_ids->end());
    }

    inline void set_lane_ids(const std::vector<int64_t>& lane_ids)
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids_=lane_ids;
    }

    inline void set_lane_ids(const uint32_t index,int64_t& lane_ids)
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids_[index] = lane_ids;
    }

    inline void add_lane_ids(const 
    int64_t& lane_ids) {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids_.emplace_back(lane_ids);
    }

    inline const int64_t& lane_ids(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        return lane_ids_[index];
    }

    inline std::vector<int64_t>* mutable_lane_ids()
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        return& lane_ids_;
    }

    inline void lane_ids(std::vector<int64_t>& lane_ids) const
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids.assign(lane_ids_.begin(),lane_ids_.end());
    }

    inline const std::vector<int64_t>& lane_ids() const
    {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        return lane_ids_;
    }

    inline uint32_t lane_ids_size() const {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        return lane_ids_.size();
    }

    inline void clear_lane_ids() {
        std::lock_guard<std::mutex> lock(*lane_ids_mutex_);
        lane_ids_.clear();
        lane_ids_.shrink_to_fit();
    }

    inline bool has_lane_ids() {
        return (lane_ids_size() != 0);
    }

void operator = (const PathPointWithLaneId& path_point_with_lane_id){
    CopyFrom(path_point_with_lane_id);
  }

  void CopyFrom(const PathPointWithLaneId& path_point_with_lane_id ){
    point_ = path_point_with_lane_id.point();
    lane_ids_ = path_point_with_lane_id.lane_ids();
  }

protected:
    std::shared_ptr<std::mutex> lane_ids_mutex_;
    motion_manager::interface::PlanningPathPoint point_;
    motion_manager::interface::PlanningPathPoint* point_ptr_ = nullptr;
    std::vector<int64_t> lane_ids_;
};
} // namespace interface
} // namespace motion_manager
