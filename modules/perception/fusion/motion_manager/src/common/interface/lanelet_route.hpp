/**
 * @file    lanelet_route.hpp
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

#include "uuid.hpp"
#include "header.hpp"
#include "geometry_pose.hpp"
#include "lanelet_segment.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class LaneletRoute
{
public:
    LaneletRoute() { 
        segments_mutex_ = std::make_shared<std::mutex>();

    clear_segments();
    allow_modification_ = false;
 }
    ~LaneletRoute() = default;

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

    inline void set_start_pose(const motion_manager::interface::GeometryPose& start_pose)
    {
        start_pose_ = start_pose;
        start_pose_ptr_ = &start_pose_;
    }

    inline const motion_manager::interface::GeometryPose& start_pose() const
    {
        return start_pose_;
    }

    inline motion_manager::interface::GeometryPose* mutable_start_pose()
    {
        return& start_pose_;
    }

    inline bool has_start_pose()
    {
        return (start_pose_ptr_ != nullptr);
    }

    inline void set_goal_pose(const motion_manager::interface::GeometryPose& goal_pose)
    {
        goal_pose_ = goal_pose;
        goal_pose_ptr_ = &goal_pose_;
    }

    inline const motion_manager::interface::GeometryPose& goal_pose() const
    {
        return goal_pose_;
    }

    inline motion_manager::interface::GeometryPose* mutable_goal_pose()
    {
        return& goal_pose_;
    }

    inline bool has_goal_pose()
    {
        return (goal_pose_ptr_ != nullptr);
    }

    inline void set_segments(std::vector<motion_manager::interface::LaneletSegment>* segments)
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments_.assign(segments->begin(), segments->end());
    }

    inline void set_segments(const std::vector<motion_manager::interface::LaneletSegment>& segments)
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments_=segments;
    }

    inline void set_segments(const uint32_t index,motion_manager::interface::LaneletSegment& segments)
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments_[index] = segments;
    }

    inline void add_segments(const 
    motion_manager::interface::LaneletSegment& segments) {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments_.emplace_back(segments);
    }

    inline const motion_manager::interface::LaneletSegment& segments(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        return segments_[index];
    }

    inline std::vector<motion_manager::interface::LaneletSegment>* mutable_segments()
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        return& segments_;
    }

    inline void segments(std::vector<motion_manager::interface::LaneletSegment>& segments) const
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments.assign(segments_.begin(),segments_.end());
    }

    inline const std::vector<motion_manager::interface::LaneletSegment>& segments() const
    {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        return segments_;
    }

    inline uint32_t segments_size() const {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        return segments_.size();
    }

    inline void clear_segments() {
        std::lock_guard<std::mutex> lock(*segments_mutex_);
        segments_.clear();
        segments_.shrink_to_fit();
    }

    inline bool has_segments() {
        return (segments_size() != 0);
    }

    inline void set_uuid(const motion_manager::interface::UUID& uuid)
    {
        uuid_ = uuid;
        uuid_ptr_ = &uuid_;
    }

    inline const motion_manager::interface::UUID& uuid() const
    {
        return uuid_;
    }

    inline motion_manager::interface::UUID* mutable_uuid()
    {
        return& uuid_;
    }

    inline bool has_uuid()
    {
        return (uuid_ptr_ != nullptr);
    }

    inline void set_allow_modification(const bool& allow_modification)
    {
        allow_modification_ = allow_modification;
        allow_modification_ptr_ = &allow_modification_;
    }

    inline const bool& allow_modification() const
    {
        return allow_modification_;
    }

    inline bool* mutable_allow_modification()
    {
        return& allow_modification_;
    }

    inline bool has_allow_modification()
    {
        return (allow_modification_ptr_ != nullptr);
    }

void operator = (const LaneletRoute& lanelet_route){
    CopyFrom(lanelet_route);
  }

  void CopyFrom(const LaneletRoute& lanelet_route ){
    header_ = lanelet_route.header();
    start_pose_ = lanelet_route.start_pose();
    goal_pose_ = lanelet_route.goal_pose();
    segments_ = lanelet_route.segments();
    uuid_ = lanelet_route.uuid();
    allow_modification_ = lanelet_route.allow_modification();
  }

protected:
    std::shared_ptr<std::mutex> segments_mutex_;
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    motion_manager::interface::GeometryPose start_pose_;
    motion_manager::interface::GeometryPose* start_pose_ptr_ = nullptr;
    motion_manager::interface::GeometryPose goal_pose_;
    motion_manager::interface::GeometryPose* goal_pose_ptr_ = nullptr;
    std::vector<motion_manager::interface::LaneletSegment> segments_;
    motion_manager::interface::UUID uuid_;
    motion_manager::interface::UUID* uuid_ptr_ = nullptr;
    bool allow_modification_;
    bool* allow_modification_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
