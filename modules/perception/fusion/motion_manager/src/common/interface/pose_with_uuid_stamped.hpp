/**
 * @file    pose_with_uuid_stamped.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "uuid.hpp"
#include "header.hpp"
#include "geometry_pose.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PoseWithUuidStamped
{
public:
    PoseWithUuidStamped() = default;
    ~PoseWithUuidStamped() = default;

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

    inline void set_pose(const motion_manager::interface::GeometryPose& pose)
    {
        pose_ = pose;
        pose_ptr_ = &pose_;
    }

    inline const motion_manager::interface::GeometryPose& pose() const
    {
        return pose_;
    }

    inline motion_manager::interface::GeometryPose* mutable_pose()
    {
        return& pose_;
    }

    inline bool has_pose()
    {
        return (pose_ptr_ != nullptr);
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

void operator = (const PoseWithUuidStamped& pose_with_uuid_stamped){
    CopyFrom(pose_with_uuid_stamped);
  }

  void CopyFrom(const PoseWithUuidStamped& pose_with_uuid_stamped ){
    header_ = pose_with_uuid_stamped.header();
    pose_ = pose_with_uuid_stamped.pose();
    uuid_ = pose_with_uuid_stamped.uuid();
  }

protected:
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    motion_manager::interface::GeometryPose pose_;
    motion_manager::interface::GeometryPose* pose_ptr_ = nullptr;
    motion_manager::interface::UUID uuid_;
    motion_manager::interface::UUID* uuid_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
