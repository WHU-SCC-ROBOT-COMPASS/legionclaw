/**
 * @file    tracked_objects.hpp
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
#include "tracked_object.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class TrackedObjects
{
public:
    TrackedObjects() { 
        objects_mutex_ = std::make_shared<std::mutex>();

    clear_objects();
 }
    ~TrackedObjects() = default;

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

    inline void set_objects(std::vector<motion_manager::interface::TrackedObject>* objects)
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects_.assign(objects->begin(), objects->end());
    }

    inline void set_objects(const std::vector<motion_manager::interface::TrackedObject>& objects)
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects_=objects;
    }

    inline void set_objects(const uint32_t index,motion_manager::interface::TrackedObject& objects)
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects_[index] = objects;
    }

    inline void add_objects(const 
    motion_manager::interface::TrackedObject& objects) {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects_.emplace_back(objects);
    }

    inline const motion_manager::interface::TrackedObject& objects(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        return objects_[index];
    }

    inline std::vector<motion_manager::interface::TrackedObject>* mutable_objects()
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        return& objects_;
    }

    inline void objects(std::vector<motion_manager::interface::TrackedObject>& objects) const
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects.assign(objects_.begin(),objects_.end());
    }

    inline const std::vector<motion_manager::interface::TrackedObject>& objects() const
    {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        return objects_;
    }

    inline uint32_t objects_size() const {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        return objects_.size();
    }

    inline void clear_objects() {
        std::lock_guard<std::mutex> lock(*objects_mutex_);
        objects_.clear();
        objects_.shrink_to_fit();
    }

    inline bool has_objects() {
        return (objects_size() != 0);
    }

void operator = (const TrackedObjects& tracked_objects){
    CopyFrom(tracked_objects);
  }

  void CopyFrom(const TrackedObjects& tracked_objects ){
    header_ = tracked_objects.header();
    objects_ = tracked_objects.objects();
  }

protected:
    std::shared_ptr<std::mutex> objects_mutex_;
    motion_manager::interface::Header header_;
    motion_manager::interface::Header* header_ptr_ = nullptr;
    std::vector<motion_manager::interface::TrackedObject> objects_;
};
} // namespace interface
} // namespace motion_manager
