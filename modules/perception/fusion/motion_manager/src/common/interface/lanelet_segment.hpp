/**
 * @file    lanelet_segment.hpp
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

#include "lanelet_primitive.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class LaneletSegment
{
public:
    LaneletSegment() { 
        primitives_mutex_ = std::make_shared<std::mutex>();

    clear_primitives();
 }
    ~LaneletSegment() = default;

    inline void set_preferred_primitive(const motion_manager::interface::LaneletPrimitive& preferred_primitive)
    {
        preferred_primitive_ = preferred_primitive;
        preferred_primitive_ptr_ = &preferred_primitive_;
    }

    inline const motion_manager::interface::LaneletPrimitive& preferred_primitive() const
    {
        return preferred_primitive_;
    }

    inline motion_manager::interface::LaneletPrimitive* mutable_preferred_primitive()
    {
        return& preferred_primitive_;
    }

    inline bool has_preferred_primitive()
    {
        return (preferred_primitive_ptr_ != nullptr);
    }

    inline void set_primitives(std::vector<motion_manager::interface::LaneletPrimitive>* primitives)
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives_.assign(primitives->begin(), primitives->end());
    }

    inline void set_primitives(const std::vector<motion_manager::interface::LaneletPrimitive>& primitives)
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives_=primitives;
    }

    inline void set_primitives(const uint32_t index,motion_manager::interface::LaneletPrimitive& primitives)
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives_[index] = primitives;
    }

    inline void add_primitives(const 
    motion_manager::interface::LaneletPrimitive& primitives) {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives_.emplace_back(primitives);
    }

    inline const motion_manager::interface::LaneletPrimitive& primitives(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        return primitives_[index];
    }

    inline std::vector<motion_manager::interface::LaneletPrimitive>* mutable_primitives()
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        return& primitives_;
    }

    inline void primitives(std::vector<motion_manager::interface::LaneletPrimitive>& primitives) const
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives.assign(primitives_.begin(),primitives_.end());
    }

    inline const std::vector<motion_manager::interface::LaneletPrimitive>& primitives() const
    {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        return primitives_;
    }

    inline uint32_t primitives_size() const {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        return primitives_.size();
    }

    inline void clear_primitives() {
        std::lock_guard<std::mutex> lock(*primitives_mutex_);
        primitives_.clear();
        primitives_.shrink_to_fit();
    }

    inline bool has_primitives() {
        return (primitives_size() != 0);
    }

void operator = (const LaneletSegment& lanelet_segment){
    CopyFrom(lanelet_segment);
  }

  void CopyFrom(const LaneletSegment& lanelet_segment ){
    preferred_primitive_ = lanelet_segment.preferred_primitive();
    primitives_ = lanelet_segment.primitives();
  }

protected:
    std::shared_ptr<std::mutex> primitives_mutex_;
    motion_manager::interface::LaneletPrimitive preferred_primitive_;
    motion_manager::interface::LaneletPrimitive* preferred_primitive_ptr_ = nullptr;
    std::vector<motion_manager::interface::LaneletPrimitive> primitives_;
};
} // namespace interface
} // namespace motion_manager
