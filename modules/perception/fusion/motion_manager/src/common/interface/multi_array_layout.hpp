/**
 * @file    multi_array_layout.hpp
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

#include "multi_array_dimension.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class MultiArrayLayout
{
public:
    MultiArrayLayout() { 
        dim_mutex_ = std::make_shared<std::mutex>();

    clear_dim();
    data_offset_ = 0;
 }
    ~MultiArrayLayout() = default;

    inline void set_dim(std::vector<motion_manager::interface::MultiArrayDimension>* dim)
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim_.assign(dim->begin(), dim->end());
    }

    inline void set_dim(const std::vector<motion_manager::interface::MultiArrayDimension>& dim)
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim_=dim;
    }

    inline void set_dim(const uint32_t index,motion_manager::interface::MultiArrayDimension& dim)
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim_[index] = dim;
    }

    inline void add_dim(const 
    motion_manager::interface::MultiArrayDimension& dim) {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim_.emplace_back(dim);
    }

    inline const motion_manager::interface::MultiArrayDimension& dim(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        return dim_[index];
    }

    inline std::vector<motion_manager::interface::MultiArrayDimension>* mutable_dim()
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        return& dim_;
    }

    inline void dim(std::vector<motion_manager::interface::MultiArrayDimension>& dim) const
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim.assign(dim_.begin(),dim_.end());
    }

    inline const std::vector<motion_manager::interface::MultiArrayDimension>& dim() const
    {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        return dim_;
    }

    inline uint32_t dim_size() const {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        return dim_.size();
    }

    inline void clear_dim() {
        std::lock_guard<std::mutex> lock(*dim_mutex_);
        dim_.clear();
        dim_.shrink_to_fit();
    }

    inline bool has_dim() {
        return (dim_size() != 0);
    }

    inline void set_data_offset(const uint32_t& data_offset)
    {
        data_offset_ = data_offset;
        data_offset_ptr_ = &data_offset_;
    }

    inline const uint32_t& data_offset() const
    {
        return data_offset_;
    }

    inline uint32_t* mutable_data_offset()
    {
        return& data_offset_;
    }

    inline bool has_data_offset()
    {
        return (data_offset_ptr_ != nullptr);
    }

void operator = (const MultiArrayLayout& multi_array_layout){
    CopyFrom(multi_array_layout);
  }

  void CopyFrom(const MultiArrayLayout& multi_array_layout ){
    dim_ = multi_array_layout.dim();
    data_offset_ = multi_array_layout.data_offset();
  }

protected:
    std::shared_ptr<std::mutex> dim_mutex_;
    std::vector<motion_manager::interface::MultiArrayDimension> dim_;
    uint32_t data_offset_;
    uint32_t* data_offset_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
