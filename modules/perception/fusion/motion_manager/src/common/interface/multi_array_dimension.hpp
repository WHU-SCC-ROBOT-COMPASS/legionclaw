/**
 * @file    multi_array_dimension.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <string>
#include <iostream>
#include <stdint.h>



/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class MultiArrayDimension
{
public:
    MultiArrayDimension() { 
    label_.clear();
    size_ = 0;
    stride_ = 0;
 }
    ~MultiArrayDimension() = default;

    inline void set_label(const std::string& label)
    {
        label_ = label;
        label_ptr_ = &label_;
    }

    inline const std::string& label() const
    {
        return label_;
    }

    inline std::string* mutable_label()
    {
        return& label_;
    }

    inline bool has_label()
    {
        return (label_ptr_ != nullptr);
    }

    inline void set_size(const uint32_t& size)
    {
        size_ = size;
        size_ptr_ = &size_;
    }

    inline const uint32_t& size() const
    {
        return size_;
    }

    inline uint32_t* mutable_size()
    {
        return& size_;
    }

    inline bool has_size()
    {
        return (size_ptr_ != nullptr);
    }

    inline void set_stride(const uint32_t& stride)
    {
        stride_ = stride;
        stride_ptr_ = &stride_;
    }

    inline const uint32_t& stride() const
    {
        return stride_;
    }

    inline uint32_t* mutable_stride()
    {
        return& stride_;
    }

    inline bool has_stride()
    {
        return (stride_ptr_ != nullptr);
    }

void operator = (const MultiArrayDimension& multi_array_dimension){
    CopyFrom(multi_array_dimension);
  }

  void CopyFrom(const MultiArrayDimension& multi_array_dimension ){
    label_ = multi_array_dimension.label();
    size_ = multi_array_dimension.size();
    stride_ = multi_array_dimension.stride();
  }

protected:
    std::string label_;
    std::string* label_ptr_ = nullptr;
    uint32_t size_;
    uint32_t* size_ptr_ = nullptr;
    uint32_t stride_;
    uint32_t* stride_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
