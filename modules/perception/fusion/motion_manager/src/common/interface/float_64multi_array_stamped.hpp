/**
 * @file    float_64multi_array_stamped.hpp
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

#include "time.hpp"
#include "multi_array_layout.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class Float64MultiArrayStamped
{
public:
    Float64MultiArrayStamped() { 
        layout_mutex_ = std::make_shared<std::mutex>();
    data_mutex_ = std::make_shared<std::mutex>();

    clear_layout();
    clear_data();
 }
    ~Float64MultiArrayStamped() = default;

    inline void set_stamp(const motion_manager::interface::Time& stamp)
    {
        stamp_ = stamp;
        stamp_ptr_ = &stamp_;
    }

    inline const motion_manager::interface::Time& stamp() const
    {
        return stamp_;
    }

    inline motion_manager::interface::Time* mutable_stamp()
    {
        return& stamp_;
    }

    inline bool has_stamp()
    {
        return (stamp_ptr_ != nullptr);
    }

    inline void set_layout(std::vector<motion_manager::interface::MultiArrayLayout>* layout)
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout_.assign(layout->begin(), layout->end());
    }

    inline void set_layout(const std::vector<motion_manager::interface::MultiArrayLayout>& layout)
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout_=layout;
    }

    inline void set_layout(const uint32_t index,motion_manager::interface::MultiArrayLayout& layout)
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout_[index] = layout;
    }

    inline void add_layout(const 
    motion_manager::interface::MultiArrayLayout& layout) {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout_.emplace_back(layout);
    }

    inline const motion_manager::interface::MultiArrayLayout& layout(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        return layout_[index];
    }

    inline std::vector<motion_manager::interface::MultiArrayLayout>* mutable_layout()
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        return& layout_;
    }

    inline void layout(std::vector<motion_manager::interface::MultiArrayLayout>& layout) const
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout.assign(layout_.begin(),layout_.end());
    }

    inline const std::vector<motion_manager::interface::MultiArrayLayout>& layout() const
    {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        return layout_;
    }

    inline uint32_t layout_size() const {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        return layout_.size();
    }

    inline void clear_layout() {
        std::lock_guard<std::mutex> lock(*layout_mutex_);
        layout_.clear();
        layout_.shrink_to_fit();
    }

    inline bool has_layout() {
        return (layout_size() != 0);
    }

    inline void set_data(std::vector<double>* data)
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data_.assign(data->begin(), data->end());
    }

    inline void set_data(const std::vector<double>& data)
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data_=data;
    }

    inline void set_data(const uint32_t index,double& data)
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data_[index] = data;
    }

    inline void add_data(const 
    double& data) {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data_.emplace_back(data);
    }

    inline const double& data(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        return data_[index];
    }

    inline std::vector<double>* mutable_data()
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        return& data_;
    }

    inline void data(std::vector<double>& data) const
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data.assign(data_.begin(),data_.end());
    }

    inline const std::vector<double>& data() const
    {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        return data_;
    }

    inline uint32_t data_size() const {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        return data_.size();
    }

    inline void clear_data() {
        std::lock_guard<std::mutex> lock(*data_mutex_);
        data_.clear();
        data_.shrink_to_fit();
    }

    inline bool has_data() {
        return (data_size() != 0);
    }

void operator = (const Float64MultiArrayStamped& float_64multi_array_stamped){
    CopyFrom(float_64multi_array_stamped);
  }

  void CopyFrom(const Float64MultiArrayStamped& float_64multi_array_stamped ){
    stamp_ = float_64multi_array_stamped.stamp();
    layout_ = float_64multi_array_stamped.layout();
    data_ = float_64multi_array_stamped.data();
  }

protected:
    std::shared_ptr<std::mutex> layout_mutex_;
    std::shared_ptr<std::mutex> data_mutex_;
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    std::vector<motion_manager::interface::MultiArrayLayout> layout_;
    std::vector<double> data_;
};
} // namespace interface
} // namespace motion_manager
