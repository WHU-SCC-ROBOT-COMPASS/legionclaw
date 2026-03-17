/**
 * @file    traffic_signal.hpp
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

#include "traffic_signal_element.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class TrafficSignal
{
public:
    TrafficSignal() { 
        elements_mutex_ = std::make_shared<std::mutex>();

    traffic_signal_id_ = 0;
    clear_elements();
 }
    ~TrafficSignal() = default;

    inline void set_traffic_signal_id(const int64_t& traffic_signal_id)
    {
        traffic_signal_id_ = traffic_signal_id;
        traffic_signal_id_ptr_ = &traffic_signal_id_;
    }

    inline const int64_t& traffic_signal_id() const
    {
        return traffic_signal_id_;
    }

    inline int64_t* mutable_traffic_signal_id()
    {
        return& traffic_signal_id_;
    }

    inline bool has_traffic_signal_id()
    {
        return (traffic_signal_id_ptr_ != nullptr);
    }

    inline void set_elements(std::vector<motion_manager::interface::TrafficSignalElement>* elements)
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements_.assign(elements->begin(), elements->end());
    }

    inline void set_elements(const std::vector<motion_manager::interface::TrafficSignalElement>& elements)
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements_=elements;
    }

    inline void set_elements(const uint32_t index,motion_manager::interface::TrafficSignalElement& elements)
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements_[index] = elements;
    }

    inline void add_elements(const 
    motion_manager::interface::TrafficSignalElement& elements) {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements_.emplace_back(elements);
    }

    inline const motion_manager::interface::TrafficSignalElement& elements(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        return elements_[index];
    }

    inline std::vector<motion_manager::interface::TrafficSignalElement>* mutable_elements()
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        return& elements_;
    }

    inline void elements(std::vector<motion_manager::interface::TrafficSignalElement>& elements) const
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements.assign(elements_.begin(),elements_.end());
    }

    inline const std::vector<motion_manager::interface::TrafficSignalElement>& elements() const
    {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        return elements_;
    }

    inline uint32_t elements_size() const {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        return elements_.size();
    }

    inline void clear_elements() {
        std::lock_guard<std::mutex> lock(*elements_mutex_);
        elements_.clear();
        elements_.shrink_to_fit();
    }

    inline bool has_elements() {
        return (elements_size() != 0);
    }

void operator = (const TrafficSignal& traffic_signal){
    CopyFrom(traffic_signal);
  }

  void CopyFrom(const TrafficSignal& traffic_signal ){
    traffic_signal_id_ = traffic_signal.traffic_signal_id();
    elements_ = traffic_signal.elements();
  }

protected:
    std::shared_ptr<std::mutex> elements_mutex_;
    int64_t traffic_signal_id_;
    int64_t* traffic_signal_id_ptr_ = nullptr;
    std::vector<motion_manager::interface::TrafficSignalElement> elements_;
};
} // namespace interface
} // namespace motion_manager
