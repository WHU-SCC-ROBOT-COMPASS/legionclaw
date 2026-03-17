/**
 * @file    traffic_signal_array.hpp
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
#include "traffic_signal.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class TrafficSignalArray
{
public:
    TrafficSignalArray() { 
        signals_mutex_ = std::make_shared<std::mutex>();

    clear_signals();
 }
    ~TrafficSignalArray() = default;

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

    inline void set_signals(std::vector<motion_manager::interface::TrafficSignal>* signals)
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals_.assign(signals->begin(), signals->end());
    }

    inline void set_signals(const std::vector<motion_manager::interface::TrafficSignal>& signals)
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals_=signals;
    }

    inline void set_signals(const uint32_t index,motion_manager::interface::TrafficSignal& signals)
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals_[index] = signals;
    }

    inline void add_signals(const 
    motion_manager::interface::TrafficSignal& signals) {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals_.emplace_back(signals);
    }

    inline const motion_manager::interface::TrafficSignal& signals(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        return signals_[index];
    }

    inline std::vector<motion_manager::interface::TrafficSignal>* mutable_signals()
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        return& signals_;
    }

    inline void signals(std::vector<motion_manager::interface::TrafficSignal>& signals) const
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals.assign(signals_.begin(),signals_.end());
    }

    inline const std::vector<motion_manager::interface::TrafficSignal>& signals() const
    {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        return signals_;
    }

    inline uint32_t signals_size() const {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        return signals_.size();
    }

    inline void clear_signals() {
        std::lock_guard<std::mutex> lock(*signals_mutex_);
        signals_.clear();
        signals_.shrink_to_fit();
    }

    inline bool has_signals() {
        return (signals_size() != 0);
    }

void operator = (const TrafficSignalArray& traffic_signal_array){
    CopyFrom(traffic_signal_array);
  }

  void CopyFrom(const TrafficSignalArray& traffic_signal_array ){
    stamp_ = traffic_signal_array.stamp();
    signals_ = traffic_signal_array.signals();
  }

protected:
    std::shared_ptr<std::mutex> signals_mutex_;
    motion_manager::interface::Time stamp_;
    motion_manager::interface::Time* stamp_ptr_ = nullptr;
    std::vector<motion_manager::interface::TrafficSignal> signals_;
};
} // namespace interface
} // namespace motion_manager
