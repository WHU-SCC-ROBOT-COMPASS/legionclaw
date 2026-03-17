/**
 * @file    predicted_path.hpp
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
#include "geometry_pose.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PredictedPath
{
public:
    PredictedPath() { 
        path_mutex_ = std::make_shared<std::mutex>();

    clear_path();
    confidence_ = 0.0;
 }
    ~PredictedPath() = default;

    inline void set_path(std::vector<motion_manager::interface::GeometryPose>* path)
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path_.assign(path->begin(), path->end());
    }

    inline void set_path(const std::vector<motion_manager::interface::GeometryPose>& path)
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path_=path;
    }

    inline void set_path(const uint32_t index,motion_manager::interface::GeometryPose& path)
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path_[index] = path;
    }

    inline void add_path(const 
    motion_manager::interface::GeometryPose& path) {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path_.emplace_back(path);
    }

    inline const motion_manager::interface::GeometryPose& path(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        return path_[index];
    }

    inline std::vector<motion_manager::interface::GeometryPose>* mutable_path()
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        return& path_;
    }

    inline void path(std::vector<motion_manager::interface::GeometryPose>& path) const
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path.assign(path_.begin(),path_.end());
    }

    inline const std::vector<motion_manager::interface::GeometryPose>& path() const
    {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        return path_;
    }

    inline uint32_t path_size() const {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        return path_.size();
    }

    inline void clear_path() {
        std::lock_guard<std::mutex> lock(*path_mutex_);
        path_.clear();
        path_.shrink_to_fit();
    }

    inline bool has_path() {
        return (path_size() != 0);
    }

    inline void set_time_step(const motion_manager::interface::Time& time_step)
    {
        time_step_ = time_step;
        time_step_ptr_ = &time_step_;
    }

    inline const motion_manager::interface::Time& time_step() const
    {
        return time_step_;
    }

    inline motion_manager::interface::Time* mutable_time_step()
    {
        return& time_step_;
    }

    inline bool has_time_step()
    {
        return (time_step_ptr_ != nullptr);
    }

    inline void set_confidence(const float& confidence)
    {
        confidence_ = confidence;
        confidence_ptr_ = &confidence_;
    }

    inline const float& confidence() const
    {
        return confidence_;
    }

    inline float* mutable_confidence()
    {
        return& confidence_;
    }

    inline bool has_confidence()
    {
        return (confidence_ptr_ != nullptr);
    }

void operator = (const PredictedPath& predicted_path){
    CopyFrom(predicted_path);
  }

  void CopyFrom(const PredictedPath& predicted_path ){
    path_ = predicted_path.path();
    time_step_ = predicted_path.time_step();
    confidence_ = predicted_path.confidence();
  }

protected:
    std::shared_ptr<std::mutex> path_mutex_;
    std::vector<motion_manager::interface::GeometryPose> path_;
    motion_manager::interface::Time time_step_;
    motion_manager::interface::Time* time_step_ptr_ = nullptr;
    float confidence_;
    float* confidence_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
