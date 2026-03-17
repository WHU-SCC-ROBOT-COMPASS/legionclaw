/**
 * @file    predicted_object_kinematics.hpp
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

#include "predicted_path.hpp"
#include "pose_with_covariance.hpp"
#include "accel_with_covariance.hpp"
#include "twist_with_covariance.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class PredictedObjectKinematics
{
public:
    PredictedObjectKinematics() { 
        ObjectClassification_mutex_ = std::make_shared<std::mutex>();

    clear_ObjectClassification();
 }
    ~PredictedObjectKinematics() = default;

    inline void set_initial_pose_with_covariance(const motion_manager::interface::PoseWithCovariance& initial_pose_with_covariance)
    {
        initial_pose_with_covariance_ = initial_pose_with_covariance;
        initial_pose_with_covariance_ptr_ = &initial_pose_with_covariance_;
    }

    inline const motion_manager::interface::PoseWithCovariance& initial_pose_with_covariance() const
    {
        return initial_pose_with_covariance_;
    }

    inline motion_manager::interface::PoseWithCovariance* mutable_initial_pose_with_covariance()
    {
        return& initial_pose_with_covariance_;
    }

    inline bool has_initial_pose_with_covariance()
    {
        return (initial_pose_with_covariance_ptr_ != nullptr);
    }

    inline void set_initial_twist_with_covariance(const motion_manager::interface::TwistWithCovariance& initial_twist_with_covariance)
    {
        initial_twist_with_covariance_ = initial_twist_with_covariance;
        initial_twist_with_covariance_ptr_ = &initial_twist_with_covariance_;
    }

    inline const motion_manager::interface::TwistWithCovariance& initial_twist_with_covariance() const
    {
        return initial_twist_with_covariance_;
    }

    inline motion_manager::interface::TwistWithCovariance* mutable_initial_twist_with_covariance()
    {
        return& initial_twist_with_covariance_;
    }

    inline bool has_initial_twist_with_covariance()
    {
        return (initial_twist_with_covariance_ptr_ != nullptr);
    }

    inline void set_initial_acceleration_with_covariance(const motion_manager::interface::AccelWithCovariance& initial_acceleration_with_covariance)
    {
        initial_acceleration_with_covariance_ = initial_acceleration_with_covariance;
        initial_acceleration_with_covariance_ptr_ = &initial_acceleration_with_covariance_;
    }

    inline const motion_manager::interface::AccelWithCovariance& initial_acceleration_with_covariance() const
    {
        return initial_acceleration_with_covariance_;
    }

    inline motion_manager::interface::AccelWithCovariance* mutable_initial_acceleration_with_covariance()
    {
        return& initial_acceleration_with_covariance_;
    }

    inline bool has_initial_acceleration_with_covariance()
    {
        return (initial_acceleration_with_covariance_ptr_ != nullptr);
    }

    inline void set_ObjectClassification(std::vector<motion_manager::interface::PredictedPath>* ObjectClassification)
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification_.assign(ObjectClassification->begin(), ObjectClassification->end());
    }

    inline void set_ObjectClassification(const std::vector<motion_manager::interface::PredictedPath>& ObjectClassification)
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification_=ObjectClassification;
    }

    inline void set_ObjectClassification(const uint32_t index,motion_manager::interface::PredictedPath& ObjectClassification)
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification_[index] = ObjectClassification;
    }

    inline void add_ObjectClassification(const 
    motion_manager::interface::PredictedPath& ObjectClassification) {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification_.emplace_back(ObjectClassification);
    }

    inline const motion_manager::interface::PredictedPath& ObjectClassification(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        return ObjectClassification_[index];
    }

    inline std::vector<motion_manager::interface::PredictedPath>* mutable_ObjectClassification()
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        return& ObjectClassification_;
    }

    inline void ObjectClassification(std::vector<motion_manager::interface::PredictedPath>& ObjectClassification) const
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification.assign(ObjectClassification_.begin(),ObjectClassification_.end());
    }

    inline const std::vector<motion_manager::interface::PredictedPath>& ObjectClassification() const
    {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        return ObjectClassification_;
    }

    inline uint32_t ObjectClassification_size() const {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        return ObjectClassification_.size();
    }

    inline void clear_ObjectClassification() {
        std::lock_guard<std::mutex> lock(*ObjectClassification_mutex_);
        ObjectClassification_.clear();
        ObjectClassification_.shrink_to_fit();
    }

    inline bool has_ObjectClassification() {
        return (ObjectClassification_size() != 0);
    }

void operator = (const PredictedObjectKinematics& predicted_object_kinematics){
    CopyFrom(predicted_object_kinematics);
  }

  void CopyFrom(const PredictedObjectKinematics& predicted_object_kinematics ){
    initial_pose_with_covariance_ = predicted_object_kinematics.initial_pose_with_covariance();
    initial_twist_with_covariance_ = predicted_object_kinematics.initial_twist_with_covariance();
    initial_acceleration_with_covariance_ = predicted_object_kinematics.initial_acceleration_with_covariance();
    ObjectClassification_ = predicted_object_kinematics.ObjectClassification();
  }

protected:
    std::shared_ptr<std::mutex> ObjectClassification_mutex_;
    motion_manager::interface::PoseWithCovariance initial_pose_with_covariance_;
    motion_manager::interface::PoseWithCovariance* initial_pose_with_covariance_ptr_ = nullptr;
    motion_manager::interface::TwistWithCovariance initial_twist_with_covariance_;
    motion_manager::interface::TwistWithCovariance* initial_twist_with_covariance_ptr_ = nullptr;
    motion_manager::interface::AccelWithCovariance initial_acceleration_with_covariance_;
    motion_manager::interface::AccelWithCovariance* initial_acceleration_with_covariance_ptr_ = nullptr;
    std::vector<motion_manager::interface::PredictedPath> ObjectClassification_;
};
} // namespace interface
} // namespace motion_manager
