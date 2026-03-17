/**
 * @file    detected_object.hpp
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

#include "shape.hpp"
#include "object_classification.hpp"
#include "detected_object_kinematics.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class DetectedObject
{
public:
    DetectedObject() { 
        classification_mutex_ = std::make_shared<std::mutex>();

    existence_probability_ = 0.0;
    clear_classification();
 }
    ~DetectedObject() = default;

    inline void set_existence_probability(const float& existence_probability)
    {
        existence_probability_ = existence_probability;
        existence_probability_ptr_ = &existence_probability_;
    }

    inline const float& existence_probability() const
    {
        return existence_probability_;
    }

    inline float* mutable_existence_probability()
    {
        return& existence_probability_;
    }

    inline bool has_existence_probability()
    {
        return (existence_probability_ptr_ != nullptr);
    }

    inline void set_classification(std::vector<motion_manager::interface::ObjectClassification>* classification)
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification_.assign(classification->begin(), classification->end());
    }

    inline void set_classification(const std::vector<motion_manager::interface::ObjectClassification>& classification)
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification_=classification;
    }

    inline void set_classification(const uint32_t index,motion_manager::interface::ObjectClassification& classification)
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification_[index] = classification;
    }

    inline void add_classification(const 
    motion_manager::interface::ObjectClassification& classification) {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification_.emplace_back(classification);
    }

    inline const motion_manager::interface::ObjectClassification& classification(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        return classification_[index];
    }

    inline std::vector<motion_manager::interface::ObjectClassification>* mutable_classification()
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        return& classification_;
    }

    inline void classification(std::vector<motion_manager::interface::ObjectClassification>& classification) const
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification.assign(classification_.begin(),classification_.end());
    }

    inline const std::vector<motion_manager::interface::ObjectClassification>& classification() const
    {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        return classification_;
    }

    inline uint32_t classification_size() const {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        return classification_.size();
    }

    inline void clear_classification() {
        std::lock_guard<std::mutex> lock(*classification_mutex_);
        classification_.clear();
        classification_.shrink_to_fit();
    }

    inline bool has_classification() {
        return (classification_size() != 0);
    }

    inline void set_kinematics(const motion_manager::interface::DetectedObjectKinematics& kinematics)
    {
        kinematics_ = kinematics;
        kinematics_ptr_ = &kinematics_;
    }

    inline const motion_manager::interface::DetectedObjectKinematics& kinematics() const
    {
        return kinematics_;
    }

    inline motion_manager::interface::DetectedObjectKinematics* mutable_kinematics()
    {
        return& kinematics_;
    }

    inline bool has_kinematics()
    {
        return (kinematics_ptr_ != nullptr);
    }

    inline void set_shape(const motion_manager::interface::Shape& shape)
    {
        shape_ = shape;
        shape_ptr_ = &shape_;
    }

    inline const motion_manager::interface::Shape& shape() const
    {
        return shape_;
    }

    inline motion_manager::interface::Shape* mutable_shape()
    {
        return& shape_;
    }

    inline bool has_shape()
    {
        return (shape_ptr_ != nullptr);
    }

void operator = (const DetectedObject& detected_object){
    CopyFrom(detected_object);
  }

  void CopyFrom(const DetectedObject& detected_object ){
    existence_probability_ = detected_object.existence_probability();
    classification_ = detected_object.classification();
    kinematics_ = detected_object.kinematics();
    shape_ = detected_object.shape();
  }

protected:
    std::shared_ptr<std::mutex> classification_mutex_;
    float existence_probability_;
    float* existence_probability_ptr_ = nullptr;
    std::vector<motion_manager::interface::ObjectClassification> classification_;
    motion_manager::interface::DetectedObjectKinematics kinematics_;
    motion_manager::interface::DetectedObjectKinematics* kinematics_ptr_ = nullptr;
    motion_manager::interface::Shape shape_;
    motion_manager::interface::Shape* shape_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
