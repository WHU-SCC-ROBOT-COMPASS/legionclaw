/**
 * @file    object_classification.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "common/enum/enum.h"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class ObjectClassification
{
public:
    ObjectClassification() { 
    label_ = motion_manager::common::DetectedObjectLabel::DO_UNKNOWN;
    probability_ = 0.0;
 }
    ~ObjectClassification() = default;

    inline void set_label(const motion_manager::common::DetectedObjectLabel& label)
    {
        label_ = label;
        label_ptr_ = &label_;
    }

    inline const motion_manager::common::DetectedObjectLabel& label() const
    {
        return label_;
    }

    inline motion_manager::common::DetectedObjectLabel* mutable_label()
    {
        return& label_;
    }

    inline bool has_label()
    {
        return (label_ptr_ != nullptr);
    }

    inline void set_probability(const float& probability)
    {
        probability_ = probability;
        probability_ptr_ = &probability_;
    }

    inline const float& probability() const
    {
        return probability_;
    }

    inline float* mutable_probability()
    {
        return& probability_;
    }

    inline bool has_probability()
    {
        return (probability_ptr_ != nullptr);
    }

void operator = (const ObjectClassification& object_classification){
    CopyFrom(object_classification);
  }

  void CopyFrom(const ObjectClassification& object_classification ){
    label_ = object_classification.label();
    probability_ = object_classification.probability();
  }

protected:
    motion_manager::common::DetectedObjectLabel label_;
    motion_manager::common::DetectedObjectLabel* label_ptr_ = nullptr;
    float probability_;
    float* probability_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
