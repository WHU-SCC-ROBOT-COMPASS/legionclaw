/**
 * @file    twist_with_covariance.hpp
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

#include "twist.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class TwistWithCovariance
{
public:
    TwistWithCovariance() { 
        covariance_mutex_ = std::make_shared<std::mutex>();

    clear_covariance();
 }
    ~TwistWithCovariance() = default;

    inline void set_twist(const motion_manager::interface::Twist& twist)
    {
        twist_ = twist;
        twist_ptr_ = &twist_;
    }

    inline const motion_manager::interface::Twist& twist() const
    {
        return twist_;
    }

    inline motion_manager::interface::Twist* mutable_twist()
    {
        return& twist_;
    }

    inline bool has_twist()
    {
        return (twist_ptr_ != nullptr);
    }

    inline void set_covariance(std::vector<double>* covariance)
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance_.assign(covariance->begin(), covariance->end());
    }

    inline void set_covariance(const std::vector<double>& covariance)
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance_=covariance;
    }

    inline void set_covariance(const uint32_t index,double& covariance)
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance_[index] = covariance;
    }

    inline void add_covariance(const 
    double& covariance) {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance_.emplace_back(covariance);
    }

    inline const double& covariance(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        return covariance_[index];
    }

    inline std::vector<double>* mutable_covariance()
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        return& covariance_;
    }

    inline void covariance(std::vector<double>& covariance) const
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance.assign(covariance_.begin(),covariance_.end());
    }

    inline const std::vector<double>& covariance() const
    {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        return covariance_;
    }

    inline uint32_t covariance_size() const {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        return covariance_.size();
    }

    inline void clear_covariance() {
        std::lock_guard<std::mutex> lock(*covariance_mutex_);
        covariance_.clear();
        covariance_.shrink_to_fit();
    }

    inline bool has_covariance() {
        return (covariance_size() != 0);
    }

void operator = (const TwistWithCovariance& twist_with_covariance){
    CopyFrom(twist_with_covariance);
  }

  void CopyFrom(const TwistWithCovariance& twist_with_covariance ){
    twist_ = twist_with_covariance.twist();
    covariance_ = twist_with_covariance.covariance();
  }

protected:
    std::shared_ptr<std::mutex> covariance_mutex_;
    motion_manager::interface::Twist twist_;
    motion_manager::interface::Twist* twist_ptr_ = nullptr;
    std::vector<double> covariance_;
};
} // namespace interface
} // namespace motion_manager
