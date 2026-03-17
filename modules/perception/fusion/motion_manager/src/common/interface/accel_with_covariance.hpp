/**
 * @file    accel_with_covariance.hpp
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

#include "accel.hpp"


/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class AccelWithCovariance
{
public:
    AccelWithCovariance() { 
        covariance_mutex_ = std::make_shared<std::mutex>();

    clear_covariance();
 }
    ~AccelWithCovariance() = default;

    inline void set_accel(const motion_manager::interface::Accel& accel)
    {
        accel_ = accel;
        accel_ptr_ = &accel_;
    }

    inline const motion_manager::interface::Accel& accel() const
    {
        return accel_;
    }

    inline motion_manager::interface::Accel* mutable_accel()
    {
        return& accel_;
    }

    inline bool has_accel()
    {
        return (accel_ptr_ != nullptr);
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

void operator = (const AccelWithCovariance& accel_with_covariance){
    CopyFrom(accel_with_covariance);
  }

  void CopyFrom(const AccelWithCovariance& accel_with_covariance ){
    accel_ = accel_with_covariance.accel();
    covariance_ = accel_with_covariance.covariance();
  }

protected:
    std::shared_ptr<std::mutex> covariance_mutex_;
    motion_manager::interface::Accel accel_;
    motion_manager::interface::Accel* accel_ptr_ = nullptr;
    std::vector<double> covariance_;
};
} // namespace interface
} // namespace motion_manager
