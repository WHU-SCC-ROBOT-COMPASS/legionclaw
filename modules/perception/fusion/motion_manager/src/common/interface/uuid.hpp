/**
 * @file    uui_d.hpp
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



/**
* @namespace motion_manager::interface
* @brief motion_manager::interface
*/
namespace motion_manager
{
namespace interface
{
class UUID
{
public:
    UUID() { 
        uuid_mutex_ = std::make_shared<std::mutex>();

    clear_uuid();
 }
    ~UUID() = default;

    inline void set_uuid(std::vector<uint8_t>* uuid)
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid_.assign(uuid->begin(), uuid->end());
    }

    inline void set_uuid(const std::vector<uint8_t>& uuid)
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid_=uuid;
    }

    inline void set_uuid(const uint32_t index,uint8_t& uuid)
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid_[index] = uuid;
    }

    inline void add_uuid(const 
    uint8_t& uuid) {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid_.emplace_back(uuid);
    }

    inline const uint8_t& uuid(uint32_t index) const
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        return uuid_[index];
    }

    inline std::vector<uint8_t>* mutable_uuid()
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        return& uuid_;
    }

    inline void uuid(std::vector<uint8_t>& uuid) const
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid.assign(uuid_.begin(),uuid_.end());
    }

    inline const std::vector<uint8_t>& uuid() const
    {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        return uuid_;
    }

    inline uint32_t uuid_size() const {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        return uuid_.size();
    }

    inline void clear_uuid() {
        std::lock_guard<std::mutex> lock(*uuid_mutex_);
        uuid_.clear();
        uuid_.shrink_to_fit();
    }

    inline bool has_uuid() {
        return (uuid_size() != 0);
    }

void operator = (const UUID& uuid){
    CopyFrom(uuid);
  }

  void CopyFrom(const UUID& uuid ){
    uuid_ = uuid.uuid();
  }

protected:
    std::shared_ptr<std::mutex> uuid_mutex_;
    std::vector<uint8_t> uuid_;
};
} // namespace interface
} // namespace motion_manager
