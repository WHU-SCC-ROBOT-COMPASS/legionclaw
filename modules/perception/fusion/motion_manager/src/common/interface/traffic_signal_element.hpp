/**
 * @file    traffic_signal_element.hpp
 * @author  dabai-motion_manager
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

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
class TrafficSignalElement
{
public:
    TrafficSignalElement() { 
    color_ = 0;
    shape_ = 0;
    status_ = 0;
    confidence_ = 0.0;
 }
    ~TrafficSignalElement() = default;

    inline void set_color(const uint8_t& color)
    {
        color_ = color;
        color_ptr_ = &color_;
    }

    inline const uint8_t& color() const
    {
        return color_;
    }

    inline uint8_t* mutable_color()
    {
        return& color_;
    }

    inline bool has_color()
    {
        return (color_ptr_ != nullptr);
    }

    inline void set_shape(const uint8_t& shape)
    {
        shape_ = shape;
        shape_ptr_ = &shape_;
    }

    inline const uint8_t& shape() const
    {
        return shape_;
    }

    inline uint8_t* mutable_shape()
    {
        return& shape_;
    }

    inline bool has_shape()
    {
        return (shape_ptr_ != nullptr);
    }

    inline void set_status(const uint8_t& status)
    {
        status_ = status;
        status_ptr_ = &status_;
    }

    inline const uint8_t& status() const
    {
        return status_;
    }

    inline uint8_t* mutable_status()
    {
        return& status_;
    }

    inline bool has_status()
    {
        return (status_ptr_ != nullptr);
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

void operator = (const TrafficSignalElement& traffic_signal_element){
    CopyFrom(traffic_signal_element);
  }

  void CopyFrom(const TrafficSignalElement& traffic_signal_element ){
    color_ = traffic_signal_element.color();
    shape_ = traffic_signal_element.shape();
    status_ = traffic_signal_element.status();
    confidence_ = traffic_signal_element.confidence();
  }

protected:
    uint8_t color_;
    uint8_t* color_ptr_ = nullptr;
    uint8_t shape_;
    uint8_t* shape_ptr_ = nullptr;
    uint8_t status_;
    uint8_t* status_ptr_ = nullptr;
    float confidence_;
    float* confidence_ptr_ = nullptr;
};
} // namespace interface
} // namespace motion_manager
