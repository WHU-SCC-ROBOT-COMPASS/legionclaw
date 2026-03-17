/**
 * @file    pavementype.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/header.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Pavementype {
public:
  Pavementype() {
    type_ = 0;
    confidence_ = 0.0;
  }
  ~Pavementype() = default;

  inline void set_header(const legionclaw::interface::Header &header) {
    header_ = header;
    header_ptr_ = &header_;
  }

  inline const legionclaw::interface::Header &header() const { return header_; }

  inline legionclaw::interface::Header *mutable_header() { return &header_; }

  inline bool has_header() { return (header_ptr_ != nullptr); }

  inline void set_type(const int32_t &type) {
    type_ = type;
    type_ptr_ = &type_;
  }

  inline const int32_t &type() const { return type_; }

  inline int32_t *mutable_type() { return &type_; }

  inline bool has_type() { return (type_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  void operator=(const Pavementype &pavementype) { CopyFrom(pavementype); }

  void CopyFrom(const Pavementype &pavementype) {
    header_ = pavementype.header();
    type_ = pavementype.type();
    confidence_ = pavementype.confidence();
  }

protected:
  //消息头
  legionclaw::interface::Header header_;
  legionclaw::interface::Header *header_ptr_ = nullptr;
  // 0-沥青 1-水泥 2-雪地 3-草地 4-泥地 5-沙地
  int32_t type_;
  int32_t *type_ptr_ = nullptr;
  //置信度
  double confidence_;
  double *confidence_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
