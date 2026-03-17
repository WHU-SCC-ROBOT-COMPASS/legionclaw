/**
 * @file    region.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <vector>

#include "modules/common/interface/point_3d.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Region {
public:
  Region() {
    region_polygon_mutex_ = std::make_shared<std::mutex>();

    name_region_ = legionclaw::interface::Region::RegionID::REGION_ID_UNKNOWN;
    score_ = 0.0;
    rank_risk_ = legionclaw::interface::Region::RankRisk::NO_RISK;
    clear_region_polygon();
  }
  ~Region() = default;

  enum RankRisk {
    NO_RISK = 0,
    FIRST_RANK_RISK = 1,
    SECOND_RANK_RISK = 2,
    THIRD_RANK_RISK = 3,
  };

  enum RegionID {
    REGION_ID_UNKNOWN = 0,
    REGION_ID_FL = 1,
    REGION_ID_FM = 2,
    REGION_ID_FR = 3,
    REGION_ID_RL = 4,
    REGION_ID_RM = 5,
    REGION_ID_RR = 6,
  };

  inline void
  set_name_region(const legionclaw::interface::Region::RegionID &name_region) {
    name_region_ = name_region;
    name_region_ptr_ = &name_region_;
  }

  inline const legionclaw::interface::Region::RegionID &name_region() const {
    return name_region_;
  }

  inline legionclaw::interface::Region::RegionID *mutable_name_region() {
    return &name_region_;
  }

  inline bool has_name_region() { return (name_region_ptr_ != nullptr); }

  inline void set_score(const double &score) {
    score_ = score;
    score_ptr_ = &score_;
  }

  inline const double &score() const { return score_; }

  inline double *mutable_score() { return &score_; }

  inline bool has_score() { return (score_ptr_ != nullptr); }

  inline void
  set_rank_risk(const legionclaw::interface::Region::RankRisk &rank_risk) {
    rank_risk_ = rank_risk;
    rank_risk_ptr_ = &rank_risk_;
  }

  inline const legionclaw::interface::Region::RankRisk &rank_risk() const {
    return rank_risk_;
  }

  inline legionclaw::interface::Region::RankRisk *mutable_rank_risk() {
    return &rank_risk_;
  }

  inline bool has_rank_risk() { return (rank_risk_ptr_ != nullptr); }

  inline void
  set_region_polygon(std::vector<legionclaw::interface::Point3D> *region_polygon) {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon_.assign(region_polygon->begin(), region_polygon->end());
  }

  inline void set_region_polygon(
      const std::vector<legionclaw::interface::Point3D> &region_polygon) {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon_ = region_polygon;
  }

  inline void set_region_polygon(const uint32_t index,
                                 legionclaw::interface::Point3D &region_polygon) {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon_[index] = region_polygon;
  }

  inline void
  add_region_polygon(const legionclaw::interface::Point3D &region_polygon) {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon_.emplace_back(region_polygon);
  }

  inline const legionclaw::interface::Point3D &region_polygon(uint32_t index) const {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    return region_polygon_[index];
  }

  inline std::vector<legionclaw::interface::Point3D> *mutable_region_polygon() {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    return &region_polygon_;
  }

  inline void
  region_polygon(std::vector<legionclaw::interface::Point3D> &region_polygon) const {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon.assign(region_polygon_.begin(), region_polygon_.end());
  }

  inline const std::vector<legionclaw::interface::Point3D> &region_polygon() const {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    return region_polygon_;
  }

  inline uint32_t region_polygon_size() const {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    return region_polygon_.size();
  }

  inline void clear_region_polygon() {
    std::lock_guard<std::mutex> lock(*region_polygon_mutex_);
    region_polygon_.clear();
    region_polygon_.shrink_to_fit();
  }

  inline bool has_region_polygon() { return (region_polygon_size() != 0); }

  void operator=(const Region &region) { CopyFrom(region); }

  void CopyFrom(const Region &region) {
    name_region_ = region.name_region();
    score_ = region.score();
    rank_risk_ = region.rank_risk();
    region_polygon_ = region.region_polygon();
  }

protected:
  std::shared_ptr<std::mutex> region_polygon_mutex_;
  //前后左右区域
  legionclaw::interface::Region::RegionID name_region_;
  legionclaw::interface::Region::RegionID *name_region_ptr_ = nullptr;
  //车辆前后左右的区域打分获得的分值(0-100表示逐步危险程度)
  double score_;
  double *score_ptr_ = nullptr;
  //危险程度(0-3四级划分，不同分级对应不同驾驶处理策略)
  legionclaw::interface::Region::RankRisk rank_risk_;
  legionclaw::interface::Region::RankRisk *rank_risk_ptr_ = nullptr;
  //世界坐标系下的区域多边形
  std::vector<legionclaw::interface::Point3D> region_polygon_;
};
} // namespace interface
} // namespace legionclaw
