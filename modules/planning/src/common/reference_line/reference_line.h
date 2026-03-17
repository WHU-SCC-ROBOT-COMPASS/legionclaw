/**
 * @file reference_line.h
 **/

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <float.h>

#include "modules/planning/src/common/reference_line/reference_point.h"
#include "modules/planning/src/interface/lane_point_inner.hpp"
#include "modules/common/math/linear_interpolation.h"

/**
 * @namespace legionclaw::reference_line
 * @brief legionclaw::reference_line
 */
namespace legionclaw {
namespace reference_line {

class ReferenceLine {
 public:
  ReferenceLine() = default;

  // explicit ReferenceLine(const ReferenceLine& reference_line) = default;

  // template <typename Iterator>
  // ReferenceLine(const Iterator begin, const Iterator end)
  //     : reference_points_(begin, end){}

  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);

  explicit ReferenceLine(
      const std::vector<legionclaw::interface::PathPoint>& path_points);

  explicit ReferenceLine(
      const std::vector<legionclaw::interface::LanePoint>& lane_points);

  explicit ReferenceLine(
      const std::vector<legionclaw::planning::LanePointInner>& lane_points);

  // std::vector<ReferencePoint> ReferencePoints() const {
  //   return reference_points_;
  // }

  inline const std::vector<ReferencePoint>& ReferencePoints() const {
    // std::lock_guard<std::mutex> lock(*reference_points_mutex_);
    return reference_points_;
  }
  ReferencePoint ReferencePoints(const int& i) const {
    if (i < 0) return reference_points_.front();
    if (i > (int)reference_points_.size() - 1) return reference_points_.back();
    return reference_points_.at(i);
  }

  bool ReferencePoints(const int& i, const ReferencePoint& p) {
    if (i < 0) return false;
    if (i > (int)reference_points_.size() - 1) return false;

    reference_points_.at(i) = p;
    return true;
  }

  void Resize(const int& num) { reference_points_.resize(num); }
  int GlobalID() const { return global_id_; }
  void SetGlobalID(const int& gid) { global_id_ = gid; }

  inline void set_remain_mileage(const double& remain_mileage) {
    remain_mileage_ = remain_mileage;
  }

  inline const double& remain_mileage() const { return remain_mileage_; }

  inline double* mutable_remain_mileage() { return &remain_mileage_; }

  void Clear() {
    reference_points_.clear();
    reference_points_.shrink_to_fit();
    global_id_ = -1;
    remain_mileage_ = DBL_MIN;
  }
  int ReferencePointSize() const { return reference_points_.size(); }
  double Mileage() { return reference_points_.back().s(); }
  inline const double Length() const {
    return reference_points_.back().s() - reference_points_.front().s();
  }
  void AddPoint(const ReferencePoint& p) { reference_points_.push_back(p); }
  void ComputeAllMileage(const double s0);
  void ComputeAllMileageAndWidth(const double s0,
                                 const std::vector<ReferencePoint> ref_points);
  void TransfertRad2Deg();

  void SetConstSpeed(const double v) {
    for (unsigned int i = 0; i < reference_points_.size(); ++i)
      reference_points_.at(i).set_limit_speed(v);
  }

  bool Stitch(const ReferenceLine& other, const int match_index);

  bool Segment(const int start_index, const int end_index);

  // private:
  // std::shared_ptr<std::mutex> reference_points_mutex_;
  std::vector<ReferencePoint> reference_points_;
  uint32_t priority_ = 0;
  int global_id_ = 0;
  double remain_mileage_;
};

}  // namespace reference_line
}  // namespace legionclaw
