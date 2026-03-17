/**
 * @file reference_point.h
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/lane_point.hpp"
#include "modules/planning/src/interface/lane_point_inner.hpp"

/**
 * @namespace legionclaw::reference_line
 * @brief legionclaw::reference_line
 */
namespace legionclaw {
namespace reference_line {

class ReferencePoint : public legionclaw::interface::PathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const legionclaw::interface::PathPoint& map_point,
                 const double longitudinal_bound, const double lateral_bound,
                 const bool enforced);
  ReferencePoint(const legionclaw::interface::LanePoint& map_point,
                 const double longitudinal_bound, const double lateral_bound,
                 const bool enforced);
  ReferencePoint(const legionclaw::planning::LanePointInner& map_point,
                 const double longitudinal_bound, const double lateral_bound,
                 const bool enforced);

  double longitudinal_bound() const { return longitudinal_bound_; }
  double lateral_bound() const { return lateral_bound_; }
  bool enforced() const { return enforced_; }
  double limit_speed() const { return limit_speed_; }
  double left_road_width() const { return left_road_width_; }
  double right_road_width() const { return right_road_width_; }
  legionclaw::common::LaneLineType left_line_type() const { return left_line_type_; }
  legionclaw::common::LaneLineType right_line_type() const { return right_line_type_; }

  void set_longitudinal_bound(const double longitudinal_bound) {
    longitudinal_bound_ = longitudinal_bound;
  }
  void set_lateral_bound(const double lateral_bound) {
    lateral_bound_ = lateral_bound;
  }
  void set_enforced(const bool enforced) { enforced_ = enforced; }
  void set_limit_speed(const double limit_speed) { limit_speed_ = limit_speed; }
  void set_left_road_width(const double left_road_width) {
    left_road_width_ = left_road_width;
  }
  void set_right_road_width(const double right_road_width) {
    right_road_width_ = right_road_width;
  }
  void set_left_line_type(const legionclaw::common::LaneLineType left_line_type) {
    left_line_type_ = left_line_type;
  }
  void set_right_line_type(const legionclaw::common::LaneLineType right_line_type) {
    right_line_type_ = right_line_type;
  }

  // static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double longitudinal_bound_ = 0.0;
  double lateral_bound_ = 0.0;
  bool enforced_ = 0.0;
  double limit_speed_ = 0.0;
  double left_road_width_ = 0.0;
  double right_road_width_ = 0.0;
  legionclaw::common::LaneLineType left_line_type_ =
      legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
  legionclaw::common::LaneLineType right_line_type_ =
      legionclaw::common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
};

}  // namespace reference_line
}  // namespace legionclaw
