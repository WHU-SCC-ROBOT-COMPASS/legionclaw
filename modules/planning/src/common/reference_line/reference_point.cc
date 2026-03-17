/**
 * @file reference_point.cc
 **/

#include "modules/planning/src/common/reference_line/reference_point.h"

/**
 * @namespace legionclaw::reference_line
 * @brief legionclaw::reference_line
 */
namespace legionclaw {
namespace reference_line {

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

ReferencePoint::ReferencePoint(const legionclaw::interface::PathPoint& map_point,
                               const double longitudinal_bound,
                               const double lateral_bound, const bool enforced)
    : legionclaw::interface::PathPoint(map_point),
      longitudinal_bound_(longitudinal_bound),
      lateral_bound_(lateral_bound),
      enforced_(enforced) {}

ReferencePoint::ReferencePoint(const legionclaw::interface::LanePoint& map_point, const double longitudinal_bound,
                  const double lateral_bound, const bool enforced):
                  longitudinal_bound_(longitudinal_bound), 
                  lateral_bound_(lateral_bound), 
                  enforced_(enforced),
                  limit_speed_(map_point.limit_speed()),
                  left_road_width_(map_point.left_road_width()),
                  right_road_width_(map_point.right_road_width()),
                  left_line_type_(map_point.left_line_type()),
                  right_line_type_(map_point.right_line_type())
    {
        x_=map_point.point().x();
        y_=map_point.point().y();
        z_=map_point.point().z();
        theta_=map_point.theta();
        s_=map_point.mileage();

    }

ReferencePoint::ReferencePoint(const legionclaw::planning::LanePointInner& map_point, const double longitudinal_bound,
                  const double lateral_bound, const bool enforced):
                  longitudinal_bound_(longitudinal_bound), 
                  lateral_bound_(lateral_bound), 
                  enforced_(enforced),
                  limit_speed_(map_point.limit_speed()),
                  left_road_width_(map_point.left_road_width()),
                  right_road_width_(map_point.right_road_width()),
                  left_line_type_(map_point.left_line_type()),
                  right_line_type_(map_point.right_line_type())
    {
        x_=map_point.point().x();
        y_=map_point.point().y();
        z_=map_point.point().z();
        theta_=map_point.theta();
        s_=map_point.mileage();
        kappa_ = map_point.kappa();

    }
}  // namespace reference_line
}  // namespace legionclaw
