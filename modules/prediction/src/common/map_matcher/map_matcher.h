
/// \file map_matcher.h
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016

#pragma once

#include <float.h>
#include <omp.h>

#include <string>

#include "modules/prediction/src/interface/lane_point_inner.hpp"
#include "modules/common/interface/lane_point.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/trajectory_point.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/curve1d/curve1d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/math/vec3d.h"

using namespace legionclaw::common;
using namespace legionclaw::interface;
// using namespace legionclaw::prediction;

namespace legionclaw {
namespace prediction {

#define distance2points(from, to) \
  sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from, to) \
  pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x *v.x + v.y * v.y)
#define angle2points(from, to) atan2(to.y - from.y, to.x - from.x)
#define LANE_CHANGE_SPEED_FACTOR 0.5
#define LANE_CHANGE_COST 3.0              // meters
#define BACKUP_STRAIGHT_PLAN_DISTANCE 75  // meters
#define LANE_CHANGE_MIN_DISTANCE 5


class MapMatcher {
 public:
  MapMatcher();
  virtual ~MapMatcher();

  std::vector<PathPoint> ToDiscretizedTrajectory(
      const std::vector<TrajectoryPoint> &trajectory);

  static double GetExactDistanceOnTrajectory(
      const std::vector<TrajectoryPoint> &trajectory, const int &i1,
      const int &i2);

  static int QueryVerticalDistanceWithBuffer(
      const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
      const double buffer, double &dist_min);

  static int QueryVerticalDistanceWithBuffer(
      const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
      const double yaw, const double buffer, double &dist_min);
  //先粗搜，再精搜
  static int QueryVerticalDistanceWithBufferII(
      const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
      const double buffer, double &dist_min);

  static int QueryVerticalDistanceWithBufferII(
      const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
      const double yaw, const double buffer, double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::interface::PathPoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
      const math::Vec2d &position, const double buffer, double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::interface::LanePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryVerticalDistanceWithBuffer(
    const std::vector<legionclaw::prediction::LanePointInner> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::prediction::LanePointInner> &trajectory,
      const math::Vec3d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::prediction::LanePointInner> &trajectory,
      const math::Vec2d &position, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
    const math::Vec3d &position, const double yaw, const double buffer,
    double &dist_min);

    static LanePointInner QueryNearestPoint(const std::vector<LanePointInner> &lane_points, 
        const double s);
};

}  // namespace prediction
}  // namespace legionclaw
