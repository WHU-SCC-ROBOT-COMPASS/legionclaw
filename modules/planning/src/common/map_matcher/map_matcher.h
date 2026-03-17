
/// \file map_matcher.h
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016

#pragma once

#include <float.h>
#include <omp.h>

#include <string>

#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/trajectory_point.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/curve1d/curve1d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/src/common/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/src/common/reference_line/reference_line.h"
#include "modules/planning/src/interface/planning_trajectory.hpp"
#include "modules/common/math/vec3d.h"
#include "modules/planning/src/common/map_matcher/linear_interpolation.h"

// #include "trajectory/trajectory1d_generator.h"
// #include "op_planner/map_matcher.h"
// #include "op_planner/MatrixOperations.h"

using namespace legionclaw::reference_line;
using namespace legionclaw::common::math;
using namespace legionclaw::interface;
using namespace legionclaw::planning;

namespace legionclaw {
namespace planning {

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

// typedef std::vector<std::shared_ptr<math::Curve1d>> Trajectory1DBundle;

class MapMatcher {
 public:
  // static std::vector<std::pair<GPSPoint, GPSPoint> > m_TestingClosestPoint;

 public:
  MapMatcher();
  virtual ~MapMatcher();

  std::vector<PathPoint> ToDiscretizedTrajectory(
      const std::vector<TrajectoryPoint> &trajectory);

  static double GetExactDistanceOnTrajectory(
      const std::vector<TrajectoryPoint> &trajectory, const int &i1,
      const int &i2);

  static double GetExactDistanceOnTrajectory(
      const std::vector<ReferencePoint> &trajectory, const int &i1,
      const int &i2);

  static double GetVelocityAhead(const std::vector<ReferencePoint> &path,
                                 const int &start_index,
                                 const double &ahead_distance);

  static void PredictConstantTimeCostForTrajectory(
      const std::vector<PathPoint> &path, const double &speed,
      const double &min_velocity, const double &max_velocity,
      PlanningTrajectory &trajectory);

  static int QueryVerticalDistanceWithBuffer(
      const std::vector<TrajectoryPoint> &trajectory, const math::Vec2d &position,
      const double buffer, double &dist_min);
  
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

  static int QueryVerticalDistanceWithBuffer(
      const std::vector<ReferencePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

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
      const std::vector<ReferencePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<ReferencePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min, const legionclaw::common::Direction &direction);

  /**
   * @brief        搜索点列A上各点在另一条点列B上的匹配点函数：
                            要求点列A的排布方向和点列B的搜索方向一致，
                            函数使用在循环内
   * @param
   * @param     &index_start 点列B上的搜索起点
   */
  static int QueryNearestPointWithBuffer(
      const std::vector<ReferencePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min, const int &index_start);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::interface::LanePoint> &trajectory,
      const math::Vec2d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryVerticalDistanceWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min);

  static int QueryVerticalDistanceWithBuffer(
    const std::vector<legionclaw::planning::LanePointInner> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min);

  static int QueryVerticalDistanceWithBuffer(
    const std::vector<legionclaw::interface::LanePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min);

  static int QueryNearestPointWithBuffer(
      const std::vector<legionclaw::planning::LanePointInner> &trajectory,
      const math::Vec3d &position, const double yaw, const double buffer,
      double &dist_min);

  static int QueryNearestPointWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min);

  static int QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
    const math::Vec3d &position, const double yaw, const double buffer,
    double &dist_min);

  static double GetDistanceToClosestStopLineAndCheck(
      const std::vector<PlanningTrajectory> &path,
      const interface::PathPoint &p, const double &giveUpDistance,
      int &stopLineID, int &stopSignID, int &trafficLightID,
      const int &prevIndex = 0);

  static ReferencePoint QueryNearestPoint(
      const std::vector<ReferencePoint> &reference_line, const double s);

  static LanePointInner QueryNearestPoint(
      const std::vector<LanePointInner> &lane_points, const double s);

  static ReferencePoint MatchToPath(const std::vector<ReferencePoint> &reference_line,
                             const double s);

  static ReferencePoint MatchToPath(
      const std::vector<ReferencePoint> &reference_line, const Vec3d &position,
      const double yaw, int &index_min);

  static ReferencePoint FindProjectionPoint(const ReferencePoint &p0,
                                            const ReferencePoint &p1,
                                            const double x, const double y);
};

}  // namespace planning
}  // namespace legionclaw
