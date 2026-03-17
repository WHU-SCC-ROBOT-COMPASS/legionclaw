
/**
 * @file              map_matcher.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-09 02:26:18
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#include "modules/planning/src/common/map_matcher/map_matcher.h"

#include <float.h>

#include <string>

#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/path_matcher.h"
// #include "modules/planning/src/common/trajectory/trajectory1d_generator.h"

using namespace std;

namespace legionclaw {
namespace planning {
using namespace legionclaw::common::math;
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;

MapMatcher::MapMatcher() {}

MapMatcher::~MapMatcher() {}

std::vector<PathPoint> MapMatcher::ToDiscretizedTrajectory(
    const std::vector<TrajectoryPoint> &trajectory) {
  std::vector<legionclaw::interface::PathPoint> path_points;
  for (const auto point : trajectory) {
    path_points.push_back(point.path_point());
  }
  return path_points;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<TrajectoryPoint> &trajectory, const math::Vec2d &position,
    const double buffer, double &dist_min) {
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].path_point().x(), trajectory[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].path_point().x(), trajectory[index_min].path_point().y()};
    end_point_2 = {trajectory[index_min + 1].path_point().x(),
                   trajectory[index_min + 1].path_point().y()};
  } else {
    start_point = {trajectory[index_min - 1].path_point().x(),
                   trajectory[index_min - 1].path_point().y()};
    end_point_2 = {trajectory[index_min].path_point().x(), trajectory[index_min].path_point().y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
    const double buffer, double &dist_min) {
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min) {
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

//先粗搜，再精搜
int MapMatcher::QueryVerticalDistanceWithBufferII(
    const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
    const double buffer, double &dist_min) {
  // ros::Time time1 = ros::Time::now();

  unsigned int len_step = 50;
  unsigned int index_min = 0;
  dist_min = std::numeric_limits<double>::max();

  // ros::Time time2 = ros::Time::now();
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  // ros::Time time3 = ros::Time::now();
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (unsigned int i = start_index;
       i < index_min + len_step && i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  math::Vec2d start_point, end_point_2;
  if (size_t(index_min) < trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }

  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sqrt(dist_min);
  if (index_min == 0) {
    // double vectorialDiff =
    // math::VectorialAngle(start_point,position,end_point_2);
    double lat_safe_dis = fabs(sin(vectorialDiff - 0.5 * M_PI) *
                               dist_min);  // check other conditions
    if (vectorialDiff > 0.5 * M_PI &&
        lat_safe_dis > 1.0)  //点在线（地一个点）外，认为没匹配上
    {
      index_min = -1;
      return index_min;
    }
  }
  if ((unsigned int)index_min == trajectory.size() - 1) {
    vectorialDiff = math::VectorialAngle(end_point_2, position, start_point);
    double lat_safe_dis = fabs(sin(vectorialDiff - 0.5 * M_PI) *
                               dist_min);  // check other conditions
    if (vectorialDiff > 0.5 * M_PI &&
        lat_safe_dis > 1.0)  //点在线（最后一个点）外，认为没匹配上
    {
      index_min = -1;
      return index_min;
    }
  }
  // double vectorialDiff =
  // math::VectorialAngle(start_point,position,end_point_2); double lon_safe_dis
  // = cos(vectorialDiff - 0.5*M_PI)*dist_min;		// check other
  // conditions if ((index_min == 0 && vectorialDiff > 0.5*M_PI && lon_safe_dis
  // > 3.5*3.5) || index_min == trajectory.size()-1)
  // {
  // 	index_min = -1;
  // 	return index_min;
  // }

  // dist_min = sqrt(dist_min);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  // ros::Time time5 = ros::Time::now();
  // std::cout << "time diff1 = " << time2-time1 << "\n";
  // std::cout << "time diff2 = " << time3-time2 << "\n";
  // std::cout << "time diff3 = " << time4-time3 << "\n";
  // std::cout << "time diff4 = " << time5-time4 << "\n" << "\n";
  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBufferII(
    const std::vector<PathPoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min) {
  // ros::Time time1 = ros::Time::now();

  unsigned int len_step = 50;
  unsigned int index_min = 0;
  dist_min = std::numeric_limits<double>::max();

  // ros::Time time2 = ros::Time::now();
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  // ros::Time time3 = ros::Time::now();
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (unsigned int i = start_index;
       i < index_min + len_step && i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  // ros::Time time4 = ros::Time::now();

  math::Vec2d start_point, end_point_2;
  if (index_min < (size_t)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }

  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sqrt(dist_min);
  if (index_min == 0) {
    // double vectorialDiff =
    // math::VectorialAngle(start_point,position,end_point_2);
    double lat_safe_dis = fabs(sin(vectorialDiff - 0.5 * M_PI) *
                               dist_min);  // check other conditions
    if (vectorialDiff > 0.5 * M_PI &&
        lat_safe_dis > 1.0)  //点在线（地一个点）外，认为没匹配上
    {
      index_min = -1;
      return index_min;
    }
  }
  if ((unsigned int)index_min == trajectory.size() - 1) {
    vectorialDiff = math::VectorialAngle(end_point_2, position, start_point);
    double lat_safe_dis = fabs(sin(vectorialDiff - 0.5 * M_PI) *
                               dist_min);  // check other conditions
    if (vectorialDiff > 0.5 * M_PI &&
        lat_safe_dis > 1.0)  //点在线（最后一个点）外，认为没匹配上
    {
      index_min = -1;
      return index_min;
    }
  }
  // double vectorialDiff =
  // math::VectorialAngle(start_point,position,end_point_2); double lon_safe_dis
  // = cos(vectorialDiff - 0.5*M_PI)*dist_min;		// check other
  // conditions if ((index_min == 0 && vectorialDiff > 0.5*M_PI && lon_safe_dis
  // > 3.5*3.5) || index_min == trajectory.size()-1)
  // {
  // 	index_min = -1;
  // 	return index_min;
  // }

  // dist_min = sqrt(dist_min);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  // ros::Time time5 = ros::Time::now();
  // std::cout << "time diff1 = " << time2-time1 << "\n";
  // std::cout << "time diff2 = " << time3-time2 << "\n";
  // std::cout << "time diff3 = " << time4-time3 << "\n";
  // std::cout << "time diff4 = " << time5-time4 << "\n" << "\n";
  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min) {
  dist_min = std::numeric_limits<double>::max();
  if(trajectory.size() == 0){
    return -1;
  }
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, position, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::PathPoint> &trajectory,
    const math::Vec2d &position, const double yaw, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  if (index_min >= 0) {
    dist_min = sqrt(dist_min);
    math::Vec2d start_point, end_point_2;
    if (index_min < (int)trajectory.size() - 1) {
      start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
      end_point_2 = {trajectory[index_min + 1].x(),
                     trajectory[index_min + 1].y()};
    } else {
      start_point = {trajectory[index_min - 1].x(),
                     trajectory[index_min - 1].y()};
      end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
    }
    double sign = math::CrossProd(start_point, position, end_point_2);
    if (sign > 0)  //点在向量右侧
      dist_min *= -1;
    else if (sign == 0)  //点在向量上
      dist_min = 0;
  }

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
    const math::Vec2d &position, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  // for (unsigned int i = 0; i < trajectory.size(); ++i) {
  //   const math::Vec2d curr_point(trajectory[i].path_point().x(),
  //                                trajectory[i].path_point().y());

  //   const double dist_sqr = curr_point.DistanceSquareTo(position);

  //   if (dist_sqr < dist_min + buffer) {
  //     dist_min = dist_sqr;
  //     index_min = i;
  //   }
  // }

  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y());
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min >= 0) {
    dist_min = sqrt(dist_min);
    math::Vec2d start_point, end_point_2;
    if (index_min < (int)trajectory.size() - 1) {
      start_point = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
      end_point_2 = {trajectory[index_min + 1].path_point().x(),
                     trajectory[index_min + 1].path_point().y()};
    } else {
      start_point = {trajectory[index_min - 1].path_point().x(),
                     trajectory[index_min - 1].path_point().y()};
      end_point_2 = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
    }
    double sign = math::CrossProd(start_point, position, end_point_2);
    if (sign > 0)  //点在向量右侧
      dist_min *= -1;
    else if (sign == 0)  //点在向量上
      dist_min = 0;
  }

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
    const math::Vec2d &position, const double yaw, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;

  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].path_point().theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].path_point().theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min >= 0) {
    dist_min = sqrt(dist_min);
    math::Vec2d start_point, end_point_2;
    if (index_min < (int)trajectory.size() - 1) {
      start_point = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
      end_point_2 = {trajectory[index_min + 1].path_point().x(),
                     trajectory[index_min + 1].path_point().y()};
    } else {
      start_point = {trajectory[index_min - 1].path_point().x(),
                     trajectory[index_min - 1].path_point().y()};
      end_point_2 = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
    }
    double sign = math::CrossProd(start_point, position, end_point_2);
    if (sign > 0)  //点在向量右侧
      dist_min *= -1;
    else if (sign == 0)  //点在向量上
      dist_min = 0;
  }

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  if (dist_min > 64) {
    return -1;
  }
  // 精搜
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(),
                                 trajectory[i].y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min < 0) return index_min;
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min,const legionclaw::common::Direction &direction) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec2d curr_point(trajectory[i].x(), trajectory[i].y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.75 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].x(),
                                 trajectory[i].y());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min < 0) return index_min;
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;
  if(direction==legionclaw::common::Direction::LEFT && dist_min > 8){
    return -1;
  }else if(direction==legionclaw::common::Direction::RIGHT&& dist_min < -8){
    return -1;
  }
  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec2d &position,
    const double yaw, const double buffer, double &dist_min,
    const int &index_start) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;

  // 粗搜
  int len_step = 50;
  double dist_sqr = 0.0;
  // #pragma omp parallel for
  for (size_t j = index_start; j < (size_t)trajectory.size(); j += len_step) {
    // const math::Vec2d curr_point(trajectory[index_start].x(),
    //                              trajectory[index_start].y());
    // const double dist_sqr = curr_point.DistanceSquareTo(position);
    dist_sqr =
        (trajectory[j].x() - position.x()) *
            (trajectory[j].x() - position.x()) +
        (trajectory[j].y() - position.y()) * (trajectory[j].y() - position.y());
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = j;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // #pragma omp parallel for
  // can not use with an && controlling condition in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    // const math::Vec2d curr_point(trajectory[i].x(),  trajectory[i].y());
    // const double dist_sqr = curr_point.DistanceSquareTo(position);
    const double dist_sqr =
        (trajectory[i].x() - position.x()) *
            (trajectory[i].x() - position.x()) +
        (trajectory[i].y() - position.y()) * (trajectory[i].y() - position.y());
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min < 0) return index_min;
  dist_min = sqrt(dist_min);

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::LanePoint> &trajectory,
    const math::Vec2d &position, const double yaw, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].point().x(),
                                 trajectory[i].point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    // double yaw_diff = fabs(math::AngleDiff(yaw, trajectory[i].theta()));
    // 等待角度更新
    // if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI)
    if (dist_sqr < dist_min + buffer) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
    end_point_2 = {trajectory[index_min + 1].point().x(),
                   trajectory[index_min + 1].point().y()};
  } else {
    start_point = {trajectory[index_min - 1].point().x(),
                   trajectory[index_min - 1].point().y()};
    end_point_2 = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
  }
  double sign = math::CrossProd(start_point, position, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].x(), trajectory[i].y(), trajectory[i].z());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, {position.x(), position.y()}, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<legionclaw::planning::LanePointInner> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;  
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].point().x(),
                                 trajectory[i].point().y(),
                                 trajectory[i].point().z());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  dist_min = sqrt(dist_min);
  if (dist_min > 5) {
    return -1;
  }
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
    end_point_2 = {trajectory[index_min + 1].point().x(),
                   trajectory[index_min + 1].point().y()};
  } else {
    start_point = {trajectory[index_min - 1].point().x(),
                   trajectory[index_min - 1].point().y()};
    end_point_2 = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, {position.x(), position.y()}, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryVerticalDistanceWithBuffer(
    const std::vector<legionclaw::interface::LanePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;  
  dist_min = std::numeric_limits<double>::max();
  int index_min = 0;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].point().x(),
                                 trajectory[i].point().y(),
                                 trajectory[i].point().z());

    const double dist = curr_point.DistanceTo(position);
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    if (dist < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist;
      index_min = i;
    }
  }

  if (dist_min > 5) {
    return -1;
  }
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
    end_point_2 = {trajectory[index_min + 1].point().x(),
                   trajectory[index_min + 1].point().y()};
  } else {
    start_point = {trajectory[index_min - 1].point().x(),
                   trajectory[index_min - 1].point().y()};
    end_point_2 = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
  }
  double vectorialDiff =
      math::VectorialAngle(start_point, {position.x(), position.y()}, end_point_2);
  dist_min = sin(vectorialDiff) * dist_min;
  double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::planning::LanePointInner> &trajectory,
    const math::Vec3d &position, const double yaw, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].point().x(),
                                 trajectory[i].point().y(),
                                 trajectory[i].point().z());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    double yaw_diff = fabs(math::AngleDiff(yaw, trajectory[i].theta()));
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
    end_point_2 = {trajectory[index_min + 1].point().x(),
                   trajectory[index_min + 1].point().y()};
  } else {
    start_point = {trajectory[index_min - 1].point().x(),
                   trajectory[index_min - 1].point().y()};
    end_point_2 = {trajectory[index_min].point().x(),
                   trajectory[index_min].point().y()};
  }
  double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<ReferencePoint> &trajectory, const math::Vec3d &position,
    const double yaw, const double buffer, double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec3d curr_point(trajectory[i].x(), trajectory[i].y(), trajectory[i].z());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].x(), trajectory[i].y(),
                                 trajectory[i].z());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min < 0) return index_min;
  dist_min = sqrt(dist_min);
  math::Vec2d start_point, end_point_2;
  if (index_min < (int)trajectory.size() - 1) {
    start_point = {trajectory[index_min].x(), trajectory[index_min].y()};
    end_point_2 = {trajectory[index_min + 1].x(),
                   trajectory[index_min + 1].y()};
  } else {
    start_point = {trajectory[index_min - 1].x(),
                   trajectory[index_min - 1].y()};
    end_point_2 = {trajectory[index_min].x(), trajectory[index_min].y()};
  }
  double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
  if (sign > 0)  //点在向量右侧
    dist_min *= -1;
  else if (sign == 0)  //点在向量上
    dist_min = 0;

  return index_min;
}

int MapMatcher::QueryNearestPointWithBuffer(
    const std::vector<legionclaw::interface::TrajectoryPoint> &trajectory,
    const math::Vec3d &position, const double yaw, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;

  // 粗搜
  int len_step = 50;
  // #pragma omp parallel for
  for (unsigned int i = 0; i < trajectory.size(); i += len_step) {
    const math::Vec3d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y(),
                                 trajectory[i].path_point().z());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].path_point().theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }
  if (index_min < 0) {
    return index_min;
  }
  // 精搜
  // #pragma omp parallel for  // can not use with an && controlling condition
  // in for loop
  unsigned int start_index = index_min < len_step ? 0 : index_min - len_step;
  for (int i = start_index;
       i < index_min + len_step && (size_t)i < trajectory.size(); ++i) {
    const math::Vec3d curr_point(trajectory[i].path_point().x(),
                                 trajectory[i].path_point().y(),
                                 trajectory[i].path_point().z());
    double yaw_diff = fabs(AngleDiff(yaw, trajectory[i].path_point().theta()));
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.5 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
  }

  if (index_min >= 0) {
    dist_min = sqrt(dist_min);
    math::Vec2d start_point, end_point_2;
    if (index_min < (int)trajectory.size() - 1) {
      start_point = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
      end_point_2 = {trajectory[index_min + 1].path_point().x(),
                     trajectory[index_min + 1].path_point().y()};
    } else {
      start_point = {trajectory[index_min - 1].path_point().x(),
                     trajectory[index_min - 1].path_point().y()};
      end_point_2 = {trajectory[index_min].path_point().x(),
                     trajectory[index_min].path_point().y()};
    }
    double sign = math::CrossProd(start_point, {position.x(), position.y()}, end_point_2);
    if (sign > 0)  //点在向量右侧
      dist_min *= -1;
    else if (sign == 0)  //点在向量上
      dist_min = 0;
  }

  return index_min;
}

double MapMatcher::GetExactDistanceOnTrajectory(
    const std::vector<TrajectoryPoint> &trajectory, const int &i1,
    const int &i2) {
  if (trajectory.size() == 0) return -1;
  if (i1 > i2 || i1 < 0 || (unsigned int)i1 >= trajectory.size() || i2 < 0 ||
      (unsigned int)i2 >= trajectory.size())
    return -1;

  return (trajectory.at(i2).path_point().s() -
          trajectory.at(i1).path_point().s());
}

double MapMatcher::GetVelocityAhead(const std::vector<ReferencePoint> &path,
                                    const int &start_index,
                                    const double &ahead_distance) {
  if (path.size() == 0 || start_index < 0) return 0;

  //速度初始值给的是参考中心线上当前匹配点的速度
  double min_v = path.at(start_index).limit_speed();
  double d = 0.0;

  //从参考中心线上当前匹配点开始，沿参考线向前搜索，找出一定长度内参考中心线上的最小速度
  unsigned int local_i = start_index;
  while (local_i < path.size() - 1 && d < ahead_distance) {
    local_i++;
    d += hypot(path.at(local_i).y() - path.at(local_i - 1).y(),
               path.at(local_i).x() - path.at(local_i - 1).x());
    if (path.at(local_i).limit_speed() < min_v)
      min_v = path.at(local_i).limit_speed();
  }

  return min_v;
}

double MapMatcher::GetExactDistanceOnTrajectory(
    const std::vector<ReferencePoint> &trajectory, const int &i1,
    const int &i2) {
  if (trajectory.size() == 0) return -1;
  if (i1 > i2 || i1 < 0 || (unsigned int)i1 >= trajectory.size() || i2 < 0 ||
      (unsigned int)i2 >= trajectory.size())
    return -1;

  return (trajectory.at(i2).s() - trajectory.at(i1).s());
}

void MapMatcher::PredictConstantTimeCostForTrajectory(
    const std::vector<PathPoint> &path, const double &speed,
    const double &min_velocity, const double &max_velocity,
    PlanningTrajectory &trajectory) {
  if (path.size() == 0 || min_velocity < 0.0) return;

  double total_distance = 0;
  double accum_time = 0;
  double current_speed = (speed < min_velocity) ? min_velocity : speed;
  current_speed = (current_speed > max_velocity) ? max_velocity : current_speed;

  std::vector<TrajectoryPoint> trajectory_points;
  trajectory_points.resize(path.size());

  // trajectory.set_trajectory_points(&trajectory_points);
  trajectory_points.at(0).set_path_point(path.at(0));
  trajectory_points.at(0).mutable_path_point()->set_s(0.0);
  trajectory_points.at(0).set_relative_time(0.0);
  trajectory_points.at(0).set_v(current_speed);
  trajectory_points.at(0).set_a(0.0);

  for (unsigned int i = 1; i < path.size(); i++) {
    trajectory_points.at(i).set_path_point(path.at(i));
    total_distance += hypot(path.at(i).x() - path.at(i - 1).x(),
                            path.at(i).y() - path.at(i - 1).y());
    trajectory_points.at(i).mutable_path_point()->set_s(total_distance);
    accum_time = total_distance / current_speed;
    trajectory_points.at(i).set_relative_time(accum_time);
    trajectory_points.at(i).set_v(current_speed);
    trajectory_points.at(i).set_a(0.0);
  }
  trajectory.set_trajectory_points(&trajectory_points);
}

double MapMatcher::GetDistanceToClosestStopLineAndCheck(
    const std::vector<PlanningTrajectory> &path, const interface::PathPoint &p,
    const double &giveUpDistance, int &stopLineID, int &stopSignID,
    int &trafficLightID, const int &prevIndex) {
  //       trafficLightID = stopSignID = stopLineID = -1;

  // for (unsigned int i = 0; i < path.size(); i++)
  // {
  // 	if (path.at(i) > 0 && path.at(i).pLane)
  // 	{

  // 		for (unsigned int j = 0; j < path.at(i).pLane->stopLines.size();
  // j++)
  // 		{

  // 			if (path.at(i).pLane->stopLines.at(j).id ==
  // path.at(i).stopLineID)
  // 			{
  // 				stopLineID = path.at(i).stopLineID;

  // 				RelativeInfo stop_info;
  // 				WayPoint stopLineWP;
  // 				stopLineWP.pos =
  // path.at(i).pLane->stopLines.at(j).points.at(0);
  // GetRelativeInfo(path,
  // stopLineWP, stop_info); 				double localDistance =
  // GetExactDistanceOnTrajectory(path, info, stop_info);

  // 				if (localDistance > giveUpDistance)
  // 				{
  // 					stopSignID =
  // path.at(i).pLane->stopLines.at(j).stopSignID;
  // trafficLightID
  // =
  // path.at(i).pLane->stopLines.at(j).trafficLightID;
  // return localDistance;
  // 				}
  // 			}
  // 		}
  // 	}
  // }
  return -1;
}

ReferencePoint MapMatcher::MatchToPath(const std::vector<ReferencePoint>& reference_line,
                                   const Vec3d& position, const double yaw,
                                   int& index_min) {
  CHECK_GT((int)reference_line.size(), 0);

  auto func_distance_square = [](const ReferencePoint& point,
                                 const Vec3d& position) {
    double dx = point.x() - position.x();
    double dy = point.y() - position.y();
    double dz = point.z() - position.z();
    return dx * dx + dy * dy + dz * dz;
  };

  double distance_min = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < reference_line.size(); ++i) {
    double distance_temp = func_distance_square(reference_line[i], position);
    double yaw_diff = fabs(AngleDiff(yaw, reference_line[i].theta()));
    if (distance_temp < distance_min && yaw_diff < 0.5 * M_PI) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end = ((std::size_t)index_min + 1 == reference_line.size())
                              ? index_min
                              : index_min + 1;

  if (index_start == index_end) {
    return reference_line[index_start];
  }

  return FindProjectionPoint(reference_line[index_start],
                             reference_line[index_end], position.x(),
                             position.y());
}

ReferencePoint MapMatcher::MatchToPath(
    const std::vector<ReferencePoint> &reference_line, const double s) {
  auto comp = [](const ReferencePoint &point, const double s) {
    return point.s() < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
  if (it_lower == reference_line.begin()) {
    if (reference_line.size() < 2) return reference_line.front();
    return InterpolateUsingLinearApproximation(
        *reference_line.begin(), *(reference_line.begin() + 1), s);
  } else if (it_lower == reference_line.end()) {
    if (reference_line.size() < 2) return reference_line.back();
    return InterpolateUsingLinearApproximation(*(reference_line.end() - 1),
                                               *reference_line.end(), s);
  }

  // interpolate between it_lower - 1 and it_lower
  // return interpolate(*(it_lower - 1), *it_lower, s);
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

ReferencePoint MapMatcher::QueryNearestPoint(const std::vector<ReferencePoint> &reference_line,
        const double s){
  auto comp = [](const ReferencePoint& point, const double s) {
    return point.s() < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
  if (it_lower == reference_line.begin()) {
    return reference_line.front();
  } else if (it_lower == reference_line.end()) {
    return reference_line.back();
  }

  return *it_lower;
}

LanePointInner MapMatcher::QueryNearestPoint(const std::vector<LanePointInner> &lane_points, 
    const double s) {
  auto comp = [](const LanePointInner& point, const double s) {
    return point.mileage() < s;
  };

  auto it_lower =
      std::lower_bound(lane_points.begin(), lane_points.end(), s, comp);
  if (it_lower == lane_points.begin()) {
    return lane_points.front();
  } else if (it_lower == lane_points.end()) {
    return lane_points.back();
  }

  return *it_lower;
}

ReferencePoint MapMatcher::FindProjectionPoint(const ReferencePoint& p0,
                                           const ReferencePoint& p1, const double x,
                                           const double y) {
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  double delta_s = dot / v1_norm;
  return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);
}

}  // namespace planning
}  // namespace legionclaw
