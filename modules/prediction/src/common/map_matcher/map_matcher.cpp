
/**
 * @file              map_matcher.cpp
 * @author       jiangchengjie (jiangchengjie@indrv.cn)
 * @brief
 * @version     1.0.0
 * @date           2021-08-09 02:26:18
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#include "modules/prediction/src/common/map_matcher/map_matcher.h"

#include <float.h>

#include <string>

#include "modules/common/math/math_utils.h"
#include "modules/common/math/path_matcher.h"

using namespace std;

namespace legionclaw {
namespace prediction {
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
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
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
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
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
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
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
    // if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI)
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
    const std::vector<legionclaw::prediction::LanePointInner> &trajectory, const math::Vec3d &position,
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
    if (dist_sqr < dist_min + buffer && yaw_diff < 0.25 * M_PI) {
      dist_min = dist_sqr;
      index_min = i;
    }
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
    const std::vector<legionclaw::prediction::LanePointInner> &trajectory,
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
    const std::vector<legionclaw::prediction::LanePointInner> &trajectory,
    const math::Vec2d &position, const double buffer,
    double &dist_min) {
  if (trajectory.size() < 2) return -1;
  dist_min = DBL_MAX;
  int index_min = -1;
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    const math::Vec2d curr_point(trajectory[i].point().x(),
                                 trajectory[i].point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
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

}  // namespace prediction
}  // namespace legionclaw
