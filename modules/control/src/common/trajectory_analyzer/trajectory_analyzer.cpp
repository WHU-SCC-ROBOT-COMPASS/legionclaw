/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "trajectory_analyzer.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <utility>

#include "common/logging/logging.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/search.h"
#include "common/time/time_tool.h"

using namespace legionclaw::common;
using namespace legionclaw::common::math;
namespace legionclaw {
namespace control {
namespace {

// Squared distance from the point to (x, y).
// 点到(x, y)的距离的平方。
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  return point.path_point();
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const legionclaw::interface::ADCTrajectory *planning_published_trajectory) {
  header_time_ = (double)TimeTool::TimeStruct2S(
      planning_published_trajectory->header().stamp());

  seq_num_ = planning_published_trajectory->header().seq();

  // int64_t current_time = legionclaw::common::TimeTool::Now2Us();
  // if(planning_published_trajectory->trajectory_point().size() > 0)
  // {
  //   std::cout << "trajectory_point().size():::" <<
  //   planning_published_trajectory->trajectory_point().size() << endl;

  //   trajectory_points_.insert(trajectory_points_.begin(),planning_published_trajectory->trajectory_point().begin(),planning_published_trajectory->trajectory_point().end());
  // }
  planning_published_trajectory->trajectory_points(trajectory_points_);
  // for (unsigned int i = 0; i <
  // planning_published_trajectory->trajectory_point().size();
  //      ++i) {
  //   trajectory_points_.push_back(
  //       planning_published_trajectory->trajectory_point()[i]);
  // }
  // int64_t time_diff_6 = legionclaw::common::TimeTool::Now2Us() - current_time;
  // std::cout << "time_diff66666:::" << time_diff_6 << endl;
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                    const double y) const {
  if (trajectory_points_.size() == 0) {
    AERROR << "trajectory size error.";
    return PathPoint();
  }

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  if (x == 0 && y == 0 && d_min > 100) {
    AERROR << "Lon not received localization message error.";
    return PathPoint();
  }

  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  const double kEpsilon = 0.001;
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point().s() -
                trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
    return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
  }

  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.

/**
 * 计算纵向误差相关参数
 * 传入参数：当前点x,y，角度thena,速度v，参考点ref_point
 * 返回参数：
 **/
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  // 笛卡尔坐标系下（这里指地图坐标系），横纵向误差
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();

  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  // 与apollo不同
  // double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  double cross_rd_nd = cos_ref_theta * dx - sin_ref_theta * dy;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  // 正北方向 顺时针
  // double dot_rd_nd = dy * cos_ref_theta + dx * sin_ref_theta;
  // 正东方向 逆时针
  double dot_rd_nd = dy * sin_ref_theta + dx * cos_ref_theta;
  *ptr_s = ref_point.s() + dot_rd_nd;
  // std::cout<< "ref_point.theta():: "<< ref_point.theta() << "dx::"<<dx<<endl;
  // std::cout<< "dy:: "<< dy << "dx::"<<dx<<endl;
  // std::cout<< "lon_ref_point.s():: "<< ref_point.s() <<
  // "lon_dot_rd_nd::"<<dot_rd_nd<<endl;

  double delta_theta = theta - ref_point.theta();
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    AERROR << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa()
           << " ref_point.x:" << ref_point.x()
           << " ref_point.y:" << ref_point.y() << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t, bool query_forward_time_point_only) const {
  // std::cout<<"sequence_num:"<<seq_num_<<"\n";
  // std::cout << "diff:" << t - header_time_ << "\n";
  // std::cout << "t:" << t << std::fixed << "\n";
  // std::cout << "header_time_:" << header_time_ << std::fixed << "\n";
  return QueryNearestPointByRelativeTime(t - header_time_,
                                         query_forward_time_point_only);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t, bool query_forward_time_point_only) const {
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    // std::cout<<"point.relative_time() : "<<point.relative_time() <<"\n";
    return point.relative_time() < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);
  // std::cout<<"trajectory_points_.size() : "<<trajectory_points_.size()
  // <<"\n";
  //  std::cout<<"it_low.relative_time() : "<<it_low->relative_time()
  //  <<"\n";
  if (it_low == trajectory_points_.begin()) {
    // std::cout<<"it_low.relative_time() : "<<it_low->relative_time()
    // <<"\n"; std::cout<<"trajectory_points_.begin() :
    // "<<trajectory_points_.begin() <<"\n";
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  if (query_forward_time_point_only) {
    return *it_low;
  } else {
    auto it_lower = it_low - 1;
    if (it_low->relative_time() - t < t - it_lower->relative_time()) {
      return *it_low;
    }

    return *it_lower;
  }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  if (trajectory_points_.size() == 0) {
    AERROR << "trajectory size error.";
    return TrajectoryPoint();
  }

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  if (x == 0 && y == 0 && d_min > 100) {
    AERROR << "Not received localization message error.";
    return TrajectoryPoint();
  }
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  return trajectory_points_[index_min];
}

size_t TrajectoryAnalyzer::QueryNearestPointIndexByPosition(
    const double x, const double y) const {
  if (trajectory_points_.size() == 0) {
    AERROR << "trajectory size error.";
    return 0;
  }

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  if (x == 0 && y == 0 && d_min > 100) {
    AERROR << "Not received localization message error.";
    return 0;
  }
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  return index_min;
}

/**
 * 查询给定位置和偏航角的轨迹点在轨迹上的投影点。
 * 通过计算给定位置和每个轨迹点之间的距离，并找到距离最小且偏航角差值小于0.5 *
 *M_PI的轨迹点。然后，它使用线性插值方法在两个最近的轨迹点之间找到投影点，并返回包含投影点信息的TrajectoryPoint对象
 * 传入参数：点position，角度yaw
 * 返回参数：包含x,y,theta,v,a状态的点
 **/
TrajectoryPoint TrajectoryAnalyzer::QueryProjectionPoint(
    const common::math::Vec2d &position, const double yaw) const {
  CHECK_GT((int)trajectory_points_.size(), 0);

  // 定义lambda函数func_distance_square,计算给定位置与轨迹点之间的距离平方
  auto func_distance_square = [](const TrajectoryPoint &point,
                                 const common::math::Vec2d &position) {
    double dx = point.path_point().x() - position.x();
    double dy = point.path_point().y() - position.y();
    return dx * dx + dy * dy;
  };

  double distance_min = std::numeric_limits<double>::max();
  int index_min = 0;

  for (std::size_t i = 0; i < trajectory_points_.size(); ++i) {
    double distance_temp =
        func_distance_square(trajectory_points_[i], position);
    double yaw_diff = fabs(common::math::AngleDiff(
        yaw, trajectory_points_[i].path_point().theta()));
    if (distance_temp < distance_min && yaw_diff < 0.5 * M_PI) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      ((std::size_t)index_min + 1 == trajectory_points_.size()) ? index_min
                                                                : index_min + 1;

  if (index_start == index_end) {
    return trajectory_points_[index_start];
  }

  TrajectoryPoint tp = trajectory_points_[index_min];
  PathPoint path_point = FindProjectionPoint(
      trajectory_points_[index_start].path_point(),
      trajectory_points_[index_end].path_point(), position.x(), position.y());

  tp.set_path_point(path_point);

  if (tp.path_point().s() < 0.0) {
    tp.path_point().set_theta(
        trajectory_points_[index_start].path_point().theta());
    tp.path_point().set_kappa(
        trajectory_points_[index_start].path_point().kappa());
    tp.path_point().set_dkappa(
        trajectory_points_[index_start].path_point().dkappa());
    tp.path_point().set_ddkappa(
        trajectory_points_[index_start].path_point().ddkappa());
  }

  return tp;
}

TrajectoryPoint TrajectoryAnalyzer::QueryProjectionPoint(
    const common::math::Vec2d &position, const double yaw, const double t,
    const double v) const {
  CHECK_GT((int)trajectory_points_.size(), 0);

  auto func_distance_square = [](const TrajectoryPoint &point,
                                 const common::math::Vec2d &position) {
    double dx = point.path_point().x() - position.x();
    double dy = point.path_point().y() - position.y();
    return dx * dx + dy * dy;
  };

  double distance_min = std::numeric_limits<double>::max();
  int index_min = 0;

  double time_diff = t - header_time_;
  // TODO:轨迹点密度设置为自适应
  int index_offset = time_diff * v / 0.1;

  for (std::size_t i = 0; i < trajectory_points_.size(); ++i) {
    double distance_temp =
        func_distance_square(trajectory_points_[i], position);
    double yaw_diff = fabs(common::math::AngleDiff(
        yaw, trajectory_points_[i].path_point().theta()));
    if (distance_temp < distance_min && yaw_diff < 0.5 * M_PI) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  index_min += index_offset;
  if (index_min >= trajectory_points_.size()) {
    index_min = trajectory_points_.size() - 1;
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      ((std::size_t)index_min + 1 == trajectory_points_.size()) ? index_min
                                                                : index_min + 1;

  if (index_start == index_end) {
    return trajectory_points_[index_start];
  }

  TrajectoryPoint tp = trajectory_points_[index_min];
  PathPoint path_point = FindProjectionPoint(
      trajectory_points_[index_start].path_point(),
      trajectory_points_[index_end].path_point(), position.x(), position.y());

  tp.set_path_point(path_point);

  return tp;
}

PathPoint TrajectoryAnalyzer::FindProjectionPoint(const PathPoint &p0,
                                                  const PathPoint &p1,
                                                  const double x,
                                                  const double y) const {
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  double delta_s = dot / v1_norm;
  return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);
}

/**
 * 查询给定位置和偏航角的轨迹点在轨迹上的投影点，然后根据里程差查找目标点（支持正向和反向）。
 * 首先使用匿名函数找到当前位置在轨迹上的投影点索引（不进行插值），获取其里程s_current，然后计算目标里程s_target = s_current + s。
 * 当s为正数时向前查找，当s为负数时向后查找。
 * 从投影点索引开始，使用二分法查找包含目标里程的区间，然后进行线性插值。
 * 传入参数：点position，角度yaw，查找的里程差s（正数向前，负数向后）
 * 返回参数：包含x,y,theta,v,a状态的点
 **/
TrajectoryPoint TrajectoryAnalyzer::QueryProjectionPoint(
    const common::math::Vec2d &position, const double yaw, const double s) const {
  CHECK_GT((int)trajectory_points_.size(), 0);

  // 定义匿名函数，仿照QueryProjectionPoint的逻辑，找到投影点的索引（不进行插值）
  auto find_projection_index = [this](const common::math::Vec2d &pos, const double y) -> std::pair<std::size_t, double> {
    // 定义lambda函数计算给定位置与轨迹点之间的距离平方
    auto func_distance_square = [](const TrajectoryPoint &point,
                                   const common::math::Vec2d &position) {
      double dx = point.path_point().x() - position.x();
      double dy = point.path_point().y() - position.y();
      return dx * dx + dy * dy;
    };

    double distance_min = std::numeric_limits<double>::max();
    std::size_t index_min = 0;

    // 遍历所有轨迹点，找到距离最小且偏航角差值小于0.5*M_PI的点
    for (std::size_t i = 0; i < trajectory_points_.size(); ++i) {
      double distance_temp = func_distance_square(trajectory_points_[i], pos);
      double yaw_diff = fabs(common::math::AngleDiff(
          y, trajectory_points_[i].path_point().theta()));
      if (distance_temp < distance_min && yaw_diff < 0.5 * M_PI) {
        distance_min = distance_temp;
        index_min = i;
      }
    }

    // 返回索引和对应的里程值（不进行插值）
    double s_current = trajectory_points_[index_min].path_point().s();
    return std::make_pair(index_min, s_current);
  };

  // 调用匿名函数获取投影点索引和里程
  auto [projection_index, s_current] = find_projection_index(position, yaw);
  double s_target = s_current + s;  // s为正数时向前，为负数时向后

  // 如果目标里程小于轨迹起点里程，返回第一个轨迹点
  if (s_target <= trajectory_points_.front().path_point().s()) {
    return trajectory_points_.front();
  }

  // 如果目标里程大于轨迹终点里程，返回最后一个轨迹点
  if (s_target >= trajectory_points_.back().path_point().s()) {
    return trajectory_points_.back();
  }

  // 从投影点索引开始，使用二分法查找包含目标里程的区间
  std::size_t index_start = 0;
  std::size_t index_end = trajectory_points_.size() - 1;

  // 确定二分查找的起始范围
  // 如果s_target >= s_current，向前查找；否则向后查找
  std::size_t left, right;
  if (s_target >= s_current) {
    // 向前查找：从投影点索引到轨迹末尾
    left = projection_index;
    right = trajectory_points_.size() - 1;
  } else {
    // 向后查找：从轨迹起点到投影点索引
    left = 0;
    right = projection_index;
  }

  // 使用二分法查找包含目标里程的区间
  // 查找满足 trajectory_points_[i].path_point().s() <= s_target 的最大索引
  std::size_t low = left;
  std::size_t high = right;
  std::size_t result_index = left;

  while (low <= high) {
    std::size_t mid = low + (high - low) / 2;
    double s_mid = trajectory_points_[mid].path_point().s();

    if (s_mid <= s_target) {
      result_index = mid;
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  // 确保result_index不会超出范围
  if (result_index >= trajectory_points_.size() - 1) {
    result_index = trajectory_points_.size() - 2;
  }

  // 确定包含目标里程的区间
  index_start = result_index;
  index_end = result_index + 1;

  // 验证区间是否包含目标里程
  double s_start = trajectory_points_[index_start].path_point().s();
  double s_end = trajectory_points_[index_end].path_point().s();
  
  if (s_target < s_start || s_target > s_end) {
    // 如果不在当前区间，使用备用线性查找
    for (std::size_t i = 0; i < trajectory_points_.size() - 1; ++i) {
      double s_i = trajectory_points_[i].path_point().s();
      double s_i1 = trajectory_points_[i + 1].path_point().s();
      if (s_target >= s_i && s_target <= s_i1) {
        index_start = i;
        index_end = i + 1;
        break;
      }
    }
  }

  // 如果找到的两个点是同一个点（理论上不应该发生），直接返回
  if (index_start == index_end) {
    return trajectory_points_[index_start];
  }

  // 使用线性插值在两个相邻轨迹点之间插值得到目标点
  const PathPoint &p0 = trajectory_points_[index_start].path_point();
  const PathPoint &p1 = trajectory_points_[index_end].path_point();
  
  PathPoint interpolated_path_point = math::InterpolateUsingLinearApproximation(p0, p1, s_target);
  
  // 创建返回的TrajectoryPoint，使用起始点的其他信息，但替换path_point
  TrajectoryPoint result = trajectory_points_[index_start];
  result.set_path_point(interpolated_path_point);
  
  // 对速度和加速度进行线性插值
  double weight = (s_target - p0.s()) / (p1.s() - p0.s());
  double v = (1 - weight) * trajectory_points_[index_start].v() + 
             weight * trajectory_points_[index_end].v();
  double a = (1 - weight) * trajectory_points_[index_start].a() + 
             weight * trajectory_points_[index_end].a();
  result.set_v(v);
  result.set_a(a);

  return result;
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint &p0,
                                                   const TrajectoryPoint &p1,
                                                   const double x,
                                                   const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    double px = math::lerp(p0.path_point().x(), p0.path_point().s(),
                           p1.path_point().x(), p1.path_point().s(), s);
    double py = math::lerp(p0.path_point().y(), p0.path_point().s(),
                           p1.path_point().y(), p1.path_point().s(), s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  PathPoint p = p0.path_point();
  double s = common::math::GoldenSectionSearch(dist_square, p0.path_point().s(),
                                               p1.path_point().s());
  p.set_s(s);
  p.set_x(common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                             p1.path_point().x(), p1.path_point().s(), s));
  p.set_y(common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                             p1.path_point().y(), p1.path_point().s(), s));
  p.set_theta(common::math::slerp(p0.path_point().theta(), p0.path_point().s(),
                                  p1.path_point().theta(), p1.path_point().s(),
                                  s));
  // approximate the curvature at the intermediate point
  p.set_kappa(common::math::lerp(p0.path_point().kappa(), p0.path_point().s(),
                                 p1.path_point().kappa(), p1.path_point().s(),
                                 s));
  return p;
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  if (trajectory_points_.size() == 0) {
    AERROR << "trajectory size error.";
    return;
  }
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point());
    trajectory_points_[i].mutable_path_point()->set_x(com.x());
    trajectory_points_[i].mutable_path_point()->set_y(com.y());
  }
}

common::math::Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return common::math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

const double TrajectoryAnalyzer::header_time() const { return header_time_; };
const unsigned int TrajectoryAnalyzer::seq_num() const { return seq_num_; }

}  // namespace control
}  // namespace legionclaw
