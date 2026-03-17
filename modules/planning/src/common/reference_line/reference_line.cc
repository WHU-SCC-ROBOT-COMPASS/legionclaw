/**
 * @namespace legionclaw::reference_line
 * @brief legionclaw::reference_line
 */
#include "modules/planning/src/common/reference_line/reference_line.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "modules/common/math/math_tools.h"
#include "modules/common/math/math_utils.h"

namespace legionclaw {
namespace reference_line {
using namespace legionclaw::common::math;
ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points) {}

ReferenceLine::ReferenceLine(
    const std::vector<legionclaw::interface::PathPoint>& path_point) {
  for (auto way_point : path_point)
    reference_points_.emplace_back(ReferencePoint(way_point, 0, 0, false));
}

ReferenceLine::ReferenceLine(
    const std::vector<legionclaw::interface::LanePoint>& lane_points) {
  for (auto lane_point : lane_points) {
    reference_points_.emplace_back(ReferencePoint(lane_point, 0, 0, false));
  }
}

ReferenceLine::ReferenceLine(
    const std::vector<legionclaw::planning::LanePointInner>& lane_points) {
  for (auto lane_point : lane_points) {
    reference_points_.emplace_back(ReferencePoint(lane_point, 0, 0, false));
  }
}

void ReferenceLine::ComputeAllMileage(const double s0) {
  int start_pos = 0;
  int end_pos = ReferencePointSize();

  if (end_pos <= 0) return;

  ReferencePoint p0;
  p0 = reference_points_[start_pos];

  double x = p0.x();
  double y = p0.y();
  double xx, yy, ss;

  double s = s0;
  reference_points_[start_pos].set_s(s);

  for (int i = start_pos + 1; i < end_pos; i++) {
    xx = reference_points_[i].x();
    yy = reference_points_[i].y();

    ss = sqrt((xx - x) * (xx - x) + (yy - y) * (yy - y));
    s += ss;

    reference_points_[i].set_s(s);

    x = xx;
    y = yy;
  }
}

void ReferenceLine::ComputeAllMileageAndWidth(
    const double s0, const std::vector<ReferencePoint> ref_points) {
  int start_pos = 0;
  int end_pos = ReferencePointSize();

  if (end_pos <= 0 || ref_points.size() <= 0) return;

  ReferencePoint p0;
  p0 = reference_points_[start_pos];

  double x = p0.x();
  double y = p0.y();
  double xx, yy, ss;

  double s = s0;
  reference_points_[start_pos].set_z(ref_points.front().z());
  reference_points_[start_pos].set_s(s);
  reference_points_[start_pos].set_left_road_width(
      ref_points.front().left_road_width());
  reference_points_[start_pos].set_right_road_width(
      ref_points.front().right_road_width());
  reference_points_[start_pos].set_left_line_type(
      ref_points.front().left_line_type());
  reference_points_[start_pos].set_right_line_type(
      ref_points.front().right_line_type());
  reference_points_[start_pos].set_limit_speed(
      ref_points.front().limit_speed());
  // reference_points_[start_pos].reverseLaneWidth =
  // ref_points.front().reverseLaneWidth;

  int index = 0;
  for (int i = start_pos + 1; i < end_pos; i++) {
    xx = reference_points_[i].x();
    yy = reference_points_[i].y();

    ss = sqrt((xx - x) * (xx - x) + (yy - y) * (yy - y));
    s += ss;

    reference_points_[i].set_s(s);

    if (s < ref_points.back().s()) {
      if (/*s>=ref_points[index].s() &&*/ s < ref_points[index + 1].s()) {
        reference_points_[i].set_z(
            lerp(ref_points[index].z(), ref_points[index].s(),
                 ref_points[index + 1].z(), ref_points[index + 1].s(), s));
        reference_points_[i].set_left_road_width(
            lerp(ref_points[index].left_road_width(), ref_points[index].s(),
                 ref_points[index + 1].left_road_width(), ref_points[index + 1].s(), s));
        reference_points_[i].set_right_road_width(
            lerp(ref_points[index].right_road_width(), ref_points[index].s(),
                 ref_points[index + 1].right_road_width(), ref_points[index + 1].s(), s));
        reference_points_[i].set_left_line_type(
            ref_points[index].left_line_type());
        reference_points_[i].set_right_line_type(
            ref_points[index].right_line_type());
        reference_points_[i].set_limit_speed(
      ref_points[index].limit_speed());
        // reference_points_[i].reverseLaneWidth =
        // ref_points[index].reverseLaneWidth;
      } else  // if(s>=ref_points[index+1].s())
      {
        index++;
        reference_points_[i].set_z(
            ref_points[index].z());
        reference_points_[i].set_left_road_width(
            ref_points[index].left_road_width());
        reference_points_[i].set_right_road_width(
            ref_points[index].right_road_width());
        reference_points_[i].set_left_line_type(
            ref_points[index].left_line_type());
        reference_points_[i].set_right_line_type(
            ref_points[index].right_line_type());
        reference_points_[i].set_limit_speed(
      ref_points[index].limit_speed());
        // reference_points_[i].reverseLaneWidth =
        // ref_points[index].reverseLaneWidth;
      }
    } else {
      reference_points_[i].set_z(
          ref_points.back().z());
      reference_points_[i].set_left_road_width(
          ref_points.back().left_road_width());
      reference_points_[i].set_right_road_width(
          ref_points.back().right_road_width());
      reference_points_[i].set_left_line_type(
          ref_points.back().left_line_type());
      reference_points_[i].set_right_line_type(
          ref_points.back().right_line_type());
      reference_points_[i].set_limit_speed(
      ref_points.back().limit_speed());
      // reference_points_[i].reverseLaneWidth =
      // ref_points.back().reverseLaneWidth;
    }

    x = xx;
    y = yy;
  }
}

// void ReferenceLine::TransfertRad2Deg() {
//   for (int i = 0; i < ReferencePointSize(); ++i) {
//     reference_points_[i].set_theta(R2D(reference_points_[i].theta()));
//   }
// }
bool ReferenceLine::Stitch(const ReferenceLine& other,const int match_index) {
  if (other.ReferencePointSize() == 0) {
    AWARN << "The other reference line is empty.";
    return true;
  }
  auto first_point = reference_points_.front();
  bool first_join = match_index > 0 && match_index < other.ReferencePointSize();

  if (!first_join) {
    AERROR << "These reference lines are not connected.";
    return false;
  }

  const auto& other_points = other.ReferencePoints();
  //计算当前平滑的参考线的第一个点距离上一次参考线的横向偏差，后续新的规划会优化这个问题，通过投影来做
  //如果两次相差的曲率大于0.001，不允许拼接
  double diff_kappa = fabs(first_point.kappa() - other_points.at(match_index).kappa());
  double kStitchingError =0.05;
  if (first_join) {
    if (diff_kappa > kStitchingError) {
      AERROR << "lateral stitching error on first join of reference line too "
                "big, diff_kappa:"<< diff_kappa;
      return false;
    }
    // AWARN << "new_points_.begin().k : " << first_point.kappa()
    //       << " , "
    //       << "pre_points_.k : " << other_points.at(match_index).kappa();
    reference_points_.insert(reference_points_.begin(), other_points.begin(),
                             other_points.begin() + match_index);
    // AWARN << "cur_points_0.x : " << reference_points_.at(match_index -2).x()
    //       << " , "
    //       << "cur_points_0.y : " << reference_points_.at(match_index -2).y()
    //       << " , "
    //       << "cur_points_1.x : " << reference_points_.at(match_index -1).x()
    //       << " , "
    //       << "cur_points_1.y : " << reference_points_.at(match_index -1).y()
    //       << " , "
    //       << "cur_points_2.x : " << reference_points_.at(match_index).x()
    //       << " , "
    //       << "cur_points_2.y : " << reference_points_.at(match_index).y()
    //       << " , "
    //       << "cur_points_3.x : " << reference_points_.at(match_index+1).x()
    //       << " , "
    //       << "cur_points_3.y : " << reference_points_.at(match_index+1).y()
    //       << " , "
    //       << "cur_points_4.x : " << reference_points_.at(match_index+2).x()
    //       << " , "
    //       << "cur_points_4.y : " << reference_points_.at(match_index+2).y();
  }
  return true;
}

bool ReferenceLine::Segment(const int start_index,
                            const int end_index) {
  if (end_index - start_index < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }

  reference_points_ =
      std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                  reference_points_.begin() + end_index);
  return true;
}

}  // namespace reference_line
}  // namespace legionclaw
