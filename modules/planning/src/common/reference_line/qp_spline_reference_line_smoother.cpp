
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

#include "modules/common/logging/logging.h"
#include "modules/common/math/curve_math.h"
#include "modules/common/interface/lane_point.hpp"
#include "modules/planning/src/common/reference_line/qp_spline_reference_line_smoother.h"


#define M_E 2.7182818284590452354         /* e */
#define M_LOG2E 1.4426950408889634074     /* log_2 e */
#define M_LOG10E 0.43429448190325182765   /* log_10 e */
#define M_LN2 0.69314718055994530942      /* log_e 2 */
#define M_LN10 2.30258509299404568402     /* log_e 10 */
#define M_PI 3.14159265358979323846       /* pi */
#define M_PI_2 1.57079632679489661923     /* pi/2 */
#define M_PI_4 0.78539816339744830962     /* pi/4 */
#define M_1_PI 0.31830988618379067154     /* 1/pi */
#define M_2_PI 0.63661977236758134308     /* 2/pi */
#define M_2_SQRTPI 1.12837916709551257390 /* 2/sqrt(pi) */
#define M_SQRT2 1.41421356237309504880    /* sqrt(2) */
#define M_SQRT1_2 0.70710678118654752440  /* 1/sqrt(2) */

using namespace std;
using legionclaw::interface::LanePoint;
using namespace legionclaw::common;
using namespace legionclaw::common::math;
using namespace legionclaw::planning::math;

namespace legionclaw {
namespace reference_line {

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother() {
  const uint32_t spline_order = 5;
  spline_solver_.reset(new Spline2dSolver(t_knots_, spline_order));
}

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother(
    const double &anchor_sampling_step, const double &knots_sampling_step, const double &lateral_bound_limit)
    : anchor_sampling_step_(anchor_sampling_step),
      knots_sampling_step_(knots_sampling_step),
      lateral_bound_limit_(lateral_bound_limit) {
  const uint32_t spline_order = 5;
  spline_solver_.reset(new Spline2dSolver(t_knots_, spline_order));
}

// QpSplineReferenceLineSmoother::~QpSplineReferenceLineSmoother()
//{
//    //dtor
//}

void QpSplineReferenceLineSmoother::Clear() { 
  t_knots_.clear(); 
  t_knots_.shrink_to_fit();
}

bool QpSplineReferenceLineSmoother::Smooth(
    const ReferenceLine &raw_reference_line,
    ReferenceLine *smoothed_reference_line, const double &path_density,
    const uint32_t &spline_order) {  // path* const smoothed_reference_line

  smoothed_reference_line->Clear();
  Clear();

  // const double kEpsilon = 1e-6;
  if (!Sampling()) {
    AINFO << "Fail to sample reference line smoother points!" << endl;
    return false;
  }

  // const uint32_t spline_order = 5;

  spline_solver_->Reset(t_knots_, spline_order);

  if (!AddConstraint()) {
    AINFO << "Add constraint for spline smoother failed" << endl;
    return false;
  }

  if (!AddKernel()) {
    AINFO << "Add kernel for spline smoother failed." << endl;
    return false;
  }

  if (!Solve()) {
    AINFO << "Solve spline smoother problem failed" << endl;
    return false;
  }

  // smoothed_reference_line->Clear();
  // mapping spline to reference line point
  // 原始ReferenceLine的起点，第一个anchor point，也是平滑后的ReferenceLine起点
  const double start_t = t_knots_.front();
  // 原始ReferenceLine的终点，最后一个anchor
  // point，也是平滑后的ReferenceLine终点
  const double end_t = t_knots_.back();
  // 采样精度，一共采样500个点。
  // uint32_t num_of_total_points = 500;
  ///******the resolution need to be modified
  const double resolution = path_density / sampling_step_;
  // std::cout <<"resolution: " << resolution <<endl;
  //      (end_t - start_t) / (num_of_total_points - 1);
  double t = start_t;
  // std::vector<navi_point> ref_points;  // 采样点保存
  ReferencePoint ref_point;
  const auto &spline = spline_solver_->spline();
  //  for (std::uint32_t i = 0; i < num_of_total_points && t < end_t;
  //       ++i, t += resolution) {
  // smoothed_reference_line->Resize((int)end_t/resolution);
  // int index = 0;
  for (; t < end_t; t += resolution) {
    const double heading = std::atan2(spline.DerivativeY(t),
                                      spline.DerivativeX(t));  // 采样点速度大小
    const double kappa = CurveMath::ComputeCurvature(
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.DerivativeY(t),
        spline.SecondDerivativeY(
            t));  // 采样点曲率，弧线越直曲率越小；弧线越弯，曲率越大
    const double dkappa = CurveMath::ComputeCurvatureDerivative(
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.ThirdDerivativeX(t), spline.DerivativeY(t),
        spline.SecondDerivativeY(t),
        spline.ThirdDerivativeY(t));  // 曲率导数，描述曲率变化
    // 求解累积距离为t时曲线的坐标，也是相对与第一个点的偏移坐标
    std::pair<double, double> xy = spline(t);
    // 加上第一个anchor point的世界系坐标，变成世界系坐标
    xy.first += ref_x_;
    xy.second += ref_y_;

    ref_point.set_x(xy.first);
    ref_point.set_y(xy.second);
    ref_point.set_theta(heading);
    ref_point.set_kappa(kappa);
    ref_point.set_dkappa(dkappa);
    smoothed_reference_line->AddPoint(ref_point);
    // smoothed_reference_line->ReferencePoints(index, ref_point);
    // index++;
  }
  smoothed_reference_line->ComputeAllMileageAndWidth(ref_mileage_,
                                                     anchor_points_);

  if (smoothed_reference_line->ReferencePointSize() < 2) {
    AINFO << "Fail to generate smoothed reference line." << endl;
    return false;
  }
  return true;
}

bool QpSplineReferenceLineSmoother::Sampling() {
  sampling_step_ = knots_sampling_step_;
  ReferencePoint first_path_point, last_path_point;
  if (anchor_points_.size() <= 0) return 0;
  first_path_point = anchor_points_.front();
  last_path_point = anchor_points_.back();
  // AINFO << "number of  anchor_points_ : " << anchor_points_.size() << endl;

  ///******Sampling : modify the way to get the knots
  //  double sampling_step = 25.0;
  //  int num_sampling_interval = 5;
  const double length = last_path_point.s() - first_path_point.s();
  uint32_t num_spline =
      std::max(1u, static_cast<uint32_t>(length / sampling_step_ + 0.5));
  if (num_spline < 2) {
    sampling_step_ = length / 2.0;
    num_spline = 2;
  }
  //  num_spline = anchor_points_.size() / num_sampling_interval;
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);

    //    if ( i*num_sampling_interval > anchor_points_.size()-3 )
    //    {
    //        t_sampling_.push_back( anchor_points_.size()*1.0 - 1.0 );
    //        break;
    //    }
    //    else
    //    {
    //        t_sampling_.push_back( i*num_sampling_interval*1.0 );
    //    }

    //    AINFO << "t_knots_ : " <<  i  << " = " << t_knots_[ i ] << endl;
    //    AINFO << "t_sampling_ : " <<  i  << " = " << t_sampling_[ i ] << endl;
  }
  // AINFO << "number of  t_knots_ : " << t_knots_.size() << endl;
  // AINFO << "num_spline : " << num_spline << endl;
  ref_x_ = first_path_point.x();
  ref_y_ = first_path_point.y();
  ref_heading_ = first_path_point.theta();
  ref_mileage_ = first_path_point.s();
  return true;
}

int QpSplineReferenceLineSmoother::FindIndex(unsigned int j,
                                             vector<double> t_sampling) {
  for (unsigned int i = 0; i < t_sampling.size() - 1; i++) {
    if (j >= t_sampling[i] && j <= t_sampling[i + 1]) {
      return i;
    }
  }
  return -1;
}
/// myy20190301
bool QpSplineReferenceLineSmoother::AddConstraint() {
  // Add x, y boundary constraint
  // std::vector<double> mileages;
  std::vector<double> headings;
  std::vector<double> longitudinal_bound;
  std::vector<double> lateral_bound;
  std::vector<math::Vec2d> xy_points;
  // std::vector<ReferencePoint> xy_points;
  ReferencePoint point_xy;
  if (anchor_points_.size() <= 0) return 0;
  for (unsigned int i = 0; i < anchor_points_.size(); i++) {
    headings.push_back(anchor_points_[i].theta());  ///- ref_heading_*M_PI/180
    longitudinal_bound.push_back(anchor_points_[i].longitudinal_bound());
    lateral_bound.push_back(anchor_points_[i].lateral_bound());
    point_xy.set_x(anchor_points_[i].x() - ref_x_);
    point_xy.set_y(anchor_points_[i].y() - ref_y_);
    // point_xy.set_mileage(anchor_points_[ i ].s() - ref_mileage_);
    // xy_points.emplace_back( point_xy );
    // mileages.push_back(anchor_points_[ i ].s() - ref_mileage_);
    xy_points.push_back({point_xy.x(), point_xy.y()});
  }

  ReferencePoint first_point, last_point;
  first_point = anchor_points_.front();
  last_point = anchor_points_.back();

  ///******AddConstraint : the scale need to be modified
  const double scale =
      (last_point.s() - first_point.s()) / (t_knots_.back() - t_knots_.front());
  //  AINFO << "scale : " << scale << endl;
  //    double scale = 1.0;
  std::vector<double> evaluated_t;
  for (unsigned int j = 0; j < anchor_points_.size(); j++) {
    //      AINFO << "------------------------" << endl;
    //      int index = FindIndex( j, t_sampling_ );
    //      if ( index < 0 )
    //        break;
    ////      AINFO << "index = " << index << endl;
    //      scale = anchor_points_[ t_sampling_[index+1] ].s
    //            - anchor_points_[ t_sampling_[index] ].s;
    ////      AINFO << "scale = " << scale << endl;
    //      double _tmp = anchor_points_[ j ].s
    //                  - anchor_points_[ t_sampling_[index] ].s;
    ////      AINFO << "(1)_tmp = " << _tmp << endl;
    //      _tmp = index + _tmp / scale;
    //      AINFO << "(2)_tmp = " << _tmp << endl;
    double _tmp = (anchor_points_[j].s() - ref_mileage_) / scale;
    evaluated_t.emplace_back(_tmp);
  }

  auto *spline_constraint = spline_solver_->mutable_constraint();

  // all points (x, y) should not deviate anchor points by a bounding box
  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
                                        longitudinal_bound, lateral_bound)) {
    AINFO << "Add 2d boundary constraint failed." << endl;
    return false;
  }

  // the heading of the first point should be identical to the anchor point.
  if (!spline_constraint->AddPointAngleConstraint(evaluated_t.front(),
                                                  headings.front())) {
    AINFO << "Add 2d point angle constraint failed." << endl;
    return false;
  }
  //
  //  // all spline should be connected smoothly to the second order derivative.
  //  各函数接壤处平滑约束
  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
    AINFO << "Add jointness constraint failed." << endl;
    return false;
  }

  return true;
}

bool QpSplineReferenceLineSmoother::AddKernel() {
  Spline2dKernel *kernel = spline_solver_->mutable_kernel();

  // add spline kernel
  const double second_derivative_weight =
      200.0;  // config_.qp_spline().second_derivative_weight()
  if (second_derivative_weight > 0.0) {
    kernel->AddSecondOrderDerivativeMatrix(second_derivative_weight);
  }
  const double third_derivative_weight =
      1000.0;  // config_.qp_spline().third_derivative_weight()
  if (third_derivative_weight > 0.0) {
    kernel->AddThirdOrderDerivativeMatrix(third_derivative_weight);
  }
  const double regularization_weight =
      1.0e-5;  // config_.qp_spline().regularization_weight()
  kernel->AddRegularization(regularization_weight);
  return true;
}

bool QpSplineReferenceLineSmoother::Solve() { return spline_solver_->Solve(); }

// mend x_knots_ to t_knots_
uint32_t QpSplineReferenceLineSmoother::FindIndex(const double x) const {
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), x);
  const uint32_t dis = std::distance(t_knots_.begin(), upper_bound);
  if (dis < t_knots_.size()) {
    return dis - 1;
  } else {
    return t_knots_.size() - 2;
  }
}

void QpSplineReferenceLineSmoother::SetAnchorPoints(
    const std::vector<ReferencePoint>& anchor_points) {
  CHECK_GE(anchor_points.size(), 2U);
  anchor_points_ = anchor_points;
}

}  // namespace reference_line
}  // namespace legionclaw
