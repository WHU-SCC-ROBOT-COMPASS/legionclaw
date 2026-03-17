/******************************************************************************
 * Copyright 2017 The LegionClaw Authors. All Rights Reserved.
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

/**
 * @file spline_smoother_solver.cc
 **/

#include "modules/planning/src/common/math/include/smoothing_spline/spline_2d_solver.h"

#include <algorithm>
#include <cstring>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
//#include <memory>

//#include "modules/common/log.h"
#include "modules/common/math/qp_solver/active_set_qp_solver.h"
#include "modules/common/math/qp_solver/qp_solver_gflags.h"
//#include "modules/common/time/time.h"
//#include
//"modules/planning/src/common/math/include/smoothing_spline/planning_gflags.h"

namespace legionclaw {
namespace planning {
namespace math {
using namespace std;
USING_NAMESPACE_QPOASES
// namespace {
//
// constexpr double kRoadBound = 1e10;
//}
constexpr double kRoadBound = 1e10;

// using legionclaw::common::time::Clock;
using Eigen::MatrixXd;

Spline2dSolver::Spline2dSolver(const std::vector<double> &t_knots,
                               const uint32_t order)
    : spline_(t_knots, order),
      kernel_(t_knots, order),
      constraint_(t_knots, order) {}

void Spline2dSolver::Reset(const std::vector<double> &t_knots,
                           const uint32_t order) {
  //  spline_(t_knots, order);
  //**********************   spline_.set_spline2d(t_knots, order);
  spline_ = Spline2d(t_knots, order);
  kernel_ = Spline2dKernel(t_knots, order);
  constraint_ = Spline2dConstraint(t_knots, order);
}

// customize setup
Spline2dConstraint *Spline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel *Spline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d *Spline2dSolver::mutable_spline() { return &spline_; }

bool Spline2dSolver::Solve() {
  const MatrixXd &kernel_matrix = kernel_.kernel_matrix();
  if (0) {
    ofstream outfile("log/kernel_matrix.log", std::ios::app);
    outfile.precision(8);

    for (unsigned int i = 0; i < kernel_matrix.rows(); i++) {
      for (unsigned int j = 0; j < kernel_matrix.cols(); j++) {
        outfile << kernel_matrix(i, j) << " ";
      }
      outfile << endl;
      // outfile << output_virtual_path.ref_points[i].k_s << endl;
    }

    outfile << endl;
    outfile.close();
  }
  const MatrixXd &offset = kernel_.offset();
  if (0) {
    ofstream outfile("log/offset.log", std::ios::app);
    outfile.precision(8);

    for (unsigned int i = 0; i < offset.rows(); i++) {
      for (unsigned int j = 0; j < offset.cols(); j++) {
        outfile << offset(i, j) << " ";
      }
      outfile << endl;
      // outfile << output_virtual_path.ref_points[i].k_s << endl;
    }

    outfile << endl;
    outfile.close();
  }
  const MatrixXd &inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  if (0) {
    ofstream outfile("log/inequality_constraint_matrix.log", std::ios::app);
    outfile.precision(8);

    for (unsigned int i = 0; i < inequality_constraint_matrix.rows(); i++) {
      for (unsigned int j = 0; j < inequality_constraint_matrix.cols(); j++) {
        outfile << inequality_constraint_matrix(i, j) << " ";
      }
      outfile << endl;
      // outfile << output_virtual_path.ref_points[i].k_s << endl;
    }

    outfile << endl;
    outfile.close();
  }
  const MatrixXd &inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();

  const MatrixXd &equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  if (0) {
    ofstream outfile("log/equality_constraint_matrix.log", std::ios::app);
    outfile.precision(8);

    for (unsigned int i = 0; i < equality_constraint_matrix.rows(); i++) {
      for (unsigned int j = 0; j < equality_constraint_matrix.cols(); j++) {
        outfile << equality_constraint_matrix(i, j) << " ";
      }
      outfile << endl;
      // outfile << output_virtual_path.ref_points[i].k_s << endl;
    }

    outfile << endl;
    outfile.close();
  }
  const MatrixXd &equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  if (kernel_matrix.rows() != kernel_matrix.cols()) {
    return false;
  }

  int num_param = kernel_matrix.rows();
  int num_constraint =
      equality_constraint_matrix.rows() + inequality_constraint_matrix.rows();

  bool use_hotstart =
      last_problem_success_ &&
      (/*FLAGS_enable_sqp_solver &&*/ sqp_solver_ != nullptr &&
       num_param == last_num_param_ && num_constraint == last_num_constraint_);

  if (!use_hotstart) {
    sqp_solver_.reset(new ::qpOASES::SQProblem(num_param, num_constraint,
                                               ::qpOASES::HST_UNKNOWN));
    ::qpOASES::Options my_options;
    my_options.enableCholeskyRefactorisation = 10;
    my_options.epsNum = FLAGS_default_active_set_eps_num;
    my_options.epsDen = FLAGS_default_active_set_eps_den;
    my_options.epsIterRef = FLAGS_default_active_set_eps_iter_ref;
    sqp_solver_->setOptions(my_options);
    if (!FLAGS_default_enable_active_set_debug_info) {
      sqp_solver_->setPrintLevel(qpOASES::PL_NONE);
    }
  }

  // definition of qpOASESproblem
  /// myy20190305  h_matrix  is more than max-value-size
  const int kNumOfMatrixElements = kernel_matrix.rows() * kernel_matrix.cols();
  //  vector<double> h_matrix;//[kNumOfMatrixElements];  // NOLINT
  // real_t *h_matrix = (real_t *)malloc(kNumOfMatrixElements * sizeof(real_t));
  real_t *h_matrix = new real_t[kNumOfMatrixElements];
  //  int h_matrix0[kNumOfMatrixElements];  // NOLINT
  for (int i = 0; i < kNumOfMatrixElements; i++) {
    h_matrix[i] = 0.0;
  }

  const int kNumOfOffsetRows = offset.rows();
  // real_t *g_matrix = (real_t *)malloc(
  //     kNumOfOffsetRows * sizeof(real_t));  //[kNumOfOffsetRows];  // NOLINT
  real_t *g_matrix = new real_t[kNumOfOffsetRows];  // NOLINT  
  for (int i = 0; i < kNumOfOffsetRows; i++) {
    g_matrix[i] = 0;
  }
  // memset(g_matrix, 0, sizeof(g_matrix) );

  int index = 0;

  for (int r = 0; r < kernel_matrix.rows(); ++r) {
    g_matrix[r] = offset(r, 0);
    for (int c = 0; c < kernel_matrix.cols(); ++c) {
      h_matrix[index++] = kernel_matrix(r, c);
    }
  }

  if (index != kernel_matrix.rows() * kernel_matrix.cols()) {
    cout << " index != kernel_matrix.rows() * kernel_matrix.cols() " << endl;
  }
  // DCHECK_EQ(index, kernel_matrix.rows() * kernel_matrix.cols());

  // search space lower bound and uppper bound
  // real_t *lower_bound =
  //     (real_t *)malloc(num_param * sizeof(real_t));  //[num_param];  // NOLINT
  real_t *lower_bound = new real_t[num_param];  // NOLINT  
  // real_t *upper_bound =
  //     (real_t *)malloc(num_param * sizeof(real_t));  //[num_param];  // NOLINT
  real_t *upper_bound = new real_t[num_param];  // NOLINT  
  // memset(lower_bound, 0, sizeof(lower_bound) );
  // memset(upper_bound, 0, sizeof(upper_bound) );
  for (int i = 0; i < num_param; i++) {
    lower_bound[i] = 0;
    upper_bound[i] = 0;
  }

  const double l_lower_bound_ = -kRoadBound;
  const double l_upper_bound_ = kRoadBound;

  for (int i = 0; i < num_param; ++i) {
    lower_bound[i] = l_lower_bound_;
    upper_bound[i] = l_upper_bound_;
  }

  // constraint matrix construction
  // real_t *affine_constraint_matrix = (real_t *)malloc(
  //     num_param * num_constraint *
  //     sizeof(real_t));  //[num_param * num_constraint];  // NOLINT
  real_t *affine_constraint_matrix = new real_t[num_param * num_constraint];  // NOLINT  
  // memset(affine_constraint_matrix, 0, sizeof(affine_constraint_matrix) );
  for (int i = 0; i < num_param * num_constraint; i++) {
    affine_constraint_matrix[i] = 0;
  }

  // real_t *constraint_lower_bound = (real_t *)malloc(
  //     num_constraint * sizeof(real_t));  //[num_constraint];  // NOLINT
  real_t *constraint_lower_bound = new real_t[num_constraint];  // NOLINT

  // real_t *constraint_upper_bound = (real_t *)malloc(
  //     num_constraint * sizeof(real_t));  //[num_constraint];  // NOLINT
  real_t *constraint_upper_bound = new real_t[num_constraint];  // NOLINT  
  //  memset(constraint_lower_bound, 0, sizeof(constraint_lower_bound) );
  //  memset(constraint_upper_bound, 0, sizeof(constraint_upper_bound) );
  for (int i = 0; i < num_constraint; i++) {
    constraint_lower_bound[i] = 0;
    constraint_upper_bound[i] = 0;
  }

  index = 0;
  for (int r = 0; r < equality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r] = equality_constraint_boundary(r, 0);
    constraint_upper_bound[r] = equality_constraint_boundary(r, 0);

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = equality_constraint_matrix(r, c);
    }
  }

  if (index != equality_constraint_matrix.rows() * num_param) {
    cout << " index != equality_constraint_matrix.rows() * num_param " << endl;
  }
  // DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param);

  const double constraint_upper_bound_ = kRoadBound;
  for (int r = 0; r < inequality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r + equality_constraint_boundary.rows()] =
        inequality_constraint_boundary(r, 0);
    constraint_upper_bound[r + equality_constraint_boundary.rows()] =
        constraint_upper_bound_;

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = inequality_constraint_matrix(r, c);
    }
  }

  if (index != equality_constraint_matrix.rows() * num_param +
                   inequality_constraint_boundary.rows() * num_param) {
    cout << " index != equality_constraint_matrix.rows() * num_param + "
            "inequality_constraint_boundary.rows() * num_param "
         << endl;
  }
  //  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param +
  //                       inequality_constraint_boundary.rows() * num_param);

  // initialize problem
  int max_iter = std::max(FLAGS_default_qp_iteration_num, num_constraint);

  ::qpOASES::returnValue ret;
  //  const double start_timestamp = Clock::NowInSeconds();
  if (!use_hotstart) {
    // cout << "Spline2dSolver is using SQP hotstart." << endl;
    ret = sqp_solver_->hotstart(
        h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
        constraint_lower_bound, constraint_upper_bound, max_iter);
    /// mend ret != qpOASES::SUCCESSFUL_RETURN
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      // cout << "Fail to hotstart spline 2d, will use re-init instead." <<
      // endl;
      ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                              lower_bound, upper_bound, constraint_lower_bound,
                              constraint_upper_bound, max_iter);
    }
  } else {
    // cout << "Spline2dSolver is NOT using SQP hotstart." << endl;
    ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                            lower_bound, upper_bound, constraint_lower_bound,
                            constraint_upper_bound, max_iter);
  }
  delete []h_matrix;
  delete []g_matrix;
  delete []lower_bound;
  delete []upper_bound;
  delete []affine_constraint_matrix;
  delete []constraint_lower_bound;
  delete []constraint_upper_bound; 
  //  const double end_timestamp = Clock::NowInSeconds();
  //  ADEBUG << "Spline2dSolver QP time: "
  //         << (end_timestamp - start_timestamp) * 1000 << " ms.";
  /// mend ret != qpOASES::SUCCESSFUL_RETURN
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      cout << "qpOASES solver failed due to reached max iteration" << endl;
    } else {
      cout << "qpOASES solver failed due to infeasibility or other internal "
           << endl;
      cout << "reasons:" << ret << endl;
    }
    last_problem_success_ = false;
    return false;
  }

  last_problem_success_ = true;
  double result[num_param];  // NOLINT
  memset(result, 0, sizeof(result));

  sqp_solver_->getPrimalSolution(result);

  MatrixXd solved_params = MatrixXd::Zero(num_param, 1);
  for (int i = 0; i < num_param; ++i) {
    solved_params(i, 0) = result[i];
  }

  last_num_param_ = num_param;
  last_num_constraint_ = num_constraint;

  return spline_.set_splines(solved_params, spline_.spline_order());
}

// extract
const Spline2d &Spline2dSolver::spline() const { return spline_; }
}  // namespace math
}  // namespace planning
}  // namespace legionclaw