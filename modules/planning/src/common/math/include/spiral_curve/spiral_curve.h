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
 * @file: spiral_curve.h
 * @brief: spiral path base class
 **/
#pragma once

#include <algorithm>
#include <vector>

#include "modules/common/interface/path_point.hpp"
#include "modules/common/math/math_utils.h"

namespace legionclaw {
namespace planning {
namespace spiral {

struct SpiralCurveConfig {
  int simpson_size;             // [default = 9];
  double newton_raphson_tol;    //[default = 0.01];
  int newton_raphson_max_iter;  //[default = 20];
};

using legionclaw::interface::PathPoint;

class SpiralCurve {
 public:
  SpiralCurve(const PathPoint &s, const PathPoint &e,
              const std::uint32_t order);
  virtual ~SpiralCurve() = default;

  /**
   * @brief Set configuration if desired (default setting was defined in
   * constructor)
   **/
  void SetSpiralConfig(const SpiralCurveConfig &spiral_config);
  /**
   * @brief Default process of calculating path without lookup table
   * @return errors of final state: fitted value vs true end point
   **/
  virtual bool CalculatePath() = 0;

  /**
   * @brief Output methods
   **/
  const std::vector<double> &p_params() const;
  const SpiralCurveConfig &spiral_config() const;
  const PathPoint &start_point() const;
  const PathPoint &end_point() const;
  double sg() const;
  double error() const;

  /**
   * @brief Get path vector with sampling size n
   * @return sequence of sampling points
   **/
  virtual bool GetPathVec(const std::uint32_t n,
                          std::vector<PathPoint> *path_points) const = 0;
  /**
   * @brief Calculate quintic path point at s locations along the whole path.
   * @return vector of path points
   **/
  virtual bool GetPathVecWithS(const std::vector<double> &vec_s,
                               std::vector<PathPoint> *path_points) const = 0;

 private:
  const PathPoint *start_point_;
  const PathPoint *end_point_;
  std::vector<double> p_params_;
  double sg_;
  double error_;
  SpiralCurveConfig spiral_config_;

 protected:
  void set_sg(const double sg);
  void set_error(const double error);

  bool ResultSanityCheck() const;

  template <typename T>
  void PrependToPParams(T begin, T end) {
    std::copy(begin, end, p_params_.begin());
  }
  static constexpr double s_two_pi_ = 2 * M_PI;
};

}  // namespace spiral
}  // namespace planning
}  // namespace legionclaw
