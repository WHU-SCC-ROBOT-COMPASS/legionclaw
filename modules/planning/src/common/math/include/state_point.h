/**
 * @file
 * @brief Defines the StatePoint class.
 */

#pragma once

#include <cmath>
#include <string>

#include "modules/common/math/vec2d.h"
namespace legionclaw {
namespace planning {
namespace math {
using  namespace legionclaw::common::math;
/**
 * @class StatePoint
 *
 */
class StatePoint : public Vec2d {
 public:
  //! Getter for heading component
  double heading() const { return heading_; }

  //! Getter for kappa component
  double kappa() const { return kappa_; }

  //! Setter for heading component
  void set_heading(const double heading) { heading_ = heading; }

  //! Setter for kappa component
  void set_kappa(const double kappa) { kappa_ = kappa; }

 protected:
  double heading_ = 0.0;
  double kappa_ = 0.0;
};

}  // namespace math
}  // namespace planning
}  // namespace legionclaw