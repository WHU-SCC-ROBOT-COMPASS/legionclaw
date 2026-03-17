/**
 * @file              rear_centered_kinematic_bicycle_model_conf.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-12-07 08:43:14
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#pragma once
/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

/**
 * @class RearCenteredKinematicBicycleModelConf
 * @brief RearCenteredKinematicBicycleModelConf.
 */
class RearCenteredKinematicBicycleModelConf {
public:
  /**
   * @brief constructor
   */
  RearCenteredKinematicBicycleModelConf() = default;

  /**
   * @brief destructor
   */
  ~RearCenteredKinematicBicycleModelConf() = default;

  void set_dt(double &dt) { dt_ = dt; }

  const double &dt() const { return dt_; }

  double *mutable_dt() { return &dt_; }

protected:
  double dt_;
};
} // namespace common
} // namespace legionclaw
