/**
 * @file              com_centered_dynamic_bicycle_model_conf.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-12-07 08:42:32
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
 * @class ComCenteredDynamicBicycleModelConf
 * @brief ComCenteredDynamicBicycleModelConf.
 */
class ComCenteredDynamicBicycleModelConf {
public:
  /**
   * @brief constructor
   */
  ComCenteredDynamicBicycleModelConf() = default;

  /**
   * @brief destructor
   */
  ~ComCenteredDynamicBicycleModelConf() = default;

  void set_dt(double &dt) { dt_ = dt; }

  const double &dt() const { return dt_; }

  double *mutable_dt() { return &dt_; }

protected:
  double dt_;
};
} // namespace common
} // namespace legionclaw
