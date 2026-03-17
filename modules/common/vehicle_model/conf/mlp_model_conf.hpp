/**
 * @file              mlp_model_conf.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-12-07 08:42:44
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
 * @class MlpModelConf
 * @brief MlpModelConf.
 */
class MlpModelConf {
public:
  /**
   * @brief constructor
   */
  MlpModelConf() = default;

  /**
   * @brief destructor
   */
  ~MlpModelConf() = default;

  void set_dt(double &dt) { dt_ = dt; }

  const double &dt() const { return dt_; }

  double *mutable_dt() { return &dt_; }

protected:
  double dt_;
};
} // namespace common
} // namespace legionclaw
