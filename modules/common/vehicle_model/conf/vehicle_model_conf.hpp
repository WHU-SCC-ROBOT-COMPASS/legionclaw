/**
 * @file              vehicle_model_conf.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-12-07 06:17:01
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#pragma once
#include "com_centered_dynamic_bicycle_model_conf.hpp"
#include "mlp_model_conf.hpp"
#include "rear_centered_kinematic_bicycle_model_conf.hpp"
/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

/**
 * @class VehicleModelConf
 * @brief VehicleModelConf.
 */
class VehicleModelConf {
public:
  /**
   * @brief constructor
   */
  VehicleModelConf() = default;

  /**
   * @brief destructor
   */
  ~VehicleModelConf() = default;
  enum ModelType {
    REAR_CENTERED_KINEMATIC_BICYCLE_MODEL = 0,
    COM_CENTERED_DYNAMIC_BICYCLE_MODEL = 1,
    MLP_MODEL = 2,
  };

  void set_model_type(ModelType &model_type) { model_type_ = model_type; }

  const ModelType &model_type() const { return model_type_; }

  ModelType *mutable_model_type() { return &model_type_; }

  void set_rc_kinematic_bicycle_model(
      RearCenteredKinematicBicycleModelConf &rc_kinematic_bicycle_model) {
    rc_kinematic_bicycle_model_ = rc_kinematic_bicycle_model;
  }

  const RearCenteredKinematicBicycleModelConf &
  rc_kinematic_bicycle_model() const {
    return rc_kinematic_bicycle_model_;
  }

  RearCenteredKinematicBicycleModelConf *mutable_rc_kinematic_bicycle_model() {
    return &rc_kinematic_bicycle_model_;
  }

  void set_comc_dynamic_bicycle_model(
      ComCenteredDynamicBicycleModelConf &comc_dynamic_bicycle_model) {
    comc_dynamic_bicycle_model_ = comc_dynamic_bicycle_model;
  }

  const ComCenteredDynamicBicycleModelConf &comc_dynamic_bicycle_model() const {
    return comc_dynamic_bicycle_model_;
  }

  ComCenteredDynamicBicycleModelConf *mutable_comc_dynamic_bicycle_model() {
    return &comc_dynamic_bicycle_model_;
  }

  void set_mlp_model(MlpModelConf &mlp_model) { mlp_model_ = mlp_model; }

  const MlpModelConf &mlp_model() const { return mlp_model_; }

  MlpModelConf *mutable_mlp_model() { return &mlp_model_; }

protected:
  ModelType model_type_;
  RearCenteredKinematicBicycleModelConf rc_kinematic_bicycle_model_;
  ComCenteredDynamicBicycleModelConf comc_dynamic_bicycle_model_;
  MlpModelConf mlp_model_;
};
} // namespace common
} // namespace legionclaw
