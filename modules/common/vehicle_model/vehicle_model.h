/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/interface/vehicle_state.hpp"
#include "modules/common/json/json.hpp"
#include "modules/common/vehicle_model/conf/vehicle_model_conf.hpp"

namespace legionclaw {
namespace common {
using legionclaw::interface::VehicleState;
using json = nlohmann::json;
class VehicleModel {
public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState &cur_vehicle_state);

private:
  //配置文件操作类
  static json vehicle_model_json_;
  static void RearCenteredKinematicBicycleModel(
      const VehicleModelConf &vehicle_model_config,
      const double predicted_time_horizon,
      const VehicleState &cur_vehicle_state,
      VehicleState *predicted_vehicle_state);
};

} // namespace common
} // namespace legionclaw
