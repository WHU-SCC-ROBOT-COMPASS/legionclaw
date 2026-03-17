/******************************************************************************
 * Copyright 2018 The LegionClaw Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include "modules/prediction/src/common/local_view.h"
#include "modules/common/status/status.h"
#include "modules/prediction/src/proto/prediction_conf.pb.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */
namespace legionclaw {
namespace prediction {
/**
 * @class Predictor
 * @brief Predictor is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Predictor {
 public:
  Predictor() = default;

  virtual ~Predictor() = default;

  virtual legionclaw::common::Status Init(const PredictionConf &prediction_conf) = 0;

  virtual std::string Name() const = 0;

  /**
   * @brief predict trajectories and intent of obstacles for execution.
   * @param local_view Current prediction local_view.
   * @return OK if prediction succeeds; error otherwise.
   */
  virtual legionclaw::common::Status Predict(const LocalView &local_view) = 0;

  virtual void Stop() = 0;
};
}  // namespace prediction
}  // namespace legionclaw
