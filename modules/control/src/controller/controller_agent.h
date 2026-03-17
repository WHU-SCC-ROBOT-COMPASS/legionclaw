/**
 * @file    controller_agent.h
 * @author  jiang <jiangchengjie@indrv.cn>
 * @date    2019-10-12
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */

/**
 * @file
 * @brief Defines the ControllerAgent class.
 */

#pragma once

#include <memory>
#include <vector>

#include "common/status/status.h"
#include "common/util/factory.h"
#include "modules/control/src/proto/control_conf.pb.h"
#include "controller.h"
#include "modules/common/interface/chassis.hpp"
#include "modules/control/src/common/dependency_injector.h"
#include "modules/common/interface/control_command.hpp"
#include "modules/common/interface/control_analysis.hpp"

#include "modules/common/interface/adc_trajectory.hpp"

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

using namespace legionclaw::common;

/**
 * @class ControllerAgent
 *
 * @brief manage all controllers declared in control config file.
 */
class ControllerAgent {
 public:
  /**
   * @brief 初始化 ControllerAgent
   * @param[in] control_conf control配置参数
   * @return Status 初始化状态
   */
  common::Status Init(std::shared_ptr<DependencyInjector> injector,
                      const ControlConf *control_conf);

  /**
   * @brief  根据当前车辆状态计算控制命令
   * @param[in]  localization     定位信息
   * @param[in]  chassis          底盘信息
   * @param[in]  trajectory       轨迹
   * @param[out]  cmd              控制命令
   * @param[out]  analysis         控制数据分析
   * @return legionclaw::common::Status 
   */
  legionclaw::common::Status ComputeControlCommand(
      const legionclaw::interface::LocalizationEstimate *localization,
      const legionclaw::interface::Chassis *chassis, const legionclaw::interface::ADCTrajectory *trajectory,
      legionclaw::interface::ControlCommand *cmd, legionclaw::interface::ControlAnalysis *analysis);

  /**
   * @brief 重置 ControllerAgent
   * @return common::Status 重置状态
   */
  common::Status Reset();

 private:
  /**
   * @brief 注册新控制器。如果需要添加新类型的控制器，应首先在此函数中注册控制器。
   * @param  control_conf     control配置参数
   */
  void RegisterControllers(const ControlConf *control_conf);

  /**
   * @brief 初始化配置参数
   * @param  control_conf     control配置参数
   * @return common::Status 初始化参数状态
   */
  common::Status InitializeConf(const ControlConf *control_conf);

  /// @brief 配置参数
  const ControlConf *control_conf_ = nullptr;
  /// @brief 控制器工厂
  common::util::Factory<ControlConf::ControllerType, Controller>
      controller_factory_;
  /// @brief 控制器列表
  std::vector<std::unique_ptr<Controller>> controller_list_;
  /// @brief 车辆状态接口
  std::shared_ptr<DependencyInjector> injector_ = nullptr;
};

}  // namespace control
}  // namespace legionclaw
