/**
 * @file    controller.h
 * @author  jiang <jiangchengjie@indrv.cn>
 * @date    2018-07-07
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */

#ifndef CONTROLLER_CONTROLLER_H_
#define CONTROLLER_CONTROLLER_H_

#include <common/status/status.h>

#include <iostream>
#include <vector>

#include "modules/control/src/proto/control_conf.pb.h"
#include "modules/control/src/proto/stop_distance_controller_conf.pb.h"
#include "modules/common/interface/chassis.hpp"
#include "modules/control/src/common/dependency_injector.h"
#include "modules/common/interface/control_command.hpp"
#include "modules/common/interface/control_analysis.hpp"

#include "modules/control/src/interface/localization_estimate.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/math/math_tools.h"


using namespace std;
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {

/**
 * @class controller
 * @brief controller base.
 */
class Controller {
 public:
  /**
   * @brief 构造函数
   */
  Controller() = default;

  /**
   * @brief 析构函数
   */
  ~Controller() = default;

  /**
   * @brief 初始化 Controller
   * @param control_conf control 配置参数
   * @return Status 初始化状态
   */
  virtual legionclaw::common::Status Init(
      std::shared_ptr<DependencyInjector> injector,
      const ControlConf *control_conf) = 0;

  /**
   * @brief  根据当前车辆状态计算控制命令
   * @param[in]  localization     定位信息
   * @param[in]  chassis          底盘信息
   * @param[in]  trajectory       轨迹
   * @param[out]  cmd              控制命令
   * @param[out]  analysis         控制数据分析        
   * @return legionclaw::common::Status 计算状态
   */
  virtual legionclaw::common::Status ComputeControlCommand(
      const legionclaw::interface::LocalizationEstimate *localization,
      const legionclaw::interface::Chassis *chassis, const legionclaw::interface::ADCTrajectory *trajectory,
      legionclaw::interface::ControlCommand *cmd, legionclaw::interface::ControlAnalysis *analysis) = 0;

  /**
   * @brief 重置控制器
   * @return legionclaw::common::Status 重置状态
   */
  virtual legionclaw::common::Status Reset() = 0;

  /**
   * @brief 控制器名字
   * @return std::string 名字
   */
  virtual std::string Name() const = 0;

  /**
   * @brief 停止控制器
   */
  virtual void Stop() = 0;
};
}  // namespace control
}  // namespace legionclaw

#endif  // COMMON_CONTROLLER_H_
