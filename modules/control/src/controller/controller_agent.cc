/**
 * @file    controller_agent.h
 * @author  jiang <jiangchengjie@indrv.cn>
 * @date    2019-10-12
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */

#include "controller_agent.h"

#include <utility>

#include "common/logging/logging.h"
#include "common/status/status.h"
#include "common/time/time_tool.h"
#include "controller/lon_controller/lon_controller.h"
#include "controller/lon_speed_controller/lon_speed_controller.h"
#include "controller/lqr_controller/lqr_lat_controller.h"
// #include "controller/mpc_controller/mpc_controller.h"
// #include "controller/stop_distance_controller/stop_distance_controller.h"
#include "controller/purepursuit_controller/purepursuit_controller.h"

namespace legionclaw
{
  namespace control
  {
    using legionclaw::common::Status;
    using namespace legionclaw::common;

    void ControllerAgent::RegisterControllers(const ControlConf *control_conf)
    {
      AINFO << "Only support Lat + Lon controllers as of now";
      //根据配置文件的激活信息注册控制器
      for (auto active_message : control_conf->active_controllers().active_message())
      {
        switch (active_message)
        {
        case ControlConf::LQR_LAT_CONTROLLER:
          controller_factory_.Register(
              ControlConf::LQR_LAT_CONTROLLER,
              []() -> Controller *
              { return new LQRLatController(); });
          break;
        case ControlConf::LON_CONTROLLER:
          controller_factory_.Register(
              ControlConf::LON_CONTROLLER,
              []() -> Controller *
              { return new LonController(); });
          break;
        case ControlConf::LON_SPEED_CONTROLLER:
          controller_factory_.Register(
              ControlConf::LON_SPEED_CONTROLLER,
              []() -> Controller *
              { return new LonSpeedController(); });
          break;
        // case ControlConf::STOP_DISTANCE__CONTROLLER:
        //   controller_factory_.Register(
        //       ControlConf::STOP_DISTANCE__CONTROLLER,
        //       []() -> Controller *
        //       { return new StopDistanceController(); });
        //   break;
        case ControlConf::PUREPURSUIT_CONTROLLER:
          controller_factory_.Register(
              ControlConf::PUREPURSUIT_CONTROLLER,
              []() -> Controller *
              { return new PurePursuitController(); });
          break;
        default:
          AERROR << "Unknown active controller type:";
        }
      }
    }

    Status ControllerAgent::InitializeConf(const ControlConf *control_conf)
    {
      if (!control_conf)
      {
        AERROR << "control_conf is null";
        return Status(Status::ErrorCode::CONTROL_INIT_ERROR,
                      "Failed to load config");
      }
      control_conf_ = control_conf;

      //根据配置文件的激活信息创建对应的控制器对象，用列表管理这些控制器
      for (auto controller_message : control_conf_->active_controllers().active_message())
      {
        auto controller = controller_factory_.CreateObject(
            static_cast<ControlConf::ControllerType>(controller_message));
        if (controller)
        {
          controller_list_.emplace_back(std::move(controller));
        }
        else
        {
          ADEBUG << "Controller: " << control_conf_->active_controllers().active_info().at(controller_message).name() << "is not supported";

          return Status(Status::ErrorCode::CONTROL_INIT_ERROR,
                        "Invalid controller type:" + control_conf_->active_controllers().active_info().at(controller_message).name());
        }
      }
      return Status::Ok();
    }

    Status ControllerAgent::Init(std::shared_ptr<DependencyInjector> injector,
                                 const ControlConf *control_conf)
    {
      injector_ = injector;
      RegisterControllers(control_conf);
      CHECK(InitializeConf(control_conf).ok()) << "Failed to initialize config.";

      //列表中的控制器初始化
      for (auto &controller : controller_list_)
      {
        stringstream sstream;
        if (controller == nullptr)
        {
          return Status(Status::ErrorCode::CONTROL_INIT_ERROR,
                        "Controller is null.");
        }
        if (!controller->Init(injector, control_conf_).ok())
        {
          AERROR << "Controller <" << controller->Name() << "> init failed!";
          return Status(Status::ErrorCode::CONTROL_INIT_ERROR, "Failed to init Controller:" + controller->Name());
        }
        AINFO << "Controller <" << controller->Name() << "> init done!";
      }
      return Status::Ok();
    }

    legionclaw::common::Status ControllerAgent::ComputeControlCommand(
        const legionclaw::interface::LocalizationEstimate *localization,
        const legionclaw::interface::Chassis *chassis, const legionclaw::interface::ADCTrajectory *trajectory,
        legionclaw::interface::ControlCommand *cmd, legionclaw::interface::ControlAnalysis *analysis)
    {
      //列表中的控制器计算控制命令
      for (auto &controller : controller_list_)
      {
        // ADEBUG << "controller:" << controller->Name() << " processing ...";

        int64_t start_timestamp = TimeTool::Now2Us();
        controller->ComputeControlCommand(localization, chassis, trajectory, cmd, analysis);
        int64_t end_timestamp = TimeTool::Now2Us();
        int64_t control_calculate_time_ = (end_timestamp - start_timestamp);
        // ADEBUG << "control command calculate time:" << control_calculate_time_;
        // std::cout << "control_calculate_time_: "<<control_calculate_time_<<endl;

        //  ADEBUG << "controller: " << controller->Name()
        //         << " calculation time is: " << time_diff_ms << " ms.";
      }
      return Status::Ok();
    }

    Status ControllerAgent::Reset()
    {
      for (auto &controller : controller_list_)
      {
        // ADEBUG << "controller:" << controller->Name() << " reset...";
        controller->Reset();
      }
      return Status::Ok();
    }

  } // namespace control
} // namespace legionclaw
