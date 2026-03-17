#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "controller/controller.h"
#include "controller/pid/pid_controller.h"
#include "modules/control/src/controller/lon_speed_controller/lon_speed_gear_state_machine_debug.hpp"
#include "modules/control/src/controller/lon_speed_controller/lon_speed_brake_state_machine_debug.hpp"
#include "modules/control/src/controller/lon_speed_controller/simple_longitudinal_debug.hpp"
#include "modules/control/src/common/trajectory_analyzer/trajectory_analyzer.h"
#include "modules/control/src/common/local_view.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/state_machine/state_context.hpp"


using legionclaw::common::state_machine::StateContext;

namespace legionclaw 
{
  namespace control 
  {
    class LonSpeedController : public Controller
    {
      public:
      LonSpeedController();

      virtual ~LonSpeedController();

      legionclaw::common::Status Init(std::shared_ptr<DependencyInjector> injector,const ControlConf *control_conf) override;

      legionclaw::common::Status ComputeControlCommand(const legionclaw::interface::LocalizationEstimate *localization,
                                                   const legionclaw::interface::Chassis *chassis,
                                                   const legionclaw::interface::ADCTrajectory *trajectory,
                                                   legionclaw::interface::ControlCommand *cmd,
                                                   legionclaw::interface::ControlAnalysis *analysis) override;

      legionclaw::common::Status Reset() override;

      void Stop() override;

      std::string Name() const override;

      private:

      protected:
      void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                     const legionclaw::interface::LocalizationEstimate *localization,
                                     const legionclaw::interface::Chassis *chassis, 
                                     const double preview_time,
                                     const double ts, 
                                     SimpleLongitudinalDebug *debug);

      void GetPathRemain(SimpleLongitudinalDebug *debug);

      void SetAnalysis(const legionclaw::interface::ControlCommand *cmd,
                       const SimpleLongitudinalDebug *debug,
                       const legionclaw::interface::Chassis *chassis,
                       legionclaw::interface::ControlAnalysis *analysis);

      void GearTransitionInit();
      void GearInitEntry(const std::string &state_name, int state);
      void GearInitUpdate(const std::string &state_name, int state);
      void GearNEntry(const std::string &state_name, int state);
      void GearNUpdate(const std::string &state_name, int state);
      void GearNExit(const std::string &state_name, int state);
      // void GearPEntry(const std::string &state_name, int state);
      // void GearPUpdate(const std::string &state_name, int state);
      // void GearPExit(const std::string &state_name, int state);
      void GearDEntry(const std::string &state_name, int state);
      void GearDUpdate(const std::string &state_name, int state);
      void GearDExit(const std::string &state_name, int state);
      void GearREntry(const std::string &state_name, int state);
      void GearRUpdate(const std::string &state_name, int state);
      void GearRExit(const std::string &state_name, int state);
      void EPBInitEntry(const std::string &state_name, int state);
      void EPBInitUpdate(const std::string &state_name, int state);
      void EPBAppliedEntry(const std::string &state_name, int state);
      void EPBAppliedUpdate(const std::string &state_name, int state);
      void EPBAppliedExit(const std::string &state_name, int state);
      void EPBReleaseEntry(const std::string &state_name, int state);
      void EPBReleaseUpdate(const std::string &state_name, int state);
      void EPBReleaseExit(const std::string &state_name, int state);
      void GearTransition(legionclaw::interface::ControlCommand *cmd);

      void BrakeStatusInit();
      void BrakeInitEntry(const std::string &state_name, int state);
      void BrakeInitUpdate(const std::string &state_name, int state);
      void BrakeOnEntry(const std::string &state_name, int state);
      void BrakeOnUpdate(const std::string &state_name, int state);
      void BrakeOnExit(const std::string &state_name, int state);
      void BrakeOffEntry(const std::string &state_name, int state);
      void BrakeOffUpdate(const std::string &state_name, int state);
      void BrakeOffExit(const std::string &state_name, int state);
      void BrakeTransition(legionclaw::interface::ControlCommand *cmd);

      void SetEventFlag(std::string key, bool value);
      bool IsEventFlagTrue(std::string key);

      VehicleParam vehicle_param_;
      GearStateMachineDebug gear_sm_debug_;
      BrakeStateMachineDebug brake_sm_debug_;

      const legionclaw::interface::LocalizationEstimate *localization_ = nullptr;
      const legionclaw::interface::Chassis *chassis_ = nullptr;
      const legionclaw::interface::ADCTrajectory *trajectory_message_ = nullptr;
      const ControlConf *control_conf_ = nullptr;
      std::string name_;

      std::shared_ptr<SimpleLongitudinalDebug> simple_longitudinal_debug_;
      std::shared_ptr<DependencyInjector> injector_; 
      std::shared_ptr<state_machine::StateContext> gear_sm_;
      std::shared_ptr<state_machine::StateContext> epb_sm_;
      std::shared_ptr<state_machine::StateContext> brake_sm_;
      std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;
      std::map<std::string, bool> eventflags_;

      bool controller_initialized_ = false;
      bool is_stopped_first_hit_ = false;
      bool is_stopped_ = false;
      bool use_station_pid_ = false;

      //周期
      double ts_ = 0.02;
      //预测时间
      double preview_time_;
      //启动速度死区
      double starting_speed_;
      //速度限制
      double speed_limit_;
      //急停时车辆加速度阈值
      double acceleration_threshold_when_brake_;

      int64_t last_time_vehicle_is_stopped_or_not_;
    };
  }
}