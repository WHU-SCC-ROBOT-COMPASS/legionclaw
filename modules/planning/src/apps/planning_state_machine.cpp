/**
 * @file    planning_state_machine.cpp
 * @author  zdhy
 * @date    2022-08-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#include <sys/time.h>
#include <time.h>

#include <fstream>

#include "planning.h"

namespace legionclaw {
namespace planning {
bool Planning::PlanningStateMachineInit() {
  planning_sm_.reset(new state_machine::StateContext(
      FLAGS_planning_state_machine_file,
      FLAGS_planning_state_machine_log_enable, FLAGS_state_machine_log_dir,
      FLAGS_planning_state_machine_name));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE, "AutoDriving",
                            std::bind(&Planning::AutoDrivingStateUpdate, this,
                                      std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE, "HumanDriving",
                            std::bind(&Planning::HumanDrivingStateUpdate, this,
                                      std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "PlanningStart",
                            std::bind(&Planning::PlanningStartStateUpdate, this,
                                      std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "DrivingPlanning",
                            std::bind(&Planning::DrivingPlanningStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "LatticePlanner",
                            std::bind(&Planning::LatticePlannerStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ESlowStopPlan",
                            std::bind(&Planning::ESlowStopPlanStateUpdate, this,
                                      std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "EQuickStopPlan",
                            std::bind(&Planning::EQuickStopPlanStateUpdate,
                                      this, std::placeholders::_1, 0));

  // planning_sm_->SetCallback(state_machine::CallbackType::UPDATE, "Publish",
  //                     std::bind(&Planning::PublisherStateUpdate, this,
  //                                 std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingPlanning",
                            std::bind(&Planning::ParkingPlanningStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingPlanner",
                            std::bind(&Planning::ParkingPlannerStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingPublisher",
                            std::bind(&Planning::ParkingPublisherStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingOutPlanning",
                            std::bind(&Planning::ParkingOutPlanningStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingOutPlanner",
                            std::bind(&Planning::ParkingOutPlannerStateUpdate,
                                      this, std::placeholders::_1, 0));

  planning_sm_->SetCallback(state_machine::CallbackType::UPDATE,
                            "ParkingOutPublisher",
                            std::bind(&Planning::ParkingOutPublishStateUpdate,
                                      this, std::placeholders::_1, 0));

  // 初始化默认进入Auto驾驶模式进行过渡
  planning_sm_->NextState("Auto");
  return true;
}

bool Planning::AutoDrivingEnvInit() {
  parking_planner_.Reset();
  plan_status_ = Status(Status::ErrorCode::PLANNING_ERROR_NOT_READY);
  frame_.Init(planning_conf_.get());
  // TODO:注意线程与进程变量的读写，拨杆功能要拿出来
  reference_line_provider_->Init();
  lattice_planner_.Init(planning_conf_.get());
  // 清上次任务发出去的变量
  adc_trajectory_ = legionclaw::interface::ADCTrajectory();
  last_adc_trajectory_ = legionclaw::interface::ADCTrajectory();
  planning_cmd_ = legionclaw::interface::PlanningCmd();
  planning_analysis_ = legionclaw::interface::PlanningAnalysis();
  faults_ = legionclaw::interface::Faults();
  events_ = legionclaw::interface::Events();

  return true;
}

void Planning::AutoDrivingStateUpdate(const std::string &state_name,
                                      int state) {
  AutoDrivingEnvInit();
  if (driving_flag_ == DrivingFlag::HUMAN) {
    Reset();
    planning_sm_->NextState("Human");
    return;
  }
  if (driving_flag_ == DrivingFlag::DRIVING_INVALID) {
    if (task_status_ == TaskStatus::TASK_FAILED) {
      // TODO: 发出 No Plan 事件和状态，或者进入人工接管请求
      TrajectoryToTakeOver();
      AERROR << "<AutoDrivingState>: TASK_FAILED No Plan ! !";
    }
    if (task_status_ == TaskStatus::TASK_COMPLETE) {
      AERROR << "<AutoDrivingState>: Task Complete ! !";
    }
    Reset();
    planning_sm_->NextState("Human");
    return;
  }
  if (driving_flag_ == DrivingFlag::AUTO) {
    if (frame_status_ == Status::Ok()) {
      planning_sm_->NextState("Planning");
      return;
    } else {
      // TODO:发出frame not ok 的故障和事件，或者进入人工接管请求
      AERROR << "<AutoDrivingState>: Frame is not ok !";
    }
  }
}

void Planning::HumanDrivingStateUpdate(const std::string &state_name,
                                       int state) {
  // TODO:发出人工接管的事件
  if (driving_flag_ == DrivingFlag::AUTO ||
      driving_flag_ == DrivingFlag::ASSISTED) {
    planning_sm_->NextState("Auto");
  }
  return;
}

void Planning::PlanningStartStateUpdate(const std::string &state_name,
                                        int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("Auto");
    AERROR << "<PlanningStartState>: Frame is not ok !";
    return;
  }
  // 退出自动驾驶，或人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN) {
    TrajectoryToTakeOver();
    planning_sm_->NextState("Auto");
    return;
  }
  // 规划成功或结束check
  if (driving_flag_ == DrivingFlag::DRIVING_INVALID) {
    planning_sm_->NextState("Auto");
    return;
  }

  // 事件、故障初始化
  std::vector<legionclaw::interface::Event> events;
  legionclaw::interface::Event event;
  faults_.clear_faults();
  events_.clear_events();

  /****目前是在进入具体行泊场景后便指定默认的规划器****/
  // 进入行车规划
  if (planning_flag_ ==
      PlanningFlag::DRIVING) {  // TODO:规划模式的条件选择改为上层下发
    planner_flag_ = PlannerFlag::LATTICE_PLANNER;
    planning_sm_->NextState("Driving");
    return;
  }

  // 进入泊车规划
  if ((adc_trajectory_.behaviour_lat_state() ==
       legionclaw::interface::ADCTrajectory::BehaviourLatState::
           PARKING_FINISH_STATE) &&
      (local_view_.chassis_.moving_status() == MovingStatus::STATIONARY) &&
      (local_view_.chassis_.epb_level() == EPBLevel::APPLIED) &&
      (parking_out_direction_ != ParkingManager::OutDirection::DirectionNone)) {
    // 泊出完成
    event.set_code(6);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    faults_.clear_faults();
    events_.clear_events();
    ResetFlag();
    planning_sm_->NextState("Auto");
    return;
  }

  if ((adc_trajectory_.behaviour_lat_state() ==
       legionclaw::interface::ADCTrajectory::BehaviourLatState::
           PARKING_FINISH_STATE) &&
      (local_view_.chassis_.moving_status() == MovingStatus::STATIONARY) &&
      (local_view_.chassis_.epb_level() == EPBLevel::APPLIED)) {
    // 泊车完成
    event.set_code(3);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    faults_.clear_faults();
    events_.clear_events();
    ResetFlag();
    planning_sm_->NextState("Auto");
    return;
  }
  if (planning_flag_ == PlanningFlag::PARKING) {
    planner_flag_ = PlannerFlag::PARKING_PLANNER;
    parking_planner_.parking_manager_.SetOutDirection(parking_out_direction_);
    planning_sm_->NextState("Parking");
    return;
  }
  if (parking_out_direction_ != ParkingManager::OutDirection::DirectionNone) {
    parking_planner_.parking_manager_.SetOutDirection(parking_out_direction_);
    planner_flag_ = PlannerFlag::PARKINGOUT_PLANNER;
    planning_sm_->NextState("ParkingOut");
    return;
  }
}

void Planning::DrivingPlanningStateUpdate(const std::string &state_name,
                                          int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    frame_.SetTerminalType(-1);
    planning_sm_->NextState("Planning");
    AERROR << "<DrivingPlanningState>: Frame is not ok !";
    return;
  }
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN) {
    TrajectoryToTakeOver();
    frame_.SetTerminalType(-1);
    planning_sm_->NextState("Planning");
    return;
  }
  // 规划成功或结束check
  if (driving_flag_ == DrivingFlag::DRIVING_INVALID) {
    frame_.SetTerminalType(-1);
    planning_sm_->NextState("Planning");
    return;
  }
  // lattice planer
  if (planner_flag_ == PlannerFlag::LATTICE_PLANNER) {
    AINFO << "Start LatticePlan.";
    planning_sm_->NextState("LatticePlan");
    return;
  }
  // EM

  // 紧急情况1
  if (planner_flag_ == PlannerFlag::E_SLOW_STOP_PLAN) {
    frame_.SetTerminalType(-1);
    planning_sm_->NextState("EmergencyIPlan");
    return;
  }
  // 紧急情况2
  if (planner_flag_ == PlannerFlag::E_QUICK_STOP_PLAN) {
    frame_.SetTerminalType(-1);
    planning_sm_->NextState("EmergencyIIPlan");
    return;
  }
}

void Planning::LatticePlannerStateUpdate(const std::string &state_name,
                                         int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("Driving");
    AERROR << "<LatticePlannerState>: Frame is not ok !";
    return;
  }
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN) {
    faults_.clear_faults();
    events_.clear_events();
    TrajectoryToTakeOver();
    planning_sm_->NextState("Driving");
    return;
  }
  // 事件、故障初始化
  std::vector<legionclaw::interface::Event> events;
  legionclaw::interface::Event event;
  faults_.clear_faults();
  events_.clear_events();
  // manual lc confirm
  bool is_manual_lc = false;
  if (frame_.LaneChangeDirection() !=
    common::Direction::DIR_INVALID) {
   is_manual_lc = true;
  }
  // 规划器计算
  // AERROR << "BEFORE-------------TRAJECTORY PATH LENGTH : " <<
  // adc_trajectory_.total_path_length();
  plan_status_ = lattice_planner_.Plan(mutable_frame(), &adc_trajectory_,
                                       &planning_analysis_, &planning_cmd_,
                                       &trajectory_array_);
  //功能1：参考线平滑耗时，可多等待几个周期，不必立马退出自动
  //功能2：支持先点自动驾驶，再下发全局导航，方便日常调试，不需要关心自动驾驶指令和全局导航的顺序
  if (plan_status_.ToString()=="ReferenceLines is Null."){
    AERROR << "ReferenceLines is Null，please wait a moment.";
    return;
  }

  // adc_trajectory_.total_path_length();
  if (plan_status_ != Status::Ok()) {
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    task_status_ = TaskStatus::TASK_FAILED;
    planning_flag_ = PlanningFlag::PLANNING_INVALID;
    waitting_state_ = 1;
    //不可行车
    event.set_code(7);
    event.set_reason("规划状态不满足，请求退出自动驾驶");
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    AERROR << "规划状态不满足，请求退出自动驾驶.";
    planning_sm_->NextState("Driving");
    return;
  } else {
    if (adc_trajectory_.behaviour_lat_state() ==
            legionclaw::interface::ADCTrajectory::BehaviourLatState::STATION_ARRIVED_STATE &&
        (local_view_.chassis_.moving_status() == MovingStatus::STATIONARY) &&
        (local_view_.chassis_.epb_level() == EPBLevel::APPLIED)) {
      driving_flag_ = DrivingFlag::DRIVING_INVALID;
      task_status_ = TaskStatus::TASK_COMPLETE;
      planning_flag_ = PlanningFlag::PLANNING_INVALID;
      adc_trajectory_.set_behaviour_lat_state(
          legionclaw::interface::ADCTrajectory::BehaviourLatState::
              LAT_NOT_ACTIVE_STATE);  // TODO:进入完成状态
      adc_trajectory_.set_behaviour_lon_state(
          legionclaw::interface::ADCTrajectory::BehaviourLonState::
              LON_NOT_ACTIVE_STATE);
      waitting_state_ = 1;
      //行车完成
      event.set_code(9);
      events.push_back(event);
      events_.set_events(events);
      PublishEvents(events_);
      planning_sm_->NextState("Driving");
      return;
    }

    // 需要接管
    if ((adc_trajectory_.total_path_length() >
         planning_conf_->emergency_lower_limit_length()) &&
        (adc_trajectory_.total_path_length() <
         frame_.VehicleState().speed *
         planning_conf_->emergency_time_interval())) {
      planner_flag_ = PlannerFlag::E_SLOW_STOP_PLAN;
      //发出退出自动驾驶事件
      AERROR << "轨迹短，请求退出自动驾驶.";
      event.set_code(10);
      event.set_reason("轨迹短，请求退出自动驾驶");
      events.push_back(event);
      events_.set_events(events);
      PublishEvents(events_);
      planning_sm_->NextState("Driving");
      return;
    } else if ((adc_trajectory_.total_path_length() > 0.1) &&
               (adc_trajectory_.total_path_length() <
                planning_conf_->emergency_lower_limit_length())) {
      planner_flag_ = PlannerFlag::E_QUICK_STOP_PLAN;
      //发出退出自动驾驶事件
      AERROR << "轨迹太短，请求退出自动驾驶.";
      event.set_code(10);
      event.set_reason("轨迹太短，请求退出自动驾驶");
      events.push_back(event);
      events_.set_events(events);
      PublishEvents(events_);
      planning_sm_->NextState("Driving");
      return;
    }

    //可以行车
    event.set_code(8);
    events.push_back(event);
    events_.set_events(events);
    //拨杆换道场景
    if (is_manual_lc == true) {
      if (frame_.LaneChangeDirection() ==
          common::Direction::DIR_INVALID &&
          frame_.LaneChangeState() ==
          Lane_Change_State::LANE_CHANGE_INVALID) {
        legionclaw::interface::Event event_lane_change;
        event_lane_change.set_code(20);
        event_lane_change.set_reason("拨杆驳回");
        events.push_back(event_lane_change);
        events_.set_events(events);
      } else {
        legionclaw::interface::Event event_lane_change;
        if (frame_.LaneChangeState() ==
              Lane_Change_State::LANE_CHANGING){
          if (frame_.LaneChangeDirection() ==
            common::Direction::LEFT){
            event_lane_change.set_code(21);
            event_lane_change.set_reason("执行向左拨杆换道");
            events.push_back(event_lane_change);
            events_.set_events(events);
          } else if (frame_.LaneChangeDirection() ==
            common::Direction::RIGHT) {
            event_lane_change.set_code(22);
            event_lane_change.set_reason("执行向右拨杆换道");
            events.push_back(event_lane_change);
            events_.set_events(events);
          }
        } else if (frame_.LaneChangeState() ==
              Lane_Change_State::LANE_CHANGE_COMPLETE){
          event_lane_change.set_code(23);
          event_lane_change.set_reason("拨杆换道完成");
          events.push_back(event_lane_change);
          events_.set_events(events);
        }
      }
    } else {
      legionclaw::interface::Event event_lane_change;
      if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_PRE_STATE){
        event_lane_change.set_code(30);
        event_lane_change.set_reason("准备向左换道");
        events.push_back(event_lane_change);
        events_.set_events(events);
      } else if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_LEFT_STATE) {
        event_lane_change.set_code(31);
        event_lane_change.set_reason("执行向左换道");
        events.push_back(event_lane_change);
        events_.set_events(events);
      } else if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_PRE_STATE) {
        event_lane_change.set_code(32);
        event_lane_change.set_reason("准备向右换道");
        events.push_back(event_lane_change);
        events_.set_events(events);
      } else if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::LANE_CHANGE_RIGHT_STATE) {
        event_lane_change.set_code(33);
        event_lane_change.set_reason("执行向右换道");
        events.push_back(event_lane_change);
        events_.set_events(events);
      } else if (frame_.LaneChangeState() ==
          Lane_Change_State::LANE_CHANGE_COMPLETE){
        event_lane_change.set_code(34);
        event_lane_change.set_reason("换道完成");
        events.push_back(event_lane_change);
        events_.set_events(events);
      }
    }

    legionclaw::interface::Event event_stop;
    if(adc_trajectory_.behaviour_lon_state() ==
      legionclaw::interface::ADCTrajectory::BehaviourLonState::STOP &&
      local_view_.chassis_.speed_mps() < planning_conf_->threshold_static_speed()){
        event_stop.set_code(40);
        event_stop.set_reason("停障中");
        events.push_back(event_stop);
        events_.set_events(events);
    }
    PublishEvents(events_);
    return;
  }
}

void Planning::ESlowStopPlanStateUpdate(const std::string &state_name,
                                        int state) {
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN) {
    faults_.clear_faults();
    events_.clear_events();
    TrajectoryToTakeOver();
    planning_sm_->NextState("Driving");
    return;
  }
  // a. 使用上一帧轨迹的最后一个点作为紧急停车点，暂时采用固定减速度
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(adc_trajectory_);
  double current_speed = frame_.VehicleState().speed;
  auto point = frame_.VehicleState().pose;
  double match_distance;
  int goal_index_global = MapMatcher::QueryNearestPointWithBuffer(
      frame_.LastPlanningTrajectory().trajectory_points(),
      Vec2d(point.x(), point.y()), point.theta(), 1.0e-6, match_distance);
  double stop_distance = MapMatcher::GetExactDistanceOnTrajectory(
      frame_.LastPlanningTrajectory().trajectory_points(), goal_index_global,
      frame_.LastPlanningTrajectorySize() - 1);
  if (stop_distance <= 0) stop_distance = 0.1;
  double accel = -0.5 * current_speed * current_speed / stop_distance;
  lon_info_provider_.ProvideTrajectoryLongitudinalInfo(current_speed, accel, 0,
                                                       *adc_trajectory_.mutable_trajectory_points());
  adc_trajectory_.set_behaviour_lat_state(
      legionclaw::interface::ADCTrajectory::BehaviourLatState::EMERGENCY_STATE);
  adc_trajectory_.set_driving_mode(common::DrivingMode::COMPLETE_AUTO_DRIVE);
  // b. 发出紧急事件和状态
  events_.clear_events();
  std::vector<legionclaw::interface::Event> events;
  legionclaw::interface::Event event_eslow_stop;
  event_eslow_stop.set_code(10);
  event_eslow_stop.set_reason("轨迹短，请求退出自动驾驶");
  events.push_back(event_eslow_stop);
  events_.set_events(events);
  PublishEvents(events_);

  if (stop_distance < planning_conf_->emergency_lower_limit_length()) {
    events_.clear_events();
    planner_flag_ = PlannerFlag::E_QUICK_STOP_PLAN;
    legionclaw::interface::Event event_equick_stop;
    event_equick_stop.set_code(10);
    event_equick_stop.set_reason("轨迹太短，请求退出自动驾驶");
    events.push_back(event_equick_stop);
    events_.set_events(events);
    PublishEvents(events_);
    planning_sm_->NextState("Driving");
    return;
  }
}

void Planning::EQuickStopPlanStateUpdate(const std::string &state_name,
                                         int state) {
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN) {
    faults_.clear_faults();
    events_.clear_events();
    TrajectoryToTakeOver();
    planning_sm_->NextState("Driving");
    return;
  }
  // TODO:下发给控制E stop的信号；轨迹上其他的信息的赋值意义待确认
  EStop estop;
  estop.set_is_estop(true);
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  legionclaw::interface::TrajectoryPoint trajectory_point;
  legionclaw::interface::PathPoint utm_pose;
  utm_pose.set_x(local_view_.location_.utm_position().x());
  utm_pose.set_y(local_view_.location_.utm_position().y());
  utm_pose.set_theta(local_view_.location_.heading());
  trajectory_point.set_path_point(utm_pose);
  trajectory_points.push_back(trajectory_point);
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(adc_trajectory_);
  adc_trajectory_.set_behaviour_lat_state(
      legionclaw::interface::ADCTrajectory::BehaviourLatState::EMERGENCY_STATE);
  adc_trajectory_.set_estop(estop);
  adc_trajectory_.set_trajectory_points(trajectory_points);
  adc_trajectory_.set_driving_mode(common::DrivingMode::COMPLETE_MANUAL);
  // b. 发出紧急事件和状态
  events_.clear_events();
  std::vector<legionclaw::interface::Event> events;
  legionclaw::interface::Event event_equick_stop;
  event_equick_stop.set_code(11);
  event_equick_stop.set_reason("轨迹太短，请求退出自动驾驶");
  events.push_back(event_equick_stop);
  driving_flag_ = DrivingFlag::DRIVING_INVALID;
  task_status_ = TaskStatus::TASK_FAILED;
  planning_flag_ = PlanningFlag::PLANNING_INVALID;

  legionclaw::interface::Event event;
  event.set_code(7);
  event.set_reason("规划状态不满足，请求退出自动驾驶");
  events.push_back(event);
  events_.set_events(events);
  PublishEvents(events_);
  planning_sm_->NextState("Driving");
}

void Planning::ParkingPlanningStateUpdate(const std::string &state_name,
                                          int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("Planning");
    AERROR << "<Parking>: Frame is not ok !";
    return;
  }
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN || waitting_state_ == 1) {
    planning_sm_->NextState("Planning");
    return;
  }
  // if (emergency_state_ == 1) {
  //     planning_sm_->NextState("E");
  //     return;
  // }
  // 规划器
  if (planner_flag_ == PlannerFlag::PARKING_PLANNER) {
    planning_sm_->NextState("ParkingPlan");
    return;
  }
}

void Planning::ParkingPlannerStateUpdate(const std::string &state_name,
                                         int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("Parking");
    AERROR << "<ParkingPlanner>: Frame is not ok !";
    return;
  }
  // 人工接管check
  // TODO:waitting_state_适用性check
  if (driving_flag_ == DrivingFlag::HUMAN || waitting_state_ == 1) {
    AfterHumanDriving();
    faults_.clear_faults();
    events_.clear_events();
    if (!plan_status_.ok()) {
      ChangeDrivingModeAndBehaviourLatState();
    }
    ResetFlag();
    // change_to_human_driving_ = 1;
    planning_sm_->NextState("Parking");
    return;
  }

  std::vector<legionclaw::interface::Event> events;
  events.clear();
  legionclaw::interface::Event event;
  if (local_view_.parking_info_.parking_status() &&
      (adc_trajectory_.behaviour_lat_state() !=
       legionclaw::interface::ADCTrajectory::BehaviourLatState::
           PARKING_FINISH_STATE)) {
    plan_status_ = parking_planner_.Plan(mutable_frame(), &adc_trajectory_,
                                         &planning_analysis_, &planning_cmd_,
                                         &trajectory_array_);
  }

  if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::
              EMERGENCY_STATE ||
      adc_trajectory_.estop().is_estop()) {
    event.set_code(0);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    // emergency_state_ = 1;
    planning_sm_->NextState("Parking");
    return;
  }

  if (planning_flag_ == PlanningFlag::PARKING_STOP) {
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    local_view_.parking_info_.set_parking_status(ParkingStatus::PARKING_DISENABLE);
    parking_planner_.Reset();
    planning_flag_ = PlanningFlag::PLANNING_INVALID;

    ChangeDrivingModeAndBehaviourLatState();
    waitting_state_ = 1;
    planning_sm_->NextState("Parking");
    return;
  }

  if (plan_status_ != Status(Status::ErrorCode::OK)) {
    stop_info_.clear_stop_points();
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    // 不可以泊车
    event.set_code(1);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    local_view_.parking_info_.set_parking_status(ParkingStatus::PARKING_DISENABLE);
    waitting_state_ = 1;
    planning_sm_->NextState("Parking");
    return;
  } else {
    // 可以泊车
    event.set_code(2);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    // planning_sm_->NextState("ParkingPublisher");
    return;
  }
}

void Planning::ParkingPublisherStateUpdate(const std::string &state_name,
                                           int state) {}

void Planning::ParkingOutPlanningStateUpdate(const std::string &state_name,
                                             int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("Planning");
    AERROR << "<ParkingOut>: Frame is not ok !";
    return;
  }
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN || waitting_state_ == 1) {
    planning_sm_->NextState("Planning");
    return;
  }
  // if (emergency_state_ == 1) {
  //     planning_sm_->NextState("E");
  //     return;
  // }
  // 规划器
  if (planner_flag_ == PlannerFlag::PARKING_PLANNER) {
    planning_sm_->NextState("ParkingOutPlan");
    return;
  }
}

void Planning::ParkingOutPlannerStateUpdate(const std::string &state_name,
                                            int state) {
  // Env check
  if (frame_status_ != Status::Ok()) {
    planning_sm_->NextState("ParkingOut");
    AERROR << "<ParkingOutPlanner>: Frame is not ok !";
    return;
  }
  // 人工接管check
  if (driving_flag_ == DrivingFlag::HUMAN || waitting_state_ == 1) {
    AfterHumanDriving();
    faults_.clear_faults();
    events_.clear_events();
    parking_out_direction_ = ParkingManager::OutDirection::DirectionNone;
    if (!plan_status_.ok()) {
      ChangeDrivingModeAndBehaviourLatState();
    }
    ResetFlag();
    // change_to_human_driving_ = 1;
    planning_sm_->NextState("ParkingOut");
    return;
  }
  // 事件
  std::vector<legionclaw::interface::Event> events;
  events.clear();
  legionclaw::interface::Event event;

  if ((parking_out_direction_ != ParkingManager::OutDirection::DirectionNone) &&
      (adc_trajectory_.behaviour_lat_state() !=
       legionclaw::interface::ADCTrajectory::BehaviourLatState::
           PARKING_FINISH_STATE)) {
  }

  if (adc_trajectory_.behaviour_lat_state() ==
          legionclaw::interface::ADCTrajectory::BehaviourLatState::
              EMERGENCY_STATE ||
      adc_trajectory_.estop().is_estop()) {
    event.set_code(0);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    emergency_state_ = 1;
    planning_sm_->NextState("ParkingOut");
    return;
  }

  if (planning_flag_ == PlanningFlag::PARKING_OUT_STOP) {
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    parking_out_direction_ = ParkingManager::OutDirection::DirectionNone;
    planning_flag_ = PlanningFlag::PLANNING_INVALID;
    ChangeDrivingModeAndBehaviourLatState();
    waitting_state_ = 1;
    planning_sm_->NextState("ParkingOut");
    return;
  }

  if (plan_status_ != Status(Status::ErrorCode::OK)) {
    stop_info_.clear_stop_points();
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    // 不可以泊出时
    event.set_code(4);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    parking_out_direction_ = ParkingManager::OutDirection::DirectionNone;
    waitting_state_ = 1;
    planning_sm_->NextState("ParkingOut");
    return;
  } else {
    // 可以泊出时
    event.set_code(5);
    events.push_back(event);
    events_.set_events(events);
    PublishEvents(events_);
    // planning_sm_->NextState("ParkingOutPublisher");
    return;
  }
}

void Planning::ParkingOutPublishStateUpdate(const std::string &state_name,
                                            int state) {}

void Planning::AfterHumanDriving() {
  adc_trajectory_.set_driving_mode(
      (legionclaw::common::DrivingMode)COMPLETE_MANUAL);
  adc_trajectory_.set_behaviour_lat_state(
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LAT_NOT_ACTIVE_STATE);
  adc_trajectory_.set_behaviour_lon_state(
      legionclaw::interface::ADCTrajectory::BehaviourLonState::LON_NOT_ACTIVE_STATE);
}

void Planning::ChangeDrivingModeAndBehaviourLatState() {
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  legionclaw::interface::TrajectoryPoint trajectory_point;
  legionclaw::interface::PathPoint utm_pose;
  utm_pose.set_x(local_view_.location_.utm_position().x());
  utm_pose.set_y(local_view_.location_.utm_position().y());
  utm_pose.set_theta(local_view_.location_.heading());
  trajectory_point.set_path_point(utm_pose);
  trajectory_points.push_back(trajectory_point);

  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(adc_trajectory_)
  adc_trajectory_.set_driving_mode(
      (legionclaw::common::DrivingMode)COMPLETE_MANUAL);
  adc_trajectory_.set_behaviour_lat_state(
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LAT_NOT_ACTIVE_STATE);
  adc_trajectory_.set_behaviour_lon_state(
      legionclaw::interface::ADCTrajectory::BehaviourLonState::LON_NOT_ACTIVE_STATE);
  adc_trajectory_.set_trajectory_points(trajectory_points);
  PublishADCTrajectory(adc_trajectory_);
}

void Planning::TrajectoryToTakeOver() {
  std::vector<legionclaw::interface::TrajectoryPoint> trajectory_points;
  legionclaw::interface::TrajectoryPoint trajectory_point;
  for (auto point : last_adc_trajectory_.trajectory_points()) {
    trajectory_point.set_path_point(point.path_point());
    trajectory_point.set_v(0);
    trajectory_points.push_back(trajectory_point);
  }

  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(adc_trajectory_)
  adc_trajectory_.set_driving_mode(
      (legionclaw::common::DrivingMode)COMPLETE_MANUAL);
  adc_trajectory_.set_behaviour_lat_state(
      legionclaw::interface::ADCTrajectory::BehaviourLatState::LAT_NOT_ACTIVE_STATE);
  adc_trajectory_.set_behaviour_lon_state(
      legionclaw::interface::ADCTrajectory::BehaviourLonState::LON_NOT_ACTIVE_STATE);
  adc_trajectory_.set_trajectory_points(trajectory_points);
  PublishADCTrajectory(adc_trajectory_);

  lattice_planner_.Reset();
}

}  // namespace planning
}  // namespace legionclaw
