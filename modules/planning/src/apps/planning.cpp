/**
 * @file    planning.cpp
 * @author  zdhy
 * @date    2021-09-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#include "modules/planning/src/apps/planning.h"

#include <sys/time.h>
#include <time.h>

#include <fstream>

#include "modules/common/file/file.h"
#include "modules/planning/src/proto/planning_conf.pb.h"

namespace legionclaw {
namespace planning {

void Planning::Init() {
  // step1 初始化状态设置为false
  {
    is_init_ = false;
    driving_flag_ = DrivingFlag::DRIVING_INVALID;
    planning_flag_ = PlanningFlag::PLANNING_INVALID;
  }

  // step2 变量初始化
  {
    VariableInit();
  }

  // step3 配置文件初始化 - 使用 protobuf 从 JSON 读取
  {
    planning_conf_ = std::make_shared<legionclaw::planning::PlanningConf>();
    if (GetProtoFromJsonFile(FLAGS_planning_config_file,
                             planning_conf_.get()) == false) {
      AERROR << "GetProtoFrom planning config file failed: "
             << FLAGS_planning_config_file;
      return;
    }
    AINFO << "Successfully loaded planning config from: " << config_file_path_;

    legionclaw::planning::LatticePlannerConf lattice_planner_conf;
    if (GetProtoFromJsonFile(FLAGS_lattice_planner_config_file,
                             &lattice_planner_conf) == false) {
      AERROR << "GetProtoFrom lattice_planner config file failed: "
             << FLAGS_lattice_planner_config_file;
      return;
    }
    planning_conf_->mutable_lattice_planner_conf()->CopyFrom(
        lattice_planner_conf);
    legionclaw::planning::ParkingConf parking_conf;
    if (GetProtoFromJsonFile(FLAGS_parking_config_file, &parking_conf) ==
        false) {
      AERROR << "GetProtoFrom parking config file failed: "
             << FLAGS_parking_config_file;
      return;
    }
    planning_conf_->mutable_parking_conf()->CopyFrom(parking_conf);

    // 加载 vehicle param 配置
    legionclaw::interface::VehicleParam vehicle_param;
    if (GetProtoFromJsonFile(FLAGS_vehicle_config_path, &vehicle_param) ==
        false) {
      AERROR << "GetProtoFrom vehicle_param config file failed: "
             << FLAGS_vehicle_config_path;
      return;
    }
    planning_conf_->mutable_vehicle_param()->CopyFrom(vehicle_param);
  }

  // step4 日志初始化 - 需要从 protobuf 配置中提取日志配置
  {
    LOGGING_INIT(planning_conf_)
  }

  // step5 IPC初始化 - 需要从 protobuf 配置中提取消息配置
  {
    MESSAGE_INIT(planning_conf_)
  }

  // step6 读取其他配置
  {
    produce_planning_command_duration_ =
        planning_conf_->produce_planning_command_duration();
    publish_planning_command_duration_ =
        planning_conf_->publish_planning_command_duration();
    vehicle_param_file_path =
        "../../common/data/vehicle_param/vehicle_param.json";
    driving_state_machine_file = "./conf/planning/driving/driving.json";
    parking_state_machine_file = "./conf/planning/parking.json";
  }

  // 调试日志初始化
  {
    enable_log_debug_ = planning_conf_->logging_data_enable();
    if (enable_log_debug_) {
      trajectory_point_log_file_.open(GetLogFileName());
      trajectory_point_log_file_ << std::fixed;
      trajectory_point_log_file_ << std::setprecision(6);
      WriteHeaders(trajectory_point_log_file_);
    }
  }

  // step6 故障码初始化
  // FaultMonitorInit();

  // instantiate reference line provider
  reference_line_provider_ =
      std::make_unique<ReferenceLineProvider>(*planning_conf_);
  reference_line_provider_->Start();

  // step7 算法初始化
  {
    // planner
    PlanningStateMachineInit();
    frame_.Init(planning_conf_.get());
    lattice_planner_.Init(planning_conf_.get());
    parking_planner_.Init(planning_conf_.get());
  }

  // step8 定时器和线程初始化
  {
    status_detect_duration_ = planning_conf_->status().status_detect_duration();

    // TimerManager<Planning>::AddTimer(produce_planning_command_duration_,
    // &Planning::ComputePlanningCommandOnTimer, this);

    // TimerManager<Planning>::AddTimer(status_detect_duration_,
    // &Planning::StatusDetectOnTimer, this);

    ad_timer_manager_ = std::make_shared<ADTimerManager<Planning, void>>();
    task_compute_ =
        std::make_shared<WheelTimer<Planning, void>>(ad_timer_manager_);
    task_publish_ =
        std::make_shared<WheelTimer<Planning, void>>(ad_timer_manager_);

    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create can task_thread_ thread.";
      return;
    }
  }

  // step9 初始化状态为true
  {
    is_init_ = true;
  }
  //TODO:临时默认激活
  TaskActivate();
}

void Planning::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
  CloseLogFile();
}

void Planning::VariableInit() {
  injector_ = std::make_shared<DependencyInjector>();
  planning_sm_ = nullptr;
  plan_status_ = Status(Status::ErrorCode::PLANNING_ERROR_NOT_READY,
                        "PLANNING_ERROR_NOT_READY.");
}

void Planning::FaultMonitorInit() {
  legionclaw::interface::FaultCodeCallback sendheart_callback_func =
      std::bind(&Planning::PublishFaults, this);
  legionclaw::interface::FaultCodeCallback fault_callback_func = nullptr;
  FAULTCODE_INIT("../../common/data/faults/faults.json", "planning",
                 faultcodeset_, sendheart_callback_func, fault_callback_func)
}

void Planning::TaskActivate() {
  if (is_init_ == false) {
    return;
  }
  // IPC激活
  MessagesActivate();
  if (function_activation_) {
    return;
  }
  task_compute_->AddTimer(produce_planning_command_duration_,
                          &Planning::ComputePlanningCommandOnTimer, this);
  task_publish_->AddTimer(publish_planning_command_duration_,
                          &Planning::PublishPlanningCommandOnTimer, this);
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout << "===================function activate=================="
            << "\n";
  function_activation_ = true;
  return;
}

void Planning::TaskStop() {
  if (is_init_ == false) {
    return;
  }

  // IPC去激活
  MessagesDeActivate();
  if (function_activation_ == false) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_compute_->Stop();
    task_publish_->Stop();
    PlanningStateMachineInit();
    plan_status_ = Status(Status::ErrorCode::PLANNING_ERROR_NOT_READY,
                          "PLANNING_ERROR_NOT_READY.");
    adc_trajectory_ = legionclaw::interface::ADCTrajectory();
    routing_response_ = legionclaw::interface::RoutingResponse();
    traffic_events_ = legionclaw::interface::TrafficEvents();
    guide_info_ = legionclaw::interface::GuideInfo();
    parking_info_ = legionclaw::interface::ParkingInfo();
    stop_info_ = legionclaw::interface::StopInfo();
    traffic_light_msg_ = legionclaw::interface::TrafficLightMsg();
    location_ = legionclaw::interface::Location();
    prediction_obstacles_ = legionclaw::interface::PredictionObstacles();
    lane_list_ = legionclaw::interface::LaneList();
    chassis_ = legionclaw::interface::Chassis();
    sotif_monitor_result_ = legionclaw::interface::SotifMonitorResult();
    obu_cmd_msg_ = legionclaw::interface::ObuCmdMsg();
    drivable_region_ = legionclaw::interface::DrivableRegion();
    parking_out_info_ = legionclaw::interface::ParkingOutInfo();
    Reset();
  }

  std::cout << "******************function stop***************" << "\n";
  function_activation_ = false;
  return;
}

void Planning::Print() {}

void Planning::Log() {
  if (enable_log_debug_ && trajectory_point_log_file_.is_open()) {
    std::vector<interface::TrajectoryPoint> trajectory_points;
    adc_trajectory_.trajectory_points(trajectory_points);
    for (auto& trajectory_point : trajectory_points) {
      stringstream sstream;
      sstream << trajectory_point.path_point().s() << ","
              << std::setprecision(8) << trajectory_point.path_point().x()
              << "," << std::setprecision(8)
              << trajectory_point.path_point().y() << ","
              << math::R2D(trajectory_point.path_point().theta()) << ","
              << trajectory_point.path_point().kappa() << ","
              << trajectory_point.relative_time() << ","
              << trajectory_point.v() * 3.6 << "," << trajectory_point.a()
              << ",";
      trajectory_point_log_file_ << sstream.str() << "\n";
    }
  }
}

std::string Planning::GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "./log/trajectory_points_log__%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void Planning::WriteHeaders(std::ofstream& file_stream) {
  file_stream << "s,"
              << "x,"
              << "y,"
              << "theta(deg),"
              << "kappa,"
              << "time,"
              << "v(km/h),"
              << "a(m/s/s)," << "\n";
}

void Planning::CloseLogFile() {
  if (enable_log_debug_ && trajectory_point_log_file_.is_open()) {
    trajectory_point_log_file_.close();
  }
}

void Planning::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<Planning>> message_manager) {
  message_manager_.insert(
      std::pair<std::string, std::shared_ptr<MessageManager<Planning>>>(
          name, message_manager));
}

void Planning::PublishADCTrajectory(
    legionclaw::interface::ADCTrajectory adc_trajectory) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishADCTrajectory(adc_trajectory);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishADCTrajectory(adc_trajectory);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishADCTrajectory(adc_trajectory);
  // AINFO << "successfully publish trajectory " << endl;
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishADCTrajectory(adc_trajectory);
#endif
}

void Planning::PublishPlanningCmd(legionclaw::interface::PlanningCmd planning_cmd) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishPlanningCmd(planning_cmd);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishPlanningCmd(planning_cmd);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishPlanningCmd(planning_cmd);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishPlanningCmd(planning_cmd);
#endif
}

void Planning::PublishPlanningAnalysis(
    legionclaw::interface::PlanningAnalysis planning_analysis) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishPlanningAnalysis(planning_analysis);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishPlanningAnalysis(planning_analysis);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishPlanningAnalysis(planning_analysis);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishPlanningAnalysis(planning_analysis);
#endif
}

void Planning::PublishTrajectoryArray(
    legionclaw::interface::TrajectoryArray trajectory_array) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishTrajectoryArray(trajectory_array);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishTrajectoryArray(trajectory_array);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishTrajectoryArray(trajectory_array);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishTrajectoryArray(trajectory_array);
#endif
}

void Planning::PublishFaults() {
  if (is_init_ == false) {
    return;
  }
  legionclaw::interface::Faults faults;
  // 版本号
  faults.set_version(10012);
  faultcodeset_->get_fault_code_list(&faults);
  // 填app的id，唯一标识符
  faults.set_app_id(faultcodeset_->get_target_id());
  // 填状态
  faults.set_is_active(function_activation_);
  // 消息头
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(faults);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_ = faults;
  }
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishFaults(faults_);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishFaults(faults_);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishFaults(faults_);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishFaults(faults_);
#endif
  faultcodeset_->clear_faults();
}

void Planning::PublishEvents(legionclaw::interface::Events events) {
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(events);
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishEvents(events);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishEvents(events);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishEvents(events);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishEvents(events);
#endif
}

std::shared_ptr<legionclaw::planning::PlanningConf> Planning::GetConf() const {
  return planning_conf_;
}

void Planning::HandleRoutingResponse(
    legionclaw::interface::RoutingResponse routing_response) {
  if (is_init_ == false) {
    return;
  }
  if (planning_conf_->enable_local_map_topic()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = routing_response.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      routing_response.set_header(header);
    }
    routing_response_ = routing_response;
    planning_flag_ = DRIVING;
    if (routing_response_.replan_flag() ==
        legionclaw::interface::RoutingResponse::ReplanFlag::REPLAN_FLAG_HUMAN) {
      legionclaw::interface::Header stop_info_header = routing_response.header();
      stop_info_header.set_stamp(TimeTool::Now2TmeStruct());
      stop_info_.clear_stop_points();
      stop_info_.set_header(stop_info_header);
    }
  }
}

void Planning::HandleLocalRoutingResponse(
    legionclaw::interface::RoutingResponse routing_response) {
  if (is_init_ == false) {
    return;
  }
  if (!planning_conf_->enable_local_map_topic()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = routing_response.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      routing_response.set_header(header);
    }
    routing_response_ = routing_response;
    planning_flag_ = DRIVING;
    if (routing_response_.replan_flag() ==
        legionclaw::interface::RoutingResponse::ReplanFlag::REPLAN_FLAG_HUMAN) {
      legionclaw::interface::Header stop_info_header = routing_response.header();
      stop_info_header.set_stamp(TimeTool::Now2TmeStruct());
      stop_info_.clear_stop_points();
      stop_info_.set_header(stop_info_header);
    }
  }
}

void Planning::HandleGuideInfo(legionclaw::interface::GuideInfo guide_info) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = guide_info.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      guide_info.set_header(header);
    }
    guide_info_ = guide_info;
  }
}

void Planning::HandleTrafficEvents(
    legionclaw::interface::TrafficEvents traffic_events) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = traffic_events.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      traffic_events.set_header(header);
    }
    traffic_events_ = traffic_events;
  }
}

void Planning::HandleParkingInfo(legionclaw::interface::ParkingInfo parking_info) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = parking_info.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      parking_info.set_header(header);
    }
    parking_info_ = parking_info;
  }
}

void Planning::HandleStopInfo(legionclaw::interface::StopInfo stop_info) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = stop_info.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      stop_info.set_header(header);
    }
    stop_info_ = stop_info;
  }
}

void Planning::HandleTrafficLightMsg(
    legionclaw::interface::TrafficLightMsg traffic_light_msg) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = traffic_light_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      traffic_light_msg.set_header(header);
    }
    traffic_light_msg_ = traffic_light_msg;
  }
}

void Planning::HandleLocation(legionclaw::interface::Location location) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = location.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      location.set_header(header);
    }
    location_ = location;
    legionclaw::interface::PointENU utm_position = location_.utm_position();
    if (!planning_conf_->enable_elevation()) {
      utm_position.set_z(0.0);
    }
    location_.set_utm_position(utm_position);
  }
}

void Planning::HandlePredictionObstacles(
    legionclaw::interface::PredictionObstacles prediction_obstacles) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = prediction_obstacles.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      prediction_obstacles.set_header(header);
    }

    prediction_obstacles_ = prediction_obstacles;
  }
}

void Planning::HandleLaneList(legionclaw::interface::LaneList lane_list) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = lane_list.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      lane_list.set_header(header);
    }
    lane_list_ = lane_list;
  }
}

void Planning::HandleChassis(legionclaw::interface::Chassis chassis) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = chassis.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      chassis.set_header(header);
    }
    chassis_ = chassis;
  }
  if (chassis_.driving_mode() == DrivingMode::MANUAL_TAKEOVER) {
    driving_flag_ = DrivingFlag::HUMAN;
    planning_flag_ = PlanningFlag::PLANNING_INVALID;
  }
}

void Planning::HandleSotifMonitorResult(
    legionclaw::interface::SotifMonitorResult sotif_monitor_result) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = sotif_monitor_result.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      sotif_monitor_result.set_header(header);
    }
    sotif_monitor_result_ = sotif_monitor_result;
  }
}

void Planning::HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = obu_cmd_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obu_cmd_msg.set_header(header);
    }
    obu_cmd_msg_ = obu_cmd_msg;
  }
  for (auto cmd : obu_cmd_msg.obu_cmd_list()) {
    // 编码值待定
    if (cmd.code() == 10086) {
      std::cout << "code : " << cmd.code() << "\n";
      switch (cmd.val()) {
        case FunctionMode::DEACTIVATE_BOTH:
          // 全部去激活
          TaskStop();
          break;
        case FunctionMode::ACTIVATE_DRIVING:
          // 行车模块激活
          TaskActivate();
          break;
        case FunctionMode::DEACTIVATE_DRIVING:
          // 行车模块去激活
          TaskStop();
          break;
        case FunctionMode::ACTIVATE_PARKING:
          // 泊车模块激活
          break;
        case FunctionMode::DEACTIVATE_PARKING:
          // 泊车模块去激活
          break;
        default:
          // 全部去激活
          TaskStop();
          break;
      }
    }
  }
}

void Planning::HandleDrivableRegion(
    legionclaw::interface::DrivableRegion drivable_region) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = drivable_region.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      drivable_region.set_header(header);
    }
    drivable_region_ = drivable_region;
  }
}

void Planning::HandleParkingOutInfo(
    legionclaw::interface::ParkingOutInfo parking_out_info) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planning_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = parking_out_info.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      parking_out_info.set_header(header);
    }
    parking_out_info_ = parking_out_info;

    if (parking_out_info_.parking_direction_type() == Direction::DIR_INVALID) {
      parking_out_direction_ =
          ParkingManager::OutDirection::DirectionNone;  // invalid
      return;
    } else if (parking_out_info_.parking_direction_type() == Direction::LEFT) {
      parking_out_direction_ = ParkingManager::OutDirection::Left;  // left
      return;
    } else if (parking_out_info_.parking_direction_type() == Direction::UP) {
      parking_out_direction_ = ParkingManager::OutDirection::UP;  // forward
      return;
    } else if (parking_out_info_.parking_direction_type() == Direction::RIGHT) {
      parking_out_direction_ = ParkingManager::OutDirection::Right;  // right
      return;
    } else if (parking_out_info_.parking_direction_type() == Direction::DOWN) {
      parking_out_direction_ = ParkingManager::OutDirection::Down;  // back
      return;
    }
  }
}

void Planning::ComputePlanningCommandOnTimer(void* param) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fabs(TimeTool::GetTimeDiff(local_view_.stop_info_.header().stamp(),
                                   stop_info_.header().stamp())) > 0.0) {
      local_view_.stop_info_ = stop_info_;
    }

    if (fabs(TimeTool::GetTimeDiff(
            local_view_.routing_response_.header().stamp(),
            routing_response_.header().stamp())) > 0.0) {
      local_view_.routing_response_ = routing_response_;
      reference_line_provider_->UpdateRoutingResponse(
          local_view_.routing_response_);
      lattice_planner_.JudgeReplanFlag(local_view_);
    }

    if (fabs(TimeTool::GetTimeDiff(local_view_.traffic_events_.header().stamp(),
                                   traffic_events_.header().stamp())) > 0.0) {
      local_view_.traffic_events_ = traffic_events_;
    }

    if (fabs(TimeTool::GetTimeDiff(local_view_.guide_info_.header().stamp(),
                                   guide_info_.header().stamp())) > 0.0) {
      local_view_.guide_info_ = guide_info_;
    }

    if (fabs(TimeTool::GetTimeDiff(local_view_.parking_info_.header().stamp(),
                                   parking_info_.header().stamp())) > 0.0) {
      local_view_.parking_info_ = parking_info_;
    }

    if (fabs(TimeTool::GetTimeDiff(
            local_view_.parking_out_info_.header().stamp(),
            parking_out_info_.header().stamp())) > 0.0) {
      local_view_.parking_out_info_ = parking_out_info_;
    }

    if (fabs(
            TimeTool::GetTimeDiff(local_view_.drivable_region_.header().stamp(),
                                  drivable_region_.header().stamp())) > 0.0) {
      local_view_.drivable_region_ = drivable_region_;
    }

    if (fabs(TimeTool::GetTimeDiff(
            local_view_.traffic_light_msg_.header().stamp(),
            traffic_light_msg_.header().stamp())) > 0.0) {
      local_view_.traffic_light_msg_ = traffic_light_msg_;
    }

    // if (fabs(TimeTool::GetTimeDiff(
    //         local_view_.obu_cmd_msg_.header().stamp(),
    //         obu_cmd_msg_.header().stamp())) > 0.0) {
    local_view_.obu_cmd_msg_ = obu_cmd_msg_;
    UpdateHMICommand();
    // }

    local_view_.location_ = location_;
    local_view_.prediction_obstacles_ = prediction_obstacles_;
    local_view_.lane_list_ = lane_list_;
    local_view_.chassis_ = chassis_;
    local_view_.sotif_monitor_result_ = sotif_monitor_result_;
  }
  Status status = CheckInput(&local_view_);

  if (!status.ok()) {
    AERROR << "Planning input data failed: " << status.error_message();
  } else {
  }

  // 算法计算
  double t1 = TimeTool::NowToSeconds();
  frame_status_ = UpdateFrame(local_view_);
  double t2 = TimeTool::NowToSeconds();
  planning_analysis_.set_frame_update_time(t2 - t1);
  // 状态机管理
  planning_sm_->OnUpdate();

  // Log();
}

void Planning::PublishPlanningCommandOnTimer(void* param) {
  if (is_init_ == false) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // 消息发布
  if (plan_status_ == Status::Ok()) {
    if (driving_flag_ == DrivingFlag::HUMAN) {
      adc_trajectory_.set_driving_mode(common::DrivingMode::COMPLETE_MANUAL);
    }
    legionclaw::interface::Header header;
    INTERFACE_HEADER_ASSIGN(adc_trajectory_)
    header.set_stamp(prediction_obstacles_.header().stamp());
    adc_trajectory_.set_header(header);
    PublishADCTrajectory(adc_trajectory_);
    INTERFACE_HEADER_ASSIGN(planning_cmd_)
    Planning::PublishPlanningCmd(planning_cmd_);
    INTERFACE_HEADER_ASSIGN(planning_analysis_)
    PublishPlanningAnalysis(planning_analysis_);
    INTERFACE_HEADER_ASSIGN(trajectory_array_)
    PublishTrajectoryArray(trajectory_array_);
    last_adc_trajectory_ = adc_trajectory_;
  } else {
    // AERROR << "<Planning>: No Plan ! !";
  }
}

Status Planning::FrameCreateReferenceLineInfo() {
  std::list<ReferenceLine> reference_lines;
  static bool failed_to_update_reference_line = false;
  failed_to_update_reference_line =
      (!reference_line_provider_->UpdatedReferenceLine());
  if (failed_to_update_reference_line) {
    const std::string msg = "Failed to update reference line !";
    AERROR << msg;
    return Status(Status::ErrorCode::PLANNING_ERROR, msg);
  }

  if (!reference_line_provider_->GetReferenceLines(&reference_lines)) {
    const std::string msg = "Failed to create reference line !";
    AERROR << msg;
    return Status(Status::ErrorCode::PLANNING_ERROR, msg);
  }

  frame_.set_smooth_lines(reference_line_provider_->GetSmoothLines());
  frame_.set_map_match_info(reference_line_provider_->GetMapMatchInfo());
  if (!frame_.ExtractReferenceLinesInfo(reference_lines)) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "failed to ExtractReferenceLinesInfo");
  }
  return Status::Ok();
}

Status Planning::CheckInput(LocalView* local_view) {
  // local_view_.obu_cmd_msg_.header =
  return Status::Ok();
}

void Planning::UpdateHMICommand() {
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  obu_cmd_msg_.obu_cmd_list(obu_cmd_list);
  for (auto obu_cmd : obu_cmd_list) {
    switch (obu_cmd.code()) {
      case 10001:  // 驾驶模式切换
        if (obu_cmd.val() == 1) {
          driving_flag_ = DrivingFlag::AUTO;
          return;
        } else if (obu_cmd.val() == 2) {
          driving_flag_ = DrivingFlag::HUMAN;
          planning_flag_ = PlanningFlag::PLANNING_INVALID;
          return;
        } else if (obu_cmd.val() == 3) {
          driving_flag_ = DrivingFlag::ASSISTED;
          return;
        }
        break;
      case 10042:  // 规划器切换
        if (obu_cmd.val() == 0) {
          planning_flag_ = PlanningFlag::PLANNING_INVALID;  // invalid
          return;
        } else if (obu_cmd.val() == 1) {
          planning_flag_ = PlanningFlag::PARKING_REQUEST;  // parking request
          return;
        } else if (obu_cmd.val() == 2) {
          planning_flag_ = PlanningFlag::PARKING;  // parking
          return;
        } else if (obu_cmd.val() == 3) {
          planning_flag_ = PlanningFlag::PARKING_STOP;  // parking stop
          return;
        } else if (obu_cmd.val() == 4) {
          planning_flag_ = PlanningFlag::PARKING_OUT;  // parkingout
          return;
        } else if (obu_cmd.val() == 5) {
          planning_flag_ = PlanningFlag::PARKING_OUT_STOP;  // parkingout stop
          return;
        }
        break;
      default:
        break;
    }
  }
}

Status Planning::UpdateFrame(const LocalView& local_view) {
  if (frame_.ExtractVehicleState(local_view.location_, local_view.chassis_,
                                 planning_conf_->vehicle_param()) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "ExtractVehicleState() failed.");
  }

  if (frame_.ExtractTrafficLightInfo(local_view.traffic_light_msg_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "ExtractTrafficLightInfo() failed.");
  }

  frame_.ExtractStopInfo(local_view_.stop_info_);

  frame_.ExtractParkingInfo(local_view_.parking_info_);

  frame_.ExtractParkingOutInfo(local_view_.parking_out_info_);
  // double t1 = TimeTool::NowToSeconds();

  // 参考线单独线程处理
  {
    if (driving_flag_ == DrivingFlag::HUMAN) {
      reference_line_provider_->Init();
    }
    injector_->vehicle_state()->Update(local_view.location_,
                                       local_view.chassis_);
    vehicle_state_ = injector_->vehicle_state()->vehicle_state();
    // Update reference line provider and reset pull over if necessary
    reference_line_provider_->UpdateVehicleState(vehicle_state_);

    if (FrameCreateReferenceLineInfo() != Status::Ok()) {
      return Status(Status::ErrorCode::PLANNING_ERROR,
                    "Frame Failed to get reference line.");
    }
  }

  frame_.UpdateLaneChangeCmd(obu_cmd_msg_);

  // frame_.IsLaneChangeComplete();

  if (frame_.ExtractObstacleList(local_view.prediction_obstacles_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "ExtractObstacleList() failed.");
  }

  if (frame_.ExtractTrafficEvents(local_view.traffic_events_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "PlanGetTrafficEvents failed.");
  }

  if (frame_.ExtractGuideInfo(local_view.guide_info_) == false) {
    return Status(Status::ErrorCode::PLANNING_ERROR,
                  "PlanExtractGuideInfo failed.");
  }

  frame_.ExtractDrivableRegion(local_view.drivable_region_);

  return Status::Ok();
}

void Planning::MessagesInit() {
  if (planning_conf_ == nullptr) return;

  for (auto it : planning_conf_->messages().active_message()) {
    auto message = planning_conf_->messages().message_info().at(it);
    switch (message.type()) {
#if LCM_ENABLE
      case legionclaw::common::MessageType::LCM: {
        AINFO << "message type:LCM";

        lcm_message_manager_ = std::make_shared<LcmMessageManager<Planning>>();
        ResigerMessageManager(message.name(), lcm_message_manager_);

        lcm_message_manager_->Init(this);
      } break;
#endif
#if DDS_ENABLE
      case legionclaw::common::MessageType::DDS: {
        AINFO << "message type:DDS";

        dds_message_manager_ = std::make_shared<DdsMessageManager<Planning>>();
        ResigerMessageManager(message.name(), dds_message_manager_);

        dds_message_manager_->Init(this);
      } break;
#endif
#if ROS_ENABLE
      case legionclaw::common::MessageType::ROS: {
        AINFO << "message type:ROS";

        ros_message_manager_ = std::make_shared<RosMessageManager<Planning>>();
        ResigerMessageManager(message.name(), ros_message_manager_);
        ros_message_manager_->Init(this);
      } break;
#endif

#if ADSFI_ENABLE
      case legionclaw::common::MessageType::ADSFI: {
        AINFO << "message type:ADSFI";

        adsfi_message_manager_ =
            std::make_shared<AdsfiMessageManager<Planning>>();
        ResigerMessageManager(message.name(), adsfi_message_manager_);

        adsfi_message_manager_->Init(this);
      } break;
#endif

#if ROS2_ENABLE
      case legionclaw::common::MessageType::ROS2: {
        AINFO << "message type:ROS2";

        ros2_message_manager_ =
            std::make_shared<Ros2MessageManager<Planning>>();
        ResigerMessageManager(message.name(), ros2_message_manager_);

        ros2_message_manager_->Init(this);
      } break;
#endif
      default: {
        AERROR << "unknown message type";
      } break;
    }
  }
}

void Planning::MessagesActivate() {
  if (planning_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->Activate();
  }
  return;
}

void Planning::MessagesDeActivate() {
  if (planning_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->DeActivate();
  }
  return;
}

void Planning::StatusDetectOnTimer() {}

void Planning::Spin() {
  while (1) {
    if (function_activation_) {
      ad_timer_manager_->DetectTimers(NULL);
      usleep(1000);
    } else
      usleep(100000);
  }
}

void Planning::Reset() {
  driving_flag_ = DrivingFlag::DRIVING_INVALID;
  task_status_ = TaskStatus::TASK_INVALID;
  planning_flag_ = PlanningFlag::PLANNING_INVALID;

  lattice_planner_.Reset();
  parking_planner_.Reset();

  local_view_.routing_response_.clear_lane_list();
  local_view_.stop_info_.clear_stop_points();
  local_view_.prediction_obstacles_.clear_prediction_obstacles();
  local_view_.obu_cmd_msg_.clear_obu_cmd_list();
  local_view_.parking_info_.set_parking_status(
      ParkingStatus::PARKING_DISENABLE);  // PARKING_DISENABLE
  local_view_.parking_info_.set_parking_type(
      ParkingType::INVALID_PARKING);  // VERTICAL_PARKING PARALLEL_PARKING
}

}  // namespace planning
}  // namespace legionclaw
