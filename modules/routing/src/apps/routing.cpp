/**
 * @file    routing.cpp
 * @author  zdhy
 * @date    2021-10-17
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include "routing.h"

#include <sys/time.h>
#include <time.h>

#include <fstream>

#include "modules/common/file/file.h"
#include "modules/common/interface/chassis.hpp"
#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/global_route_msg.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/common/interface/odometry.hpp"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/polygon_2d.hpp"
#include "modules/common/interface/routing_request.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/stop_info.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"

/**
 * @namespace legionclaw::routing
 * @brief legionclaw::routing
 */

namespace legionclaw {
namespace routing {
// 仅在本文件中使用，计算两个角度制的夹角（返回0~pi）
double calculate_diff_angle(double first, double second) {
  double diff = first - second;

  if (diff < 0) diff = second - first;
  while (diff >= 2 * M_PI) {
    diff -= 2 * M_PI;
  }
  if (diff > M_PI) diff = 2 * M_PI - diff;

  return diff;
}

void Routing::Init() {
  // step1 初始化状态设置为false
  {
    is_init_ = false;
    function_activation_ = false;
  }

  // step2 变量初始化
  {
    map_loaded_ = false;
    new_goal_received_ = false;
    new_parking_space_ = false;
    start_pose_inited_ = false;
    new_request_received_ = false;
    first_send_ = false;
    send_flag_ = false;
    mileage_enable_ = false;
    stop_check_ = false;
    location_init_ = false;
    match_stop_points_ = false;
    traffic_light_msg_.set_contain_lights(false);
    cur_event_.mutable_limit_speed_info()->set_limitspeed_valid_flag(
        legionclaw::common::IsValid(0));
    cur_speed_limit.set_limitspeed_valid_flag(legionclaw::common::IsValid(0));
    transformer_init_ = false;
  }

  // step3 配置文件初始化
  {
    routing_conf_ = std::make_shared<RoutingConf>();
    if (GetProtoFromJsonFile(FLAGS_routing_config_file, routing_conf_.get()) ==
        false) {
      AERROR << "GetProtoFrom routing_config_file failed.";
      return;
    }
  }
  // step4 日志初始化
  {
    LOGGING_INIT(routing_conf_)
  }

  // step4 IPC初始化
  {
    MESSAGE_INIT(routing_conf_)
  }

  // step5 读取配置文件
  {
  }
  // step6 故障码初始化
  FaultMonitorInit();

  traffic_rule_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany,
      lanelet::Participants::Vehicle);  // 生成路由的交通规则

  ad_timer_manager_ = std::make_shared<ADTimerManager<Routing, void>>();
  task_mainloop_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_check_station_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_check_stopline_1000ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_merge_polygons_1000ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_global_msg_gen_1000ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_guide_info_gen_1000ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_lanelines_gen_1000ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_navi_info_gen_100ms_ =
      std::make_shared<WheelTimer<Routing, void>>(ad_timer_manager_);
  task_thread_.reset(new std::thread([this] { Spin(); }));
  if (task_thread_ == nullptr) {
    std::cerr << "Unable to create task_thread_ thread " << "\n";
    return;
  }
  // step9 初始化状态为true
  {
    is_init_ = true;
  }
  //TODO:临时默认激活
  TaskActivate(); // 暂时修改为初始化就后激活
}

void Routing::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    std::cout << "task_thread_ stopped [ok]." << "\n";
  }
}

void Routing::TaskActivate() {
  if (is_init_ == false) {
    return;
  }
  // IPC激活
  MessagesActivate();
  if (function_activation_) {
    return;
  }
  {
    // 定时器激活
    task_mainloop_->AddTimer(routing_conf_->loop_rate(), &Routing::MainLoop,
                             this);

    task_check_station_->AddTimer(routing_conf_->loop_rate(),
                                  &Routing::Check_Sation_Loop, this);
    if (routing_conf_->enable_traffic_junction()) {
      task_check_stopline_1000ms_->AddTimer(
          routing_conf_->check_junction_rate(), &Routing::CheckStopLine, this);
    }
    if (routing_conf_->enable_polygon_send()) {
      task_merge_polygons_1000ms_->AddTimer(routing_conf_->polygon_gen_rate(),
                                            &Routing::MergePolygons, this);
    }
    if (routing_conf_->enable_route_msg_send()) {
      task_global_msg_gen_1000ms_->AddTimer(routing_conf_->route_msg_gen_rate(),
                                            &Routing::GlobalMsgGen, this);
    }
    if (routing_conf_->enable_guide_info_send()) {
      task_guide_info_gen_1000ms_->AddTimer(
          routing_conf_->guide_info_gen_rate(), &Routing::GuideInfoGen, this);
    }
    if (routing_conf_->enable_lanelines_send()) {
      task_lanelines_gen_1000ms_->AddTimer(routing_conf_->lanelines_gen_rate(),
                                           &Routing::LanelinesGen, this);
    }
    if (routing_conf_->enable_navi_info_send()) {
      task_navi_info_gen_100ms_->AddTimer(routing_conf_->navi_info_gen_rate(),
                                          &Routing::NaviInfoPublisher, this);
    }
  }
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout << "===================function activate=================="
            << "\n";
  function_activation_ = true;
  return;
}

void Routing::TaskStop() {
  if (is_init_ == false) {
    return;
  }
  // IPC去激活
  MessagesDeActivate();
  if (function_activation_ == false) {
    return;
  }
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // 定时器激活
    task_mainloop_->Stop();

    task_check_station_->Stop();
    if (routing_conf_->enable_traffic_junction()) {
      task_check_stopline_1000ms_->Stop();
    }
    if (routing_conf_->enable_polygon_send()) {
      task_merge_polygons_1000ms_->Stop();
    }
    if (routing_conf_->enable_route_msg_send()) {
      task_global_msg_gen_1000ms_->Stop();
    }
    if (routing_conf_->enable_guide_info_send()) {
      task_guide_info_gen_1000ms_->Stop();
    }
    if (routing_conf_->enable_lanelines_send()) {
      task_lanelines_gen_1000ms_->Stop();
    }
    if (routing_conf_->enable_navi_info_send()) {
      task_navi_info_gen_100ms_->Stop();
    }
  }
  {
    // 清除所有内部计算的中间结果，保证回到刚init完的状态
    new_goal_received_ = false;
    start_pose_inited_ = false;
    transformer_init_ = false;
    stop_lane_ids_.clear();
    match_stop_points_ = false;
    stop_check_ = false;
    stop_lanes_.clear();
    stop_indexs_.clear();
    location_init_ = false;
    new_request_received_ = false;
    new_parking_space_ = false;
    mileage_enable_ = false;
    send_flag_ = false;
    mileage_enable_ = false;
    first_send_ = false;

    total_mileage_ = 0;
    cur_mileage_ = 0;
    path_id_map_.clear();
    local_ids.clear();
    send_flag_ = false;
    junction_check_ = false;

    section_mileage_ = 0;
    local_paths_.clear();
    all_stop_lines_.clear();
    one_way_plan_.clear();
    no_change_lane_.clear();
    junctions_directions_.clear();

    location_init_ = false;
    traffic_light_msg_.set_contain_lights(false);
    cur_event_.mutable_limit_speed_info()->set_limitspeed_valid_flag(
        legionclaw::common::IsValid(0));
    cur_speed_limit.set_limitspeed_valid_flag(legionclaw::common::IsValid(0));
  }
  std::cout << "******************function stop***************" << "\n";
  function_activation_ = false;
  return;
}

void Routing::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<Routing>> message_manager) {
  message_manager_.insert(
      std::pair<std::string, std::shared_ptr<MessageManager<Routing>>>(
          name, message_manager));
}

void Routing::PublishRoutingResponse(
    legionclaw::interface::RoutingResponse routing_response) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishRoutingResponse(routing_response);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishRoutingResponse(routing_response);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishRoutingResponse(routing_response);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishRoutingResponse(routing_response);
#endif
}

void Routing::PublishNaviInfoMsg(legionclaw::interface::NaviInfoMsg navi_info_msg) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishNaviInfoMsg(navi_info_msg);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishNaviInfoMsg(navi_info_msg);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishNaviInfoMsg(navi_info_msg);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishNaviInfoMsg(navi_info_msg);
#endif
}

void Routing::PublishGuideInfo(legionclaw::interface::GuideInfo guide_info) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishGuideInfo(guide_info);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishGuideInfo(guide_info);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishGuideInfo(guide_info);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishGuideInfo(guide_info);
#endif
}

void Routing::PublishLaneList(legionclaw::interface::LaneList lane_list) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishLaneList(lane_list);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishLaneList(lane_list);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishLaneList(lane_list);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["DDS"]->PublishLaneList(lane_list);
#endif
}

void Routing::PublishParkingInfo(legionclaw::interface::ParkingInfo parking_info) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishParkingInfo(parking_info);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishParkingInfo(parking_info);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishParkingInfo(parking_info);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishParkingInfo(parking_info);
#endif
}

void Routing::PublishStopInfo(legionclaw::interface::StopInfo stop_info) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishStopInfo(stop_info);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishStopInfo(stop_info);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishStopInfo(stop_info);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishStopInfo(stop_info);
#endif
}

void Routing::PublishFaults() {
  if (is_init_ == false) {
    return;
  }
  legionclaw::interface::Faults faults;
  faults.set_version(10002);
  faultcodeset_->get_fault_code_list(&faults);
  // 填app的id，唯一标识符
  faults.set_app_id(faultcodeset_->get_target_id());
  // 填状态
  faults.set_is_active(function_activation_);
  // 消息头
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(faults);
  {
    std::unique_lock<std::mutex> lock(mutex_);
    faults_ = faults;
    lock.unlock();
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

  // faultcodeset_->clear_fault_code();//通号项目使用
}

void Routing::PublishGlobalRouteMsg(
    legionclaw::interface::GlobalRouteMsg global_route_msg) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishGlobalRouteMsg(global_route_msg);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishGlobalRouteMsg(global_route_msg);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishGlobalRouteMsg(global_route_msg);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishGlobalRouteMsg(global_route_msg);
#endif
}

void Routing::PublishPolygon2D(legionclaw::interface::Polygon2D polygon_2d) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishPolygon2D(polygon_2d);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishPolygon2D(polygon_2d);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishPolygon2D(polygon_2d);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishPolygon2D(polygon_2d);
#endif
}

void Routing::PublishTrafficEventsOutput(
    legionclaw::interface::TrafficEvents traffic_events) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishTrafficEventsOutput(traffic_events);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishTrafficEventsOutput(traffic_events);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishTrafficEventsOutput(traffic_events);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishTrafficEventsOutput(traffic_events);
#endif
}

void Routing::PublishTrafficLightMsgOutput(
    legionclaw::interface::TrafficLightMsg traffic_light_msg_output) {
  // #if LCM_ENABLE
  //       if (message_manager_.count("LCM") > 0)
  //         message_manager_["LCM"]->PublishTrafficLightMsgOutput(traffic_light_msg_output);
  // #endif

  // #if ROS_ENABLE
  //       if (message_manager_.count("ROS") > 0)
  //         message_manager_["ROS"]->PublishTrafficLightMsgOutput(traffic_light_msg_output);
  // #endif

  // #if DDS_ENABLE
  //       if (message_manager_.count("DDS") > 0)
  //         message_manager_["DDS"]->PublishTrafficLightMsgOutput(traffic_light_msg_output);
  // #endif

  // #if ROS2_ENABLE
  //       if (message_manager_.count("ROS2") > 0)
  //         message_manager_["ROS2"]->PublishTrafficLightMsgOutput(traffic_light_msg_output);
  // #endif
}

std::shared_ptr<RoutingConf> Routing::GetConf() const { return routing_conf_; }

void Routing::HandleTrafficLightMsg(
    legionclaw::interface::TrafficLightMsg traffic_light_msg) {
  if (is_init_ == false) {
    return;
  }
  {
    if (routing_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = traffic_light_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      traffic_light_msg.set_header(header);
    }
    traffic_light_msg_ = traffic_light_msg;
  }
}

void Routing::HandleTrafficEventsInput(
    legionclaw::interface::TrafficEvents traffic_events_input) {
  if (is_init_ == false) {
    return;
  }
  {
    if (routing_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = traffic_events_input.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      traffic_events_input.set_header(header);
    }
    cur_speed_limit = traffic_events_input.limit_speed_info();
  }
}

void Routing::HandleOdometry(legionclaw::interface::Odometry odometry) {
  if (is_init_ == false) {
    return;
  }
  {
    if (routing_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = odometry.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      odometry.set_header(header);
    }
    Eigen::Quaterniond quaternion(
        odometry.orientation().qw(), odometry.orientation().qx(),
        odometry.orientation().qy(), odometry.orientation().qz());
    transformer_.set_quaternion(quaternion);
    Eigen::Vector3d translation(odometry.position().x(),
                                odometry.position().y(),
                                odometry.position().z());
    transformer_.set_translation(translation);
    transformer_init_ = true;
  }
}

void Routing::HandleLocation(legionclaw::interface::Location location) {
  if (function_activation_ == false) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  // 坐标原点设置
  if (!location_init_) {
    routing_conf_->set_origin_lat(location.origin_lat());
    routing_conf_->set_origin_lon(location.origin_lon());
    std::cout << "Origin  : " << "  lat:  " << setprecision(10)
              << routing_conf_->origin_lat() << " lon:  " << setprecision(10)
              << routing_conf_->origin_lon() << "\n";
    projector_.set_origin_ll(routing_conf_->origin_lat(),
                             routing_conf_->origin_lon());
  }
  if (routing_conf_->use_tf_flag()) {
    if (transformer_init_) {
      double odom_heading = location.heading();
      location.set_heading(odom_heading - transformer_.get_yaw());
      // auto
      // utm_point=projector_.forward(location.position().lat(),location.position().lon());
      // odom ==> gps
      Eigen::Vector3d input(location.utm_position().x(),
                            location.utm_position().y(),
                            location.utm_position().z());
      Eigen::Vector3d output = transformer_.trans_inv(input);
      location.mutable_utm_position()->set_x(output.x());
      location.mutable_utm_position()->set_y(output.y());
    } else {
      std::cout << "tf not init" << "\n";
      return;
    }
  }
  if (!location_init_) {
    LoadLanelet2Map();
    location_init_ = true;
  }
  // auto
  // utm_point=projector_.forward(location.position().lat(),location.position().lon());
  // location.mutable_utm_position()->set_x(utm_point.x);
  // location.mutable_utm_position()->set_y(utm_point.y);
  location_ = location;
  if (!routing_conf_->enable_3d_map()) {
    location_.mutable_utm_position()->set_z(0);
  }
  if (!start_pose_inited_) {
    current_pose_ = location;
    start_pose_inited_ = true;
  }
  section_last_point_ = current_pose_;
  current_pose_ = location;
  if (mileage_enable_) {
    double distance = sqrt(pow(current_pose_.utm_position().x() -
                                   section_last_point_.utm_position().x(),
                               2) +
                           pow(current_pose_.utm_position().y() -
                                   section_last_point_.utm_position().y(),
                               2));
    section_mileage_ += distance;
    if (section_mileage_ >= routing_conf_->mileage_length()) {
      send_flag_ = true;
    }
  }
}

void Routing::HandleChassis(legionclaw::interface::Chassis chassis) {
  if (is_init_ == false) {
    return;
  }
  {
    if (routing_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = chassis.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      chassis.set_header(header);
    }
    chassis_ = chassis;
  }
}

void Routing::HandleRoutingRequest(
    legionclaw::interface::RoutingRequest routing_request) {
  if (is_init_ == false) {
    return;
  }
  if (!location_init_) {
    std::cerr << "location not init" << "\n";
    return;
  }
  routing_request_ = routing_request;
  new_request_received_ = true;
}

void Routing::MessagesInit() {
  if (routing_conf_ == nullptr) return;

  for (auto it : routing_conf_->messages().active_message()) {
    auto message = routing_conf_->messages().message_info().at(it);
    switch (message.type()) {
#if LCM_ENABLE
      case legionclaw::common::MessageType::LCM: {
        AINFO << "message type:LCM";

        lcm_message_manager_ = std::make_shared<LcmMessageManager<Routing>>();
        ResigerMessageManager(message.name(), lcm_message_manager_);

        lcm_message_manager_->Init(this);
      } break;
#endif
#if DDS_ENABLE
      case legionclaw::common::MessageType::DDS: {
        AINFO << "message type:DDS";

        dds_message_manager_ = std::make_shared<DdsMessageManager<Routing>>();
        ResigerMessageManager(message.name(), dds_message_manager_);

        dds_message_manager_->Init(this);
      } break;
#endif
#if ROS_ENABLE
      case legionclaw::common::MessageType::ROS: {
        AINFO << "message type:ROS";

        ros_message_manager_ = std::make_shared<RosMessageManager<Routing>>();
        ResigerMessageManager(message.name(), ros_message_manager_);
        ros_message_manager_->Init(this);
      } break;
#endif

#if ADSFI_ENABLE
      case legionclaw::common::MessageType::ADSFI: {
        AINFO << "message type:ADSFI";

        adsfi_message_manager_ =
            std::make_shared<AdsfiMessageManager<Routing>>();
        ResigerMessageManager(message.name(), adsfi_message_manager_);

        adsfi_message_manager_->Init(this);
      } break;
#endif

#if ROS2_ENABLE
      case legionclaw::common::MessageType::ROS2: {
        AINFO << "message type:ROS2";

        ros2_message_manager_ = std::make_shared<Ros2MessageManager<Routing>>();
        ResigerMessageManager(message.name(), ros2_message_manager_);

        ros2_message_manager_->Init(this);
      } break;
#endif
      default: {
        std::cerr << "unknown message type" << "\n";
      } break;
    }
  }
}

void Routing::FaultMonitorInit() {
  legionclaw::interface::FaultCodeCallback sendheart_callback_func =
      std::bind(&Routing::PublishFaults, this);
  legionclaw::interface::FaultCodeCallback fault_callback_func = nullptr;
  FAULTCODE_INIT("../../common/data/faults/faults.json", "routing",
                 faultcodeset_, sendheart_callback_func, fault_callback_func)
}

void Routing::MessagesActivate() {
  if (routing_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->Activate();
  }
  return;
}

void Routing::MessagesDeActivate() {
  if (routing_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->DeActivate();
  }
  return;
}

void Routing::Spin() {
  while (1) {
    if (function_activation_) {
      ad_timer_manager_->DetectTimers(NULL);
      usleep(1000);
    } else
      usleep(100000);
  }
}

void Routing::HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg) {
  if (!is_init_) {
    return;
  }
  // if (!location_init_)
  // {
  //   std::cerr << "location not init" << "\n";
  //   return;
  // }
  if (!routing_conf_->enable_obucmd_handle()) {
    return;
  }
  std::vector<legionclaw::interface::ObuCmd> obu_cmd_list;
  obu_cmd_msg.obu_cmd_list(obu_cmd_list);
  for (auto obucmd : obu_cmd_list) {
    if (obucmd.code() == 10042 && obucmd.val() == 1) {
      ParkingPlan();
    } else if (obucmd.code() == 10042 && obucmd.val() == 2) {
      if (new_parking_space_) {
        parking_info_.mutable_header()->set_seq(unique::getParkingInfoSeq());
        PublishParkingInfo(parking_info_);
        new_parking_space_ = false;
      } else {
        std::cout << "No Parking Space Choosed" << "\n";
      }
    } else if (obucmd.code() == 10001 && obucmd.val() == 2) {
      std::unique_lock<std::mutex> lock(mutex_);
      Clear();
      std::cout << "Exit Global Planning" << "\n";
    } else if (obucmd.code() == 10086) {
      std::cout << "code : " << obucmd.code() << "\n";
      switch (obucmd.val()) {
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

int Routing::GetIndex(lanelet::ConstLanelets lanes,
                      lanelet::ConstLanelet& lane) {
  int index = -1;

  // 使用 std::find 查找元素
  auto it = std::find(lanes.begin(), lanes.end(), lane);

  // 检查元素是否存在
  if (it != lanes.end()) {
    // 计算索引
    index = std::distance(lanes.begin(), it);
  }

  return index;
}

bool Routing::GetLane(double x, double y, double z, double yaw,
                      lanelet::ConstLanelet& lane) {
  lanelet::Point3d point(lanelet::utils::getId(), x, y, z);
  // 由于统一角度坐标系，故在此更改
  double north_yaw = yaw;
  auto lane_pair = lanelet2_ex::GetClosestPoint_LaneletInRange(
      const_map_, point, north_yaw, routing_conf_->search_range(),
      routing_conf_->yaw_weight(), routing_conf_->search_height_range());
  int lane_id = lane_pair.second;
  if (lane_id == -1) {
    return false;
  }
  if (map_->laneletLayer.exists(lane_id)) {
    lane = map_->laneletLayer.get(lane_id);
    return true;
  } else {
    return false;
  }
}

bool Routing::ParkingPlan() {
  std::cout << "Choosing the Nearest Parking Area " << "\n";
  if (parking_areas_.empty()) {
    std::cout << "No Parking Space in Map" << "\n";
    return false;
  }
  double minDistance = DBL_MAX;
  lanelet::ConstLanelet start_lane;
  bool get_it =
      GetLane(location_.utm_position().x(), location_.utm_position().y(),
              location_.utm_position().z(), location_.heading(), start_lane);
  if (!get_it) {
    std::cout << "Can not Locate Vehicle in map" << "\n";
  }
  int index = -1;

  for (unsigned int j = 0; j < parking_areas_.size(); j++) {
    ParkStop area = parking_areas_[j];
    auto b_point = projector_.forward(area.stop_point_.latitude(),
                                      area.stop_point_.longitude());
    lanelet::ConstLanelet stop_lane;
    bool get_stop = GetLane(b_point.x, b_point.y, area.stop_point_.ele(),
                            area.stop_point_.heading(), stop_lane);
    if (!get_stop) {
      continue;
    }

    lanelet::Optional<lanelet::routing::Route> route =
        routing_map_->getRoute(start_lane, stop_lane, 0, true);
    lanelet::routing::LaneletPaths shortest_paths;

    if (!route) {
      continue;
    }
    lanelet::routing::LaneletPath shortest_path = route->shortestPath();
    // std::cout << "shortest path in lanelet2" << "\n";

    double distance = 0;
    for (unsigned int i = 0; i < shortest_path.size(); i++) {
      auto lane = shortest_path[i];
      if (i > 1) {
        auto before = shortest_path[i - 1];
        auto besides = routing_map_->besides(lane);
        bool is_beside = false;
        for (auto beside : besides) {
          if (beside.id() == before.id()) {
            is_beside = true;
            break;
          }
        }
        if (is_beside) {
          continue;
        }
      }

      distance += lanelet::geometry::length2d(lane);
    }
    if (distance < minDistance) {
      minDistance = distance;
      index = j;
    }
  }
  if (index == -1) {
    std::cout << "No Parking Area Selected" << "\n";
    return false;
  }
  auto parking_selected = parking_areas_[index];

  interface::RoutingRequest request;
  std::vector<legionclaw::interface::KeyPoint> key_points;
  interface::KeyPoint start_point;
  auto start_ll = projector_.reverse(location_.utm_position().x(),
                                     location_.utm_position().y());
  start_point.set_latitude(start_ll.lat);
  start_point.set_longitude(start_ll.lon);
  start_point.set_heading(location_.heading());
  key_points.push_back(start_point);
  key_points.push_back(parking_selected.stop_point_);
  request.set_num_of_kp(key_points.size());
  request.set_key_point_list(&key_points);

  std::cout << "Parking Area Choosed	"
            << " Distance :	" << minDistance << "\n";
  routing_request_ = request;
  parking_info_ = parking_selected.parking_info_;
  new_request_received_ = true;
  new_parking_space_ = true;
  return true;
}

void Routing::LoadStopLines() {
  all_stop_lines_.clear();
  std::set<lanelet::TrafficLight::Ptr> lights;
  for (auto lane : local_paths_) {
    // bool has_stopline = false;
    for (auto id : lane) {
      auto cur_lane = map_->laneletLayer.get(id);
      auto traffic_lights =
          cur_lane.regulatoryElementsAs<lanelet::TrafficLight>();
      for (auto light : traffic_lights) {
        lights.insert(light);
      }
    }
  }
  for (auto light : lights) {
    StopData temp_line;
    auto lanes = map_->laneletLayer.findUsages(light);
    auto stop_line_option = light->stopLine();
    if (!stop_line_option) {
      continue;
    }
    auto stop_line = stop_line_option.value();
    temp_line.line_id = stop_line.id();
    for (auto lane : lanes) {
      std::cout << "test_stoplane: " << lane.id() << "\n";
      auto temp_dir = FindNextJunctionDirection(lane.id());
      std::cout << "NextJunctionDirection= " << temp_dir << "\n";
      if (temp_dir != legionclaw::common::Direction::DIR_INVALID) {
        temp_line.direction = temp_dir;
      }
      interface::Point3D temp_point;
      // calculate point
      auto center_point = lane.centerline().back();
      // auto last_point=lane.centerline()[lane.centerline().size()-2];
      // double dist=lanelet::geometry::distance2d(center_point,last_point);
      // double
      // x=routing_conf_->stop_line_distance()/dist*(last_point.x()-center_point.x())+center_point.x();
      // double
      // y=routing_conf_->stop_line_distance()/dist*(last_point.y()-center_point.y())+center_point.y();
      temp_point.set_x(center_point.x());
      temp_point.set_y(center_point.y());
      temp_point.set_z(center_point.z());
      temp_line.lane_ids.push_back(lane.id());
      temp_line.stop_points.push_back(temp_point);
      auto temp_length = lanelet::geometry::length3d(lane);
      if (temp_length < routing_conf_->stopline_reserved_length()) {
        lanelet::Lanelets temp_lanes;
        FindSuitableStopLane(lane, temp_lanes,
                             routing_conf_->stopline_reserved_length());
        if (!temp_lanes.empty()) {
          for (auto lane : temp_lanes) {
            temp_line.lane_ids.push_back(lane.id());
          }
        }
      }
    }
    all_stop_lines_.push_back(temp_line);
  }
  junction_check_ = true;
  for (int i = 0; i < all_stop_lines_.size(); i++) {
    std::cout << "stop_line_" << i << "\n";
    for (auto id : all_stop_lines_[i].lane_ids) {
      std::cout << id << " -- ";
    }
    std::cout << "\nstop_line_" << i << "\n";
  }
  return;
}

legionclaw::common::Direction Routing::FindNextJunctionDirection(int id) {
  legionclaw::common::Direction dir = legionclaw::common::Direction::DIR_INVALID;
  for (auto ids : local_paths_) {
    // std::cout<<"--local_path"<<"\n";

    for (int i = 0; i < ids.size() - 1; i++) {
      // std::cout<<"LaneId= "<<ids[i]<<"\n";
      if (ids[i] == id) {
        auto next_lane = map_->laneletLayer.get(ids[i + 1]);
        // std::cout<<"NextJunctionLaneId= "<<ids[i + 1]<<"  LaneType=
        // "<<next_lane.attributeOr("subtype","unknown")<<"\n";
        if (next_lane.attributeOr("subtype", "unknown") == "virtual") {
          std::string next_dir =
              next_lane.attributeOr("turn_direction", "unknown");
          if (next_dir == std::string("left") ||
              next_dir == std::string("turnaround") ||
              next_dir == std::string("left_turnaround")) {
            dir = legionclaw::common::Direction::LEFT;
          } else if (next_dir == std::string("right")) {
            dir = legionclaw::common::Direction::RIGHT;
          } else if (next_dir == std::string("straight")) {
            dir = legionclaw::common::Direction::UP;
          } else {
            dir = legionclaw::common::Direction::DIR_INVALID;
          }
        }
      }
    }

    // std::cout<<"local_path--"<<"\n";
  }
  return dir;
}

legionclaw::common::Direction Routing::FindJunctionDirection(int id) {
  auto lane = map_->laneletLayer.get(id);
  legionclaw::common::Direction dir = legionclaw::common::Direction::DIR_INVALID;
  // std::cout<<"lane.attribute =
  // "<<lane.attributeOr("subtype","unknown")<<"\n"; if
  // (lane.attributeOr("subtype","unknown") == "virtual")
  {
    std::string dir_string = lane.attributeOr("turn_direction", "unknown");
    if (dir_string == std::string("left") ||
        dir_string == std::string("turnaround") ||
        dir_string == std::string("left_turnaround")) {
      dir = legionclaw::common::Direction::LEFT;
    } else if (dir_string == std::string("right")) {
      dir = legionclaw::common::Direction::RIGHT;
    } else if (dir_string == std::string("straight")) {
      dir = legionclaw::common::Direction::UP;
    } else {
      dir = legionclaw::common::Direction::DIR_INVALID;
    }
  }

  return dir;
}

int Routing::GetLaneInfoType(int id) {
  auto lane = map_->laneletLayer.get(id);
  std::string lane_guidance = lane.attributeOr("turn_direction", "unknown");
  legionclaw::routing::LaneInfoType lane_type;
  if (lane_guidance == std::string("left")) {
    lane_type = legionclaw::routing::LaneInfoType::LEFT_TURN_NO_TURN_AROUND;
  } else if (lane_guidance == std::string("straight")) {
    lane_type = legionclaw::routing::LaneInfoType::STRAIGHT;
  } else if (lane_guidance == std::string("right")) {
    lane_type = legionclaw::routing::LaneInfoType::RIGHT_TURN;
  } else if (lane_guidance == std::string("straight_left")) {
    lane_type = legionclaw::routing::LaneInfoType::STRAIGHT_AND_LEFT_TURN;
  } else if (lane_guidance == std::string("straight_right")) {
    lane_type = legionclaw::routing::LaneInfoType::STRAIGHT_AND_RIGHT_TURN;
  } else if (lane_guidance == std::string("left_turnaround")) {
    lane_type = legionclaw::routing::LaneInfoType::LEFT_TURN_AND_TURN_AROUND;
  } else if (lane_guidance == std::string("turnaround")) {
    lane_type = legionclaw::routing::LaneInfoType::NO_LEFT_TURN_ONLY_TURN_AROUND;
  } else {
    lane_type = legionclaw::routing::LaneInfoType::LANE_TYPE_UNKNOWN;
  }
  return lane_type;
}

int Routing::GetLaneRoadType(int id) {
  auto lane = map_->laneletLayer.get(id);
  std::string lane_roadtype = lane.attributeOr("roadtype", "unknown");
  return 0;
}

void Routing::LoadJunctionInfo(const std::vector<int>& ids) {
  one_way_plan_.clear();
  junctions_directions_.clear();
  // std::list<int> no_change_lane_; //剔除最短路径下的左右换道路段
  no_change_lane_.clear();
  for (unsigned int i = 0; i < ids.size(); i++) {
    int id = ids[i];
    if (i == 0) {
      no_change_lane_.push_back(id);
    } else {
      auto lane = map_->laneletLayer.get(id);
      auto besides = routing_map_->besides(lane);
      int last_id = no_change_lane_.back();
      bool not_beside = true;
      for (auto beside : besides) {
        if (beside.id() == last_id) {
          not_beside = false;
        }
      }
      if (not_beside) {
        no_change_lane_.push_back(id);
      }
    }
  }
  // 计算总里程
  for (auto id : no_change_lane_) {
    auto lane = map_->laneletLayer.get(id);
    auto lane_length = lanelet::geometry::length3d(lane);
    total_mileage_ += lane_length;
  }
  global_route_msg_.set_total_mileage(total_mileage_);
  // 开始对路口进行打断
  std::vector<int> temp_seq;  // 当前路段
  for (auto id : no_change_lane_) {
    auto cur_dir = FindJunctionDirection(id);
    // if (map_->laneletLayer.get(id).attributeOr("subtype","unknown") ==
    // "virtual")
    std::string lanetype =
        map_->laneletLayer.get(id).attributeOr("subtype", "unknown");
    if (lanetype == "virtual" &&
        cur_dir != legionclaw::common::Direction::DIR_INVALID) {
      temp_seq.push_back(id);
      one_way_plan_.push_back(temp_seq);
      junctions_directions_.push_back(cur_dir);
      temp_seq.clear();
    } else {
      temp_seq.push_back(id);
    }
  }
  std::cout << "Junction  Size:   " << junctions_directions_.size()
            << "\n";
  std::cout << "Junction Directions : " << "\n";
  for (int i = 0; i < junctions_directions_.size(); i++) {
    std::cout << i << " : " << junctions_directions_[i] << "  " << "\n";
  }
  for (int i = 0; i < one_way_plan_.size(); i++) {
    std::cout << "one_way_plan_" << i << "\n";
    for (auto id : one_way_plan_[i]) {
      std::cout << id << " -- ";
    }
    std::cout << "\none_way_plan_" << i << "\n";
  }

  return;
}

void Routing::LoadParkingAreas() {
  int num_count = 0;
  parking_areas_.clear();
  for (auto area : map_->areaLayer) {
    if (area.hasAttribute("subtype") &&
        area.attribute("subtype") == "parking") {
      ParkStop parking_area;
      interface::Point3D parking_point;
      // std::cout << " id:  " << area.id() << "\n";
      double lat, lon, yaw, stop_lat, stop_lon, stop_yaw, stop_ele, park_ele;
      lat = area.attributeOr("lat", 0.0);
      lon = area.attributeOr("lon", 0.0);
      yaw = area.attributeOr("yaw", 0.0);
      stop_lat = area.attributeOr("stop_lat", 0.0);
      stop_lon = area.attributeOr("stop_lon", 0.0);
      stop_yaw = area.attributeOr("stop_yaw", 0.0);
      stop_ele = area.attributeOr("stop_ele", 0.0);
      park_ele = area.attributeOr("park_ele", 0.0);
      parking_area.stop_point_.set_latitude(stop_lat);
      parking_area.stop_point_.set_longitude(stop_lon);
      parking_area.stop_point_.set_ele(stop_ele);
      parking_area.stop_point_.set_heading(stop_yaw);
      interface::Polygon3D polygon;
      interface::Point3D p1, p2, p3, p4;
      auto basic_point = projector_.forward(lat, lon);
      parking_point.set_x(basic_point.x);
      parking_point.set_y(basic_point.y);
      parking_point.set_z(park_ele);
      parking_area.parking_info_.set_center_point_of_parking(parking_point);
      parking_area.parking_info_.set_parking_status(
          legionclaw::interface::ParkingStatus::PARKING_ENABLE);
      parking_area.parking_info_.set_parking_space_id(area.id());
      parking_area.parking_info_.set_parking_type(
          common::ParkingType(area.attributeOr("parking_type", 0)));
      parking_area.parking_info_.set_theta(Yaw2Angle(yaw));
      // parking_area.parking_info_.set_theta(yaw* M_PI / 180);
      parking_area.parking_info_.set_length(area.attributeOr("length", 0.0));
      parking_area.parking_info_.set_width(area.attributeOr("width", 0.0));
      double sin_v = sin(parking_area.parking_info_.theta());
      double cos_v = cos(parking_area.parking_info_.theta());
      double half_l = parking_area.parking_info_.length() / 2;
      double half_w = parking_area.parking_info_.width() / 2;
      p1.set_x(parking_point.x() + half_l * cos_v + half_w * sin_v);
      p1.set_y(parking_point.y() + half_l * sin_v - half_w * cos_v);
      p1.set_z(parking_point.z());
      // 前左点
      p2.set_x(parking_point.x() + half_l * cos_v - half_w * sin_v);
      p2.set_y(parking_point.y() + half_l * sin_v + half_w * cos_v);
      p2.set_z(parking_point.z());
      // 后左点
      p3.set_x(parking_point.x() - half_l * cos_v - half_w * sin_v);
      p3.set_y(parking_point.y() - half_l * sin_v + half_w * cos_v);
      p3.set_z(parking_point.z());
      // 后右点
      p4.set_x(parking_point.x() - half_l * cos_v + half_w * sin_v);
      p4.set_y(parking_point.y() - half_l * sin_v - half_w * cos_v);
      p4.set_z(parking_point.z());
      polygon.clear_points();
      std::vector<interface::Point3D> points;
      points.push_back(p1);
      points.push_back(p2);
      points.push_back(p3);
      points.push_back(p4);
      polygon.set_points(&points);
      parking_area.parking_info_.set_polygon(polygon);
      parking_areas_.push_back(parking_area);
      num_count++;
    }
  }
  std::cout << "Loaded Parking Space Num : 	" << num_count << "\n";
}

void Routing::Clear() {
  response_.clear_lane_list();
  first_send_ = false;
  path_id_map_.clear();
  local_ids.clear();
  all_stop_lines_.clear();
  one_way_plan_.clear();
  no_change_lane_.clear();
  junctions_directions_.clear();
  send_flag_ = false;
  mileage_enable_ = false;
  local_paths_.clear();
}

bool Routing::UsagedIndex(int lane_id) {
  bool lane_used = false;
  if (std::count(count_id.begin(), count_id.end(), lane_id)) {
    lane_used = true;
  }
  return lane_used;
}

bool Routing::MergedIndex(int lane_id) {
  bool lane_merged = false;
  if (std::count(merged_id.begin(), merged_id.end(), lane_id)) {
    lane_merged = true;
  }
  return lane_merged;
}

lanelet::BasicPolygon2d Routing::merger(
    lanelet::BasicPolygon2d existing_polygon, lanelet::Lanelet lanelet) {
  lanelet::CompoundPolygon2d polygon = lanelet.invert().polygon2d();
  lanelet::BasicPolygon2d basic_polygon = polygon.basicPolygon();
  lanelet::BasicPolygon2d res_polygon;
  std::vector<lanelet::BasicPolygon2d> output_polygon;
  boost::geometry::union_(existing_polygon, basic_polygon, output_polygon);
  if (output_polygon.size() == 0) {
    std::cout << lanelet.id() << " merged failed " << "\n";
    res_polygon = existing_polygon;
  } else if (output_polygon.size() >= 2) {
    // std::cout<<lanelet.id()<<" merged failed "<<"\n";
    res_polygon = output_polygon.front();
  } else {
    // std::cout<<lanelet.id()<<" merged successed "<<"\n";
    res_polygon = output_polygon.front();
  }
  return res_polygon;
}

lanelet::BasicPolygon2d Routing::MergeLoop(lanelet::BasicPolygon2d res_polygon,
                                           lanelet::Lanelet start_lane) {
  lanelet::Lanelets unmerged_lanes;
  if (UsagedIndex(start_lane.id()) == false) {
    // 合并左边相邻车道
    auto local_left_lanes = local_routing->lefts(start_lane);
    if (local_left_lanes.size() == 0) {
      // std::cout << "Lanelet : " << start_lane.id() << " has no left lane" <<
      // "\n";
    } else {
      lanelet::ConstLanelet const_left_lane = local_left_lanes.front();
      lanelet::Lanelet left_lane =
          localmap->laneletLayer.get(const_left_lane.id());
      if (MergedIndex(left_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, left_lane);
        // std::cout << "Lanelet : " << start_lane.id() << " merges with left
        // lane : " << left_lane.id() << "\n";
        merged_id.push_back(left_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout <<  "Lanelet : " << start_lane.id() << "'s left lane : "
        // <<left_lane.id() << " has been merged before" << "\n";
      }
      if (UsagedIndex(left_lane.id()) == false) {
        unmerged_lanes.push_back(left_lane);
      }
    }

    // 合并左边相邻不可路由车道
    auto local_adjleft_lanes = local_routing->adjacentLefts(start_lane);
    if (local_adjleft_lanes.size() == 0) {
      // std::cout << "Lanelet : " << start_lane.id() << " has no adjacentLeft
      // lane" << "\n";
    } else {
      lanelet::ConstLanelet const_adjleft_lane = local_adjleft_lanes.front();
      lanelet::Lanelet adjleft_lane =
          localmap->laneletLayer.get(const_adjleft_lane.id());
      if (MergedIndex(adjleft_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, adjleft_lane);
        // std::cout << "Lanelet : " << start_lane.id() << " merges with
        // adjacentLeft lane : " << adjleft_lane.id() << "\n";
        merged_id.push_back(adjleft_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout <<  "Lanelet : " << start_lane.id() << "'s adjacentLeft
        // lane : " <<adjleft_lane.id() << " has been merged before" <<
        // "\n";
      }
      if (UsagedIndex(adjleft_lane.id()) == false) {
        unmerged_lanes.push_back(adjleft_lane);
      }
    }

    // 合并右边相邻车道
    auto local_right_lanes = local_routing->rights(start_lane);
    if (local_right_lanes.size() == 0) {
      // std::cout << "Lanelet : " << start_lane.id() << " has no right lane" <<
      // "\n";
    } else {
      lanelet::ConstLanelet const_right_lane = local_right_lanes.front();
      lanelet::Lanelet right_lane =
          localmap->laneletLayer.get(const_right_lane.id());
      if (MergedIndex(right_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, right_lane);
        // std::cout << "Lanelet : " << start_lane.id() << " merges with right
        // lane : " << right_lane.id() << "\n";
        merged_id.push_back(right_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout <<  "Lanelet : " << start_lane.id() << "'s right lane : "
        // <<right_lane.id() << " has been merged before" << "\n";
      }
      if (UsagedIndex(right_lane.id()) == false) {
        unmerged_lanes.push_back(right_lane);
      }
    }

    // 合并右边相邻不可路由车道
    auto local_adjright_lanes = local_routing->adjacentRights(start_lane);
    if (local_adjright_lanes.size() == 0) {
      // std::cout << "Lanelet : " << start_lane.id() << " has no adjacentRight
      // lane" << "\n";
    } else {
      lanelet::ConstLanelet const_adjright_lane = local_adjright_lanes.front();
      lanelet::Lanelet adjright_lane =
          localmap->laneletLayer.get(const_adjright_lane.id());
      if (MergedIndex(adjright_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, adjright_lane);
        // std::cout << "Lanelet : " << start_lane.id() << " merges with
        // adjacentRight lane : " << adjright_lane.id() << "\n";
        merged_id.push_back(adjright_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout <<  "Lanelet : " << start_lane.id() << "'s adjacentRight
        // lane : " <<adjright_lane.id() << " has been merged before" <<
        // "\n";
      }
      if (UsagedIndex(adjright_lane.id()) == false) {
        unmerged_lanes.push_back(adjright_lane);
      }
    }

    // 合并前车道
    auto local_forward_lanes = local_routing->following(start_lane, false);
    if (local_forward_lanes.size() == 0) {
      // std::cout <<"lanelet : " << start_lane.id() << " has no forward lane"
      // << "\n";
    } else {
      lanelet::ConstLanelet const_forward_lane = local_forward_lanes.front();
      lanelet::Lanelet forward_lane =
          localmap->laneletLayer.get(const_forward_lane.id());
      if (MergedIndex(forward_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, forward_lane);
        // std::cout << "lanelet : " << start_lane.id() << " merges with forward
        // lane : " << forward_lane.id() << "\n";
        merged_id.push_back(forward_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout << "lanelet : " << start_lane.id() << "'s forward lane : "
        // <<forward_lane.id() << " has been merged before" << "\n";
      }
      if (UsagedIndex(forward_lane.id()) == false) {
        unmerged_lanes.push_back(forward_lane);
      }
    }

    // 合并后车道
    auto local_behind_lanes = local_routing->previous(start_lane, false);
    if (local_behind_lanes.size() == 0) {
      // std::cout << "lanelet : " << start_lane.id() << " has no behind lane"
      // <<"\n";
    } else {
      lanelet::ConstLanelet const_behind_lane = local_behind_lanes.front();
      lanelet::Lanelet behind_lane =
          localmap->laneletLayer.get(const_behind_lane.id());
      if (MergedIndex(behind_lane.id()) == false) {
        auto res_polygon_ = merger(res_polygon, behind_lane);
        // std::cout << "lanelet : " << start_lane.id() << " merges with behind
        // lane : " << behind_lane.id()  << "\n";
        merged_id.push_back(behind_lane.id());
        res_polygon = res_polygon_;
      } else {
        // std::cout << "lanelet : " << start_lane.id() << "'s behind lane : "
        // <<behind_lane.id() << " has been merged before" << "\n";
      }
      if (UsagedIndex(behind_lane.id()) == false) {
        unmerged_lanes.push_back(behind_lane);
      }
    }
    count_id.push_back(start_lane.id());
  }
  if (unmerged_lanes.empty()) {
    return res_polygon;
  } else {
    for (auto unmerged_lane : unmerged_lanes) {
      res_polygon = MergeLoop(res_polygon, unmerged_lane);
    }
  }
  return res_polygon;
}

void Routing::MergePolygons(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  // std::cout << "location x : " << location_.utm_position().x() << "\t"
  // <<"location y : "<< location_.utm_position().y() << "\n"; auto
  // location_point = lanelet::BasicPoint2d(location_.utm_position().x(),
  // location_.utm_position().y());
  auto low_point = lanelet::BasicPoint2d(
      location_.utm_position().x() - routing_conf_->polygon_range(),
      location_.utm_position().y() - routing_conf_->polygon_range());
  auto high_point = lanelet::BasicPoint2d(
      location_.utm_position().x() + routing_conf_->polygon_range(),
      location_.utm_position().y() + routing_conf_->polygon_range());
  auto lanes_in_range =
      map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));

  // 构建局部路由图合并车道
  localmap = lanelet::utils::createMap(lanes_in_range);
  local_routing =
      lanelet::routing::RoutingGraph::build(*localmap, *traffic_rule_);

  lanelet::Point3d temp_point(
      lanelet::utils::getId(), location_.utm_position().x(),
      location_.utm_position().y(), location_.utm_position().z());
  auto local_locate_lanepair = (lanelet2_ex::GetClosestPoint_LaneletInRange(
      lanes_in_range, temp_point, location_.heading(),
      routing_conf_->search_range(), routing_conf_->yaw_weight(),
      routing_conf_->search_height_range()));
  if (!localmap->laneletLayer.exists(local_locate_lanepair.second)) {
    // std::cout<<"not locate  lanelet in polygon gen "<<"\n";
    return;
  }
  lanelet::Lanelet start_lane =
      localmap->laneletLayer.get(local_locate_lanepair.second);
  lanelet::BasicPolygon2d polygon0 =
      start_lane.invert().polygon2d().basicPolygon();
  lanelet::BasicPolygon2d res_polygon;

  count_id.clear();
  merged_id.clear();

  merged_id.push_back(start_lane.id());
  res_polygon = MergeLoop(polygon0, start_lane);
  legionclaw::interface::Polygon2D polygon_to_send;
  polygon_to_send.clear_points();
  for (auto point : res_polygon) {
    legionclaw::interface::Point2D new_point;
    new_point.set_x(point.x());
    new_point.set_y(point.y());
    polygon_to_send.mutable_points()->push_back(new_point);
  }
  PublishPolygon2D(polygon_to_send);
}

void Routing::GlobalMsgGen(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  std::vector<int> no_change_lane;
  std::unique_lock<std::mutex> lock_(mutex_);
  no_change_lane = no_change_lane_;
  lock_.unlock();
  // if (!send_global_)
  // {
  //   // std::cout << "Send global route msg error! " << "\n";
  //   return;
  // }
  // 初始化
  global_route_msg_.clear_route();
  global_route_msg_.clear_cur_slice();
  total_mileage_ = 0;
  cur_mileage_ = 0;
  // auto location_point = lanelet::BasicPoint2d(location_.utm_position().x(),
  // location_.utm_position().y());
  auto low_point = lanelet::BasicPoint2d(
      location_.utm_position().x() - routing_conf_->local_route_visual_range(),
      location_.utm_position().y() - routing_conf_->local_route_visual_range());
  auto high_point = lanelet::BasicPoint2d(
      location_.utm_position().x() + routing_conf_->local_route_visual_range(),
      location_.utm_position().y() + routing_conf_->local_route_visual_range());
  /*
  // auto lanes_region =
  map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  lanelet::Lanelets lanes_region;
  for (auto local_path : local_paths_)
  {
    for (auto id : local_path)
    {
      lanelet::Lanelet temp_lane = map_->laneletLayer.get(id);
      lanes_region.push_back(temp_lane);
    }
  }
  auto areas = map_->areaLayer.search(lanelet::BoundingBox2d(low_point,
  high_point));

  //构造局部地图
  auto temp_map = lanelet::utils::createMap(lanes_region);

  lanelet::Point3d temp_point(lanelet::utils::getId(),
  location_.utm_position().x(), location_.utm_position().y(),
  location_.utm_position().z()); auto temp_locate_lanepair =
  (lanelet2_ex::GetClosestPoint_LaneletInRange(lanes_region, temp_point,
  location_.heading(), routing_conf_->search_range(),
  routing_conf_->yaw_weight(), routing_conf_->search_height_range()));

  double min_dist = lanelet::geometry::distance2d(temp_point,
  temp_locate_lanepair.first); if (min_dist > 10)   //
  若当前定位和规划路线的最短2D距离超过10米则重新匹配车道
  {
    lanes_region = map_->laneletLayer.search(lanelet::BoundingBox2d(low_point,
  high_point)); temp_locate_lanepair =
  lanelet2_ex::GetClosestPoint_LaneletInRange(lanes_region, temp_point,
  location_.heading(), routing_conf_->search_range(),
  routing_conf_->yaw_weight(), routing_conf_->search_height_range());
  }
  */

  // 直接使用当前定位点获取所有的车道、区域等局部地图信息
  lanelet::Point3d temp_point(
      lanelet::utils::getId(), location_.utm_position().x(),
      location_.utm_position().y(), location_.utm_position().z());
  auto lanes_region =
      map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  auto areas =
      map_->areaLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  auto temp_locate_lanepair = lanelet2_ex::GetClosestPoint_LaneletInRange(
      lanes_region, temp_point, location_.heading(),
      routing_conf_->search_range(), routing_conf_->yaw_weight(),
      routing_conf_->search_height_range());
  auto temp_map = lanelet::utils::createMap(lanes_region);

  for (auto lane : lanes_region) {
    interface::LaneletInfo lanelet_info;
    lanelet_info.set_lanelet_id(lane.id());
    lanelet_info.set_length(lanelet::geometry::length3d(lane));
    global_route_msg_.add_cur_slice(lanelet_info);
  }
  for (auto area : areas) {
    interface::LaneletInfo lanelet_info;
    lanelet_info.set_lanelet_id(area.id());
    lanelet_info.set_length(-1);
    global_route_msg_.add_cur_slice(lanelet_info);
    // std::cout<<"pub region"<<"\n";
  }

  if (!temp_map->laneletLayer.exists(temp_locate_lanepair.second)) {
    // std::cout<<"Vehicle out of routing map , no mileage info "<<"\n";
    // std::cout<<"Please send a new routing request !"<<"\n";
    std::cout << "Vehicle out of map , no mileage info " << "\n";
    return;
    // PublishGlobalRouteMsg(global_route_msg_);
  }

  auto current_lanelet =
      temp_map->laneletLayer.get(temp_locate_lanepair.second);
  auto cur_lane_length = lanelet::geometry::length3d(current_lanelet);
  current_laneletinfo_.set_lanelet_id(temp_locate_lanepair.second);
  current_laneletinfo_.set_length(cur_lane_length);
  global_route_msg_.set_current_lanelet(
      current_laneletinfo_);  // 当前所在车道的信息

  auto cur_lanes = routing_map_->besides(current_lanelet);

  std::vector<legionclaw::interface::PointLLH> global_path;
  for (auto id : no_change_lane) {
    global_lane_info_.set_lanelet_id(id);
    auto lane = map_->laneletLayer.get(id);
    auto lane_length = lanelet::geometry::length3d(lane);
    global_lane_info_.set_length(lane_length);
    global_route_msg_.add_route(
        global_lane_info_);  // 一次导航的全程道路集合，从起点到终点排序
    auto tmp_lane = lane.centerline3d();
    for (auto point : tmp_lane) {
      legionclaw::interface::PointLLH point_llh;
      auto platlon = projector_.reverse(point.x(), point.y());
      point_llh.set_lat(platlon.lat);
      point_llh.set_lon(platlon.lon);
      point_llh.set_height(point.z());
      global_path.push_back(point_llh);
    }
  }
  double min_d = DBL_MAX;
  double temp_d = 0;
  unsigned int path_index = 0;
  // auto start_point = routing_request_.key_point_list(0);
  // auto goal_point =
  // routing_request_.key_point_list(routing_request_.key_point_list_size() -
  // 1); unsigned int start_index = 0, goal_index = global_path.size() - 1;
  for (unsigned int i = 0; i < global_path.size(); i++) {
    auto tmp_p = projector_.forward(global_path[i].lat(), global_path[i].lon());
    temp_d = std::sqrt(std::pow(location_.utm_position().x() - tmp_p.x, 2) +
                       std::pow(location_.utm_position().y() - tmp_p.y, 2));
    if (temp_d < min_d) {
      path_index = i;
      min_d = temp_d;
    }
  }
  // global_route_msg_.set_global_path(global_path); //当前规划的全局路径点列表
  // global_route_msg_.set_global_path_index(path_index);
  // //当前定位所在全局路径索引

  auto cur_cline = current_lanelet.centerline3d();
  auto res = lanelet2_ex::GetRemainDistance(temp_point, cur_cline);
  double res_mileage = res.first;
  double pre_mileage = cur_lane_length - res_mileage;
  int cur_index = 0;
  for (auto lane : cur_lanes) {
    if (count(no_change_lane.begin(), no_change_lane.end(), lane.id())) {
      cur_index =
          find(no_change_lane.begin(), no_change_lane.end(), lane.id()) -
          no_change_lane.begin();
    }
  }
  for (unsigned int i = 0; i < cur_index; i++) {
    auto lane = map_->laneletLayer.get(no_change_lane[i]);
    auto length = lanelet::geometry::length3d(lane);
    cur_mileage_ += length;
  }
  cur_mileage_ += pre_mileage;
  global_route_msg_.set_cur_mileage(cur_mileage_);  // 当前里程
  PublishGlobalRouteMsg(global_route_msg_);
}

void Routing::CheckStopLine(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  std::vector<std::vector<int>> one_way_plan;
  std::vector<StopData> all_stop_lines;
  std::vector<legionclaw::common::Direction> junctions_directions;
  std::unique_lock<std::mutex> lock(mutex_);
  bool junction_check = junction_check_;
  one_way_plan = one_way_plan_;
  all_stop_lines = all_stop_lines_;
  junctions_directions = junctions_directions_;
  lock.unlock();
  if (!junction_check) {
    return;
  }
  auto location_point = lanelet::BasicPoint2d(location_.utm_position().x(),
                                              location_.utm_position().y());
  auto low_point = lanelet::BasicPoint2d(
      location_.utm_position().x() - routing_conf_->local_map_range(),
      location_.utm_position().y() - routing_conf_->local_map_range());
  auto high_point = lanelet::BasicPoint2d(
      location_.utm_position().x() + routing_conf_->local_map_range(),
      location_.utm_position().y() + routing_conf_->local_map_range());
  // auto lanes_region =
  // map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  int match_id = -1;
  double min_total = DBL_MAX;
  double temp_d_total = DBL_MAX;
  // match location lanelet and traffic_light
  // match lanelet
  // for (auto lane : lanes_region) // match lanelet in region
  for (auto lane_ids : local_paths_)  // match lanelet in local slice paths
  {
    for (auto lane_id : lane_ids) {
      auto lane = map_->laneletLayer.get(lane_id);
      if (lanelet::geometry::distance2d(location_point, lane.polygon2d()) >
          routing_conf_->search_range()) {
        continue;
      }
      lanelet::ConstLineString3d cline = lane.centerline();
      if (cline.size() < 2) {
        continue;
      }
      double d = 0;
      double min_d = DBL_MAX;
      // bool belast = false;
      lanelet::ConstPoint3d front, behind;
      for (unsigned int i = 0; i < cline.size(); i++) {
        d = lanelet::geometry::distance2d(cline[i], location_point);
        if (d < min_d) {
          min_d = d;
          if (i < cline.size() - 1) {
            front = cline[i];
            behind = cline[i + 1];
          } else {
            front = cline[i - 1];
            behind = cline[i];
            // belast = true;
          }
        }
      }
      double true_min_d = lanelet::geometry::distance2d(cline, location_point);
      double yaw = lanelet2_ex::calculate_angle(front, behind);
      double angle = location_.heading() * 180.0 / M_PI;  // 弧度转角度
      double hang_diff = calculate_diff_angle(yaw, angle);
      temp_d_total =
          hang_diff * routing_conf_->yaw_weight() / 180.0 + true_min_d;
      if (temp_d_total < min_total) {
        min_total = temp_d_total;
        match_id = lane.id();
      }
    }
  }

  cur_event_.mutable_route_fusion_info()->set_fusion_flag(
      legionclaw::common::IsValid::INVALID);  // 默认不融合直接透传
  if (map_->laneletLayer.exists(match_id)) {
    auto match_lane = map_->laneletLayer.get(match_id);
    std::string location_area =
        match_lane.attributeOr("location", "urban_default");
    // std::cout<<location_area<<"\n";
    if (location_area == std::string("tunnel")) {
      cur_event_.mutable_route_fusion_info()->set_fusion_flag(
          legionclaw::common::IsValid::VALID);
      cur_event_.mutable_route_fusion_info()->set_fusion_reason("tunnel");
    }
  }
  // match speed_limit
  double map_speed = (routing_conf_->speed_limit()) / 3.6;
  if (map_->laneletLayer.exists(match_id)) {
    auto cur_lane = map_->laneletLayer.get(match_id);
    map_speed =
        cur_lane.attributeOr("speed_limit", routing_conf_->speed_limit()) / 3.6;
  }
  if (!cur_speed_limit.limitspeed_valid_flag()) {
    cur_event_.mutable_limit_speed_info()->set_limit_speed(map_speed);
    cur_event_.mutable_limit_speed_info()->set_limitspeed_valid_flag(
        legionclaw::common::IsValid(1));
  } else {
    if (map_speed < cur_speed_limit.limit_speed()) {
      cur_event_.mutable_limit_speed_info()->set_limit_speed(map_speed);
      cur_event_.mutable_limit_speed_info()->set_limitspeed_valid_flag(
          legionclaw::common::IsValid(1));
    } else {
      cur_event_.mutable_limit_speed_info()->set_limit_speed(
          cur_speed_limit.limit_speed());
      cur_event_.mutable_limit_speed_info()->set_limitspeed_valid_flag(
          legionclaw::common::IsValid(1));
    }
  }
  // 匹配线路导航的路口信息
  //
  bool match_junction_success = false;
  unsigned int junction_index = 0;
  unsigned int junction_seq_index = 0;
  if (junctions_directions_.empty()) {
    cur_junc_.set_direction_flag(legionclaw::common::IsValid::INVALID);
  } else if (map_->laneletLayer.exists(match_id)) {
    auto match_lane = map_->laneletLayer.get(match_id);
    auto besides = routing_map_->besides(match_lane);
    for (unsigned int i = 0; i < one_way_plan.size(); i++) {
      if (match_junction_success) {
        break;
      }
      for (unsigned int j = 0; j < one_way_plan[i].size(); j++) {
        if (match_junction_success) {
          break;
        }
        int temp_id = one_way_plan[i][j];
        for (auto beside : besides) {
          if (match_junction_success) {
            break;
          }
          if (beside.id() == temp_id) {
            junction_index = i;
            junction_seq_index = j;
            match_junction_success = true;
          }
        }
      }
    }
  }
  // 计算到路口距离
  cur_junc_.set_direction_flag(legionclaw::common::IsValid::INVALID);
  if (match_junction_success) {
    auto cur_seq = one_way_plan[junction_index];
    int cur_id = cur_seq[junction_seq_index];
    if (junction_seq_index < cur_seq.size() - 1)  // 快到拐弯处
    {
      // 和当前车定位到的车道还是有一定区别，有可能是左右相邻的关系
      auto cur_lane = map_->laneletLayer.get(cur_id);
      auto cur_cline = cur_lane.centerline3d();
      lanelet::Point3d cur_point(
          lanelet::utils::getId(), location_.utm_position().x(),
          location_.utm_position().y(), location_.utm_position().z());
      auto res = lanelet2_ex::GetRemainDistance(cur_point, cur_cline);
      double dis = res.first;

      for (unsigned int i = junction_seq_index + 1; i < cur_seq.size() - 1;
           i++) {
        auto temp_lane = map_->laneletLayer.get(cur_seq[i]);
        dis += lanelet::geometry::length3d(temp_lane);
      }
      cur_junc_.set_distance_to_junction(dis);
      navi_info_msg_.set_distance_to_cross(
          dis -
          routing_conf_->stop_line_distance());  // navi_info 距离路口的距离
      cur_junc_.set_distance_to_stop(dis - routing_conf_->stop_line_distance());
      if (dis < routing_conf_->junction_distance()) {
        cur_junc_.set_direction_flag(legionclaw::common::IsValid::VALID);
        auto curr_dir = junctions_directions_[junction_index];
        cur_junc_.set_direction(curr_dir);
        // navi_info_msg_ crossing_behavior 路口决策行为：0: 直行; 1: 左转; 2:
        // 右转; 3: 掉头
        navi_info_msg_.set_crossing_behavior(0);  // 默认直行,暂无掉头
        if (curr_dir == legionclaw::common::Direction::LEFT) {
          navi_info_msg_.set_crossing_behavior(1);
        } else if (curr_dir == legionclaw::common::Direction::RIGHT) {
          navi_info_msg_.set_crossing_behavior(2);
        }
      }
    }
  }
  // match traffic light
  bool match_success = false;
  for (auto data : all_stop_lines) {
    if (match_success) {
      break;
    }
    for (auto id : data.lane_ids) {
      if (match_id == id) {
        match_success = true;
        // cur_junc_.set_direction(data.direction);
        cur_junc_.set_light_flag(legionclaw::common::IsValid::VALID);
        cur_junc_.set_stop_line(data.stop_points);

        legionclaw::interface::TrafficLightMsg TrafficLightsOutput;
        std::vector<legionclaw::interface::TrafficLight> lights_vec;
        auto cur_lane = map_->laneletLayer.get(match_id);
        auto traffic_lights =
            cur_lane.regulatoryElementsAs<lanelet::TrafficLight>();
        for (auto trafic_light : traffic_lights) {
          // auto lanes = map_->laneletLayer.findUsages(light);
          // auto stopLine = light->stopLine();
          lanelet::LineStringsOrPolygons3d lights_linestr =
              trafic_light->trafficLights();

          for (int i = 0; i < lights_linestr.size(); i++) {
            lanelet::LineString3d light_line =
                map_->lineStringLayer.get(lights_linestr[i].id());
            for (int j = 0; j < light_line.size(); j++) {
              // 根据地图中交通灯的转向信息,设置消息通信中的交通灯转向信息
              // 后续封装成函数调用,目前对于交通灯实际长宽等属性,消息未定义后续补充完善
              legionclaw::interface::TrafficLightType lightType =
                  legionclaw::interface::TrafficLightType::TYPE_UNKNOWN;
              std::string light_dir =
                  light_line[j].attributeOr("direction", "unknown");
              if (light_dir == "unknown") {
                lightType = legionclaw::interface::TrafficLightType::TYPE_UNKNOWN;
              } else if (light_dir == "straight") {
                lightType = legionclaw::interface::TrafficLightType::STRAIGHT;
              } else if (light_dir == "left") {
                lightType = legionclaw::interface::TrafficLightType::TURN_LEFT;
              } else if (light_dir == "right") {
                lightType = legionclaw::interface::TrafficLightType::TURN_RIGHT;
              } else if (light_dir == "straight_left") {
                lightType =
                    legionclaw::interface::TrafficLightType::STRAIGHT_TURN_LEFT;
              } else if (light_dir == "straight_right") {
                lightType =
                    legionclaw::interface::TrafficLightType::STRAIGHT_TURN_RIGHT;
              } else {
                lightType = legionclaw::interface::TrafficLightType::TYPE_UNKNOWN;
              }
              /* // 后续根据需求新增其他交通灯类型
              // legionclaw::interface::TrafficLightType::CIRCULAR;
              // legionclaw::interface::TrafficLightType::PEDESTRIAN;
              // legionclaw::interface::TrafficLightType::CYCLIST; */

              double light_length = light_line[j].attributeOr("length", 0.0);
              double light_width = light_line[j].attributeOr("width", 0.0);

              legionclaw::interface::Point3D light_pt;
              light_pt.set_x(light_line[j].attributeOr("x", 0.0));
              light_pt.set_y(light_line[j].attributeOr("y", 0.0));
              light_pt.set_z(light_line[j].z());

              legionclaw::interface::TrafficLight trafficLight;
              trafficLight.set_position(light_pt);
              trafficLight.set_type(lightType);
              if (lightType !=
                  legionclaw::interface::TrafficLightType::TYPE_UNKNOWN) {
                lights_vec.push_back(trafficLight);
              }
            }
          }
        }
        TrafficLightsOutput.set_traffic_light(lights_vec);
        bool contain_lights = lights_vec.size() > 0;
        if (contain_lights) {
          TrafficLightsOutput.set_contain_lights(contain_lights);
          PublishTrafficLightMsgOutput(TrafficLightsOutput);
        }

        // gps===>odom
        if (routing_conf_->use_tf_flag()) {
          if (transformer_init_) {
            for (int i = 0; i < cur_junc_.stop_line_size(); i++) {
              Eigen::Vector3d input(cur_junc_.stop_line()[i].x(),
                                    cur_junc_.stop_line()[i].y(),
                                    cur_junc_.stop_line()[i].z());
              Eigen::Vector3d output = transformer_.trans(input);
              cur_junc_.mutable_stop_line()->at(i).set_x(output.x());
              cur_junc_.mutable_stop_line()->at(i).set_y(output.y());
            }
          } else {
            std::cout << "tf not init" << "\n";
          }
        }

        cur_junc_.set_light_color(
            legionclaw::common::TrafficLightColor::COLOR_UNKNOWN);
        break;
      }
    }
  }
  // macth color
  // if (!match_success)
  // {
  //   cur_junc_.set_light_flag(legionclaw::common::IsValid::INVALID);
  //   cur_event_.set_junction_info(cur_junc_);
  //   PublishTrafficEventsOutput(cur_event_);
  //   return;
  // }

  // 根据交通灯类型颜色等和导航信息匹配
  cur_junc_.set_light_flag(legionclaw::common::IsValid::INVALID);
  navi_info_msg_.set_traffic_light_stop(false);  // 默认不需要停车
  if (traffic_light_msg_.has_traffic_light() &&
      cur_junc_
          .direction_flag())  // direction_flag为1,才存在路口和对应的导航信息
  {
    std::vector<legionclaw::interface::TrafficLight> trafficLights =
        traffic_light_msg_.traffic_light();
    int light_idx = MatchTrafficlight(cur_junc_.direction(), trafficLights);
    if (light_idx != -1) {
      legionclaw::common::TrafficLightColor light_color =
          trafficLights[light_idx].color();
      cur_junc_.set_light_flag(legionclaw::common::IsValid::VALID);
      cur_junc_.set_light_color(light_color);
      if (light_color == legionclaw::common::TrafficLightColor::RED) {
        navi_info_msg_.set_traffic_light_stop(true);
      }
    }
  }

  cur_event_.set_junction_info(cur_junc_);
  // legionclaw::interface::Header header;
  // header.set_stamp(TimeTool::Now2TmeStruct());
  // cur_event_.set_header(header);
  cur_event_.set_header(location_.header());
  // cur_event_.mutable_route_fusion_info()->set_fusion_flag(legionclaw::common::IsValid::VALID);
  // cur_event_.mutable_route_fusion_info()->set_fusion_reason("test_odom");

  PublishTrafficEventsOutput(cur_event_);
}

void Routing::LanelinesGen(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  if (!local_paths_.size()) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  lock.unlock();

  // publish routing lanelines

  // local_lane_list_.clear_camera_laneline();
  // local_lane_list_.set_header(location_.header());

  lanelet::Lanelets local_lanes;
  legionclaw::interface::LaneList local_lanelist = legionclaw::interface::LaneList();
  std::vector<legionclaw::interface::LaneLine> local_lanelines;

  lanelet::BasicPoint2d location_point = lanelet::BasicPoint2d(
      location_.utm_position().x(), location_.utm_position().y());
  // auto low_point = lanelet::BasicPoint2d(location_.utm_position().x() -
  // routing_conf_->polygon_range(), location_.utm_position().y() -
  // routing_conf_->polygon_range()); auto high_point =
  // lanelet::BasicPoint2d(location_.utm_position().x() +
  // routing_conf_->polygon_range(), location_.utm_position().y() +
  // routing_conf_->polygon_range()); auto lanes_in_range =
  // map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  // 构建一定范围内的局部路由图
  // auto tmp_localmap = lanelet::utils::createMap(lanes_in_range);
  // auto tmp_local_routing =
  // lanelet::routing::RoutingGraph::build(*tmp_localmap, *traffic_rule_);

  // for (auto local_path : local_paths_)
  // {
  //   for (auto id : local_path)
  //   {
  //     lanelet::Lanelet temp_lane = map_->laneletLayer.get(id);
  //     local_lanes.push_back(temp_lane);
  //   }
  // }
  // //构建全局导航路线上的局部路由图
  // auto tmp_localmap = lanelet::utils::createMap(local_lanes);
  // auto tmp_local_routing =
  // lanelet::routing::RoutingGraph::build(*tmp_localmap, *traffic_rule_);

  std::vector<std::vector<int>> leftmost_lines;
  std::vector<std::vector<int>> rightmost_lines;
  std::vector<std::vector<int>> lane_lines;
  for (int i = 0; i < local_paths_.size(); i++) {
    lanelet::Lanelet temp_startlane =
        map_->laneletLayer.get(local_paths_[i][0]);
    auto local_leftRelations = routing_map_->leftRelations(
        temp_startlane);  // 获取车道左边的所有车道及邻接关系
    auto local_rightRelations = routing_map_->rightRelations(
        temp_startlane);  // 获取车道右边的所有车道及邻接关系
    std::vector<int> lane_leftline;
    std::vector<int> lane_rightline;
    for (auto id : local_paths_[i]) {
      lanelet::Lanelet temp_lane = map_->laneletLayer.get(id);
      // int64_t leftbound_id = temp_lane.leftBound().id();
      // int64_t rightbound_id = temp_lane.rightBound().id();
      lane_leftline.push_back(temp_lane.leftBound().id());
      lane_rightline.push_back(temp_lane.rightBound().id());
    }
    if (local_leftRelations.size() == 0) {
      leftmost_lines.push_back(lane_leftline);
    }
    if (local_rightRelations.size() == 0) {
      rightmost_lines.push_back(lane_rightline);
    }
    // if (std::find_if(lane_lines.begin(), lane_lines.end(), lane_leftline) !=
    // lane_lines.end())
    // {
    //   lane_lines.push_back(lane_leftline);
    // }
    // if (std::find_if(lane_lines.begin(), lane_lines.end(), lane_rightline) !=
    // lane_lines.end())
    // {
    //   lane_lines.push_back(lane_rightline);
    // }
    bool left_use_flag = true, right_use_flag = true;
    for (auto lane_line : lane_lines) {
      if (lane_leftline[0] == lane_line[0])  // 判断车道左边线是否已经记录
      {
        left_use_flag = false;
      }
      if (lane_rightline[0] == lane_line[0])  // 判断车道右边线是否已经记录
      {
        right_use_flag = false;
      }
    }
    if (left_use_flag) {
      lane_lines.push_back(lane_leftline);
    }
    if (right_use_flag) {
      lane_lines.push_back(lane_rightline);
    }
  }

  double distance_threshold = 141;   // 距离阈值
  for (auto lane_line : lane_lines)  // 车道线
  {
    legionclaw::interface::LaneLine temp_laneline = legionclaw::interface::LaneLine();
    std::vector<legionclaw::interface::Point3D> laneline_points;

    for (auto line_id : lane_line) {
      lanelet::LineString3d line = map_->lineStringLayer.get(line_id);
      std::string line_attr = line.attributeOr("subtype", "solid");
      temp_laneline.set_lane_type(
          legionclaw::interface::LaneLine::LaneType::SOLID);  // 默认给实线
      if (line_attr == "Dashed") {
        temp_laneline.set_lane_type(
            legionclaw::interface::LaneLine::LaneType::DASHED);
      }
      for (int i = 0; i < line.size(); i++) {
        lanelet::Point3d line_point = line[i];
        double d = lanelet::geometry::distance2d(line_point, location_point);
        if (d < distance_threshold) {
          legionclaw::interface::Point3D tmp_p;
          tmp_p.set_x(line_point.x());
          tmp_p.set_y(line_point.y());
          tmp_p.set_z(line_point.z());
          laneline_points.push_back(tmp_p);
        }
      }
    }
    temp_laneline.set_pts_abs(laneline_points);
    // temp_laneline.set_lane_type(legionclaw::interface::LaneLine::LaneType::SOLID);
    temp_laneline.set_lane_color(legionclaw::interface::LaneLine::LaneColor::WHITE);
    local_lanelines.push_back(temp_laneline);
  }

  for (auto leftmost_line : leftmost_lines)  // 道路左边线
  {
    legionclaw::interface::LaneLine temp_laneline = legionclaw::interface::LaneLine();
    std::vector<legionclaw::interface::Point3D> laneline_points;

    for (auto line_id : leftmost_line) {
      lanelet::LineString3d line = map_->lineStringLayer.get(line_id);
      for (int i = 0; i < line.size(); i++) {
        lanelet::Point3d line_point = line[i];
        double d = lanelet::geometry::distance2d(line_point, location_point);
        if (d < distance_threshold) {
          legionclaw::interface::Point3D tmp_p;
          tmp_p.set_x(line_point.x());
          tmp_p.set_y(line_point.y());
          tmp_p.set_z(line_point.z());
          laneline_points.push_back(tmp_p);
        }
      }
    }
    temp_laneline.set_pts_abs(laneline_points);
    temp_laneline.set_lane_type(legionclaw::interface::LaneLine::LaneType::CURB);
    temp_laneline.set_lane_color(
        legionclaw::interface::LaneLine::LaneColor::LANE_COLOR_UNKNOWN);
    local_lanelines.push_back(temp_laneline);
  }

  for (auto rightmost_line : rightmost_lines)  // 道路右边线
  {
    legionclaw::interface::LaneLine temp_laneline = legionclaw::interface::LaneLine();
    std::vector<legionclaw::interface::Point3D> laneline_points;

    for (auto line_id : rightmost_line) {
      lanelet::LineString3d line = map_->lineStringLayer.get(line_id);
      for (int i = 0; i < line.size(); i++) {
        lanelet::Point3d line_point = line[i];
        double d = lanelet::geometry::distance2d(line_point, location_point);
        if (d < distance_threshold) {
          legionclaw::interface::Point3D tmp_p;
          tmp_p.set_x(line_point.x());
          tmp_p.set_y(line_point.y());
          tmp_p.set_z(line_point.z());
          laneline_points.push_back(tmp_p);
        }
      }
    }
    temp_laneline.set_pts_abs(laneline_points);
    temp_laneline.set_lane_type(legionclaw::interface::LaneLine::LaneType::CURB);
    temp_laneline.set_lane_color(
        legionclaw::interface::LaneLine::LaneColor::LANE_COLOR_UNKNOWN);
    local_lanelines.push_back(temp_laneline);
  }

  local_lanelist.set_header(location_.header());
  local_lanelist.set_camera_laneline(local_lanelines);
  PublishLaneList(local_lanelist);
}

void Routing::GuideInfoGen(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  if (!local_paths_.size()) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  lock.unlock();
  // if(routing_conf_->enable_guide_info_send())
  if (1) {
    auto location_point = lanelet::BasicPoint2d(location_.utm_position().x(),
                                                location_.utm_position().y());
    // auto low_point = lanelet::BasicPoint2d(location_.utm_position().x() -
    // routing_conf_->local_map_range(), location_.utm_position().y() -
    // routing_conf_->local_map_range()); auto high_point =
    // lanelet::BasicPoint2d(location_.utm_position().x() +
    // routing_conf_->local_map_range(), location_.utm_position().y() +
    // routing_conf_->local_map_range()); auto lanes_region =
    // map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
    int match_id = -1;
    double min_dist = DBL_MAX;
    // match lanelet
    lanelet::Lanelets local_lanes;
    for (auto local_path : local_paths_) {
      for (auto id : local_path) {
        lanelet::Lanelet lane = map_->laneletLayer.get(id);
        local_lanes.push_back(lane);
        if (lanelet::geometry::distance2d(location_point, lane.polygon2d()) >
            routing_conf_->search_range()) {
          continue;
        }
        lanelet::ConstLineString3d cline = lane.centerline();
        if (cline.size() < 2) {
          continue;
        }
        double true_min_d =
            lanelet::geometry::distance2d(cline, location_point);
        if (true_min_d < min_dist) {
          min_dist = true_min_d;
          match_id = lane.id();
        }
      }
    }
    if (!map_->laneletLayer.exists(match_id)) {
      std::cout << "The vehicle is not positioned within the lane of the "
                   "routing graph !!! "
                << "\n";
      return;
    }
    lanelet::Lanelet cur_lane = map_->laneletLayer.get(match_id);
    auto cur_cline = cur_lane.centerline3d();
    lanelet::Point3d cur_point(
        lanelet::utils::getId(), location_.utm_position().x(),
        location_.utm_position().y(), location_.utm_position().z());
    std::pair<double, int> residual_info =
        lanelet2_ex::GetRemainDistance(cur_point, cur_cline);
    double residual_dis = residual_info.first;
    std::string cur_lanetype = cur_lane.attributeOr("subtype", "unknown");
    std::string cur_roadtype = cur_lane.attributeOr("roadtype", "unknown");
    // std::cout<<"cur_lane.id = "<<cur_lane.id()<<"  lanetype: " <<
    // cur_lanetype << "\nresidual_length: " << residual_dis <<"\n";

    // 构建局部路由图合并车道
    auto tmp_localmap = lanelet::utils::createMap(local_lanes);
    auto tmp_local_routing =
        lanelet::routing::RoutingGraph::build(*tmp_localmap, *traffic_rule_);

    legionclaw::interface::GuideInfo guide_info = legionclaw::interface::GuideInfo();
    legionclaw::interface::GuideRoad current_guide_road =
        legionclaw::interface::GuideRoad();
    legionclaw::interface::GuideRoad next_guide_road =
        legionclaw::interface::GuideRoad();

    guide_info.set_round_status(0);  // 环岛通行状态 0：未知或非环岛状态
    guide_info.set_roads_status(3);  // 跨路口转向引导无效状态

    // 路口通行状态判定
    if (cur_lanetype == "virtual") {
      current_guide_road.set_road_type(2);    // 2：交叉路口
      guide_info.set_intersection_status(2);  // 2：交叉路口通行状态
    } else if (cur_roadtype == "roundabout") {
      current_guide_road.set_road_type(1);    // 1：环岛
      guide_info.set_round_status(2);         // 环岛中行驶
      guide_info.set_intersection_status(1);  // 1：非交叉路口状态
    } else {
      current_guide_road.set_road_type(0);    // 0：未知或一般道路
      guide_info.set_intersection_status(1);  // 1：非交叉路口状态
    }
    // 当前道路的引导信息
    current_guide_road.set_road_id(cur_lane.id());
    current_guide_road.set_turn_type(FindJunctionDirection(cur_lane.id()));

    // 规划路线上下一条道路的引导信息
    auto next_lane = tmp_local_routing->following(cur_lane);
    next_guide_road.set_road_id(0);
    next_guide_road.set_road_type(0);
    next_guide_road.set_turn_type(0);

    if (!next_lane.empty()) {
      next_guide_road.set_road_id(next_lane[0].id());
      next_guide_road.set_turn_type(FindJunctionDirection(next_lane[0].id()));
      std::string next_lanetype =
          next_lane[0].attributeOr("subtype", "unknown");
      std::string next_roadtype =
          next_lane[0].attributeOr("roadtype", "unknown");
      if (next_lanetype == "virtual") {
        next_guide_road.set_road_type(2);  // 2：交叉路口
      } else if (next_roadtype == "roundabout") {
        next_guide_road.set_road_type(1);  // 1：环岛
      } else {
        next_guide_road.set_road_type(0);  // 0：未知或一般道路
      }
      // 环岛通行状态信息
      if (cur_roadtype != "roundabout" && next_roadtype == "roundabout") {
        guide_info.set_round_status(1);  // 即将驶入环岛
      }
      // if (cur_roadtype == "roundabout" && next_roadtype == "roundabout")
      // {
      //   guide_info.set_round_status(2); //
      //   环岛中行驶(next_road不存在时则没有该状态)
      // }
      if (cur_roadtype == "roundabout" && next_roadtype != "roundabout") {
        guide_info.set_round_status(3);  // 即将驶出环岛
      }
    }

    // 规划路线上当前道路的引导信息
    guide_info.set_header(location_.header());
    guide_info.set_current_road(current_guide_road);
    guide_info.set_next_road(next_guide_road);
    guide_info.set_next_dis(residual_dis);
    // guide_info.set_round_status(0);   // 环岛通行状态，暂时无效
    // guide_info.set_intersection_status(0);
    // guide_info.set_roads_status(3);   // 跨路口转向引导无效状态

    PublishGuideInfo(guide_info);
  }
}

void Routing::NaviInfoPublisher(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!map_loaded_) {
    return;
  }
  if (!local_paths_.size()) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  lock.unlock();

  auto location_point = lanelet::BasicPoint2d(location_.utm_position().x(),
                                              location_.utm_position().y());
  int match_id = -1;
  double min_dist = DBL_MAX;
  // match lanelet
  for (auto local_path : local_paths_) {
    for (auto id : local_path) {
      lanelet::Lanelet lane = map_->laneletLayer.get(id);
      if (lanelet::geometry::distance2d(location_point, lane.polygon2d()) >
          routing_conf_->search_range()) {
        continue;
      }
      lanelet::ConstLineString3d cline = lane.centerline();
      if (cline.size() < 2) {
        continue;
      }
      double true_min_d = lanelet::geometry::distance2d(cline, location_point);
      if (true_min_d < min_dist) {
        min_dist = true_min_d;
        match_id = lane.id();
      }
    }
  }
  if (!map_->laneletLayer.exists(match_id)) {
    return;
  }
  lanelet::Lanelet cur_lane = map_->laneletLayer.get(match_id);
  lanelet::routing::LaneletRelations cur_leftRelations =
      routing_map_->leftRelations(
          cur_lane);  // 获取当前车道左边的所有车道及邻接关系
  lanelet::routing::LaneletRelations cur_rightRelations =
      routing_map_->rightRelations(
          cur_lane);                // 获取当前车道右边的所有车道及邻接关系
  std::vector<int> cur_road_slice;  // 当前车道所在的道路切片
  std::string cur_lanetype = cur_lane.attributeOr("subtype", "unknown");
  double cur_map_speed =
      cur_lane.attributeOr("speed_limit", routing_conf_->speed_limit()) / 3.6;
  if (cur_lanetype == "virtual") {
    navi_info_msg_.set_lane_type(1);  // 道路类型 0: 常规道路
  } else {
    navi_info_msg_.set_lane_type(0);  // 道路类型 1: 路口
  }
  navi_info_msg_.set_lane_count(cur_leftRelations.size() +
                                cur_rightRelations.size() +
                                1);  // 当前道路切面车道个数
  navi_info_msg_.set_lane_index(
      cur_leftRelations.size());  // 当前车所在道路索引, 最左侧为0

  // 获取当前车道所在的道路切片
  for (auto left_relation : cur_leftRelations) {
    cur_road_slice.push_back(left_relation.lanelet.id());
  }
  cur_road_slice.push_back(cur_lane.id());
  for (auto right_relation : cur_rightRelations) {
    cur_road_slice.push_back(right_relation.lanelet.id());
  }

  // 获取当前道路切片中目标行驶车道索引
  navi_info_msg_.set_lane_target(
      navi_info_msg_.lane_index());  // 目标车道索引,默认给当前车道,最左侧为0
  for (int i = 0; i < cur_road_slice.size(); i++) {
    auto it = std::find(no_change_lane_.begin(), no_change_lane_.end(),
                        cur_road_slice[i]);
    if (it != no_change_lane_.end()) {
      auto it_index = std::distance(no_change_lane_.begin(), it);
      auto it_lane = map_->laneletLayer.get(no_change_lane_[it_index]);
      auto it_following_lanes = routing_map_->following(it_lane);
      if (!it_following_lanes.empty()) {
        if (it_following_lanes.front().id() == no_change_lane_[it_index + 1]) {
          navi_info_msg_.set_lane_target(i);  // 目标行驶车道索引, 最左侧为0
          break;
        }
      }
      if (it_index == no_change_lane_.size() - 1) {
        continue;
      }
      auto next_it_lane = map_->laneletLayer.get(no_change_lane_[it_index + 1]);
      auto tmp_lanes = routing_map_->previous(next_it_lane);
      if (!tmp_lanes.empty()) {
        auto tmp_it = std::find(cur_road_slice.begin(), cur_road_slice.end(),
                                tmp_lanes.front().id());
        if (tmp_it != cur_road_slice.end()) {
          auto tmp_it_index = std::distance(cur_road_slice.begin(), tmp_it);
          navi_info_msg_.set_lane_target(tmp_it_index);
          break;
        }
      }
    }
  }

  navi_info_msg_.set_road_speed(cur_map_speed);  // 地图限速
  navi_info_msg_.set_turning_speed(DBL_MAX);     // 转弯建议速度,暂无置为极大值
  navi_info_msg_.set_turning_deriction(
      0);  // 转弯完成后道路朝向,单位:角度,暂无置为0
  navi_info_msg_.set_distance_to_stop(global_route_msg_.total_mileage() -
                                      cur_mileage_);  // 全局剩余里程
  navi_info_msg_.set_header(location_.header());

  PublishNaviInfoMsg(navi_info_msg_);
}

void Routing::MainLoop(void* param) {
  if (!function_activation_) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  bool bMakeNewPlan = false;
  if (new_request_received_) {
    if (routing_conf_->enable_replan()) {
      bMakeNewPlan = true;
      new_request_received_ = false;
    }

    if (bMakeNewPlan) {
      Clear();
      bool bNewPlan = false;
      lock.lock();
      if (GlobalPlan(routing_request_) ==
          interface::RoutingResponse::PlanStatus::PLAN_SUCCESS) {
        bNewPlan = true;
      }
      lock.unlock();
      if (bNewPlan) {
        junction_check_ = false;
        // LoadStopLines();
        stop_check_ = false;
        bMakeNewPlan = false;
        // send_global_ = true;
        send_flag_ = true;   // 规划完成初始下发第一个切片
        first_send_ = true;  // 第一次下发标志
        goal_pose_ = ll2xy(routing_request_.key_point_list(
            routing_request_.key_point_list_size() - 1));
        stop_info_ = GenStopInfo(goal_pose_);
        // stop_info_.mutable_header()->set_seq(unique::getStopInfoSeq());
        mileage_enable_ = true;
      } else {
        std::cout << "Plan Status Error : REQUEST_ERROR" << "\n";
        PublishRoutingResponse(response_);
      }
    }
  }
  while (send_flag_) {
    lock.lock();
    if (GetSection(no_change_lane_, local_paths_, location_,
                   routing_conf_->back_length(),
                   routing_conf_->forward_length()) ==
        interface::RoutingResponse::PlanStatus::OUT_OF_MAP) {
      section_mileage_ = routing_conf_->mileage_length() / 2;
      send_flag_ = false;
      std::cout << "Plan Status Error : OUT_OF_MAP " << "\n";
      PublishRoutingResponse(response_);
      return;
    }
    LoadStopLines();
    Section2Response(local_paths_, response_);
    if (first_send_) {
      response_.set_replan_flag(
          interface::RoutingResponse::ReplanFlag::REPLAN_FLAG_HUMAN);
      first_send_ = false;
    } else {
      response_.set_replan_flag(
          interface::RoutingResponse::ReplanFlag::REPLAN_FLAG_NONE);
    }
    response_.mutable_header()->set_seq(unique::getRoutingResponseSeq());
    section_mileage_ = 0;
    send_flag_ = false;
    std::cout << "Plan Status Success : PLAN_SUCCESS " << "\n";
    PublishRoutingResponse(response_);
    std::cout << "publish section ok !" << "\n";
    usleep(1000 * routing_conf_->sleep_time());
    if (match_stop_points_) {
      std::cout << "Stop Path Check Num : " << stop_indexs_.size() << "\n";
      stop_check_ = true;
    } else {
      std::cout << "Stop Point Not Match" << "\n";
    }
    lock.unlock();
  }
  return;
}

void Routing::LoadLanelet2Map() {
  if (!map_loaded_) {
    std::cout << "Loading Map" << "\n";
    double origin_lat = routing_conf_->origin_lat();
    double origin_lon = routing_conf_->origin_lon();
    std::string osm_map_path = routing_conf_->map_file_path();
    std::cout << osm_map_path << "\n";
    lanelet::projection::UtmProjector projector(
        lanelet::Origin({origin_lat, origin_lon}));
    map_ = lanelet::load(osm_map_path, projector);
    routing_map_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rule_);
    const_map_ = routing_map_->passableSubmap()->laneletMap();
    map_loaded_ = true;
    std::cout << "Map Loaded !" << "\n";
    // LoadParkingAreas();
  }
  return;
}

interface::KeyPoint Routing::ll2xy(const interface::KeyPoint keypoint) {
  auto basic_point =
      projector_.forward(keypoint.latitude(), keypoint.longitude());
  interface::KeyPoint new_key;
  new_key.set_id(keypoint.id());
  new_key.set_heading(keypoint.heading());
  new_key.set_longitude(basic_point.x);
  new_key.set_latitude(basic_point.y);
  new_key.set_ele(keypoint.ele());
  new_key.set_name(keypoint.name());
  return new_key;
}

interface::StopInfo Routing::GenStopInfo(const interface::KeyPoint goal) {
  interface::StopPoint stop_point;
  interface::Point3D point_3d;
  point_3d.set_x(goal.longitude());
  point_3d.set_y(goal.latitude());
  // odometry坐标系
  if (routing_conf_->enable_3d_map()) {
    point_3d.set_z(goal.ele());
  } else {
    point_3d.set_z(0);
  }

  stop_point.set_point(point_3d);
  stop_point.set_stop_distance(routing_conf_->stop_distance());
  // 统一角度坐标系，在此更改
  double angle = goal.heading();
  // double angle = goal.heading()*M_PI/180.0;
  stop_point.set_theta(angle);
  stop_point.set_type(0);
  interface::StopInfo stopinfo;
  std::vector<interface::StopPoint> stop_points;
  stop_points.clear();
  stop_points.push_back(stop_point);
  stopinfo.set_stop_points(&stop_points);
  // header
  interface::Header header;
  header.set_frame_id(routing_conf_->app_name());
  if (routing_conf_->enable_system_time()) {
    header.set_stamp(common::TimeTool::Now2TmeStruct());
  } else {
    header.set_stamp(current_pose_.header().stamp());
  }
  header.set_seq(unique::getStopInfoSeq());
  stopinfo.set_header(header);
  return stopinfo;
}

void Routing::InStation(const interface::KeyPoint goal,
                        const interface::Location pose,
                        const interface::Chassis chassis) {
  // double distance = sqrt(pow(goal.longitude() - pose.utm_position().x(), 2) +
  // pow(goal.latitude() - pose.utm_position().y(), 2));
}

void Routing::Check_Sation_Loop(void* param) {
  if (!function_activation_) {
    return;
  }
  if (!stop_check_) {
    return;
  }

  double stop_distance = chassis_.speed_mps() * chassis_.speed_mps() /
                         routing_conf_->normal_deceleration();
  if (stop_distance < routing_conf_->stop_distance()) {
    stop_distance = routing_conf_->stop_distance();
  }
  bool res = false;
  for (unsigned int i = 0; i < stop_indexs_.size(); i++) {
    bool temp_res = MatchPointInPath(
        location_.utm_position().x(), location_.utm_position().y(),
        location_.utm_position().z(), location_.heading(),
        routing_conf_->search_range(), routing_conf_->search_height_range(),
        stop_distance, stop_indexs_[i], stop_lanes_[i]);
    if (temp_res) {
      res = true;
      break;
    }
  }
  if (res) {
    // gps==>odom
    if (routing_conf_->use_tf_flag()) {
      if (transformer_init_) {
        Eigen::Vector3d input(stop_info_.stop_points().front().point().x(),
                              stop_info_.stop_points().front().point().y(),
                              stop_info_.stop_points().front().point().z());
        Eigen::Vector3d output = transformer_.trans(input);
        stop_info_.mutable_stop_points()->at(0).mutable_point()->set_x(
            output.x());
        stop_info_.mutable_stop_points()->at(0).mutable_point()->set_y(
            output.y());
        double yaw_diff = transformer_.get_yaw();
        double theta = stop_info_.stop_points().front().theta();
        std::cout << "stop theta in utm " << theta << "\n";
        std::cout << "yaw_diff " << yaw_diff << "\n";
        std::cout << "stop theta in odom " << theta + yaw_diff << "\n";
        stop_info_.mutable_stop_points()->at(0).set_theta(theta + yaw_diff);
      } else {
        std::cout << "tf not init" << "\n";
        return;
      }
    }
    PublishStopInfo(stop_info_);
    std::cout << "publish stop info ok !" << "\n";
    stop_check_ = false;
    mileage_enable_ = false;
  }
  return;
}

bool Routing::SingleRoad2FullRoute(const std::vector<int>& single_road,
                                   std::vector<std::vector<int>>& local_route) {
  lanelet::ConstLanelets sequence_start_lanes;  // 起点车道集合
  std::vector<int> start_ids;                   // 起点车道id集合
  lanelet::ConstLanelets sequence_lanes;        // 所有路由车道集合
  std::vector<int> all_ids;                     // 所有路由车道id集合

  // single road 首尾车道
  auto start_lane = map_->laneletLayer.get(single_road.front());
  auto end_lane = map_->laneletLayer.get(single_road.back());

  std::vector<lanelet::Ids> paths_ids;  // path车道id集合

  std::vector<std::vector<int>> lanelets_ids;  // 所有path车道id集合
  path_id_map_.clear();
  lanelet::ConstLanelet temp;
  lanelet::ConstLanelets lanelet_besides;

  // 寻找最短路径上的可路由车道
  for (unsigned int i = 0; i < single_road.size();
       i++)  // to find the all possible lanelets around the shortest path
  {
    temp = map_->laneletLayer.get(single_road[i]);
    lanelet_besides = routing_map_->besides(temp);
    // 筛选可路由的相邻不可换道车道
    lanelet::ConstLanelets adj_lefts;
    lanelet::ConstLanelets adj_rights;
    auto adjacent_lefts = routing_map_->adjacentLefts(temp);
    for (auto lane : adjacent_lefts) {
      if (route->contains(lane)) {
        adj_lefts.push_back(lane);
      }
    }
    auto adjacent_rights = routing_map_->adjacentRights(temp);
    for (auto lane : adjacent_rights) {
      if (route->contains(lane)) {
        adj_rights.push_back(lane);
      }
    }
    // 筛选的相邻不可换道车道集合加入相邻车道集合中
    if (!adj_lefts.empty()) {
      lanelet_besides.insert(lanelet_besides.begin(), adj_lefts.begin(),
                             adj_lefts.end());
      auto left_lanelet_besides = routing_map_->lefts(adj_lefts.front());
      if (!left_lanelet_besides.empty()) {
        lanelet_besides.insert(lanelet_besides.begin(),
                               left_lanelet_besides.begin(),
                               left_lanelet_besides.end());
      }
    }
    if (!adj_rights.empty()) {
      auto right_lanelet_besides = routing_map_->rights(adj_rights.front());
      if (!right_lanelet_besides.empty()) {
        lanelet_besides.insert(lanelet_besides.end(),
                               right_lanelet_besides.begin(),
                               right_lanelet_besides.end());
      }
      lanelet_besides.insert(lanelet_besides.end(), adj_rights.begin(),
                             adj_rights.end());
    }
    // 相邻车道加入到所有车道集合中
    for (unsigned int j = 0; j < lanelet_besides.size(); j++) {
      // if (std::find(all_ids.begin(), all_ids.end(), lanelet_besides[j].id())
      // == all_ids.end()&&route->contains(lanelet_besides[j]))
      if (std::find(all_ids.begin(), all_ids.end(), lanelet_besides[j].id()) ==
          all_ids.end()) {
        sequence_lanes.push_back(lanelet_besides[j]);
        all_ids.push_back(lanelet_besides[j].id());
        // if (!route->contains(lanelet_besides[j]))
        // {
        //   std::cout << "lanelet besides(non-route) id : " <<
        //   lanelet_besides[j].id() << "\n";
        // }
      }
    }
  }

  // lanelet::ConstLanelets temp_previous_lanes;   // 前任车道集合
  std::vector<int> temp_previous_ids;  // 前任车道id集合

  // lanelet::ConstLanelets temp_following_lanes;   // 后继车道集合
  std::vector<int> temp_following_ids;  // 后继车道id集合

  // 筛选有相同前任的车道
  for (unsigned int m = 0; m < sequence_lanes.size(); m++) {
    lanelet::ConstLanelet temp_start_lane = sequence_lanes[m];
    lanelet::ConstLanelets pre_lanes =
        routing_map_->previous(temp_start_lane, false);
    if (pre_lanes.empty()) {
      continue;
    }
    if (std::find(all_ids.begin(), all_ids.end(), pre_lanes.front().id()) ==
        all_ids.end()) {
      lanelet::ConstLanelets pre_pre_lanes =
          routing_map_->previous(pre_lanes.front(), false);
      if (pre_pre_lanes.empty()) {
        continue;
      }
      if (std::find(all_ids.begin(), all_ids.end(),
                    pre_pre_lanes.front().id()) != all_ids.end()) {
        // 一对多车道场景
        if (std::find(temp_previous_ids.begin(), temp_previous_ids.end(),
                      pre_lanes.front().id()) == temp_previous_ids.end()) {
          temp_previous_ids.push_back(pre_lanes.front().id());
        }
        lanelet::ConstLanelets temp_lanelet_besides =
            routing_map_->besides(temp_start_lane);
        int temp_index = GetIndex(temp_lanelet_besides, temp_start_lane);
        for (unsigned int p = 0; p < temp_lanelet_besides.size(); p++) {
          lanelet::ConstLanelets temp_pre_lanes =
              routing_map_->previous(temp_lanelet_besides[p], false);
          if (std::find(all_ids.begin(), all_ids.end(),
                        temp_pre_lanes.front().id()) != all_ids.end()) {
            if (std::find(temp_previous_ids.begin(), temp_previous_ids.end(),
                          temp_pre_lanes.front().id()) ==
                temp_previous_ids.end()) {
              temp_previous_ids.push_back(temp_pre_lanes.front().id());
            }
            int refer_index =
                GetIndex(temp_lanelet_besides, temp_lanelet_besides[p]);
            int rel_index = temp_index - refer_index;
            int insert_start_index =
                GetIndex(sequence_lanes, temp_pre_lanes.front());
            sequence_lanes.insert(
                sequence_lanes.begin() + insert_start_index + rel_index,
                pre_lanes.front());
            all_ids.insert(all_ids.begin() + insert_start_index + rel_index + 1,
                           pre_lanes.front().id());
            break;
          }
        }
      }
    }
  }

  // 筛选有相同后继的车道
  for (unsigned int m = 0; m < sequence_lanes.size(); m++) {
    lanelet::ConstLanelet temp_start_lane = sequence_lanes[m];
    lanelet::ConstLanelets next_lanes =
        routing_map_->following(temp_start_lane);
    if (next_lanes.empty()) {
      continue;
    }
    if (std::find(all_ids.begin(), all_ids.end(), next_lanes.front().id()) ==
        all_ids.end()) {
      lanelet::ConstLanelets next_next_lanes =
          routing_map_->following(next_lanes.front());
      if (next_next_lanes.empty()) {
        break;
      }
      if (std::find(all_ids.begin(), all_ids.end(),
                    next_next_lanes.front().id()) != all_ids.end()) {
        // 多对一车道场景
        if (std::find(temp_following_ids.begin(), temp_following_ids.end(),
                      next_next_lanes.front().id()) ==
            temp_following_ids.end()) {
          temp_following_ids.push_back(next_next_lanes.front().id());
        }
        lanelet::ConstLanelets temp_lanelet_besides =
            routing_map_->besides(temp_start_lane);
        int temp_index = GetIndex(temp_lanelet_besides, temp_start_lane);
        for (unsigned int p = 0; p < temp_lanelet_besides.size(); p++) {
          lanelet::ConstLanelets temp_fol_lanes =
              routing_map_->following(temp_lanelet_besides[p], false);
          if (std::find(all_ids.begin(), all_ids.end(),
                        temp_fol_lanes.front().id()) != all_ids.end()) {
            int refer_index =
                GetIndex(temp_lanelet_besides, temp_lanelet_besides[p]);
            int rel_index = temp_index - refer_index;
            int insert_start_index =
                GetIndex(sequence_lanes, temp_fol_lanes.front());
            sequence_lanes.insert(
                sequence_lanes.begin() + insert_start_index + rel_index,
                next_lanes.front());
            all_ids.insert(all_ids.begin() + insert_start_index + rel_index + 1,
                           next_lanes.front().id());
            break;
          }
        }
      }
    }
  }

  lanelet::ConstLanelets start_lanelet_besides =
      routing_map_->besides(start_lane);
  // 寻找所有起点车道（单纯使用是否存在前任车道搜寻）
  for (unsigned int i = 0; i < sequence_lanes.size();
       i++)  // to find all the start lanelet
  {
    lanelet::ConstLanelet temp_start_lane = sequence_lanes[i];
    lanelet::ConstLanelets pre_lanes =
        routing_map_->previous(temp_start_lane, false);
    bool start_flag = true;  // whether a lane is a sequence start ,that depends
                             // on whether any of its previous  is in the list
    if (pre_lanes.size() != 0) {
      for (unsigned int j = 0; j < pre_lanes.size(); j++) {
        if (std::find(all_ids.begin(), all_ids.end(), pre_lanes[j].id()) !=
            all_ids.end()) {
          start_flag = false;
        }
      }
      if (start_flag) {
        sequence_start_lanes.push_back(temp_start_lane);
        start_ids.push_back(temp_start_lane.id());
      }
    } else {
      sequence_start_lanes.push_back(temp_start_lane);
      start_ids.push_back(temp_start_lane.id());
    }
  }
  // 没有搜寻到起点车道是添加single road 起点车道的相邻车道（是否多余？）
  if (sequence_start_lanes.size() == 0)  // if find no start lanelet
  {
    for (unsigned int i = 0; i < start_lanelet_besides.size();
         i++)  // put start_lanelet and its besides into sequence_start_lanes
    {
      if (std::find(sequence_start_lanes.begin(), sequence_start_lanes.end(),
                    start_lanelet_besides[i]) == sequence_start_lanes.end()) {
        sequence_start_lanes.push_back(start_lanelet_besides[i]);
        start_ids.push_back(start_lanelet_besides[i].id());
      }
    }
  }

  // 根据起始lanelet的sequence来补全all_ids
  for (unsigned int i = 0; i < sequence_start_lanes.size(); i++) {
    lanelet::LaneletSequence temp_seq = route->remainingLane(
        sequence_start_lanes[i]);  // 此处会找很长的一段不换道的路径
    // 进行筛选，只找重合路段
    int end_index = -1;
    for (int k = temp_seq.size() - 1; k >= 0; k--) {
      int id = temp_seq[k].id();
      if (std::find(start_ids.begin(), start_ids.end(), id) !=
          start_ids.end()) {
        end_index = k;
        break;
      }
    }
    // 默认索引为end_index的车道的前任车道不在所有路由车道集合中，进行添加
    for (int j = 0; j <= end_index; j++) {
      auto lane = temp_seq[j];
      if (find(all_ids.begin(), all_ids.end(), lane.id()) == all_ids.end()) {
        all_ids.push_back(lane.id());
      }
    }
  }

  // 补全之后 再修正start lanes （重复上述操作）
  lanelet::ConstLanelets temp_start_lanes;
  for (auto temp_lane : sequence_start_lanes) {
    lanelet::ConstLanelets pre_lanes = routing_map_->previous(temp_lane, false);
    bool start_flag = true;  // whether a lane is a sequence start ,that depends
                             // on whether any of its previous  is in the list
    if (pre_lanes.size() != 0) {
      for (unsigned int j = 0; j < pre_lanes.size(); j++) {
        if (std::find(all_ids.begin(), all_ids.end(), pre_lanes[j].id()) !=
            all_ids.end()) {
          start_flag = false;
        }
      }
      if (start_flag) {
        temp_start_lanes.push_back(temp_lane);
      }
    } else {
      temp_start_lanes.push_back(temp_lane);
    }
  }
  sequence_start_lanes = temp_start_lanes;
  // 是否多余？是否all_ids中含有不符条件的id？
  if (sequence_start_lanes.empty()) {
    sequence_start_lanes.push_back(start_lane);
  }

  // 对起点车道进行更复杂的判断 （是否有车道数量的增减）
  lanelet::ConstLanelets fix_start_lanes;
  std::vector<int> fix_start_ids;
  std::vector<int> used_besides_ids;
  for (unsigned int i = 0; i < sequence_start_lanes.size(); i++) {
    lanelet::ConstLanelet temp_lane = sequence_start_lanes[i];
    lanelet::ConstLanelets follow_lanes =
        route->remainingLane(temp_lane).lanelets();
    // 方便测试分析问题的打印
    if (!route->contains(temp_lane)) {
      auto route_graph = route->shortestPath();
      std::cout << "*********** Route Graph-->";
      for (auto route_lane : route_graph) {
        std::cout << route_lane.id() << "-->";
      }
      std::cout << "Route Over" << "\n";
    }

    std::cout << "start lane id : " << temp_lane.id() << "\n";
    std::cout << "start following lanes size : " << follow_lanes.size()
              << "\n";
    for (auto follow_lane : follow_lanes) {
      std::cout << follow_lane.id() << "----->";
    }
    std::cout << "Following Over" << "\n";
    // remainingLane函数返回的首个车道是当前车道
    if (!follow_lanes.empty()) {
      follow_lanes.erase(follow_lanes.begin());
    } else {
      // 当前车道也没有返回，为什么？
      break;
    }
    // 当前车道无后继车道（断头路情况）
    if (follow_lanes.empty()) {
      lanelet::ConstLanelets end_lanes_besides =
          routing_map_->besides(temp_lane);
      for (auto lane : end_lanes_besides) {
        if (std::find(used_besides_ids.begin(), used_besides_ids.end(),
                      lane.id()) == used_besides_ids.end()) {
          used_besides_ids.push_back(lane.id());
          lanelet::ConstLanelets temp_follow_lanes =
              route->remainingLane(lane).lanelets();
          // 为什么会为空，只能是besides车道为空，怀疑是内存泄漏，需要检查
          if (!temp_follow_lanes.empty()) {
            temp_follow_lanes.erase(temp_follow_lanes.begin());
          }
          if (!temp_follow_lanes.empty()) {
            lanelet::ConstLanelet temp_follow_lane =
                temp_follow_lanes.front();  // 这里是不是用following比较好？
            if (std::find(all_ids.begin(), all_ids.end(),
                          temp_follow_lane.id()) != all_ids.end() &&
                std::find(start_ids.begin(), start_ids.end(),
                          temp_follow_lane.id()) == start_ids.end()) {
              fix_start_lanes.push_back(temp_follow_lane);
              fix_start_ids.push_back(temp_follow_lane.id());
            }
          }
        }
      }
    }
    // 当前车道有后继车道
    else {
      lanelet::ConstLanelet end_follow_lane;
      for (int k = follow_lanes.size() - 1; k >= 0; k--) {
        if (std::find(all_ids.begin(), all_ids.end(), follow_lanes[k].id()) !=
            all_ids.end()) {
          end_follow_lane = follow_lanes[k];
          break;
        }
      }
      if (map_->laneletLayer.find(end_follow_lane.id()) ==
          map_->laneletLayer.end()) {
        continue;
      }
      lanelet::ConstLanelets end_lanes_besides =
          routing_map_->besides(end_follow_lane);
      for (auto lane : end_lanes_besides) {
        if (std::find(used_besides_ids.begin(), used_besides_ids.end(),
                      lane.id()) == used_besides_ids.end()) {
          used_besides_ids.push_back(lane.id());
          lanelet::ConstLanelets temp_follow_lanes =
              route->remainingLane(lane).lanelets();
          if (!temp_follow_lanes.empty()) {
            temp_follow_lanes.erase(temp_follow_lanes.begin());
          }
          if (!temp_follow_lanes.empty()) {
            lanelet::ConstLanelet temp_follow_lane = temp_follow_lanes.front();
            if (std::find(all_ids.begin(), all_ids.end(),
                          temp_follow_lane.id()) != all_ids.end() &&
                std::find(start_ids.begin(), start_ids.end(),
                          temp_follow_lane.id()) == start_ids.end()) {
              fix_start_lanes.push_back(temp_follow_lane);
              fix_start_ids.push_back(temp_follow_lane.id());
            }
          }
        }
      }
    }
  }

  // 调整 start id 顺序
  fix_start_ids.insert(fix_start_ids.end(), start_ids.begin(), start_ids.end());
  fix_start_ids.insert(fix_start_ids.end(), temp_previous_ids.begin(),
                       temp_previous_ids.end());
  fix_start_ids.insert(fix_start_ids.end(), temp_following_ids.begin(),
                       temp_following_ids.end());
  std::vector<int> fix_ids = all_ids;
  for (auto it = fix_ids.begin(); it != fix_ids.end();) {
    if (std::find(fix_start_ids.begin(), fix_start_ids.end(), *it) ==
        fix_start_ids.end()) {
      it = fix_ids.erase(it);
    } else {
      it++;
    }
  }
  lanelet::ConstLanelets fix_lanes;
  for (auto id : fix_ids) {
    fix_lanes.push_back(const_map_->laneletLayer.get(id));
  }
  sequence_start_lanes = fix_lanes;
  start_ids = fix_ids;

  used_besides_ids.clear();
  fix_start_lanes = sequence_start_lanes;
  fix_start_ids = start_ids;
  int insert_count = 0;
  for (unsigned int i = 0; i < fix_start_lanes.size(); i++) {
    int insert_value = insert_count;
    lanelet::ConstLanelet temp_sequence_start_lane = fix_start_lanes[i];
    lanelet::ConstLanelets sequence_start_lanes_besides =
        routing_map_->besides(temp_sequence_start_lane);
    // int lane_size = sequence_start_lanes_besides.size();
    int basic_index = std::find(sequence_start_lanes_besides.begin(),
                                sequence_start_lanes_besides.end(),
                                temp_sequence_start_lane) -
                      sequence_start_lanes_besides.begin();
    for (auto lane : sequence_start_lanes_besides) {
      if (std::find(used_besides_ids.begin(), used_besides_ids.end(),
                    lane.id()) == used_besides_ids.end()) {
        used_besides_ids.push_back(lane.id());
        if (std::find(fix_start_ids.begin(), fix_start_ids.end(), lane.id()) ==
            fix_start_ids.end()) {
          int lane_index = std::find(sequence_start_lanes_besides.begin(),
                                     sequence_start_lanes_besides.end(), lane) -
                           sequence_start_lanes_besides.begin();
          if (lane_index > basic_index) {
            sequence_start_lanes.insert(sequence_start_lanes.begin() + i +
                                            insert_value - basic_index +
                                            lane_index,
                                        lane);
            start_ids.insert(
                start_ids.begin() + i + insert_value - basic_index + lane_index,
                lane.id());
            insert_count += 1;
          } else if (lane_index < basic_index) {
            sequence_start_lanes.insert(
                sequence_start_lanes.begin() + i + insert_value + lane_index,
                lane);
            start_ids.insert(start_ids.begin() + i + insert_value + lane_index,
                             lane.id());
            insert_count += 1;
          }
        }
      }
    }
  }

  for (unsigned int i = 0; i < sequence_start_lanes.size();
       i++)  // to find relevant sequence and ids_list
  {
    // lanelet::LaneletSequence
    // temp_seq=route->remainingLane(sequence_start_lanes[i]); lanelet::Ids
    // ids=temp_seq.ids();
    lanelet::ConstLanelet temp_end = sequence_start_lanes[i];
    std::cout << "sequence_start_lanes_" << i << ": " << temp_end.id()
              << "\n";
    std::vector<int> final_ids;
    final_ids.clear();
    final_ids.push_back(temp_end.id());
    bool path_not_end = false;
    do {
      lanelet::ConstLanelets follows = routing_map_->following(temp_end);
      if (follows.empty()) {
        path_not_end = false;
      } else {
        bool has_follow = false;
        for (auto follow : follows) {
          if (find(all_ids.begin(), all_ids.end(), follow.id()) !=
                  all_ids.end() &&
              find(sequence_start_lanes.begin(), sequence_start_lanes.end(),
                   follow) == sequence_start_lanes.end()) {
            if (GetLaneInfoType(temp_end.id()) ==
                GetLaneInfoType(follow.id())) {
              temp_end = follow;
              path_not_end = true;
              has_follow = true;
              final_ids.push_back(follow.id());
              break;
            } else  // 若前后车道的道路类型不一致，则不能进行合并，需要打断
            {
              lanelets_ids.push_back(final_ids);
              final_ids.clear();
              final_ids.push_back(follow.id());
              temp_end = follow;
              path_not_end = true;
              has_follow = true;
              break;
            }
          } else {
            has_follow &= false;
          }
        }
        path_not_end = has_follow;
      }

    } while (path_not_end);

    lanelets_ids.push_back(final_ids);
  }

  legionclaw::routing::GetPathID(start_lane.id(), all_ids, lanelets_ids,
                             path_id_map_, routing_map_, map_);

  local_route = lanelets_ids;

  if (local_route.size() > 0 && local_route.at(0).size() > 0) {
    // for(int i= 0 ; i< local_route.size(); i++)
    // {
    //   std::cout << "local_route_"<<i<< "\n";
    //   for (auto id : local_route[i])
    //   {
    //     std::cout << id <<" -- ";
    //   }
    //   std::cout << "\nlocal_route_"<<i<< "\n";
    // }
    return true;
  } else {
    std::cout << "Failed To Get Local Route" << "\n";
    return false;
  }
}

legionclaw::interface::RoutingResponse::PlanStatus Routing::GetSection(
    const std::vector<int>& singleroad,
    std::vector<std::vector<int>>& local_route, const interface::Location pose,
    const double back_length, const double forward_length) {
  // 改为shortest path 作为输入
  //  auto location_point = lanelet::BasicPoint2d(pose.utm_position().x(),
  //  pose.utm_position().y());
  auto low_point = lanelet::BasicPoint2d(
      pose.utm_position().x() - routing_conf_->local_map_range(),
      pose.utm_position().y() - routing_conf_->local_map_range());
  auto high_point = lanelet::BasicPoint2d(
      pose.utm_position().x() + routing_conf_->local_map_range(),
      pose.utm_position().y() + routing_conf_->local_map_range());
  // auto lanes_region =
  // map_->laneletLayer.search(lanelet::BoundingBox2d(low_point, high_point));
  auto const_lanes_region = const_map_->laneletLayer.search(
      lanelet::BoundingBox2d(low_point, high_point));
  lanelet::Lanelets lanes_region;
  // std::cout << "\n-- lanes_region --" << "\n";
  // for (auto const_lane : const_lanes_region)
  // {
  //   auto lane_region = map_->laneletLayer.get(const_lane.id());
  //   lanes_region.push_back(lane_region);
  //   // std::cout<<const_lane.id()<<"\n";
  // }
  // std::cout << "\n-- lanes_region --" << "\n";
  for (auto local_path : local_paths_) {
    for (auto id : local_path) {
      lanelet::Lanelet temp_lane = map_->laneletLayer.get(id);
      lanes_region.push_back(temp_lane);
    }
  }

  lanelet::Point3d point(lanelet::utils::getId(), pose.utm_position().x(),
                         pose.utm_position().y(), pose.utm_position().z());
  auto locate_lanepair = lanelet2_ex::GetClosestPoint_LaneletInRange(
      lanes_region, point, pose.heading(), routing_conf_->search_range(),
      routing_conf_->yaw_weight(), routing_conf_->search_height_range());
  // auto closest_point = locate_lanepair.first;
  double min_dist = lanelet::geometry::distance2d(point, locate_lanepair.first);
  if (min_dist > 10)  // 若当前定位和规划路线的最短2D距离超过10米则重新匹配车道
  {
    lanes_region.clear();
    for (auto const_lane : const_lanes_region) {
      auto lane_region = map_->laneletLayer.get(const_lane.id());
      lanes_region.push_back(lane_region);
      // std::cout<<const_lane.id()<<"\n";
    }
    locate_lanepair = lanelet2_ex::GetClosestPoint_LaneletInRange(
        lanes_region, point, pose.heading(), routing_conf_->search_range(),
        routing_conf_->yaw_weight(), routing_conf_->search_height_range());
  }
  int locate_id = locate_lanepair.second;
  lanelet::Lanelet locate_lane;
  if (!map_->laneletLayer.exists(locate_id)) {
    std::cout << "vehicle not in map, can not decide local route" << "\n";
    response_.set_plan_status(
        interface::RoutingResponse::PlanStatus::OUT_OF_MAP);
    return response_.plan_status();
  } else {
    locate_lane = map_->laneletLayer.get(locate_id);
  }
  // auto locate_besides = routing_map_->besides(locate_lane);
  // std::vector<int> locate_ids;
  // std::cout << "\n-- locate_id -- " <<locate_id<< "\n";
  // std::cout << "\n-- locate_lanes --" << "\n";
  // for (auto lane : locate_besides)
  // {
  //   locate_ids.emplace_back(lane.id());
  //   std::cout<<lane.id()<<"\n";
  // }
  // std::cout << "\n-- locate_lanes --" << "\n";

  auto locate_leftRelations = routing_map_->leftRelations(
      locate_lane);  // 获取当前车道左边的所有车道及邻接关系
  auto locate_rightRelations = routing_map_->rightRelations(
      locate_lane);             // 获取当前车道右边的所有车道及邻接关系
  std::vector<int> locate_ids;  // 当前车道以及左右两边的所有车道id
  locate_ids.emplace_back(locate_id);
  std::cout << "\n-- locate_id -- " << locate_id << "\n";
  std::cout << "\n-- Surrounding lanes --" << "\n";
  for (auto leftRelation : locate_leftRelations) {
    locate_ids.emplace_back(leftRelation.lanelet.id());
    std::cout << leftRelation.lanelet.id() << "\n";
  }
  for (auto rightRelation : locate_rightRelations) {
    locate_ids.emplace_back(rightRelation.lanelet.id());
    std::cout << rightRelation.lanelet.id() << "\n";
  }
  std::cout << "\n-- Surrounding lanes --" << "\n";

  int locate_index = -1;
  bool match_success = false;
  for (int index = 0; index < singleroad.size(); index++) {
    if (match_success) {
      break;
    }
    for (auto slice_id : locate_ids) {
      if (match_success) {
        break;
      }
      if (slice_id == singleroad[index]) {
        locate_index = index;
        match_success = true;
      }
    }
  }
  if (!match_success) {
    std::cout << " warning : vehicle not match route , can not decide local "
                 "route,replan"
              << "\n";
    std::cout << "\n-- singleroad --" << "\n";
    for (int i = 0; i < singleroad.size(); i++) {
      std::cout << "singleroad_" << i << "  " << singleroad[i] << "\n";
    }
    std::cout << "\n-- singleroad --" << "\n";
    auto start_lane = locate_lane;
    auto end_lane = map_->laneletLayer.get(singleroad.back());
    lanelet::Optional<lanelet::routing::Route> route_temp;
    route_temp = routing_map_->getRoute(start_lane, end_lane);
    if (!route_temp) {
      std::cout << "replan failed" << "\n";
      response_.set_plan_status(
          interface::RoutingResponse::PlanStatus::OUT_OF_MAP);
      return response_.plan_status();
    } else {
      auto shortest_path = route_temp->shortestPath();
      route = std::move(route_temp);
      std::vector<int> shortest_ids;
      std::cout << "shortest path in lanelet2" << "\n";
      for (unsigned int i = 0; i < shortest_path.size(); i++) {
        int lanelet_id = shortest_path[i].id();
        std::cout << lanelet_id << "\n";
        shortest_ids.push_back(lanelet_id);
      }
      LoadJunctionInfo(shortest_ids);  // 更新no_change_lane_
      locate_index = 0;
    }
  }
  std::vector<int> local_singleroad;
  local_singleroad.emplace_back(no_change_lane_[locate_index]);
  auto remain =
      lanelet2_ex::GetRemainDistance(point, locate_lane.centerline3d());
  double cur_length = lanelet::geometry::length3d(locate_lane);
  double remain_back = back_length - (cur_length - remain.first);
  double remain_forward = forward_length - remain.first;
  for (int i = locate_index - 1; i >= 0; i--) {
    // 往后找
    if (remain_back < 0) {
      break;
    } else {
      lanelet::ConstLanelet temp_lane =
          map_->laneletLayer.get(no_change_lane_[i]);
      double temp_length = lanelet::geometry::length3d(temp_lane);
      local_singleroad.insert(local_singleroad.begin(), no_change_lane_[i]);
      remain_back = remain_back - temp_length;
    }
  }
  for (int i = locate_index + 1; i < no_change_lane_.size(); i++) {
    // 往前找
    if (remain_forward < 0) {
      break;
    } else {
      lanelet::ConstLanelet temp_lane =
          map_->laneletLayer.get(no_change_lane_[i]);
      double temp_length = lanelet::geometry::length3d(temp_lane);
      local_singleroad.insert(local_singleroad.end(), no_change_lane_[i]);
      remain_forward = remain_forward - temp_length;
    }
  }
  if (local_singleroad.size() <= 0) {
    std::cout << "local route size =0 warning" << "\n";
    response_.set_plan_status(
        interface::RoutingResponse::PlanStatus::OUT_OF_MAP);
    return response_.plan_status();
  }
  SingleRoad2FullRoute(local_singleroad, local_route);
  response_.set_plan_status(
      interface::RoutingResponse::PlanStatus::PLAN_SUCCESS);
  return response_.plan_status();
}

void Routing::Section2Response(const std::vector<std::vector<int>>& local_route,
                               interface::RoutingResponse& response) {
  response.clear_lane_list();
  bool temp_stop_check = false;
  std::vector<interface::LaneInfo> temp_stop_lanes;
  stop_lanes_.clear();
  stop_indexs_.clear();
  // 构建局部路由图搜寻车道拓扑关系
  lanelet::Lanelets seg_lanes;
  for (auto seg_route : local_route) {
    for (auto seg_lane_id : seg_route) {
      lanelet::Lanelet seg_lane = map_->laneletLayer.get(seg_lane_id);
      seg_lanes.push_back(seg_lane);
    }
  }
  lanelet::LaneletMapUPtr seg_map = lanelet::utils::createMap(seg_lanes);
  lanelet::routing::RoutingGraphUPtr seg_routing =
      lanelet::routing::RoutingGraph::build(*seg_map, *traffic_rule_);

  for (unsigned int i = 0; i < local_route.size();
       i++)  // lanelets_ids_list to vector_map LanePoints_list
  {
    std::vector<int> lane_id_list;
    std::vector<int> match_lane_id_list;
    lane_id_list.clear();
    interface::LaneInfo path;
    path.clear_lane_points();
    lane_id_list = local_route[i];
    path.set_global_id(path_id_map_.at(lane_id_list[0]));
    // std::cout << "lane_id[0]: " << lane_id_list[0]<<"\n";
    path.set_type(GetLaneInfoType(lane_id_list[0]));
    // path.set_priority(); //TODO

    for (unsigned int l = 0; l < local_route.size(); l++) {
      if (l == i) {
        continue;
      } else {
        match_lane_id_list = local_route[l];
        lanelet::ConstLanelet front_lane =
            const_map_->laneletLayer.get(lane_id_list.front());
        lanelet::ConstLanelet back_lane =
            const_map_->laneletLayer.get(lane_id_list.back());
        // 查找上一车道
        lanelet::ConstLanelets previous_lanes =
            seg_routing->previous(front_lane);
        if (!previous_lanes.empty()) {
          lanelet::ConstLanelet previous_lane = previous_lanes.front();
          if (std::find(match_lane_id_list.begin(), match_lane_id_list.end(),
                        previous_lane.id()) != match_lane_id_list.end()) {
            path.set_predecessor_id(path_id_map_.at(match_lane_id_list[0]));
          }
        }
        // 查找下一车道
        lanelet::ConstLanelets following_lanes =
            seg_routing->following(back_lane);
        if (!following_lanes.empty()) {
          lanelet::ConstLanelet following_lane = following_lanes.front();
          if (std::find(match_lane_id_list.begin(), match_lane_id_list.end(),
                        following_lane.id()) != match_lane_id_list.end()) {
            path.set_successor_id(path_id_map_.at(match_lane_id_list[0]));
          }
        }
        // 查找左边相邻车道
        lanelet::ConstLanelets left_lanes = seg_routing->lefts(front_lane);
        if (!left_lanes.empty()) {
          lanelet::ConstLanelet left_lane = left_lanes.front();
          if (std::find(match_lane_id_list.begin(), match_lane_id_list.end(),
                        left_lane.id()) != match_lane_id_list.end()) {
            path.set_left_neighbor_id(path_id_map_.at(match_lane_id_list[0]));
          }
        }
        // 查找右边相邻车道
        lanelet::ConstLanelets right_lanes = seg_routing->rights(front_lane);
        if (!right_lanes.empty()) {
          lanelet::ConstLanelet right_lane = right_lanes.front();
          if (std::find(match_lane_id_list.begin(), match_lane_id_list.end(),
                        right_lane.id()) != match_lane_id_list.end()) {
            path.set_right_neighbor_id(path_id_map_.at(match_lane_id_list[0]));
          }
        }
      }
    }
    double last_mileage = 0;
    bool is_stop_lane = false;
    for (unsigned int j = 0; j < lane_id_list.size(); j++) {
      int lanelet2_id = lane_id_list[j];
      for (auto stop_id : stop_lane_ids_) {
        if (lanelet2_id == stop_id) {
          is_stop_lane = true;
        }
      }

      lanelet::Lanelet temp_lane = map_->laneletLayer.get(lanelet2_id);
      lanelet::ConstLineString3d cline = temp_lane.centerline();
      lanelet::ConstLineString3d temp_leftline;
      lanelet::ConstLineString3d temp_rightline;
      temp_leftline = temp_lane.leftBound();
      temp_rightline = temp_lane.rightBound();
      std::string left_attr = temp_leftline.attributeOr("subtype", "curbstone");
      std::string right_attr =
          temp_rightline.attributeOr("subtype", "curbstone");
      double map_speed =
          temp_lane.attributeOr("speed_limit", routing_conf_->speed_limit()) /
          3.6;
      common::LaneLineType left_type =
                               common::LaneLineType::LANE_LINE_TYPE_UNKNOWN,
                           right_type =
                               common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
      lanelet::LineString3d
          left_base = temp_lane.leftBound(),
          right_base =
              temp_lane
                  .rightBound();  // 左右车道的物理边线，如果有对向可借道车道会换成对应的边线
      if (left_attr == std::string("solid"))  // get left line type enum
      {
        left_type = common::LaneLineType::WHITE_SOLID;
      } else if (left_attr == std::string("dashed_solid")) {
        left_type = common::LaneLineType::WHITE_SOLID;
      } else if (left_attr == std::string("curbstone")) {
        left_type = common::LaneLineType::CURB;
      } else if (left_attr == std::string("dashed") ||
                 left_attr == std::string("solid_dashed")) {
        left_type = common::LaneLineType::WHITE_DASHED;
        auto laneletsOwningLinestring =
            map_->laneletLayer.findUsages(temp_lane.leftBound().invert());
        if (laneletsOwningLinestring.size() == 0) {
          // std::cerr << "lanelet :" << temp_lane.id() << " the left bound
          // owning by " << laneletsOwningLinestring.size() << " opposite lane"
          // << "\n";
        } else if (laneletsOwningLinestring.size() == 1) {
          left_base = laneletsOwningLinestring.front().rightBound();
        } else {
          // std::cerr << "lanelet :" << temp_lane.id() << " the left bound
          // owning by " << laneletsOwningLinestring.size() << " opposite lane"
          // << "\n";
        }
      } else {
        left_type = common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
      }

      if (right_attr == std::string("solid"))  // get right line type enum
      {
        right_type = common::LaneLineType::WHITE_SOLID;
      } else if (right_attr == std::string("solid_dashed")) {
        right_type = common::LaneLineType::WHITE_SOLID;
      } else if (right_attr == std::string("curbstone")) {
        right_type = common::LaneLineType::CURB;
      } else if (right_attr == std::string("dashed") ||
                 right_attr == std::string("dashed_solid")) {
        right_type = common::LaneLineType::WHITE_DASHED;
        auto laneletsOwningLinestring =
            map_->laneletLayer.findUsages(temp_lane.rightBound().invert());
        if (laneletsOwningLinestring.size() == 0) {
          // std::cerr << "lanelet :" << temp_lane.id() << " the right bound
          // owning by " << laneletsOwningLinestring.size() << " opposite lane"
          // << "\n";
        } else if (laneletsOwningLinestring.size() == 1) {
          right_base = laneletsOwningLinestring.front().leftBound();
        } else {
          // std::cerr << "lanelet :" << temp_lane.id() << " the right bound
          // owning by " << laneletsOwningLinestring.size() << " opposite lane"
          // << "\n";
        }
      } else {
        right_type = common::LaneLineType::LANE_LINE_TYPE_UNKNOWN;
      }

      double mileage = last_mileage;
      double last_theta = 0;
      unsigned int last_index = 0;
      for (unsigned int k = 0; k < cline.size() - 1; k++) {
        lanelet::ConstPoint3d cpoint = cline[k];
        double theta;
        if (k > 0) {
          theta = atan2(cpoint.y() - cline[last_index].y(),
                        cpoint.x() - cline[last_index].x());
          double gap = lanelet::geometry::distance2d(cpoint, cline[last_index]);
          // if ((calculate_diff_angle(theta, last_theta) > M_PI / 2) ||
          // (routing_conf_->enable_downsample() && gap <=
          // routing_conf_->path_density()))
          // {
          //   continue;
          // }
          // else
          {
            mileage += lanelet::geometry::distance3d(cline[last_index], cpoint);
            last_index = k;
            last_theta = theta;
          }
        } else {
          theta = atan2(cline[k + 1].y() - cpoint.y(),
                        cline[k + 1].x() - cpoint.x());
          last_theta = theta;
          last_index = 0;
        }

        interface::LanePoint point;
        interface::Point3D basic_point;
        basic_point.set_x(cpoint.x());
        basic_point.set_y(cpoint.y());
        basic_point.set_z(cpoint.z());
        double left_dis = lanelet::geometry::distance3d(left_base, cpoint);
        double right_dis = lanelet::geometry::distance3d(right_base, cpoint);
        left_dis = left_dis > 1.7 ? left_dis : 1.7;
        right_dis = right_dis > 1.7 ? right_dis : 1.7;
        point.set_point(basic_point);
        point.set_left_road_width(left_dis);
        point.set_right_road_width(right_dis);
        point.set_limit_speed(map_speed);
        point.set_mileage(mileage);
        point.set_left_line_type(left_type);
        point.set_right_line_type(right_type);
        point.set_theta(theta);
        path.add_lane_points(point);
      }
      last_mileage = mileage + lanelet::geometry::distance3d(cline[last_index],
                                                             cline.back());
    }
    if (routing_conf_->enable_path_smooth()) {
      legionclaw::routing::FixPathDensity(path, routing_conf_->path_density());
      legionclaw::routing::SmoothPath(path, 0.49, 0.35, 0.01);
    }
    if (path.lane_points_size() > 0)
      std::cout << "size -> " << path.lane_points_size()
                << "  gid = " << int32_t(path.global_id()) << "\n";
    response.add_lane_list(path);
    if (is_stop_lane) {
      temp_stop_lanes.push_back(path);
      temp_stop_check = true;
    }
  }
  if (temp_stop_check) {
    auto stop_point = stop_info_.stop_points().front();
    for (auto stop_lane : temp_stop_lanes) {
      int stop_index = GetNearestPointInPath(
          stop_point.point().x(), stop_point.point().y(),
          stop_point.point().z(), stop_point.theta(), stop_lane);
      if (stop_index < 0 || stop_index >= stop_lane.lane_points_size()) {
        std::cout << "ERROR: Can not Match Stop Point in CenterLine"
                  << "\n";
      } else {
        match_stop_points_ = true;
        stop_indexs_.push_back(stop_index);
        stop_lanes_.push_back(stop_lane);
      }
    }
  }
  // header
  interface::Header header;
  header.set_frame_id(routing_conf_->app_name());
  if (routing_conf_->enable_system_time()) {
    header.set_stamp(common::TimeTool::Now2TmeStruct());
  } else {
    header.set_stamp(current_pose_.header().stamp());
  }
  header.set_seq(unique::getRoutingResponseSeq());
  response.set_header(header);
}

legionclaw::interface::RoutingResponse::PlanStatus Routing::GlobalPlan(
    interface::RoutingRequest request) {
  if (request.num_of_kp() < 2) {
    std::cerr << legionclaw::common::Status::ROUTING_ERROR_REQUEST << "\n";
    response_.set_plan_status(
        interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
    return response_.plan_status();
  }
  lanelet::ConstLanelets lanes;
  lanes.clear();
  stop_lane_ids_.clear();
  unsigned int start_index = 0;
  unsigned int end_index = 0;
  if (routing_conf_->use_tf_flag()) {
    request.mutable_key_point_list()->at(0).set_heading(location_.heading());
  }
  for (unsigned int i = 0; i < request.key_point_list_size(); i++) {
    auto key = request.key_point_list(i);
    if (!routing_conf_->enable_3d_map()) {
      key.set_ele(0);
    }
    auto b_point = projector_.forward(key.latitude(), key.longitude());
    lanelet::Point3d point(lanelet::utils::getId(), b_point.x, b_point.y,
                           key.ele());
    double yaw =
        key.heading();  // 更新角度坐标系，此处传进来正东为0，逆时针增加的弧度
    std::cout << "key point   " << i << "    :   " << point
              << "  heading :" << key.heading() << "\n";
    auto lane_pair = lanelet2_ex::GetClosestPoint_LaneletInRange(
        const_map_, point, yaw, routing_conf_->search_range(),
        routing_conf_->yaw_weight(), routing_conf_->search_height_range());
    double match_dis_2d = lanelet::geometry::distance2d(lane_pair.first, point);
    if (match_dis_2d > routing_conf_->search_range()) {
      std::cout << "error : key point not match lanelet : " << lane_pair.second
                << " in range, distance : " << match_dis_2d
                << " , check your  x y poistion" << "\n";
      std::cout << "closest point :" << lane_pair.first << "\n";
      response_.set_plan_status(
          interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
      return response_.plan_status();
    }
    double z_dis = fabs(lane_pair.first.z() - point.z());
    if (z_dis > routing_conf_->search_height_range()) {
      std::cout << "error : key point not match lanelet : " << lane_pair.second
                << " in range, distance : " << z_dis
                << " , check your elevation " << "\n";
      std::cout << "closest point :" << lane_pair.first << "\n";
      response_.set_plan_status(
          interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
      return response_.plan_status();
    }
    int lane_id = lane_pair.second;
    if (!map_->laneletLayer.exists(lane_id)) {
      std::cerr << "Can not find lanelet in range !"
                << legionclaw::common::Status::ROUTING_ERROR_REQUEST << "\n";
      response_.set_plan_status(
          interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
      return response_.plan_status();
    }
    auto lane = map_->laneletLayer.get(lane_id);
    lanes.push_back(lane);
    if (i == 0) {
      start_index =
          lanelet2_ex::GetPointIndex(lane_pair.first, lane.centerline3d());
    }
    if (i == request.key_point_list_size() - 1) {
      end_index =
          lanelet2_ex::GetPointIndex(lane_pair.first, lane.centerline3d());
    }
  }

  lanelet::ConstLanelets mid_lanes = lanes;
  mid_lanes.erase(mid_lanes.begin());
  mid_lanes.erase(mid_lanes.end());
  int start_id = lanes.front().id();
  int end_id = lanes.back().id();
  // 寻找所有终点车道
  lanelet::ConstLanelets stop_lanelets = routing_map_->besides(lanes.back());
  for (auto stop_lanelet : stop_lanelets) {
    stop_lane_ids_.push_back(stop_lanelet.id());
  }
  if (start_id == end_id)  // 如果起始和终点在同一个车道，判断是否需要绕一圈
  {
    if (start_index > end_index) {
      auto follows = routing_map_->following(lanes.front());
      if (follows.empty()) {
        std::cerr
            << "start point is in front of stop point but there is no circle"
            << "\n";
        response_.set_plan_status(
            interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
        return response_.plan_status();
      } else {
        for (auto follow : follows) {
          mid_lanes.insert(mid_lanes.begin(), follow);
          route = routing_map_->getRouteVia(lanes.front(), mid_lanes,
                                            lanes.back(), 0, true);
          if (route) {
            break;
          }
        }
      }
    } else {
      route = routing_map_->getRouteVia(lanes.front(), mid_lanes, lanes.back(),
                                        0, true);
    }
  } else {
    route = routing_map_->getRouteVia(lanes.front(), mid_lanes, lanes.back(), 0,
                                      true);
  }
  if (!route) {
    std::cout << "failed to generate route graph" << "\n";
    std::cout << "start_lanelet: " << lanes.front().id()
              << " goal_lanelet: " << lanes.back().id() << "\n";
    response_.set_plan_status(
        interface::RoutingResponse::PlanStatus::REQUEST_ERROR);
    return response_.plan_status();
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  std::vector<int> shortest_ids;
  std::cout << "shortest path in lanelet2" << "\n";
  for (unsigned int i = 0; i < shortest_path.size(); i++) {
    int lanelet_id = shortest_path[i].id();
    std::cout << lanelet_id << "\n";
    shortest_ids.push_back(lanelet_id);
  }
  LoadJunctionInfo(shortest_ids);
  if (no_change_lane_.size() > 0) {
    response_.set_plan_status(
        interface::RoutingResponse::PlanStatus::PLAN_SUCCESS);
    return response_.plan_status();
  }
  return {};
}  // TODO: -Werror=return-type

void SmoothPath(interface::LaneInfo& lane_info, double weight_data,
                double weight_smooth, double tolerance) {
  if (lane_info.lane_points_size() <= 2) {
    return;
  }
  std::vector<interface::LanePoint> path_in;
  lane_info.lane_points(path_in);
  std::vector<interface::LanePoint> smoothPath_out = path_in;
  double change = tolerance;
  double xtemp, ytemp;
  int nIterations = 0;
  int size = path_in.size();
  while (change >= tolerance) {
    change = 0.0;
    for (int i = 1; i < size - 1; i++) {
      xtemp = smoothPath_out[i].point().x();
      ytemp = smoothPath_out[i].point().y();

      interface::Point3D basic_point;
      basic_point.set_x(xtemp);
      basic_point.set_y(ytemp);

      basic_point.set_x(basic_point.x() +
                        weight_data *
                            (path_in[i].point().x() - basic_point.x()));
      basic_point.set_y(basic_point.y() +
                        weight_data *
                            (path_in[i].point().y() - basic_point.y()));

      basic_point.set_x(basic_point.x() +
                        weight_smooth * (smoothPath_out[i - 1].point().x() +
                                         smoothPath_out[i + 1].point().x() -
                                         (2.0 * basic_point.x())));
      basic_point.set_y(basic_point.y() +
                        weight_smooth * (smoothPath_out[i - 1].point().y() +
                                         smoothPath_out[i + 1].point().y() -
                                         (2.0 * basic_point.y())));

      change += fabs(xtemp - basic_point.x());
      change += fabs(ytemp - basic_point.y());

      smoothPath_out[i].set_point(basic_point);
    }
    nIterations++;
  }

  lane_info.set_lane_points(&smoothPath_out);
}

void FixPathDensity(interface::LaneInfo& lane_info,
                    const double& distanceDensity) {
  if (lane_info.lane_points_size() == 0 || distanceDensity == 0) {
    return;
  }

  double d = 0, a = 0;
  double margin = distanceDensity * 0.01;
  double remaining = 0;
  int nPoints = 0;
  std::vector<interface::LanePoint> fixedPath;
  fixedPath.push_back(lane_info.lane_points(0));
  for (unsigned int si = 0, ei = 1; ei < lane_info.lane_points_size();) {
    d += hypot(lane_info.lane_points(ei).point().x() -
                   lane_info.lane_points(ei - 1).point().x(),
               lane_info.lane_points(ei).point().y() -
                   lane_info.lane_points(ei - 1).point().y()) +
         remaining;
    a = atan2(lane_info.lane_points(ei).point().y() -
                  lane_info.lane_points(si).point().y(),
              lane_info.lane_points(ei).point().x() -
                  lane_info.lane_points(si).point().x());

    if (d < distanceDensity - margin)  // skip
    {
      ei++;
      remaining = 0;
    } else if (d > (distanceDensity + margin))  // skip
    {
      interface::LanePoint pm = lane_info.lane_points(si);
      nPoints = d / distanceDensity;
      for (int k = 0; k < nPoints; k++) {
        interface::Point3D basic_point = pm.point();
        basic_point.set_x(basic_point.x() + distanceDensity * cos(a));
        basic_point.set_y(basic_point.y() + distanceDensity * sin(a));
        pm.set_point(basic_point);
        fixedPath.push_back(pm);
      }
      remaining = d - nPoints * distanceDensity;
      si++;
      lane_info.set_lane_points(si, pm);
      d = 0;
      ei++;
    } else {
      d = 0;
      remaining = 0;
      fixedPath.push_back(lane_info.lane_points(ei));
      ei++;
      si = ei - 1;
    }
  }

  lane_info.set_lane_points(&fixedPath);
}

// 找到lists中的包含input的list的索引
unsigned int GetIndex(int input, std::vector<std::vector<int>> lists) {
  for (unsigned int i = 0; i < lists.size(); i++) {
    std::vector<int> list = lists[i];
    for (unsigned int j = 0; j < list.size(); j++) {
      if (list[j] == input) {
        return i;
      }
    }
  }
  return -1;
}

double Yaw2Angle(double input) {
  double temp_angle = 90 - input;
  double format_angle = temp_angle;
  if (temp_angle < -180) {
    format_angle = 360 + temp_angle;
  }
  double output = format_angle * M_PI / 180;
  return output;
}

void GetPathID(int start_id, std::vector<int> all_ids,
               std::vector<std::vector<int>>& lanes, std::map<int, int>& map,
               lanelet::routing::RoutingGraphUPtr& graph,
               lanelet::LaneletMapUPtr& osm_map) {
  int cur_value = 1;
  for (unsigned int i = 0; i < lanes.size(); i++) {
    std::vector<int> cur_lane = lanes[i];
    for (auto cur_id : cur_lane) {
      if (map.find(cur_id) == map.end()) {
        map.insert(std::make_pair(cur_id, cur_value));
      }
    }
    cur_value += 1;
  }
}

bool Routing::MatchPointInPath(double x, double y, double z, double yaw,
                               const double range, const double height_range,
                               const double max_distance, const double index,
                               const interface::LaneInfo& lane) {
  double stop_mile = lane.lane_points(index).mileage();
  bool is_near_station = false;
  double yaw_weight = routing_conf_->yaw_weight();
  for (int i = index; i > 0; i--) {
    auto cpoint = lane.lane_points(i);
    if (stop_mile - cpoint.mileage() > max_distance) {
      break;
    }
    if (fabs(x - cpoint.point().x()) > range) {
      continue;
    }
    if (fabs(y - cpoint.point().y()) > range) {
      continue;
    }
    if (fabs(z - cpoint.point().z()) > height_range) {
      continue;
    }
    double temp_score =
        calculate_diff_angle(yaw, cpoint.theta()) * yaw_weight / M_PI +
        sqrt(pow(x - cpoint.point().x(), 2) + pow(y - cpoint.point().y(), 2));
    if (temp_score < range) {
      is_near_station = true;
      break;
    }
  }

  return is_near_station;
}

int Routing::GetNearestPointInPath(double x, double y, double z, double yaw,
                                   const interface::LaneInfo& lane) {
  int min_index = -1;
  double min_score = DBL_MAX;
  // double match_range = GetConf()->search_range();
  double match_height_range = GetConf()->search_height_range();
  double stop_range =
      GetConf()
          ->stop_distance();  // 此处放款x,y的匹配距离，以达到多车道匹配的效果
  double yaw_weight =
      GetConf()
          ->yaw_weight();  // 此处权重系数为比例值，例如角度差为10度，则权值=10*yaw_weight/180
  for (int i = 0; i < lane.lane_points().size(); i++) {
    auto cpoint = lane.lane_points()[i];
    if (fabs(x - cpoint.point().x()) > stop_range) {
      continue;
    }
    if (fabs(y - cpoint.point().y()) > stop_range) {
      continue;
    }
    if (fabs(z - cpoint.point().z()) > match_height_range) {
      continue;
    }
    double temp_score =
        calculate_diff_angle(yaw, cpoint.theta()) * yaw_weight / M_PI +
        sqrt(pow(x - cpoint.point().x(), 2) + pow(y - cpoint.point().y(), 2) +
             pow(z - cpoint.point().z(), 2));
    if (temp_score < min_score) {
      min_score = temp_score;
      min_index = i;
    }
  }

  return min_index;
}

void Routing::FindSuitableStopLane(lanelet::Lanelet lane,
                                   lanelet::Lanelets& lanes,
                                   const double range) {
  auto length = lanelet::geometry::length3d(lane);
  auto next_lanes = routing_map_->previous(lane, false);
  if (next_lanes.size() != 0) {
    lanelet::ConstLanelet const_next_lane = next_lanes.front();
    lanelet::Lanelet next_lane = map_->laneletLayer.get(const_next_lane.id());
    lanes.push_back(next_lane);
    auto next_length = lanelet::geometry::length3d(next_lane);
    if (next_length > range - length) {
      return;
    } else {
      FindSuitableStopLane(next_lane, lanes, range - length);
    }
  }
}

// 基于导航信息,匹配交通信号灯;返回交通信号灯的索引;如果未找到返回-1;
// 若存在多个交通信号灯方向与导航方向一致,则返回索引最小的交通信号灯index;
int Routing::MatchTrafficlight(
    legionclaw::common::Direction dir,
    const std::vector<legionclaw::interface::TrafficLight>& lights) {
  int idx = -1;
  for (int i = 0; i < lights.size(); i++) {
    if (dir == legionclaw::common::Direction::UP)  // 直行
    {
      if (lights[i].type() == legionclaw::common::TrafficLightType::STRAIGHT ||
          lights[i].type() ==
              legionclaw::common::TrafficLightType::STRAIGHT_TURN_LEFT ||
          lights[i].type() ==
              legionclaw::common::TrafficLightType::STRAIGHT_TURN_RIGHT) {
        idx = i;
        break;
      }
    }
    if (dir == legionclaw::common::Direction::LEFT)  // 左转
    {
      if (lights[i].type() == legionclaw::common::TrafficLightType::TURN_LEFT ||
          lights[i].type() ==
              legionclaw::common::TrafficLightType::STRAIGHT_TURN_LEFT) {
        idx = i;
        break;
      }
    }
    if (dir == legionclaw::common::Direction::RIGHT)  // 右转
    {
      if (lights[i].type() == legionclaw::common::TrafficLightType::TURN_RIGHT ||
          lights[i].type() ==
              legionclaw::common::TrafficLightType::STRAIGHT_TURN_RIGHT) {
        idx = i;
        break;
      }
    }
  }
  return idx;
}

// 作用：判断点是否在多边形内
// p指目标点， ptPolygon指多边形的点集合， nCount指多边形的边数
bool Routing::PointInPolygon(legionclaw::interface::Point2D p,
                             std::vector<legionclaw::interface::Point2D>& ptPolygon,
                             int nCount) {
  // 交点个数
  int nCross = 0;
  for (int i = 0; i < nCount; i++) {
    legionclaw::interface::Point2D p1 = ptPolygon[i];
    legionclaw::interface::Point2D p2 =
        ptPolygon[(i + 1) % nCount];  // 点P1与P2形成连线

    if (p1.y() == p2.y()) continue;
    if (p.y() < min(p1.y(), p2.y())) continue;
    if (p.y() >= max(p1.y(), p2.y())) continue;
    // 求交点的x坐标（由直线两点式方程转化而来）
    double x = (double)(p.y() - p1.y()) * (double)(p2.x() - p1.x()) /
                   (double)(p2.y() - p1.y()) +
               p1.x();

    // 只统计p1p2与p向右射线的交点
    if (x > p.x()) {
      nCross++;
    }
  }

  // 交点为偶数，点在多边形之外
  // 交点为奇数，点在多边形之内
  if ((nCross % 2) == 1) {
    return true;  // 点在区域内
  } else {
    return false;  // 点在区域外
  }
}

std::vector<legionclaw::interface::Point2D> Routing::getRectVertex(
    legionclaw::interface::Point2D center, float theta, float w, float h) {
  std::vector<legionclaw::interface::Point2D> RectVertex;
  legionclaw::interface::Point2D a, b, c, d;
  a.set_x(center.x() + w / 2 * cos(theta) - h / 2 * sin(theta));
  a.set_y(center.y() - w / 2 * sin(theta) - h / 2 * cos(theta));

  b.set_x(center.x() - w / 2 * cos(theta) - h / 2 * sin(theta));
  b.set_y(center.y() + w / 2 * sin(theta) - h / 2 * cos(theta));

  c.set_x(center.x() - w / 2 * cos(theta) + h / 2 * sin(theta));
  c.set_y(center.y() + w / 2 * sin(theta) + h / 2 * cos(theta));

  d.set_x(center.x() + w / 2 * cos(theta) + h / 2 * sin(theta));
  d.set_y(center.y() - w / 2 * sin(theta) + h / 2 * cos(theta));

  RectVertex.push_back(a);
  RectVertex.push_back(b);
  RectVertex.push_back(c);
  RectVertex.push_back(d);

  return RectVertex;
}

namespace unique {
std::atomic<uint32_t> fault_code_counter{1};
std::atomic<uint32_t> stop_info_counter{1};
std::atomic<uint32_t> parking_info_counter{1};
std::atomic<uint32_t> route_result_counter{1};

uint32_t getFaultCodeSeq() { return fault_code_counter++; }

uint32_t getStopInfoSeq() { return stop_info_counter++; }

uint32_t getParkingInfoSeq() { return parking_info_counter++; }

uint32_t getRoutingResponseSeq() { return route_result_counter++; }

}  // namespace unique

}  // namespace routing
}  // namespace legionclaw
