/**
 * @file    prediction.cpp
 * @author  legionclaw
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <time.h>
#include <fstream>
#include <sys/time.h>

#include "prediction.h"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/interface/faults.hpp"
#include "modules/common/interface/location.hpp"
#include "modules/common/interface/odometry.hpp"
#include "modules/common/interface/lane_list.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/adc_trajectory.hpp"
#include "modules/common/interface/traffic_events.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/prediction_obstacles.hpp"
#include "modules/common/interface/obu_cmd_msg.hpp"
#include "modules/prediction/src/common/prediction_gflags.h"

/**
 * @namespace legionclaw::prediction
 * @brief legionclaw::prediction
 */

namespace legionclaw {
namespace prediction {

void Prediction::Init() {
  // step1 初始化状态设置为false
  { 
    is_init_ = false;
    location_init_ = false;
    map_loaded_ = false;
    local_view_.localiztion_mode_ = 0;  //localiztion_mode: //0:ins, 1:odometry
    local_view_.map_mode_ = 0;  //0=use hdmap, 1=use localmap
  }

  // step2 变量初始化
  { VariableInit(); }

  // step3 配置文件初始化
  {
    prediction_conf_ = std::make_shared<legionclaw::prediction::PredictionConf>();
    if (GetProtoFromJsonFile(FLAGS_prediction_config_file,
                             prediction_conf_.get()) == false) {
      AERROR << "GetProtoFrom prediction config file failed: "
             << FLAGS_prediction_config_file;
      return;
    }
    AINFO << "Successfully loaded prediction config from: " << config_file_path_;
  }
  // step4 日志初始化
  {LOGGING_INIT(prediction_conf_)}

  // // step4 IPC初始化
  {MESSAGE_INIT(prediction_conf_)}

  // step5 读取配置文件
  {
    produce_prediction_command_duration_ =
        prediction_conf_->produce_prediction_command_duration();
    publish_prediction_command_duration_ =
        prediction_conf_->publish_prediction_command_duration();
  }

  // step6 故障码初始化
  FaultMonitorInit();

  // step7 算法初始化
  {
    vector_map_predictor_.Init(*prediction_conf_);
  }

  // step8 定时器和线程初始化
  {
    status_detect_duration_ =
        prediction_conf_->status().status_detect_duration();
            
    ad_timer_manager_ = std::make_shared<ADTimerManager<Prediction, void>>();
    task_10ms_ =
        std::make_shared<WheelTimer<Prediction, void>>(ad_timer_manager_);
    // task_10ms_->AddTimer(10, &Prediction::Task10ms, this);
    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create can task_thread_ thread.";
      return;
    }
  }

  // step9 初始化状态为true
  { is_init_ = true; }
  //TODO:临时默认激活
  TaskActivate();
}

void Prediction::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void Prediction::VariableInit() {
  prediction_conf_ = std::make_shared<PredictionConf>();
}

void Prediction::Print() {}

void Prediction::Log() {}

void Prediction::MessagesActivate()
{
  if (prediction_conf_ == nullptr)
  {
    return;
  }
  for (auto message_manager:message_manager_)
  {
    message_manager.second->Activate();
  }
  return;
}

void Prediction::MessagesDeActivate()
{
  if (prediction_conf_ == nullptr)
  {
    return;
  }
  for (auto message_manager:message_manager_)
  {
    message_manager.second->DeActivate();
  }
  return;
}

void Prediction::TaskActivate() 
{
  if(is_init_==false)
  {
    return;
  }
  // IPC激活
  MessagesActivate();
  if(function_activation_)
  {
    return;
  }
  task_10ms_->AddTimer(produce_prediction_command_duration_, &Prediction::Task10ms, this);
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout<<"===================function activate=================="<<"\n";
  function_activation_=true;
  return;
}

void Prediction::TaskStop() 
{
  if(is_init_==false)
  {
    return;
  }
  // IPC去激活
  MessagesDeActivate();
  if(function_activation_==false)
  {
    return;
  }
  {
    // 清除所有内部计算的中间结果，保证回到刚init完的状态
    std::lock_guard<std::mutex> lock(mutex_);
    task_10ms_->Stop();
    location_ = legionclaw::interface::Location();
    adc_trajectory_ = legionclaw::interface::ADCTrajectory();
    obstacle_list_ = legionclaw::interface::ObstacleList();
    odometry_ = legionclaw::interface::Odometry();
    routing_response_ = legionclaw::interface::RoutingResponse();
    lane_list_ = legionclaw::interface::LaneList();
    // map_loaded_ = false;
    location_init_ = false;
  }
  std::cout<<"******************function stop***************"<<"\n";
  function_activation_=false;
  return;
}

void Prediction::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<Prediction>> message_manager) {
  message_manager_.insert(
      std::pair<std::string, std::shared_ptr<MessageManager<Prediction>>>(
          name, message_manager));
}

void Prediction::Task10ms(void* param) {
  ComputePredictionCommandOnTimer();
}

void Prediction::PublishPredictionObstacles(
    legionclaw::interface::PredictionObstacles prediction_obstacles) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0) message_manager_["LCM"]->PublishPredictionObstacles(prediction_obstacles);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0) message_manager_["ROS"]->PublishPredictionObstacles(prediction_obstacles);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0) message_manager_["DDS"]->PublishPredictionObstacles(prediction_obstacles);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0) message_manager_["ROS2"]->PublishPredictionObstacles(prediction_obstacles);
#endif
}

void Prediction::PublishFaults() {
  if (is_init_ == false) {
    return;
  }  
  legionclaw::interface::Faults faults;
  faults.set_version(10001);
  faultcodeset_->get_fault_code_list(&faults);
  // 填app的id，唯一标识符
  faults.set_app_id(faultcodeset_->get_target_id());
  // 填状态
  faults.set_is_active(function_activation_);  
  // 消息头
  legionclaw::interface::Header header;
  INTERFACE_HEADER_ASSIGN(faults);
  // header.set_stamp(location_.header().stamp());
  faults.set_header(header);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_ = faults;
  }
 
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0) message_manager_["LCM"]->PublishFaults(faults_);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0) message_manager_["ROS"]->PublishFaults(faults_);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0) message_manager_["DDS"]->PublishFaults(faults_);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0) message_manager_["ROS2"]->PublishFaults(faults_);
#endif
  // faultcodeset_->clear_faults();
}

std::shared_ptr<PredictionConf> Prediction::GetConf() const {
  return prediction_conf_;
}

void Prediction::HandleLocation(legionclaw::interface::Location location) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = location.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      location.set_header(header);
    }
    location_ = location;
    location_init_ = true;
  }
}

void Prediction::HandleADCTrajectory(
    legionclaw::interface::ADCTrajectory adc_trajectory) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = adc_trajectory.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      adc_trajectory.set_header(header);
    }
    adc_trajectory_ = adc_trajectory;
  }
}

void Prediction::HandleObstacleList(
    legionclaw::interface::ObstacleList obstacle_list) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = obstacle_list.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obstacle_list.set_header(header);
    }
    obstacle_list_ = obstacle_list;
  }
}

void Prediction::HandleOdometry(legionclaw::interface::Odometry odometry) {
  if (is_init_ == false) {
      return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
      std::lock_guard<std::mutex> lock(mutex_);
      if (prediction_conf_->use_system_timestamp() == true) {
          legionclaw::interface::Header header = odometry.header();
          header.set_stamp(TimeTool::Now2TmeStruct());
          odometry.set_header(header);
      }
      odometry_ = odometry;
    
      //localiztion_mode: //0:ins, 1:odometry
      //默认是ins模式，收到Odometry（TF）消息则进入odom模式。不需要动态切回ins
      local_view_.localiztion_mode_ = 1;
  }
}

void Prediction::HandleTrafficEvents(
    legionclaw::interface::TrafficEvents traffic_events) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = traffic_events.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      traffic_events.set_header(header);
    }
      //fusion_flag: 0=use hdmap, 1=use localmap
      if (traffic_events.route_fusion_info().fusion_flag())
        local_view_.map_mode_ = 1;
      else if (prediction_conf_->map_mode() == 0)
        local_view_.map_mode_ = 0;
  }
}

void Prediction::HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg) {
  if (is_init_ == false) {
    return;
  }
  
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = obu_cmd_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obu_cmd_msg.set_header(header);
    }
    obu_cmd_msg_ = obu_cmd_msg;
  }
  for (auto cmd:obu_cmd_msg.obu_cmd_list())
  {
    // 编码值待定
    if(cmd.code()==10086)
    {
      std::cout<<"code : "<<cmd.code()<<"\n";
      switch (cmd.val())
      {
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

void Prediction::HandleRoutingResponse(
    legionclaw::interface::RoutingResponse routing_response) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = routing_response.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      routing_response.set_header(header);
    }
    routing_response_ = routing_response;
  }
}

void Prediction::HandleLaneList(legionclaw::interface::LaneList lane_list) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_==false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prediction_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = lane_list.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      lane_list.set_header(header);
    }
    lane_list_ = lane_list;
  }
}

void Prediction::ComputePredictionCommandOnTimer() {

  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.location_ = location_;
    local_view_.adc_trajectory_ = adc_trajectory_;
    local_view_.obstacle_list_ = obstacle_list_;
    local_view_.odometry_ = odometry_;
    // local_view_.traffic_events_ = traffic_events_;
    local_view_.routing_response_ = routing_response_;
    local_view_.lane_list_ = lane_list_;
  }

  Status status = CheckInput(&local_view_);
  // check data

  if (!map_loaded_ && location_init_)
  {
    local_view_.map_mode_ = prediction_conf_->map_mode();
    if (prediction_conf_->map_mode() == 0)
      vector_map_predictor_.LoadLanelet2Map(location_.origin_lat(), location_.origin_lon());
    map_loaded_ = true;
  }

  if (!status.ok() || !map_loaded_) {
  } else {
    //算法计算
    // double time1=legionclaw::common::TimeTool::NowToSeconds();
    //  double time2;

    if (vector_map_predictor_.Predict(local_view_) == Status(Status::ErrorCode::OK))
    {
      // time2=legionclaw::common::TimeTool::NowToSeconds();
      // std::cout<<"1111111111111111111 pred_time  :   "<<time2-time1<<"     seconds "<<"\n"<<"\n";
      legionclaw::interface::Header header;
      prediction_obstacles_ = vector_map_predictor_.PredictionObstacles();
      // double time3=legionclaw::common::TimeTool::NowToSeconds();
    // std::cout<<"222222222222222222222 pred_time  :   "<<time3-time2<<"     seconds "<<"\n"<<"\n";
      // if (prediction_out_array_.prediction_obstacles_size() > 0)
      // {
        INTERFACE_HEADER_ASSIGN(prediction_obstacles_);
        header.set_stamp(obstacle_list_.header().stamp());
        prediction_obstacles_.set_header(header);

        // double time4=legionclaw::common::TimeTool::NowToSeconds();
        // double time3=legionclaw::common::TimeTool::TimeStruct2S(obstacle_list_.header().stamp());
        // std::cout<<std::setprecision(17)<<time4<<"  "<<time1<<" pred_time = "<<time4-time1<<"  seconds "<<"\n";
        // std::cout<<"33333333333333333333 pred_time  :   "<<time4-time3<<"     seconds "<<"\n"<<"\n";
        PublishPredictionObstacles(prediction_obstacles_);
        // double time5=legionclaw::common::TimeTool::NowToSeconds();
        // std::cout<<"44444444444444444 pred_time  :   "<<time5-time4<<"     seconds "<<"\n"<<"\n";
      // }
    }
  }
}

Status Prediction::CheckInput(LocalView *local_view) 
{ 
  // if (local_view->obstacle_list_.obstacle_size() > 0)
    return Status::Ok(); 
  // else
  //   return Status(Status::ErrorCode::PREDICTION_ERROR,
  //                 "Obstaclelist is empty!.");

}

void Prediction::MessagesInit() {
  if (prediction_conf_ == nullptr)
    return;

  for (auto it : prediction_conf_->messages().active_message()) {
    auto message = prediction_conf_->messages().message_info().at(it);
    switch (message.type()) {
#if LCM_ENABLE
    case legionclaw::common::MessageType::LCM: {
      AINFO << "message type:LCM";

      lcm_message_manager_ = std::make_shared<LcmMessageManager<Prediction>>();
      ResigerMessageManager(message.name(), lcm_message_manager_);

      lcm_message_manager_->Init(this);
    } break;
#endif
#if DDS_ENABLE
    case legionclaw::common::MessageType::DDS: {
      AINFO << "message type:DDS";

      dds_message_manager_ = std::make_shared<DdsMessageManager<Prediction>>();
      ResigerMessageManager(message.name(), dds_message_manager_);

      dds_message_manager_->Init(this);
    } break;
#endif
#if ROS_ENABLE
    case legionclaw::common::MessageType::ROS: {
      AINFO << "message type:ROS";

      ros_message_manager_ = std::make_shared<RosMessageManager<Prediction>>();
      ResigerMessageManager(message.name(), ros_message_manager_);
      ros_message_manager_->Init(this);
    } break;
#endif
#if ROS2_ENABLE
    case legionclaw::common::MessageType::ROS2: {
      AINFO << "message type:ROS2";

      ros2_message_manager_ =
          std::make_shared<Ros2MessageManager<Prediction>>();
      ResigerMessageManager(message.name(), ros2_message_manager_);

      ros2_message_manager_->Init(this);
    } break;
#endif

#if ADSFI_ENABLE
    case legionclaw::common::MessageType::ADSFI: {
      AINFO << "message type:ADSFI";

      adsfi_message_manager_ =
          std::make_shared<AdsfiMessageManager<Prediction>>();
      ResigerMessageManager(message.name(), adsfi_message_manager_);

      adsfi_message_manager_->Init(this);
    } break;
#endif
    default: { AERROR << "unknown message type"; } break;
    }
  }
}

void Prediction::FaultMonitorInit() {
  legionclaw::interface::FaultCodeCallback sendheart_callback_func = std::bind(&Prediction::PublishFaults, this);
  legionclaw::interface::FaultCodeCallback fault_callback_func = nullptr;
  FAULTCODE_INIT("../../common/data/faults/faults.json", "prediction", faultcodeset_, sendheart_callback_func, fault_callback_func)
}

void Prediction::StatusDetectOnTimer() {}

void Prediction::Spin() {
  while (1) {
    if (function_activation_) {
      ad_timer_manager_->DetectTimers(NULL);
      usleep(1000);
    } else usleep(100000);
  }
}

} // namespace prediction
} // namespace legionclaw
