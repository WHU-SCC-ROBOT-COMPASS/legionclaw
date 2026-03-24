/**
 * @file    lidar_ground_segmentation.cpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <time.h>
#include <fstream>
#include <sys/time.h>

#include "lidar_ground_segmentation.h"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/interface/point_cloud.hpp"

/**
 * @namespace legion::perception::lidar
 * @brief legion::perception::lidar
 */

namespace legion {
namespace perception {
namespace lidar {

void LidarGroundSegmentation::Init() {
  // step1 初始化状态设置为false
  {
    is_init_ = false;
    function_activation_ == false;
  }

  // step2 变量初始化
  { VariableInit(); }

  // step3 配置文件初始化
  {
    std::ifstream in(config_file_path_);
    in >> lidar_ground_segmentation_json_;
    if (lidar_ground_segmentation_json_.is_null()) {
      std::cout << "lidar_ground_segmentation_json_ is null" << std::endl;
      return;
    }
  }
  // step4 日志初始化
  {LOGGING_INIT(lidar_ground_segmentation_conf_, lidar_ground_segmentation_json_)}

  // step4 IPC初始化
  {MESSAGE_INIT(lidar_ground_segmentation_conf_, lidar_ground_segmentation_json_)}

  // step5 读取配置文件
  {
    produce_lidar_ground_segmentation_command_duration_ = lidar_ground_segmentation_json_
        ["produce_lidar_ground_segmentation_command_duration"];
    publish_lidar_ground_segmentation_command_duration_ = lidar_ground_segmentation_json_
        ["publish_lidar_ground_segmentation_command_duration"];
    lidar_ground_segmentation_conf_->set_use_system_timestamp(
        lidar_ground_segmentation_json_["use_system_timestamp"]);
  }

  patchwork::Params params = LoadParamsFromJSON(lidar_ground_segmentation_json_["params_file"]);
  patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);

  // step6 故障码初始化
  FaultMonitorInit();

  // step7 算法初始化
  {}

  // step8 定时器和线程初始化
  {
    status_detect_duration_ = (uint32_t)(
        double)lidar_ground_segmentation_json_["status"]["status_detect_duration"];

    ad_timer_manager_ =
        std::make_shared<ADTimerManager<LidarGroundSegmentation, void>>();
    task_1000ms_ = std::make_shared<WheelTimer<LidarGroundSegmentation, void>>(
        ad_timer_manager_);
    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create task_thread_ thread.";
      return;
    }
  }
  std::cout << "lidar_ground_segmentation init" << std::endl;

  // step9 初始化状态为true
  { is_init_ = true; }

  // 手动激活
  TaskActivate();
}

void LidarGroundSegmentation::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void LidarGroundSegmentation::VariableInit() {
  lidar_ground_segmentation_conf_ = std::make_shared<LidarSegmentGroundConf>();
}

patchwork::Params LidarGroundSegmentation::LoadParamsFromJSON(const std::string& config_file) {
    patchwork::Params params;
    
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open config file: " << config_file << std::endl;
        std::cerr << "Using default parameters." << std::endl;
        return params;
    }
    
    nlohmann::json j;
    file >> j;
    file.close();
    
    if (j.find("patchworkpp") != j.end()) {
        auto& pp = j["patchworkpp"];
        
        if (pp.find("verbose") != pp.end()) params.verbose = pp["verbose"];
        if (pp.find("sensor_height") != pp.end()) params.sensor_height = pp["sensor_height"];
        if (pp.find("filter_ground_height") != pp.end()) params.filter_ground_height = pp["filter_ground_height"];
        if (pp.find("num_iter") != pp.end()) params.num_iter = pp["num_iter"];
        if (pp.find("num_lpr") != pp.end()) params.num_lpr = pp["num_lpr"];
        if (pp.find("num_min_pts") != pp.end()) params.num_min_pts = pp["num_min_pts"];
        if (pp.find("th_seeds") != pp.end()) params.th_seeds = pp["th_seeds"];
        if (pp.find("th_dist") != pp.end()) params.th_dist = pp["th_dist"];
        if (pp.find("th_seeds_v") != pp.end()) params.th_seeds_v = pp["th_seeds_v"];
        if (pp.find("th_dist_v") != pp.end()) params.th_dist_v = pp["th_dist_v"];
        if (pp.find("max_range") != pp.end()) params.max_range = pp["max_range"];
        if (pp.find("min_range") != pp.end()) params.min_range = pp["min_range"];
        if (pp.find("uprightness_thr") != pp.end()) params.uprightness_thr = pp["uprightness_thr"];
        if (pp.find("adaptive_seed_selection_margin") != pp.end()) params.adaptive_seed_selection_margin = pp["adaptive_seed_selection_margin"];
        if (pp.find("RNR_ver_angle_thr") != pp.end()) params.RNR_ver_angle_thr = pp["RNR_ver_angle_thr"];
        if (pp.find("RNR_intensity_thr") != pp.end()) params.RNR_intensity_thr = pp["RNR_intensity_thr"];
        if (pp.find("max_flatness_storage") != pp.end()) params.max_flatness_storage = pp["max_flatness_storage"];
        if (pp.find("max_elevation_storage") != pp.end()) params.max_elevation_storage = pp["max_elevation_storage"];
        if (pp.find("enable_RNR") != pp.end()) params.enable_RNR = pp["enable_RNR"];
        if (pp.find("enable_RVPF") != pp.end()) params.enable_RVPF = pp["enable_RVPF"];
        if (pp.find("enable_TGR") != pp.end()) params.enable_TGR = pp["enable_TGR"];
        
        if (pp.find("czm") != pp.end()) {
            auto& czm = pp["czm"];
            if (czm.find("num_zones") != czm.end()) params.num_zones = czm["num_zones"];
            if (czm.find("num_rings_of_interest") != czm.end()) params.num_rings_of_interest = czm["num_rings_of_interest"];
            if (czm.find("num_sectors_each_zone") != czm.end()) {
                params.num_sectors_each_zone = czm["num_sectors_each_zone"].get<std::vector<int>>();
            }
            if (czm.find("mum_rings_each_zone") != czm.end()) {
                params.num_rings_each_zone = czm["mum_rings_each_zone"].get<std::vector<int>>();
            }
            if (czm.find("elevation_thr") != czm.end()) {
                params.elevation_thr = czm["elevation_thr"].get<std::vector<double>>();
            }
            if (czm.find("flatness_thr") != czm.end()) {
                params.flatness_thr = czm["flatness_thr"].get<std::vector<double>>();
            }
        }
    }
    
    return params;
}

void LidarGroundSegmentation::Print() {}

void LidarGroundSegmentation::Log() {}

void LidarGroundSegmentation::TaskActivate() {
  if (is_init_ == false) {
    return;
  }
  // IPC激活
  MessagesActivate();
  if (function_activation_) {
    return;
  }
  task_1000ms_->AddTimer(1000, &LidarGroundSegmentation::Task1000ms, this);
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout << "===================function activate=================="
            << std::endl;
  function_activation_ = true;
  return;
}

void LidarGroundSegmentation::TaskStop() {
  if (is_init_ == false) {
    return;
  }
  // IPC去激活
  MessagesDeActivate();
  if (function_activation_ == false) {
    return;
  }
  task_1000ms_->Stop();
  {
    // 清除所有内部计算的中间结果，保证回到刚init完的状态
  }
  std::cout << "******************function stop***************" << std::endl;
  function_activation_ = false;
  return;
}

void LidarGroundSegmentation::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<LidarGroundSegmentation>> message_manager) {
  message_manager_.insert(
      std::pair<std::string,
                std::shared_ptr<MessageManager<LidarGroundSegmentation>>>(
          name, message_manager));
}

void LidarGroundSegmentation::Task1000ms(void* param) {}

void LidarGroundSegmentation::PublishGroundPoints(
    legion::interface::PointCloud &ground_points) {
#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishGroundPoints(ground_points);
#endif
}
void LidarGroundSegmentation::PublishNoGroundPoints(
    legion::interface::PointCloud &no_ground_points) {
#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishNoGroundPoints(no_ground_points);
#endif
}
void LidarGroundSegmentation::PublishFaults() {
  if (is_init_ == false) {
    return;
  }
  legion::interface::Faults faults;
  faultcodeset_->get_fault_code_list(&faults);
  // 填app的id，唯一标识符
  faults.set_app_id(faultcodeset_->get_target_id());
  // 填状态
  faults.set_is_active(function_activation_);
  // 消息头
  legion::interface::Header header;
  INTERFACE_HEADER_ASSIGN(faults);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_ = faults;
  }
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishFaults(faults);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishFaults(faults);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishFaults(faults);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishFaults(faults);
#endif
  faultcodeset_->clear_faults();
}

std::shared_ptr<LidarSegmentGroundConf> LidarGroundSegmentation::GetConf() const {
  return lidar_ground_segmentation_conf_;
}

void LidarGroundSegmentation::HandleObuCmdMsg(legion::interface::ObuCmdMsg obu_cmd_msg) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lidar_ground_segmentation_conf_->use_system_timestamp() == true) {
      legion::interface::Header header = obu_cmd_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obu_cmd_msg.set_header(header);
    }
    obu_cmd_msg_ = obu_cmd_msg;
  }
  for (auto cmd : obu_cmd_msg.obu_cmd_list()) {
    // 编码值待定
    if (cmd.code() == 10086) {
      std::cout << "code : " << cmd.code() << std::endl;
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

void LidarGroundSegmentation::HandlePointCloudInput(legion::interface::PointCloud cloud) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_ == false) {
    return;
  }
  if (patchworkpp_ == nullptr) {
    return;
  }

  const auto &cloud_mat = patchworkpp_ros::utils::PointCloud2ToEigenMat(cloud);
  // 调用patchworkpp进行地面分割
  patchworkpp_->estimateGround(cloud_mat);

  // 获取地面点和非地面点
  Eigen::MatrixX3f ground_points = patchworkpp_->getGround();
  Eigen::MatrixX3f nonground_points = patchworkpp_->getNonground();

  legion::interface::PointCloud legion_ground_points = patchworkpp_ros::utils::EigenMatToPointCloud2(ground_points, cloud.header());
  legion::interface::PointCloud legion_nonground_points = patchworkpp_ros::utils::EigenMatToPointCloud2(nonground_points, cloud.header());
  // 发布地面点和非地面点
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishGroundPoints(legion_ground_points);
    PublishNoGroundPoints(legion_nonground_points);
  }
}

void LidarGroundSegmentation::ComputeLidarSegmentGroundCommandOnTimer() {

  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.point_cloud_input_ = point_cloud_input_;
  }

  Status status = CheckInput(&local_view_);
  // check data

  if (!status.ok()) {
  } else {
  }

  //算法计算
}
Status LidarGroundSegmentation::CheckInput(LocalView* local_view) {
  return Status::Ok();
}

void LidarGroundSegmentation::MessagesInit() {
  if (lidar_ground_segmentation_conf_ == nullptr)
    return;

  std::map<std::string, legion::common::Message>::iterator iter;
  for (auto& iter : lidar_ground_segmentation_conf_->messages()) {
    auto message = iter.second;

    switch (message.type) {
#if LCM_ENABLE
    case legion::common::MessageType::LCM: {
      AINFO << "message type:LCM";

      lcm_message_manager_ =
          std::make_shared<LcmMessageManager<LidarGroundSegmentation>>();
      ResigerMessageManager(message.name, lcm_message_manager_);

      lcm_message_manager_->Init(this);
    } break;
#endif
#if DDS_ENABLE
    case legion::common::MessageType::DDS: {
      AINFO << "message type:DDS";

      dds_message_manager_ =
          std::make_shared<DdsMessageManager<LidarGroundSegmentation>>();
      ResigerMessageManager(message.name, dds_message_manager_);

      dds_message_manager_->Init(this);
    } break;
#endif
#if ROS_ENABLE
    case legion::common::MessageType::ROS: {
      AINFO << "message type:ROS";

      ros_message_manager_ =
          std::make_shared<RosMessageManager<LidarGroundSegmentation>>();
      ResigerMessageManager(message.name, ros_message_manager_);
      ros_message_manager_->Init(this);
    } break;
#endif
#if ROS2_ENABLE
    case legion::common::MessageType::ROS2: {
      AINFO << "message type:ROS2";

      ros2_message_manager_ =
          std::make_shared<Ros2MessageManager<LidarGroundSegmentation>>();
      ResigerMessageManager(message.name, ros2_message_manager_);

      ros2_message_manager_->Init(this);
    } break;
#endif

#if ADSFI_ENABLE
    case legion::common::MessageType::ADSFI: {
      AINFO << "message type:ADSFI";

      adsfi_message_manager_ =
          std::make_shared<AdsfiMessageManager<LidarGroundSegmentation>>();
      ResigerMessageManager(message.name, adsfi_message_manager_);

      adsfi_message_manager_->Init(this);
    } break;
#endif
    default: {
      AERROR << "unknown message type";
    } break;
    }
  }
}

void LidarGroundSegmentation::FaultMonitorInit() {
  legion::interface::FaultCodeCallback sendheart_callback_func =
      std::bind(&LidarGroundSegmentation::PublishFaults, this);
  legion::interface::FaultCodeCallback fault_callback_func = nullptr;
  FAULTCODE_INIT("../../../../common/data/faults/faults.json",
                 "lidar_ground_segmentation", faultcodeset_, sendheart_callback_func,
                 fault_callback_func)
}

void LidarGroundSegmentation::MessagesActivate() {
  if (lidar_ground_segmentation_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->Activate();
  }
  return;
}

void LidarGroundSegmentation::MessagesDeActivate() {
  if (lidar_ground_segmentation_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->DeActivate();
  }
  return;
}

void LidarGroundSegmentation::StatusDetectOnTimer() {}

void LidarGroundSegmentation::Spin() {
  while (1) {
    if (function_activation_) {
      ad_timer_manager_->DetectTimers(NULL);
      usleep(1000);
    } else
      usleep(100000);
  }
}

} // namespace lidar
} // namespace perception
} // namespace legion
