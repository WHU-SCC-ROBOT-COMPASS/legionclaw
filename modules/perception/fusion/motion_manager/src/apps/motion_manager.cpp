/**
 * @file    motion_manager.cpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <fstream>
#include <sys/time.h>
#include <time.h>

#include "modules/common/interface/location.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"
#include "motion_manager.h"

#include <chrono>
#include <fstream>
#include <ctime>

/**
 * @namespace legion::perception::fusion
 * @brief legion::perception::fusion
 */

using Label = motion_manager::interface::ObjectClassification;

// 对比函数
bool cmp(legion::interface::Obstacle &a, legion::interface::Obstacle &b)
{
  return a.id() < b.id();
}

namespace legion{
namespace perception{
namespace fusion{

void MotionManager::Init()
{
  // step1 初始化状态设置为false
  {
    is_init_ = false;
  }

  // step2 变量初始化
  {
    VariableInit();
  }

  // step3 配置文件初始化
  {
    std::ifstream in(config_file_path_);
    in >> motion_manager_json_;
    if (motion_manager_json_.is_null())
    {
      std::cout << "motion_manager_json_ is null" << "\n";
      return;
    }
  }
  // step4 日志初始化
  {
    LOGGING_INIT(motion_manager_conf_, motion_manager_json_)
  }

  // step4 IPC初始化
  {
    MESSAGE_INIT(motion_manager_conf_, motion_manager_json_)
  }

  // step5 读取配置文件
  {
    produce_motion_manager_command_duration_ =
        motion_manager_json_["produce_motion_manager_command_duration"];
    publish_motion_manager_command_duration_ =
        motion_manager_json_["publish_motion_manager_command_duration"];
    motion_manager_conf_->set_use_system_timestamp(
        motion_manager_json_["use_system_timestamp"]);
    id_num_max = motion_manager_json_["id_num_max"];
    use_location_time = motion_manager_json_["use_location_time"];
    heading_error = motion_manager_json_["heading_error"];
    use_sync = motion_manager_json_["use_sync"];
    obj_rescale = motion_manager_json_["obj_rescale"];
    // ========== 融合匹配相关参数 ==========
    iou_threshold = motion_manager_json_.value("iou_threshold", 0.01);  // IOU阈值，默认0.01
    data_timeout_ms = motion_manager_json_.value("data_timeout_ms", 3000.0);  // 数据超时时间（ms），默认3000ms
    grid_size = motion_manager_json_.value("grid_size", 0.1);  // 位置网格化精度（米），默认0.1米
    history_match_max_distance = motion_manager_json_.value("history_match_max_distance", 5.0);  // 历史匹配最大距离（米），默认5.0米
    close_match_distance = motion_manager_json_.value("close_match_distance", 2.0);  // 近距离匹配阈值（米），默认2.0米
    very_close_match_distance = motion_manager_json_.value("very_close_match_distance", 1.0);  // 非常近距离匹配阈值（米），默认1.0米
    
    // ========== ID管理相关参数 ==========
    lcd_id_offset = motion_manager_json_.value("lcd_id_offset", 10000);  // LCD ID偏移量，默认10000
    
    // ========== 时间同步相关参数 ==========
    duplicate_frame_threshold = motion_manager_json_.value("duplicate_frame_threshold", 0.01);  // 重复帧检测阈值（秒），默认0.01秒
    time_sync_max_diff = motion_manager_json_.value("time_sync_max_diff", 1.0);  // 时间同步最大时间差（秒），默认1.0秒
    match_quality_distance_offset = motion_manager_json_.value("match_quality_distance_offset", 0.1);  // 匹配质量计算距离偏移，默认0.1

    fixed_obstacle = motion_manager_json_["fixed_obstacle"];

    config_path = motion_manager_json_["config_path"];

    std::ifstream file(fixed_obstacle);
    file >> confjson;
    file.close();
  }

  // step6 故障码初始化
  FaultMonitorInit();

  // step7 算法初始化
  {
    cout << "config_path: " << config_path << endl;
    tracker_.parseConfig(config_path);
  }

  // step8 定时器和线程初始化
  {
    status_detect_duration_ = (uint32_t)(double)motion_manager_json_["status"]["status_detect_duration"];

    // TimerManager<MotionManager>::AddTimer(
    //     produce_motion_manager_command_duration_,
    //     &MotionManager::ComputeMotionManagerCommandOnTimer, this);

    // TimerManager<MotionManager>::AddTimer(
    //     status_detect_duration_, &MotionManager::StatusDetectOnTimer, this);

    ad_timer_manager_ =
        std::make_shared<ADTimerManager<MotionManager, void>>();
    task_compute_ = std::make_shared<WheelTimer<MotionManager, void>>(
        ad_timer_manager_);
    // task_10ms_->AddTimer(10, &MotionManager::Task10ms, this);
    task_thread_.reset(new std::thread([this]
                                        { Spin(); }));
    if (task_thread_ == nullptr)
    {
      AERROR << "Unable to create can task_thread_ thread.";
      return;
    }
  }

  // step9 初始化状态为true
  {
    is_init_ = true;
  }

  // 手动激活
  TaskActivate();
}

void MotionManager::Join()
{
  if (task_thread_ != nullptr && task_thread_->joinable())
  {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void MotionManager::VariableInit()
{
  motion_manager_conf_ = std::make_shared<MotionManagerConf>();
  // 初始化数据源最后更新时间
  last_obstacle_list_input_time_ms_ = 0;
  last_lcd_obstacle_list_input_time_ms_ = 0;
  // 初始化融合ID稳定化相关变量
  fusion_global_idx_ = 0;
  fusion_id_map_.clear();
  lidar_id_history_.clear();
  id_to_position_.clear();
  obstacle_history_.clear();
}

void MotionManager::Print() {}

void MotionManager::Log() {}

void MotionManager::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<MotionManager>> message_manager)
{
  message_manager_.insert(
      std::pair<std::string,
                std::shared_ptr<MessageManager<MotionManager>>>(
          name, message_manager));
}

void MotionManager::Task10ms(void *param) {}

void MotionManager::PublishObstacleListOutput(
    legion::interface::ObstacleList obstacle_list_output)
{


#if LCM_ENABLE
  message_manager_["LCM"]->PublishObstacleListOutput(obstacle_list_output);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishObstacleListOutput(obstacle_list_output);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishObstacleListOutput(obstacle_list_output);
#endif

#if ROS2_ENABLE
std::cout << "PublishObstacleListOutput ROS2" << "\n";
  message_manager_["ROS2"]->PublishObstacleListOutput(obstacle_list_output);
#endif
}

std::shared_ptr<MotionManagerConf> MotionManager::GetConf() const
{
  return motion_manager_conf_;
}

void MotionManager::HandleObuCmdMsg(legion::interface::ObuCmdMsg obu_cmd_msg)
{
  if (is_init_ == false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_manager_conf_->use_system_timestamp() == true)
    {
      legion::interface::Header header = obu_cmd_msg.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obu_cmd_msg.set_header(header);
    }
    obu_cmd_msg_ = obu_cmd_msg;
  }
  for (auto cmd : obu_cmd_msg.obu_cmd_list())
  {
    // 编码值待定
    if (cmd.code() == 10086)
    {
      std::cout << "code : " << cmd.code() << "\n";
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

void MotionManager::HandleLocation(legion::interface::Location location)
{
  if (is_init_ == false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_manager_conf_->use_system_timestamp() == true)
    {
      legion::interface::Header header = location.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      location.set_header(header);
    }
    location_ = location;
    // 2024_01_15 考虑到激光雷达到惯导存在1.58度左右的偏差，修正之  
    // 通过配置文件配置heading_error
    double temp_heading = location_.heading() - heading_error * M_PI / 180;
    location_.set_heading(temp_heading);
  }
}

void MotionManager::HandleObstacleListInput(
    legion::interface::ObstacleList obstacle_list_input)
{
  if (is_init_ == false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_manager_conf_->use_system_timestamp() == true)
    {
      legion::interface::Header header = obstacle_list_input.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      obstacle_list_input.set_header(header);
    }
    obstacle_list_input_ = obstacle_list_input;
    for (auto &ob : obstacle_list_input_.obstacle())
    {
      ob.set_id(-1);
    }
    // 更新最后接收时间
    last_obstacle_list_input_time_ms_ = legion::common::TimeTool::Now2Ms();
    AINFO << "=====================receive obstacles=====================";
  }
}

void MotionManager::HandleLCDObstacleList(
    legion::interface::ObstacleList lcd_obstacle_list)
{
  if (is_init_ == false)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_manager_conf_->use_system_timestamp() == true)
    {
      legion::interface::Header header = lcd_obstacle_list.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      lcd_obstacle_list.set_header(header);
    }
    // 将 LCDObstacleList 存储到单独的变量中
    lcd_obstacle_list_input_ = lcd_obstacle_list;
    // 确保header被正确设置（赋值操作可能不会保留header_ptr_）
    if (lcd_obstacle_list.has_header())
    {
      lcd_obstacle_list_input_.set_header(lcd_obstacle_list.header());
    }
    // 更新最后接收时间
    last_lcd_obstacle_list_input_time_ms_ = legion::common::TimeTool::Now2Ms();
    // for (auto &ob : lcd_obstacle_list_input_.obstacle())
    // {
    //   ob.set_id(-1);
    // }
    AINFO << "=====================receive LCD obstacles from lidar_cluster_detect=====================";

  }
}

void MotionManager::ComputeMotionManagerCommandOnTimer(void*)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.location_ = location_;
    local_view_.obstacle_list_input_ = obstacle_list_input_;
  }

  Status status = CheckInput(&local_view_);
  // check data

  if (!status.ok())
  {
  }
  else
  {
    std::lock_guard<std::mutex> lock(mutex_);
  }
  // 算法计算
  MotionManagerRun();
}
Status MotionManager::CheckInput(LocalView *local_view)
{
  return Status::Ok();
}

void MotionManager::MotionManagerRun()
{
  legion::interface::Location location = local_view_.location_;
  legion::interface::ObstacleList obstacle_list_input = local_view_.obstacle_list_input_;

  double time_begin = legion::common::TimeTool::Now2Ms();

  // 检查是否有输入数据（obstacle_list_input 或 lcd_obstacle_list_input_）
  bool has_lcd_only = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 如果 obstacle_list_input 为空，但 lcd_obstacle_list_input_ 不为空，则直接进入融合流程
    if (obstacle_list_input.obstacle_size() == 0 && lcd_obstacle_list_input_.obstacle_size() > 0)
    {
      has_lcd_only = true;
      
      // 过滤重复帧
      if (!is_first_frame_)
      {
        double time_last = last_frame_obs_.header().stamp().sec() + last_frame_obs_.header().stamp().nsec() * 1e-9;
        double time_curr = lcd_obstacle_list_input_.header().stamp().sec() + lcd_obstacle_list_input_.header().stamp().nsec() * 1e-9;
        double del_time = time_curr - time_last;
        if (abs(del_time) <= duplicate_frame_threshold)
        {
          std::cout << std::to_string(del_time) << "\n";
          AINFO << "Recieve repeated LCD obstacle message";
          return;
        }
      }

      // 确保 obstacle_list_output_ 为空，以便 FuseObstacleLists 正确处理
      obstacle_list_output_.clear_obstacle();
      obstacle_list_output_.set_is_valid(false);
      
      // 障碍物与location时间同步
      if (use_sync == 1)
      {
        // 对于 LCDObstacleList，使用其时间戳进行同步
        SyncStamp(lcd_obstacle_list_input_, location);
      }
      else
      {
        location_current = location;
      }
    }
  }
  
  // 在锁外调用 FuseObstacleLists，避免死锁（FuseObstacleLists 内部会获取锁）
  if (has_lcd_only)
  {
    // 直接融合并发布 LCDObstacleList
    FuseObstacleLists();
    
      // 在发布之前，进行ID稳定化处理，确保ID一致性
      {
        std::lock_guard<std::mutex> lock(mutex_);
        StabilizeFusionIds(fusion_obstacle_list_, last_frame_obs_);
        UpdateObstacleHistory(fusion_obstacle_list_);
        
        // 在发布前打印所有障碍物的ID
        std::cout << "=== Publish Obstacle IDs (LCD only) ===" << "\n";
        for (const auto &ob : fusion_obstacle_list_.obstacle())
        {
          std::cout << "Obstacle ID: " << ob.id() << "\n";
        }
        std::cout << "Total obstacles: " << fusion_obstacle_list_.obstacle_size() << "\n";
        std::cout << "=======================================" << "\n";
        
        PublishObstacleListOutput(fusion_obstacle_list_);

      is_first_frame_ = 0;
      last_frame_obs_ = fusion_obstacle_list_;
    }

    double time_end = legion::common::TimeTool::Now2Ms();
    std::cout << "Run time : " << time_end - time_begin << " ms " << "\n";
    cout << "Publish obstacle size (LCD only): " << fusion_obstacle_list_.obstacle_size();
    cout << "***************" << endl;
    return;
  }

  // 如果 obstacle_list_input 也为空，则直接返回
  if (obstacle_list_input.obstacle_size() == 0)
  {
    AINFO << "No obstacle input, skip processing";
    return;
  }

  //  过滤重复帧
  if (!is_first_frame_)
  {
    double time_last = last_frame_obs_.header().stamp().sec() + last_frame_obs_.header().stamp().nsec() * 1e-9;
    double time_curr = obstacle_list_input.header().stamp().sec() + obstacle_list_input.header().stamp().nsec() * 1e-9;
    double del_time = time_curr - time_last;
    if (abs(del_time) <= 0.01)
    {
      std::cout << std::to_string(del_time) << "\n";
      AINFO << "Recieve repeated obstacle message";
      return;
    }
  }

  // 障碍物与location时间同步
  if (use_sync == 1)
  {
    SyncStamp(obstacle_list_input, location);
  }
  else
  {
    location_current = location;
  }

  cout << "Receive obstaclelist input size: " << obstacle_list_input.obstacle().size() << endl;

  GetPolygon(obstacle_list_input);

  VehicleToWorld(obstacle_list_input, location);

  motion_manager::interface::DetectedObjects objects;
  TransObstacle(objects, obstacle_list_input);

  std::shared_ptr<motion_manager::interface::DetectedObjects> input_objects_msg = std::make_shared<motion_manager::interface::DetectedObjects>(objects);

  motion_manager::interface::TrackedObjects output_msg;
  output_msg = tracker_.onMeasurement(input_objects_msg, location_current);

  // 使用成员变量而不是局部变量，以便 FuseObstacleLists 可以访问
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_list_output_.set_header(obstacle_list_input.header());
  }

  TransObstacle(obstacle_list_output_, output_msg);

  legion::interface::Header header = obstacle_list_output_.header();
  for (auto &ob : obstacle_list_output_.obstacle())
  {
    ob.set_timestamp(header.stamp());
  }

  // 计算加速度
  Getacc(obstacle_list_output_);

  // 由于卡尔曼滤波输出的id较长且不连续，对目标id进行重新赋值
  ResetId(obstacle_list_output_);

  FilterVel(obstacle_list_output_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_list_output_.set_is_valid(true);
  }

  // 融合 lcd_obstacle_list_input_ 和 obstacle_list_output_
  FuseObstacleLists();
  
  // 在发布之前，进行ID稳定化处理，确保ID一致性
  // 这可以处理数据源闪烁和形状差异带来的ID跳变问题
  {
    std::lock_guard<std::mutex> lock(mutex_);
    StabilizeFusionIds(fusion_obstacle_list_, last_frame_obs_);
    UpdateObstacleHistory(fusion_obstacle_list_);
  }

  // 在发布前打印所有障碍物的ID
  std::cout << "=== Publish Obstacle IDs ===" << "\n";
  for (const auto &ob : fusion_obstacle_list_.obstacle())
  {
    std::cout << "Obstacle ID: " << ob.id() << "\n";
  }
  std::cout << "Total obstacles: " << fusion_obstacle_list_.obstacle_size() << "\n";
  std::cout << "===========================" << "\n";
  
  PublishObstacleListOutput(fusion_obstacle_list_);

  is_first_frame_ = 0;
  last_frame_obs_ = fusion_obstacle_list_;

  double time_end = legion::common::TimeTool::Now2Ms();

  std::cout << "Run time : " << time_end - time_begin << " ms " << "\n";
  cout << "Publish obstacle size: " << fusion_obstacle_list_.obstacle_size();
  cout << "***************" << endl;
}

void MotionManager::MessagesInit()
{
  if (motion_manager_conf_ == nullptr)
    return;

  std::map<std::string, legion::common::Message>::iterator iter;
  for (auto &iter : motion_manager_conf_->messages())
  {
    auto message = iter.second;

    switch (message.type)
    {
#if LCM_ENABLE
    case legion::common::MessageType::LCM:
    {
      AINFO << "message type:LCM";

      lcm_message_manager_ =
          std::make_shared<LcmMessageManager<MotionManager>>();
      ResigerMessageManager(message.name, lcm_message_manager_);

      lcm_message_manager_->Init(this);
    }
    break;
#endif
#if DDS_ENABLE
    case legion::common::MessageType::DDS:
    {
      AINFO << "message type:DDS";

      dds_message_manager_ =
          std::make_shared<DdsMessageManager<MotionManager>>();
      ResigerMessageManager(message.name, dds_message_manager_);

      dds_message_manager_->Init(this);
    }
    break;
#endif
#if ROS_ENABLE
    case legion::common::MessageType::ROS:
    {
      AINFO << "message type:ROS";

      ros_message_manager_ =
          std::make_shared<RosMessageManager<MotionManager>>();
      ResigerMessageManager(message.name, ros_message_manager_);
      ros_message_manager_->Init(this);
    }
    break;
#endif
#if ROS2_ENABLE
    case legion::common::MessageType::ROS2:
    {
      AINFO << "message type:ROS2";

      ros2_message_manager_ =
          std::make_shared<Ros2MessageManager<MotionManager>>();
      ResigerMessageManager(message.name, ros2_message_manager_);

      ros2_message_manager_->Init(this);
    }
    break;
#endif

#if ADSFI_ENABLE
    case legion::common::MessageType::ADSFI:
    {
      AINFO << "message type:ADSFI";

      adsfi_message_manager_ =
          std::make_shared<AdsfiMessageManager<MotionManager>>();
      ResigerMessageManager(message.name, adsfi_message_manager_);

      adsfi_message_manager_->Init(this);
    }
    break;
#endif
    default:
    {
      AERROR << "unknown message type";
    }
    break;
    }
  }
}

void MotionManager::FilterVel(legion::interface::ObstacleList &obstacle_list_output_)
{
  for (auto &ob : obstacle_list_output_.obstacle())
  {
    double vel = sqrt(pow(ob.velocity_abs().x(), 2) + pow(ob.velocity_abs().y(), 2));
    if (ob.sub_type() == 10 && vel > 3)
    {
      legion::interface::Point3D temp_vel;
      temp_vel.set_x(0);
      temp_vel.set_y(0);
      temp_vel.set_z(0);
      ob.set_velocity_abs(temp_vel);
    }
  }
}

void MotionManager::ResetId(legion::interface::ObstacleList &obstacle_list_output_)
{
  if (is_first_frame_)
  {
    for (auto &ob : obstacle_list_output_.obstacle())
    {
      int ori_id = ob.id();
      ob.set_id(global_idx);
      id_convert_map_.insert(pair<int, int>(ori_id, global_idx));
      global_idx++;
    }
    return;
  }
  set<int> id_set;
  for (auto ob : obstacle_list_output_.obstacle())
  {
    id_set.insert(ob.id());
  }

  for (auto &ob : obstacle_list_output_.obstacle())
  {
    int ori_id = ob.id();
    auto it = id_convert_map_.find(ori_id);
    if (it != id_convert_map_.end())
    { // 对于已跟踪上的目标，直接赋值即可
      ob.set_id(it->second);
    }
    else
    { // 对于新进目标，赋新id
      ob.set_id(global_idx);
      id_convert_map_.insert(pair<int, int>(ori_id, global_idx));
      global_idx++;
      while (true)
      {
        int is_repeated = 0;
        auto it = id_set.find(global_idx);
        if (it == id_set.end())
        {
          break;
        }
        else
        {
          global_idx++;
        }
      }
    }
  }

  if (global_idx > id_num_max)
  {
    global_idx = 0;
  }
}

void MotionManager::Settype(legion::interface::Obstacle &ob, motion_manager::interface::ObjectClassification &cls)
{
  if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_CAR)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_CAR);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_TRUCK)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_TRUCK);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_BUS)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_BUS);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_MOTORCYCLIST);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_BICYCLE)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_CYCLIST);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_PEDESTRIAN)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_PEDESTRIAN);
  }
  else if (cls.label() == motion_manager::common::DetectedObjectLabel::DO_UNKNOWN)
  {
    ob.set_sub_type(legion::common::ObstacleSubType::ST_UNKNOWN);
  }
}

void MotionManager::TransObstacle(motion_manager::interface::DetectedObjects &objects, legion::interface::ObstacleList &ob_list)
{
  // Convert Header from legion::interface to motion_manager::interface
  motion_manager::interface::Header header;
  const legion::interface::Header &legion_header = ob_list.header();
  header.set_seq(legion_header.seq());
  header.set_frame_id(legion_header.frame_id());
  // Convert Time if needed
  motion_manager::interface::Time stamp;
  const legion::interface::Time &legion_stamp = legion_header.stamp();
  stamp.set_sec(legion_stamp.sec());
  stamp.set_nsec(legion_stamp.nsec());
  header.set_stamp(stamp);
  objects.set_header(header);
  
  for (auto &obstacle : ob_list.obstacle())
  {
    motion_manager::interface::DetectedObject object;
    object.set_existence_probability(obstacle.existence_prob());

    motion_manager::interface::Shape shape;
    shape.set_type(0);

    motion_manager::interface::Point3D dimensions;
    dimensions.set_x(obstacle.length());
    dimensions.set_y(obstacle.width());
    dimensions.set_z(obstacle.height());
    shape.set_dimensions(dimensions);

    object.set_shape(shape);

    motion_manager::interface::ObjectClassification cls;
    if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_CAR)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_CAR);
    }
    else if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_TRUCK)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_TRUCK);
    }
    else if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_BUS)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_BUS);
    }
    else if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_MOTORCYCLIST ||
              obstacle.sub_type() == legion::common::ObstacleSubType::ST_TRICYCLIST)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_MOTORCYCLE);
    }
    else if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_CYCLIST)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_BICYCLE);
    }
    else if (obstacle.sub_type() == legion::common::ObstacleSubType::ST_PEDESTRIAN)
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_PEDESTRIAN);
    }
    else
    {
      cls.set_label(motion_manager::common::DetectedObjectLabel::DO_UNKNOWN);
    }

    cls.set_probability(obstacle.confidence());
    cls.set_probability(0.7);
    object.add_classification(cls);

    motion_manager::interface::DetectedObjectKinematics metric;

    // 世界坐标系
    motion_manager::interface::PoseWithCovariance pose_with_covariance;
    motion_manager::interface::GeometryPose pose;
    pose.mutable_position()->set_x(obstacle.center_pos_abs().x());
    pose.mutable_position()->set_y(obstacle.center_pos_abs().y());
    pose.mutable_position()->set_z(obstacle.center_pos_abs().z());
    motion_manager::interface::Quaternion quat;
    tf2::Quaternion quat_tmp;
    quat_tmp = tf2::createQuaternionFromRPY(0, 0, obstacle.theta_vehicle() + local_view_.location_.heading());
    quat.set_qx(quat_tmp.getX());
    quat.set_qy(quat_tmp.getY());
    quat.set_qz(quat_tmp.getZ());
    quat.set_qw(quat_tmp.getW());
    pose.set_orientation(quat);
    pose_with_covariance.set_pose(pose);

    motion_manager::interface::PoseWithCovariance pose_with_covariance_vehicle;

    // 车身坐标系
    motion_manager::interface::GeometryPose pose_vehicle;
    pose_vehicle.mutable_position()->set_x(obstacle.center_pos_vehicle().x());
    pose_vehicle.mutable_position()->set_y(obstacle.center_pos_vehicle().y());
    pose_vehicle.mutable_position()->set_z(obstacle.center_pos_vehicle().z());

    motion_manager::interface::Quaternion quat_vehicle;
    tf2::Quaternion quat_vehicle_tmp;
    quat_vehicle_tmp = tf2::createQuaternionFromRPY(0, 0, obstacle.theta_vehicle());
    quat_vehicle.set_qx(quat_vehicle_tmp.getX());
    quat_vehicle.set_qy(quat_vehicle_tmp.getY());
    quat_vehicle.set_qz(quat_vehicle_tmp.getZ());
    quat_vehicle.set_qw(quat_vehicle_tmp.getW());
    pose_vehicle.set_orientation(quat_vehicle);

    pose_with_covariance_vehicle.set_pose(pose_vehicle);

    metric.set_pose_with_covariance(pose_with_covariance);
    metric.set_pose_with_covariance_vehicle(pose_with_covariance_vehicle);

    object.set_kinematics(metric);

    objects.add_objects(object);
  }
}

void MotionManager::TransObstacle(legion::interface::ObstacleList &obl_list, motion_manager::interface::TrackedObjects &objects)
{
  std::vector<legion::interface::Obstacle> output_obstacles;
  for (auto track : objects.objects())
  {
    legion::interface::Obstacle ob;

    ob.center_pos_abs().set_x(track.kinematics().pose_with_covariance().pose().position().x());
    ob.center_pos_abs().set_y(track.kinematics().pose_with_covariance().pose().position().y());
    ob.center_pos_abs().set_z(track.kinematics().pose_with_covariance().pose().position().z());
    ob.center_pos_vehicle().set_x(track.kinematics().pose_with_covariance_vehicle().pose().position().x());
    ob.center_pos_vehicle().set_y(track.kinematics().pose_with_covariance_vehicle().pose().position().y());
    ob.center_pos_vehicle().set_z(track.kinematics().pose_with_covariance_vehicle().pose().position().z());

    motion_manager::interface::ObjectClassification cls = track.classification()[0];
    Settype(ob, cls);

    ob.set_lane_position((int)(track.classification()[1].probability() * 1000) - 3);

    std::string object_id = utils::toHexString(track.object_id());
    int decimalNumber = std::stoi(object_id, nullptr, 16);
    ob.set_id(decimalNumber);

    tf2::Quaternion quat;
    tf2_geometry_msgs::Quaternion quaternion;
    quaternion.x = track.kinematics().pose_with_covariance().pose().orientation().qx();
    quaternion.y = track.kinematics().pose_with_covariance().pose().orientation().qy();
    quaternion.z = track.kinematics().pose_with_covariance().pose().orientation().qz();
    quaternion.w = track.kinematics().pose_with_covariance().pose().orientation().qw();
    tf2::quaternionMsgToTF(quaternion, quat);
    double roll, pitch, heading; // 定义存储roll,pitch and yaw的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, heading);

    tf2::Quaternion quat_vehicle;
    tf2_geometry_msgs::Quaternion quaternion_vehicle;
    quaternion_vehicle.x = track.kinematics().pose_with_covariance_vehicle().pose().orientation().qx();
    quaternion_vehicle.y = track.kinematics().pose_with_covariance_vehicle().pose().orientation().qy();
    quaternion_vehicle.z = track.kinematics().pose_with_covariance_vehicle().pose().orientation().qz();
    quaternion_vehicle.w = track.kinematics().pose_with_covariance_vehicle().pose().orientation().qw();
    tf2::quaternionMsgToTF(quaternion_vehicle, quat_vehicle);
    double roll_vehicle, pitch_vehicle, heading_vehicle; // 定义存储roll,pitch and yaw的容器
    tf2::Matrix3x3(quat_vehicle).getRPY(roll_vehicle, pitch_vehicle, heading_vehicle);

    ob.set_theta_vehicle(heading_vehicle);
    ob.set_theta_abs(heading);

    ob.velocity_abs().set_x(track.kinematics().twist_with_covariance().twist().linear().x());
    ob.velocity_abs().set_y(track.kinematics().twist_with_covariance().twist().linear().y());
    ob.velocity_abs().set_z(track.kinematics().twist_with_covariance().twist().linear().z());

    // // 速度小的侧方来车速度设为0
    // double vel = pow(pow(ob.velocity_abs().x(), 2) + pow(ob.velocity_abs().y(), 2), 0.5);
    // if ((fabs(heading - location_current.heading()) > 0.7)&&(fabs(heading - location_current.heading()) <3.14-0.7))
    // {
    //   if (vel < 3)
    //   {
    //     ob.velocity_abs().set_x(0);
    //     ob.velocity_abs().set_y(0);
    //   }
    // }

    ob.set_length(track.shape().dimensions().x());
    ob.set_width(track.shape().dimensions().y());
    ob.set_height(track.shape().dimensions().z());

    ob.set_existence_prob(track.existence_probability());

    set_polygon_abs(ob.length(), ob.width(), ob.height(), ob.center_pos_abs().x(), ob.center_pos_abs().y(), ob.center_pos_abs().z(), ob.theta_abs(), ob);

    output_obstacles.push_back(ob);
  }
  obl_list.set_obstacle(output_obstacles);
}

void MotionManager::SyncStamp(legion::interface::ObstacleList &obl_list, legion::interface::Location &location)
{
  if (location_list_.size() > 100)
  {
    location_list_.erase(location_list_.begin());
  }

  double ob_time = (obl_list.header().stamp().sec() + obl_list.header().stamp().nsec() * 1e-9) * 1e+3;
  int has_sync = 0;
  double del_t;
  double del_time;
  int min_id;
  double min_del_t = 9999;

  // 遍历location队列，获取id
  for (size_t id = 0; id < location_list_.size(); id++)
  {
    auto loca = location_list_[id];
    double loca_time = (loca.header().stamp().sec() + loca.header().stamp().nsec() * 1e-9) * 1e+3;
    del_t = abs(ob_time - loca_time);
    if (del_t < min_del_t)
    {
      min_del_t = del_t;
      min_id = id;
    }
  }

  // 如果匹配成功，则更新location_current
  if (min_del_t < 100)
  {
    location_current = location_list_[min_id];
    has_sync = 1;
    del_time = min_del_t;
  }

  // 如果未匹配成功，使用当前location
  if (has_sync == 0)
  {
    AINFO << "=================警告！ 警告！ 时间戳未匹配成功=================" << endl;
    location_current = location;
    del_time = 9999;
  }

  location_list_.push_back(location);
}

void MotionManager::VehicleToWorld(legion::interface::ObstacleList &data, legion::interface::Location &location)
{
  legion::interface::Header header;
  legion::interface::Time stamp;
  stamp.set_sec(location.header().stamp().sec());
  stamp.set_nsec(location.header().stamp().nsec());
  header.set_stamp(stamp);
  header.set_frame_id(data.header().frame_id());
  header.set_seq(data.header().seq());
  data.set_header(header);
  for (auto ob : data.obstacle())
  {
    legion::interface::Time obstacle_timestamp;
    obstacle_timestamp.set_sec(location.header().stamp().sec());
    obstacle_timestamp.set_nsec(location.header().stamp().nsec());
    ob.set_timestamp(obstacle_timestamp);
  }

  std::vector<legion::interface::Obstacle> data_fused_object_array;
  data.obstacle(data_fused_object_array);
  size_t obj_size = data_fused_object_array.size();

  for (size_t obj_index = 0; obj_index < obj_size; obj_index++) // obstacle copy
  {
    legion::interface::Point3D to_pos = data_fused_object_array[obj_index].center_pos_vehicle();
    ConvertPoint(to_pos, location);
    data_fused_object_array[obj_index].set_center_pos_abs(to_pos);

    double theta_vehicle = data_fused_object_array[obj_index].theta_vehicle();
    double theta_abs = theta_vehicle + location.heading();
    if (theta_abs > M_PI)
      theta_abs -= M_PI * 2.0;
    else if (theta_abs < -M_PI)
      theta_abs += M_PI * 2.0;

    data_fused_object_array[obj_index].set_theta_abs(theta_abs);

    legion::interface::Point3D acc;
    acc = data_fused_object_array[obj_index].acceleration_vehicle();
    legion::preprocessor::coordinate::convert_acc(&acc, &location);
    data_fused_object_array[obj_index].set_acceleration_abs(acc);

    std::vector<legion::interface::Point3D> from_polygon;
    data_fused_object_array[obj_index].polygon_point_vehicle(from_polygon);
    size_t polygon_point_size = from_polygon.size();
    std::vector<legion::interface::Point3D> to_polygon;
    to_polygon.resize(polygon_point_size);
    for (size_t polygon_point_index = 0; polygon_point_index < polygon_point_size; polygon_point_index++) // polygon_point copy
    {
      legion::interface::Point3D pt = from_polygon[polygon_point_index];
      ConvertPoint(pt, location);
      to_polygon[polygon_point_index] = pt;
    }
    data_fused_object_array[obj_index].clear_polygon_point_abs();
    data_fused_object_array[obj_index].set_polygon_point_abs(&to_polygon);
    legion::interface::Point3D anchor_point = data_fused_object_array[obj_index].anchor_point_vehicle();
    ConvertPoint(anchor_point, location);
    data_fused_object_array[obj_index].set_anchor_point_abs(anchor_point);
  }
  data.clear_obstacle();
  data.set_obstacle(data_fused_object_array);
}

void MotionManager::ConvertPoint(legion::interface::Point3D &point, legion::interface::Location &location)
{
  std::vector<double> values = legion::preprocessor::coordinate::convert_point(location.utm_position().x(),
                                                                                location.utm_position().y(),
                                                                                location.utm_position().z(),
                                                                                location.roll(),
                                                                                location.pitch(),
                                                                                location.heading(),
                                                                                point.x(),
                                                                                point.y(),
                                                                                point.z());

  point.set_x(values[0]);
  point.set_y(values[1]);
  point.set_z(values[2]);
}

void MotionManager::GetPolygon(legion::interface::ObstacleList &result_obstacle_list)
{
  for (auto &ob : result_obstacle_list.obstacle())
  {
    legion::interface::Point3D center_pos = ob.center_pos_vehicle();
    double theta = ob.theta_vehicle();
    double wid = ob.width();
    double len = ob.length();
    double hei = ob.height();
    std::vector<legion::interface::Point3D> poly_list;
    legion::interface::Point3D point1, point2, point3, point4;
    point1.set_x(center_pos.x() + len / 2);
    point1.set_y(center_pos.y() - wid / 2);
    point1.set_z(center_pos.z() - hei / 2);
    point2.set_x(center_pos.x() + len / 2);
    point2.set_y(center_pos.y() + wid / 2);
    point2.set_z(center_pos.z() - hei / 2);
    point3.set_x(center_pos.x() - len / 2);
    point3.set_y(center_pos.y() + wid / 2);
    point3.set_z(center_pos.z() - hei / 2);
    point4.set_x(center_pos.x() - len / 2);
    point4.set_y(center_pos.y() - wid / 2);
    point4.set_z(center_pos.z() - hei / 2);
    poly_list.push_back(point1);
    poly_list.push_back(point2);
    poly_list.push_back(point3);
    poly_list.push_back(point4);
    ob.set_polygon_point_vehicle(poly_list);

    std::vector<legion::interface::Point3D> new_poly_list;
    for (auto poly : ob.polygon_point_vehicle())
    {
      double xx = (poly.x() - center_pos.x()) * cos(theta) - (poly.y() - center_pos.y()) * sin(theta) + center_pos.x();
      double yy = (poly.x() - center_pos.x()) * sin(theta) + (poly.y() - center_pos.y()) * cos(theta) + center_pos.y();
      double zz = poly.z() - ob.height() / 2;
      poly.set_x(xx);
      poly.set_y(yy);
      new_poly_list.push_back(poly);
    }
    ob.set_polygon_point_vehicle(new_poly_list);
  }
}

void MotionManager::set_polygon_abs(double len, double wid, double hei, double x, double y, double z, double theta, legion::interface::Obstacle &ob_)
{
  std::vector<legion::interface::Point3D> poly_list;
  legion::interface::Point3D point1, point2, point3, point4;
  point1.set_x(x + len / 2);
  point1.set_y(y - wid / 2);
  point1.set_z(z - hei / 2);
  point2.set_x(x + len / 2);
  point2.set_y(y + wid / 2);
  point2.set_z(z - hei / 2);
  point3.set_x(x - len / 2);
  point3.set_y(y + wid / 2);
  point3.set_z(z - hei / 2);
  point4.set_x(x - len / 2);
  point4.set_y(y - wid / 2);
  point4.set_z(z - hei / 2);
  poly_list.push_back(point1);
  poly_list.push_back(point2);
  poly_list.push_back(point3);
  poly_list.push_back(point4);
  ob_.set_polygon_point_abs(poly_list);

  std::vector<legion::interface::Point3D> new_poly_list;
  for (auto poly : ob_.polygon_point_abs())
  {
    double xx = (poly.x() - x) * cos(theta) - (poly.y() - y) * sin(theta) + x;
    double yy = (poly.x() - x) * sin(theta) + (poly.y() - y) * cos(theta) + y;
    double zz = poly.z() - ob_.height() / 2;
    poly.set_x(xx);
    poly.set_y(yy);
    new_poly_list.push_back(poly);
  }

  ob_.set_polygon_point_abs(new_poly_list);
}

void MotionManager::FaultMonitorInit() {}

void MotionManager::MessagesActivate()
{
  if (motion_manager_conf_ == nullptr)
  {
    return;
  }
  for (auto message_manager : message_manager_)
  {
    message_manager.second->Activate();
  }
  return;
}

void MotionManager::MessagesDeActivate()
{
  if (motion_manager_conf_ == nullptr)
  {
    return;
  }
  for (auto message_manager : message_manager_)
  {
    message_manager.second->DeActivate();
  }
  return;
}

void MotionManager::TaskActivate()
{
  if (is_init_ == false)
  {
    return;
  }
  // IPC激活
  MessagesActivate();
  if (function_activation_)
  {
    return;
  }
  task_compute_->AddTimer(produce_motion_manager_command_duration_,
                          &MotionManager::ComputeMotionManagerCommandOnTimer, this);
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout << "===================function activate=================="
            << "\n";
  function_activation_ = true;
  return;
}

void MotionManager::TaskStop()
{
  if (is_init_ == false)
  {
    return;
  }
  // IPC去激活
  MessagesDeActivate();
  if (function_activation_ == false)
  {
    return;
  }
  {
    // 清除所有内部计算的中间结果，保证回到刚init完的状态
  }
  std::cout << "******************function stop***************" << "\n";
  function_activation_ = false;
  return;
}

void MotionManager::StatusDetectOnTimer() {}

void MotionManager::Spin()
{
  while (1)
  {
    ad_timer_manager_->DetectTimers(NULL);
    usleep(1000);
  }
}

void MotionManager::GetObstacleHeader(legion::interface::ObstacleList &obstacle_list)
{
  legion::interface::Time time_ = obstacle_list.header().stamp();
  for (auto &ob : obstacle_list.obstacle())
  {
    ob.set_timestamp(time_);
  }
}

void MotionManager::Getacc(legion::interface::ObstacleList &obstacle_list_output)
{
  if (is_first_frame_)
  {
    return;
  }
  for (auto &ob : obstacle_list_output.obstacle())
  {
    auto it = frames_obs_for_acc_.find(ob.id());
    if (it == frames_obs_for_acc_.end()) // 若该id未记录，记录该id
    {
      vector<legion::interface::Obstacle> temp_ob_l;
      temp_ob_l.push_back(ob);
      frames_obs_for_acc_.insert(pair<int, vector<legion::interface::Obstacle>>(ob.id(), temp_ob_l));
    }
    else // 若该id已记录，判断跟踪是否满5帧
    {
      if (it->second.size() < 4)
      {
        it->second.push_back(ob); // 若未满5帧，继续记录
      }
      else
      { // 若已满5帧，计算加速度
        it->second.push_back(ob);
        double time_curr = obstacle_list_output.header().stamp().sec() + obstacle_list_output.header().stamp().nsec() * 1e-9;
        double time_begin = it->second[0].timestamp().sec() + it->second[0].timestamp().nsec() * 1e-9;
        double delta_time = time_curr - time_begin;
        if (delta_time < duplicate_frame_threshold)
        {
          it->second.erase(it->second.begin());
          return;
        }
        double this_vx = ob.velocity_abs().x();
        double this_vy = ob.velocity_abs().y();
        double this_v = sqrt(pow(this_vx, 2) + pow(this_vy, 2));
        double last_vx = it->second[0].velocity_abs().x();
        double last_vy = it->second[0].velocity_abs().y();
        double last_v = sqrt(pow(last_vx, 2) + pow(last_vy, 2));

        double cal_acc = (this_v - last_v) / delta_time;
        legion::interface::Point3D acc;
        acc.set_x(cal_acc);
        acc.set_y(0);
        acc.set_z(0);
        ob.set_acceleration_abs(acc);

        it->second.erase(it->second.begin());
      }
    }
  }
}

double MotionManager::CalculatePolygonIoU(const legion::interface::Obstacle &ob1, const legion::interface::Obstacle &ob2)
{
  // 获取两个障碍物的多边形点
  std::vector<legion::interface::Point3D> polygon1, polygon2;
  ob1.polygon_point_abs(polygon1);
  ob2.polygon_point_abs(polygon2);
  
  if (polygon1.size() < 3 || polygon2.size() < 3)
  {
    return 0.0;
  }
  
  // 判断哪个是矩形（4个点），哪个是多边形（很多点）
  bool is_rect1 = (polygon1.size() == 4);
  bool is_rect2 = (polygon2.size() == 4);
  const std::vector<legion::interface::Point3D> *rect_poly = nullptr;
  const std::vector<legion::interface::Point3D> *poly_poly = nullptr;
  
  if (is_rect1 && !is_rect2)
  {
    rect_poly = &polygon1;
    poly_poly = &polygon2;
  }
  else if (!is_rect1 && is_rect2)
  {
    rect_poly = &polygon2;
    poly_poly = &polygon1;
  }
  // 如果都是矩形或都不是矩形，使用通用方法
  else
  {
    rect_poly = nullptr;
    poly_poly = nullptr;
  }
  
  // 计算边界框（用于快速筛选）
  auto getBoundingBox = [](const std::vector<legion::interface::Point3D> &points) -> std::pair<std::pair<double, double>, std::pair<double, double>> {
    double min_x = points[0].x(), max_x = points[0].x();
    double min_y = points[0].y(), max_y = points[0].y();
    for (const auto &p : points)
    {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    return {{min_x, max_x}, {min_y, max_y}};
  };
  
  auto bbox1 = getBoundingBox(polygon1);
  auto bbox2 = getBoundingBox(polygon2);
  
  // 检查边界框是否重叠
  double overlap_x = std::max(0.0, std::min(bbox1.first.second, bbox2.first.second) - std::max(bbox1.first.first, bbox2.first.first));
  double overlap_y = std::max(0.0, std::min(bbox1.second.second, bbox2.second.second) - std::max(bbox1.second.first, bbox2.second.first));
  double bbox_intersection = overlap_x * overlap_y;
  
  // 如果边界框不重叠，IOU 为 0
  if (bbox_intersection < 1e-6)
  {
    return 0.0;
  }
  
  // 计算面积
  double area1, area2;
  
  if (is_rect1)
  {
    // 矩形面积：使用边界框面积（对于矩形，边界框就是实际面积）
    area1 = (bbox1.first.second - bbox1.first.first) * (bbox1.second.second - bbox1.second.first);
  }
  else
  {
    // 多边形面积：使用鞋带公式
    area1 = 0.0;
    for (size_t i = 0; i < polygon1.size(); ++i)
    {
      size_t j = (i + 1) % polygon1.size();
      area1 += polygon1[i].x() * polygon1[j].y();
      area1 -= polygon1[j].x() * polygon1[i].y();
    }
    area1 = std::abs(area1) / 2.0;
  }
  
  if (is_rect2)
  {
    area2 = (bbox2.first.second - bbox2.first.first) * (bbox2.second.second - bbox2.second.first);
  }
  else
  {
    area2 = 0.0;
    for (size_t i = 0; i < polygon2.size(); ++i)
    {
      size_t j = (i + 1) % polygon2.size();
      area2 += polygon2[i].x() * polygon2[j].y();
      area2 -= polygon2[j].x() * polygon2[i].y();
    }
    area2 = std::abs(area2) / 2.0;
  }
  
  if (area1 < 1e-6 || area2 < 1e-6)
  {
    return 0.0;
  }
  
  // 计算交集面积
  double intersection_area = 0.0;
  
  if (rect_poly != nullptr && poly_poly != nullptr)
  {
    // 矩形与多边形的相交：优化算法
    // 获取矩形的边界框
    auto rect_bbox = getBoundingBox(*rect_poly);
    double rect_min_x = rect_bbox.first.first;
    double rect_max_x = rect_bbox.first.second;
    double rect_min_y = rect_bbox.second.first;
    double rect_max_y = rect_bbox.second.second;
    double rect_area = (rect_max_x - rect_min_x) * (rect_max_y - rect_min_y);
    
    // 对于大多边形，先简化（采样）以提高计算效率
    const size_t MAX_POLY_POINTS = 500;  // 最多使用500个点
    std::vector<legion::interface::Point3D> simplified_poly;
    if (poly_poly->size() > MAX_POLY_POINTS)
    {
      size_t step = poly_poly->size() / MAX_POLY_POINTS;
      for (size_t i = 0; i < poly_poly->size(); i += step)
      {
        simplified_poly.push_back((*poly_poly)[i]);
      }
      // 确保闭合
      if (simplified_poly.back().x() != simplified_poly.front().x() ||
          simplified_poly.back().y() != simplified_poly.front().y())
      {
        simplified_poly.push_back(simplified_poly.front());
      }
    }
    else
    {
      simplified_poly = *poly_poly;
    }
    
    // 判断矩形四个角点是否在多边形内
    auto isPointInPolygon = [](double px, double py, const std::vector<legion::interface::Point3D> &poly) -> bool {
      int crossings = 0;
      for (size_t i = 0; i < poly.size() - 1; ++i)
      {
        const auto &p1 = poly[i];
        const auto &p2 = poly[i + 1];
        
        if (((p1.y() > py) != (p2.y() > py)) &&
            (px < (p2.x() - p1.x()) * (py - p1.y()) / (p2.y() - p1.y()) + p1.x()))
        {
          crossings++;
        }
      }
      return (crossings % 2 == 1);
    };
    
    // 检查矩形四个角点
    bool corner1_in = isPointInPolygon(rect_min_x, rect_min_y, simplified_poly);
    bool corner2_in = isPointInPolygon(rect_max_x, rect_min_y, simplified_poly);
    bool corner3_in = isPointInPolygon(rect_max_x, rect_max_y, simplified_poly);
    bool corner4_in = isPointInPolygon(rect_min_x, rect_max_y, simplified_poly);
    int corners_in = (corner1_in ? 1 : 0) + (corner2_in ? 1 : 0) + 
                     (corner3_in ? 1 : 0) + (corner4_in ? 1 : 0);
    
    // 检查多边形点有多少在矩形内
    size_t poly_points_in_rect = 0;
    for (const auto &p : simplified_poly)
    {
      if (p.x() >= rect_min_x && p.x() <= rect_max_x &&
          p.y() >= rect_min_y && p.y() <= rect_max_y)
      {
        poly_points_in_rect++;
      }
    }
    double poly_in_rect_ratio = simplified_poly.size() > 0 ? 
        static_cast<double>(poly_points_in_rect) / simplified_poly.size() : 0.0;
    
    // 估算交集面积
    if (corners_in == 4)
    {
      // 矩形完全在多边形内
      intersection_area = rect_area;
    }
    else if (corners_in == 0 && poly_in_rect_ratio < 0.1)
    {
      // 矩形和多边形基本不重叠
      intersection_area = bbox_intersection * 0.3; // 使用较小的系数
    }
    else
    {
      // 部分重叠：使用边界框重叠和面积比率的加权平均
      double rect_in_poly_ratio = corners_in / 4.0;
      double estimated_intersection_from_rect = rect_area * rect_in_poly_ratio;
      double estimated_intersection_from_poly = area2 * poly_in_rect_ratio;
      intersection_area = (estimated_intersection_from_rect + estimated_intersection_from_poly) / 2.0;
      // 确保不超过边界框重叠
      intersection_area = std::min(intersection_area, bbox_intersection);
    }
  }
  else
  {
    // 两个都是矩形或都不是矩形，使用边界框重叠作为近似
    double bbox1_area = (bbox1.first.second - bbox1.first.first) * (bbox1.second.second - bbox1.second.first);
    double bbox2_area = (bbox2.first.second - bbox2.first.first) * (bbox2.second.second - bbox2.second.first);
    double bbox_union = bbox1_area + bbox2_area - bbox_intersection;
    
    if (bbox_union < 1e-6)
    {
      return 0.0;
    }
    
    double bbox_iou = bbox_intersection / bbox_union;
    double area_ratio1 = area1 / std::max(bbox1_area, 1e-6);
    double area_ratio2 = area2 / std::max(bbox2_area, 1e-6);
    double avg_area_ratio = (area_ratio1 + area_ratio2) / 2.0;
    
    intersection_area = bbox_intersection * avg_area_ratio * bbox_iou;
  }
  
  // 计算并集和 IOU
  double union_area = area1 + area2 - intersection_area;
  
  if (union_area < 1e-6)
  {
    return 0.0;
  }
  
  double iou = intersection_area / union_area;
  return std::min(1.0, std::max(0.0, iou));
}

double MotionManager::CalculateObstacleArea(const legion::interface::Obstacle &ob)
{
  std::vector<legion::interface::Point3D> polygon;
  ob.polygon_point_abs(polygon);
  
  if (polygon.size() < 3)
  {
    return 0.0;
  }
  
  // 判断是否为矩形（4个点）
  bool is_rect = (polygon.size() == 4);
  
  // 计算边界框
  double min_x = polygon[0].x(), max_x = polygon[0].x();
  double min_y = polygon[0].y(), max_y = polygon[0].y();
  for (const auto &p : polygon)
  {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  }
  
  double area = 0.0;
  if (is_rect)
  {
    // 矩形面积：使用边界框面积
    area = (max_x - min_x) * (max_y - min_y);
  }
  else
  {
    // 多边形面积：使用鞋带公式
    for (size_t i = 0; i < polygon.size(); ++i)
    {
      size_t j = (i + 1) % polygon.size();
      area += polygon[i].x() * polygon[j].y();
      area -= polygon[j].x() * polygon[i].y();
    }
    area = std::abs(area) / 2.0;
  }
  
  return area;
}

// 从多边形计算最小矩形框（轴对齐）
std::tuple<double, double, double, double> MotionManager::GetBoundingBox(
    const std::vector<legion::interface::Point3D> &polygon)
{
  if (polygon.empty())
  {
    return std::make_tuple(0.0, 0.0, 0.0, 0.0);
  }
  
  double min_x = polygon[0].x(), max_x = polygon[0].x();
  double min_y = polygon[0].y(), max_y = polygon[0].y();
  
  for (const auto &p : polygon)
  {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  }
  
  return std::make_tuple(min_x, min_y, max_x, max_y);
}

// 计算两个障碍物基于最小矩形框的IOU
double MotionManager::CalculateBoundingBoxIoU(const legion::interface::Obstacle &ob1, 
                                              const legion::interface::Obstacle &ob2)
{
  std::vector<legion::interface::Point3D> polygon1, polygon2;
  ob1.polygon_point_abs(polygon1);
  ob2.polygon_point_abs(polygon2);
  
  if (polygon1.size() < 3 || polygon2.size() < 3)
  {
    return 0.0;
  }
  
  // 计算两个障碍物的最小矩形框
  auto bbox1 = GetBoundingBox(polygon1);
  auto bbox2 = GetBoundingBox(polygon2);
  
  double min_x1 = std::get<0>(bbox1), min_y1 = std::get<1>(bbox1);
  double max_x1 = std::get<2>(bbox1), max_y1 = std::get<3>(bbox1);
  double min_x2 = std::get<0>(bbox2), min_y2 = std::get<1>(bbox2);
  double max_x2 = std::get<2>(bbox2), max_y2 = std::get<3>(bbox2);
  
  // 计算交集
  double inter_min_x = std::max(min_x1, min_x2);
  double inter_min_y = std::max(min_y1, min_y2);
  double inter_max_x = std::min(max_x1, max_x2);
  double inter_max_y = std::min(max_y1, max_y2);
  
  double inter_area = 0.0;
  if (inter_min_x < inter_max_x && inter_min_y < inter_max_y)
  {
    inter_area = (inter_max_x - inter_min_x) * (inter_max_y - inter_min_y);
  }
  
  // 计算并集
  double area1 = (max_x1 - min_x1) * (max_y1 - min_y1);
  double area2 = (max_x2 - min_x2) * (max_y2 - min_y2);
  double union_area = area1 + area2 - inter_area;
  
  if (union_area < 1e-6)
  {
    return 0.0;
  }
  
  return inter_area / union_area;
}

/**
 * @brief 从历史帧中匹配障碍物ID
 * @param current_ob 当前障碍物
 * @return 匹配到的历史Lidar ID，如果未找到则返回-1
 * 
 * 该函数在历史帧窗口（最多HISTORY_WINDOW_SIZE帧）中搜索匹配的障碍物ID。
 * 匹配策略（按优先级）：
 * 1. IOU大于阈值：使用矩形框IOU进行匹配
 * 2. 位置很近（< close_match_distance）：即使IOU为0也认为匹配成功

 * 
 * 只匹配Lidar障碍物（ID < lcd_id_offset），优先使用Lidar ID作为稳定ID
 */
int MotionManager::MatchHistoricalObstacle(const legion::interface::Obstacle &current_ob)
{
  int best_match_id = -1;
  double max_iou = 0.0;
  double best_distance = 1e10;
  
  // 遍历历史帧（从最近的帧开始，最多HISTORY_WINDOW_SIZE帧）
  for (size_t frame_idx = 0; frame_idx < obstacle_history_.size(); ++frame_idx)
  {
    const auto &frame_obstacles = obstacle_history_[frame_idx];
    
    // 遍历该帧中的所有障碍物
    for (const auto &pair : frame_obstacles)
    {
      const HistoricalObstacle &hist_ob = pair.second;
      
      // 只匹配Lidar障碍物（ID < lcd_id_offset）
      if (hist_ob.id >= lcd_id_offset)
      {
        continue;
      }
      
      // 检查位置是否相近（快速筛选）
      double dist = std::sqrt(
        std::pow(current_ob.center_pos_abs().x() - hist_ob.center_pos.x(), 2) +
        std::pow(current_ob.center_pos_abs().y() - hist_ob.center_pos.y(), 2)
      );
      
      // 如果距离太远（超过配置的最大距离），跳过
      if (dist > history_match_max_distance)
      {
        continue;
      }
      
      // 构建历史障碍物对象用于IOU计算
      legion::interface::Obstacle hist_ob_for_iou;
      hist_ob_for_iou.set_center_pos_abs(hist_ob.center_pos);
      hist_ob_for_iou.set_polygon_point_abs(hist_ob.polygon_points);
      
      // 计算基于矩形框的IOU
      double iou = CalculateBoundingBoxIoU(current_ob, hist_ob_for_iou);
      
      // 对于形状变化的障碍物，使用更宽松的匹配策略
      // 策略1：IOU大于阈值
      // 策略2：位置很近（< close_match_distance），可以没有IOU（处理形状变化导致的IOU降低）

      bool is_close_match = (dist < close_match_distance);

      
      // 匹配条件：IOU大于阈值，或者位置很近，或者位置非常近
      bool match_condition = (iou > iou_threshold || is_close_match );
      
      // 选择最佳匹配：优先IOU高的，IOU相同时选择距离近的
      // 如果当前匹配的IOU更高，或者IOU相同但距离更近，或者当前IOU为0但距离更近且之前的IOU也小于阈值
      if (match_condition && 
          (iou > max_iou || 
           (iou == max_iou && dist < best_distance) || 
           (max_iou < iou_threshold && dist < best_distance)))
      {
        max_iou = iou;
        best_distance = dist;
        best_match_id = hist_ob.id;
      }
    }
  }
  
  return best_match_id;
}

// 更新历史障碍物记录
void MotionManager::UpdateObstacleHistory(const legion::interface::ObstacleList &current_fusion_list)
{
  // 创建当前帧的障碍物映射
  std::map<int, HistoricalObstacle> current_frame_obstacles;
  
  for (const auto &ob : current_fusion_list.obstacle())
  {
    HistoricalObstacle hist_ob;
    hist_ob.id = ob.id();
    hist_ob.center_pos = ob.center_pos_abs();
    ob.polygon_point_abs(hist_ob.polygon_points);
    hist_ob.frame_age = 0;
    
    current_frame_obstacles[ob.id()] = hist_ob;
  }
  
  // 将当前帧添加到历史窗口
  obstacle_history_.push_front(current_frame_obstacles);
  
  // 保持历史窗口大小不超过10帧
  while (obstacle_history_.size() > HISTORY_WINDOW_SIZE)
  {
    obstacle_history_.pop_back();
  }
  
  // 更新所有历史障碍物的帧年龄
  for (auto &frame_obstacles : obstacle_history_)
  {
    for (auto &pair : frame_obstacles)
    {
      pair.second.frame_age++;
    }
  }
}

// 辅助函数：根据障碍物位置生成网格化键值
MotionManager::PositionKey MotionManager::GetPositionKey(const legion::interface::Obstacle &ob)
{
  PositionKey key;
  // 使用配置的网格精度
  key.grid_x = static_cast<int>(std::floor(ob.center_pos_abs().x() / grid_size));
  key.grid_y = static_cast<int>(std::floor(ob.center_pos_abs().y() / grid_size));
  return key;
}

void MotionManager::StabilizeFusionIds(legion::interface::ObstacleList &current_fusion_list, 
                                      const legion::interface::ObstacleList &last_fusion_list)
{
  // 如果是第一帧，保持原ID并记录Lidar ID到位置映射
  if (is_first_frame_ || last_fusion_list.obstacle_size() == 0)
  {
    for (auto &ob : current_fusion_list.obstacle())
    {
      int original_id = ob.id();
      // 第一帧保持原ID，不重新分配
      fusion_id_map_[original_id] = original_id;
      
      // 如果是Lidar障碍物（ID < lcd_id_offset），记录到位置映射表
      if (original_id < lcd_id_offset)
      {
        PositionKey key = GetPositionKey(ob);
        lidar_id_history_[key] = original_id;
        id_to_position_[original_id] = key;
      }
    }
    return;
  }
  
  // 使用IOU匹配当前帧和上一帧的障碍物
  std::vector<bool> current_matched(current_fusion_list.obstacle_size(), false);
  std::vector<bool> last_matched(last_fusion_list.obstacle_size(), false);
  
  // 维护已使用的ID集合，确保每个ID只分配给一个障碍物
  std::set<int> used_ids;
  
  // 第一轮：IOU匹配
  for (size_t i = 0; i < current_fusion_list.obstacle_size(); ++i)
  {
    auto &current_ob = current_fusion_list.obstacle(i);
    double max_iou = 0.0;
    int best_match_idx = -1;
    
    // 在上一帧中寻找最佳匹配
    for (size_t j = 0; j < last_fusion_list.obstacle_size(); ++j)
    {
      if (last_matched[j]) continue;
      
      const auto &last_ob = last_fusion_list.obstacle(j);
      
      // 使用polygon_points_abs进行IOU匹配（对形状差异更鲁棒）
      // 同时计算多边形IOU和矩形框IOU，选择更好的匹配
      double polygon_iou = CalculatePolygonIoU(current_ob, last_ob);
      double bbox_iou = CalculateBoundingBoxIoU(current_ob, last_ob);
      
      // 使用两者中的较大值作为匹配IOU
      double iou = std::max(polygon_iou, bbox_iou);
      
      // 检查位置距离（用于形状差异时的匹配）
      double dist = std::sqrt(
        std::pow(current_ob.center_pos_abs().x() - last_ob.center_pos_abs().x(), 2) +
        std::pow(current_ob.center_pos_abs().y() - last_ob.center_pos_abs().y(), 2)
      );
      
      // 如果位置很近且矩形框IOU足够，也考虑匹配（处理形状差异）
      bool is_close_and_bbox_match = (dist < close_match_distance && bbox_iou > iou_threshold);
      
      if (iou > max_iou || (is_close_and_bbox_match && bbox_iou > max_iou))
      {
        max_iou = iou;
        best_match_idx = j;
      }
    }
    
    // 如果IOU大于阈值，或者位置很近且矩形框IOU足够，认为匹配成功，优先使用Lidar的ID作为稳定ID
    bool match_success = (max_iou > iou_threshold && best_match_idx >= 0);
    if (!match_success && best_match_idx >= 0)
    {
      // 如果多边形IOU不够，但位置很近且矩形框IOU足够，也认为匹配成功
      const auto &last_ob = last_fusion_list.obstacle(best_match_idx);
      double dist = std::sqrt(
        std::pow(current_ob.center_pos_abs().x() - last_ob.center_pos_abs().x(), 2) +
        std::pow(current_ob.center_pos_abs().y() - last_ob.center_pos_abs().y(), 2)
      );
      double bbox_iou = CalculateBoundingBoxIoU(current_ob, last_ob);
      if (dist < close_match_distance && bbox_iou > iou_threshold)
      {
        match_success = true;
      }
    }
    
    if (match_success && best_match_idx >= 0)
    {
      const auto &last_ob = last_fusion_list.obstacle(best_match_idx);
      int last_stable_id = last_ob.id();
      int current_id = current_ob.id();
      
      // 优先使用Lidar的ID（ID < lcd_id_offset）作为稳定ID
      int stable_id;
      if (current_id < lcd_id_offset && last_stable_id >= lcd_id_offset)
      {
        // 当前帧是Lidar，上一帧是LCD，使用当前帧的Lidar ID
        stable_id = current_id;
      }
      else if (current_id >= lcd_id_offset && last_stable_id < lcd_id_offset)
      {
        // 当前帧是LCD，上一帧是Lidar，使用上一帧的Lidar ID
        stable_id = last_stable_id;
      }
      else
      {
        // 都是Lidar或都是LCD，使用上一帧的ID保持连续性
        stable_id = last_stable_id;
      }
      
      // 检查该ID是否已经被使用
      if (used_ids.find(stable_id) != used_ids.end())
      {
        // ID已被使用，跳过此匹配，让已匹配的障碍物独占该ID
        continue;
      }
      
      // 更新当前障碍物的ID为稳定ID
      int original_id = current_ob.id();
      current_ob.set_id(stable_id);
      fusion_id_map_[original_id] = stable_id;
      used_ids.insert(stable_id);  // 标记ID已使用
      
      // 如果是Lidar ID，更新位置映射
      if (stable_id < lcd_id_offset)
      {
        PositionKey key = GetPositionKey(current_ob);
        lidar_id_history_[key] = stable_id;
        id_to_position_[stable_id] = key;
      }
      
      current_matched[i] = true;
      last_matched[best_match_idx] = true;
    }
  }
  
  // ========== 第二轮：基于历史帧匹配（优先处理LCD障碍物） ==========
  // 优先处理LCD障碍物（ID >= lcd_id_offset），因为它们可能是形状变化导致的匹配失败
  // 使用候选列表机制，收集所有匹配到同一ID的障碍物，然后选择最佳匹配
  std::map<int, std::vector<std::pair<size_t, double>>> id_candidates;  // ID -> [(障碍物索引, 匹配质量)]
  
  for (size_t i = 0; i < current_fusion_list.obstacle_size(); ++i)
  {
    if (current_matched[i]) continue;
    
    auto &current_ob = current_fusion_list.obstacle(i);
    int current_id = current_ob.id();
    
    // 优先处理LCD障碍物，尝试匹配历史Lidar ID
    if (current_id >= lcd_id_offset)
    {
      // 尝试从历史帧中匹配障碍物ID（优先匹配Lidar ID）
      int historical_id = MatchHistoricalObstacle(current_ob);
      
      if (historical_id >= 0 && historical_id < lcd_id_offset)
      {
        // 计算匹配质量（使用IOU或距离）
        double match_quality = 0.0;
        // 尝试从历史帧中找到匹配的障碍物，计算IOU
        for (size_t frame_idx = 0; frame_idx < obstacle_history_.size(); ++frame_idx)
        {
          const auto &frame_obstacles = obstacle_history_[frame_idx];
          auto it = frame_obstacles.find(historical_id);
          if (it != frame_obstacles.end())
          {
            const HistoricalObstacle &hist_ob = it->second;
            legion::interface::Obstacle hist_ob_for_iou;
            hist_ob_for_iou.set_center_pos_abs(hist_ob.center_pos);
            hist_ob_for_iou.set_polygon_point_abs(hist_ob.polygon_points);
            double iou = CalculateBoundingBoxIoU(current_ob, hist_ob_for_iou);
            double dist = std::sqrt(
              std::pow(current_ob.center_pos_abs().x() - hist_ob.center_pos.x(), 2) +
              std::pow(current_ob.center_pos_abs().y() - hist_ob.center_pos.y(), 2)
            );
            // 使用IOU作为匹配质量，如果IOU为0则使用距离的倒数
            match_quality = (iou > 0.0) ? iou : (1.0 / (dist + match_quality_distance_offset));
            break;
          }
        }
        id_candidates[historical_id].push_back({i, match_quality});
      }
    }
  }
  
  // 为每个ID选择最佳匹配的障碍物（IOU最高或距离最近）
  for (const auto &pair : id_candidates)
  {
    int candidate_id = pair.first;
    const auto &candidates = pair.second;
    
    // 如果ID已被使用，跳过
    if (used_ids.find(candidate_id) != used_ids.end())
    {
      continue;
    }
    
    // 选择匹配质量最高的障碍物
    size_t best_idx = 0;
    double best_quality = candidates[0].second;
    for (size_t j = 1; j < candidates.size(); ++j)
    {
      if (candidates[j].second > best_quality)
      {
        best_quality = candidates[j].second;
        best_idx = j;
      }
    }
    
    // 将ID分配给最佳匹配的障碍物
    size_t ob_idx = candidates[best_idx].first;
    auto &current_ob = current_fusion_list.obstacle(ob_idx);
    int original_id = current_ob.id();
    current_ob.set_id(candidate_id);
    fusion_id_map_[original_id] = candidate_id;
    used_ids.insert(candidate_id);  // 标记ID已使用
    current_matched[ob_idx] = true;
  }
  
  // 第三轮：对于仍未匹配的障碍物（包括Lidar和LCD），再次尝试历史匹配
  // 使用候选列表，收集所有匹配到同一ID的障碍物，然后选择最佳匹配
  std::map<int, std::vector<std::pair<size_t, double>>> id_candidates_round3;
  
  for (size_t i = 0; i < current_fusion_list.obstacle_size(); ++i)
  {
    if (current_matched[i]) continue;
    
    auto &current_ob = current_fusion_list.obstacle(i);
    int current_id = current_ob.id();
    
    // 尝试从历史帧中匹配障碍物ID（优先匹配Lidar ID）
    int historical_id = MatchHistoricalObstacle(current_ob);
    
    if (historical_id >= 0 && historical_id < 10000)
    {
      // 计算匹配质量
      double match_quality = 0.0;
      for (size_t frame_idx = 0; frame_idx < obstacle_history_.size(); ++frame_idx)
      {
        const auto &frame_obstacles = obstacle_history_[frame_idx];
        auto it = frame_obstacles.find(historical_id);
        if (it != frame_obstacles.end())
        {
          const HistoricalObstacle &hist_ob = it->second;
          legion::interface::Obstacle hist_ob_for_iou;
          hist_ob_for_iou.set_center_pos_abs(hist_ob.center_pos);
          hist_ob_for_iou.set_polygon_point_abs(hist_ob.polygon_points);
          double iou = CalculateBoundingBoxIoU(current_ob, hist_ob_for_iou);
          double dist = std::sqrt(
            std::pow(current_ob.center_pos_abs().x() - hist_ob.center_pos.x(), 2) +
            std::pow(current_ob.center_pos_abs().y() - hist_ob.center_pos.y(), 2)
          );
          match_quality = (iou > 0.0) ? iou : (1.0 / (dist + 0.1));
          break;
        }
      }
      id_candidates_round3[historical_id].push_back({i, match_quality});
    }
    
    // 如果当前是LCD障碍物（ID >= lcd_id_offset），尝试基于位置的简单匹配
    if (current_id >= lcd_id_offset)
    {
      PositionKey key = GetPositionKey(current_ob);
      
      // 检查当前位置附近是否有历史Lidar ID（检查相邻网格）
      int best_lidar_id = -1;
      double min_distance = 1e10;
      
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          PositionKey search_key;
          search_key.grid_x = key.grid_x + dx;
          search_key.grid_y = key.grid_y + dy;
          
          auto it = lidar_id_history_.find(search_key);
          if (it != lidar_id_history_.end())
          {
            // 计算实际距离（使用网格大小）
            double dist = std::sqrt(dx * dx + dy * dy) * grid_size;
            if (dist < min_distance && dist < close_match_distance)
            {
              min_distance = dist;
              best_lidar_id = it->second;
            }
          }
        }
      }
      
      // 如果找到历史Lidar ID，添加到候选列表
      if (best_lidar_id >= 0 && best_lidar_id < lcd_id_offset)
      {
        double match_quality = 1.0 / (min_distance + match_quality_distance_offset);  // 距离越近，质量越高
        id_candidates_round3[best_lidar_id].push_back({i, match_quality});
      }
    }
    
    // 如果当前是Lidar障碍物，记录到位置映射（但先不分配ID，等候选列表处理完）
    if (current_id < lcd_id_offset)
    {
      PositionKey key = GetPositionKey(current_ob);
      lidar_id_history_[key] = current_id;
      id_to_position_[current_id] = key;
    }
  }
  
  // 为每个ID选择最佳匹配的障碍物
  for (const auto &pair : id_candidates_round3)
  {
    int candidate_id = pair.first;
    const auto &candidates = pair.second;
    
    // 如果ID已被使用，跳过
    if (used_ids.find(candidate_id) != used_ids.end())
    {
      continue;
    }
    
    // 选择匹配质量最高的障碍物
    size_t best_idx = 0;
    double best_quality = candidates[0].second;
    for (size_t j = 1; j < candidates.size(); ++j)
    {
      if (candidates[j].second > best_quality)
      {
        best_quality = candidates[j].second;
        best_idx = j;
      }
    }
    
    // 将ID分配给最佳匹配的障碍物
    size_t ob_idx = candidates[best_idx].first;
    auto &current_ob = current_fusion_list.obstacle(ob_idx);
    int original_id = current_ob.id();
    current_ob.set_id(candidate_id);
    fusion_id_map_[original_id] = candidate_id;
    used_ids.insert(candidate_id);  // 标记ID已使用
    current_matched[ob_idx] = true;
    
    // 如果是Lidar ID，更新位置映射
    if (candidate_id < lcd_id_offset)
    {
      PositionKey key = GetPositionKey(current_ob);
      lidar_id_history_[key] = candidate_id;
      id_to_position_[candidate_id] = key;
    }
  }
  
  // 第四轮：为仍未匹配的障碍物分配新ID
  for (size_t i = 0; i < current_fusion_list.obstacle_size(); ++i)
  {
    if (current_matched[i]) continue;
    
    auto &current_ob = current_fusion_list.obstacle(i);
    int original_id = current_ob.id();
    
    // 确保新ID不与已存在的ID冲突（包括已使用的ID）
    while (true)
    {
      bool id_exists = false;
      // 检查是否在当前障碍物列表中使用
      for (size_t j = 0; j < current_fusion_list.obstacle_size(); ++j)
      {
        if (j != i && current_fusion_list.obstacle(j).id() == fusion_global_idx_)
        {
          id_exists = true;
          break;
        }
      }
      // 检查是否在已使用的ID集合中
      if (!id_exists && used_ids.find(fusion_global_idx_) != used_ids.end())
      {
        id_exists = true;
      }
      if (!id_exists)
      {
        break;
      }
      fusion_global_idx_++;
      if (fusion_global_idx_ > id_num_max)
      {
        fusion_global_idx_ = 0;
      }
    }
    
    current_ob.set_id(fusion_global_idx_);
    fusion_id_map_[original_id] = fusion_global_idx_;
    used_ids.insert(fusion_global_idx_);  // 标记ID已使用
    
    // 如果是新分配的Lidar ID，记录到位置映射
    if (fusion_global_idx_ < lcd_id_offset)
    {
      PositionKey key = GetPositionKey(current_ob);
      lidar_id_history_[key] = fusion_global_idx_;
      id_to_position_[fusion_global_idx_] = key;
    }
    
    fusion_global_idx_++;
    if (fusion_global_idx_ > id_num_max)
    {
      fusion_global_idx_ = 0;
    }
  }
  
  // 最终检查：确保所有障碍物的ID都是唯一的（防止遗漏）
  std::map<int, std::vector<size_t>> id_to_indices;
  for (size_t i = 0; i < current_fusion_list.obstacle_size(); ++i)
  {
    int id = current_fusion_list.obstacle(i).id();
    id_to_indices[id].push_back(i);
  }
  
  // 对于有重复ID的障碍物，重新分配ID
  for (const auto &pair : id_to_indices)
  {
    if (pair.second.size() > 1)
    {
      // 保留第一个障碍物的ID，为其他障碍物重新分配
      for (size_t idx = 1; idx < pair.second.size(); ++idx)
      {
        size_t ob_idx = pair.second[idx];
        auto &current_ob = current_fusion_list.obstacle(ob_idx);
        
        // 分配新的唯一ID
        while (true)
        {
          bool id_exists = false;
          for (size_t j = 0; j < current_fusion_list.obstacle_size(); ++j)
          {
            if (j != ob_idx && current_fusion_list.obstacle(j).id() == fusion_global_idx_)
            {
              id_exists = true;
              break;
            }
          }
          if (!id_exists && used_ids.find(fusion_global_idx_) != used_ids.end())
          {
            id_exists = true;
          }
          if (!id_exists)
          {
            break;
          }
          fusion_global_idx_++;
          if (fusion_global_idx_ > id_num_max)
          {
            fusion_global_idx_ = 0;
          }
        }
        
        current_ob.set_id(fusion_global_idx_);
        used_ids.insert(fusion_global_idx_);
        
        fusion_global_idx_++;
        if (fusion_global_idx_ > id_num_max)
        {
          fusion_global_idx_ = 0;
        }
      }
    }
  }
  
  // 清理过期的位置映射（只保留最近使用的ID）
  // 这里简化处理，实际可以维护时间戳
}

void MotionManager::FuseObstacleLists()
{
  if (is_init_ == false)
  {
    return;
  }
  
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 0. 检查数据源是否超时（数据中断检测）
  int64_t current_time_ms = legion::common::TimeTool::Now2Ms();
  bool obstacle_list_output_valid = true;
  bool lcd_obstacle_list_input_valid = true;
  
  // 检查 obstacle_list_output_ 是否超时
  // 注意：obstacle_list_output_ 是基于 obstacle_list_input_ 生成的，所以检查 obstacle_list_input_ 的更新时间
  if (last_obstacle_list_input_time_ms_ > 0)
  {
    int64_t time_since_last_update = current_time_ms - last_obstacle_list_input_time_ms_;
    if (time_since_last_update > static_cast<int64_t>(data_timeout_ms))
    {
      obstacle_list_output_valid = false;
      AINFO << "ObstacleListInput data timeout: " << time_since_last_update << "ms (threshold: " << data_timeout_ms << "ms), clearing obstacle_list_output_";
    }
  }
  else if (obstacle_list_output_.obstacle_size() == 0)
  {
    // 如果从未接收过数据且当前列表为空，认为无效
    obstacle_list_output_valid = false;
  }
  
  // 检查 lcd_obstacle_list_input_ 是否超时
  if (last_lcd_obstacle_list_input_time_ms_ > 0)
  {
    int64_t time_since_last_update = current_time_ms - last_lcd_obstacle_list_input_time_ms_;
    if (time_since_last_update > static_cast<int64_t>(data_timeout_ms))
    {
      lcd_obstacle_list_input_valid = false;
      AINFO << "LCDObstacleList data timeout: " << time_since_last_update << "ms (threshold: " << data_timeout_ms << "ms), clearing lcd_obstacle_list_input_";
    }
  }
  else if (lcd_obstacle_list_input_.obstacle_size() == 0)
  {
    // 如果从未接收过数据且当前列表为空，认为无效
    lcd_obstacle_list_input_valid = false;
  }
  
  // 如果某个数据源超时，清空该数据源
  if (!obstacle_list_output_valid)
  {
    AINFO << "ObstacleListInput timeout, clearing obstacle_list_output_";
    obstacle_list_output_.clear_obstacle();
  }
  
  if (!lcd_obstacle_list_input_valid)
  {
    AINFO << "LCDObstacleList timeout, clearing lcd_obstacle_list_input_";
    lcd_obstacle_list_input_.clear_obstacle();
  }
  
  // 如果两个数据源都无效，清空融合结果
  if (!obstacle_list_output_valid && !lcd_obstacle_list_input_valid)
  {
    AINFO << "Both data sources timeout or empty, clear fusion result";
    fusion_obstacle_list_.clear_obstacle();
    fusion_obstacle_list_.set_is_valid(false);
    return;
  }
  
  // 0. 对 LCDObstacleList 中的障碍物进行坐标转换（从车体坐标系转换为绝对坐标系）
  std::vector<legion::interface::Obstacle> lcd_obstacles;
  lcd_obstacle_list_input_.obstacle(lcd_obstacles);
  for (size_t obj_index = 0; obj_index < lcd_obstacles.size(); obj_index++)
  {
    // 转换 center_pos_vehicle 到 center_pos_abs
    legion::interface::Point3D center_pos = lcd_obstacles[obj_index].center_pos_vehicle();
    ConvertPoint(center_pos, location_);
    lcd_obstacles[obj_index].set_center_pos_abs(center_pos);
    
    // 转换 polygon_points_vehicle 到 polygon_point_abs
    std::vector<legion::interface::Point3D> from_polygon;
    lcd_obstacles[obj_index].polygon_point_vehicle(from_polygon);
    size_t polygon_point_size = from_polygon.size();
    std::vector<legion::interface::Point3D> to_polygon;
    to_polygon.resize(polygon_point_size);
    for (size_t polygon_point_index = 0; polygon_point_index < polygon_point_size; polygon_point_index++)
    {
      legion::interface::Point3D pt = from_polygon[polygon_point_index];
      ConvertPoint(pt, location_);
      to_polygon[polygon_point_index] = pt;
    }
    lcd_obstacles[obj_index].clear_polygon_point_abs();
    lcd_obstacles[obj_index].set_polygon_point_abs(&to_polygon);
    
    // 转换 theta_vehicle 到 theta_abs
    double theta_vehicle = lcd_obstacles[obj_index].theta_vehicle();
    double theta_abs = theta_vehicle + location_.heading();
    if (theta_abs > M_PI)
      theta_abs -= M_PI * 2.0;
    else if (theta_abs < -M_PI)
      theta_abs += M_PI * 2.0;
    lcd_obstacles[obj_index].set_theta_abs(theta_abs);
  }
  // 更新转换后的障碍物列表
  lcd_obstacle_list_input_.clear_obstacle();
  lcd_obstacle_list_input_.set_obstacle(lcd_obstacles);


  std::cout<<"lcd_obstacle_list_input_size: "<<lcd_obstacle_list_input_.obstacle_size()<<"\n";

  for(auto ob:lcd_obstacle_list_input_.obstacle())
  {
    std::cout << "id: " << ob.id() << " theta_vehicle: " << ob.theta_vehicle() <<" theta_abs: " << ob.theta_abs() <<" center_pos_abs: (" << ob.center_pos_abs().x() <<" , "<< ob.center_pos_abs().y() <<"  , "<< ob.center_pos_abs().z() <<")";
    std::vector<legion::interface::Point3D> poly_vehicle, poly_abs;
    ob.polygon_point_vehicle(poly_vehicle);
    ob.polygon_point_abs(poly_abs);
    if (poly_vehicle.size() > 0)
    {
      std::cout << " polygen_points_vehicle[0]: (" << poly_vehicle[0].x() <<" , "<< poly_vehicle[0].y() <<"  , "<< poly_vehicle[0].z() <<")";
    }
    if (poly_abs.size() > 0)
    {
      std::cout << " polygen_points_abs[0]: (" << poly_abs[0].x() <<" , "<< poly_abs[0].y() <<"  , "<< poly_abs[0].z() <<")";
    }
    std::cout << "\n";
  }
  std::cout<<"--------------------------------"<<"\n";
  std::cout<<"obstacle_list_output_size: "<<obstacle_list_output_.obstacle_size()<<"\n";
  for(auto ob:obstacle_list_output_.obstacle())
  {
    std::cout << "id: " << ob.id() << " theta_vehicle: " << ob.theta_vehicle() <<" theta_abs: " << ob.theta_abs() <<" center_pos_abs: (" << ob.center_pos_abs().x() <<" , "<< ob.center_pos_abs().y() <<"  , "<< ob.center_pos_abs().z() <<")";
    std::vector<legion::interface::Point3D> poly_vehicle, poly_abs;
    ob.polygon_point_vehicle(poly_vehicle);
    ob.polygon_point_abs(poly_abs);
    if (poly_vehicle.size() > 0)
    {
      std::cout << " polygen_points_vehicle[0]: (" << poly_vehicle[0].x() <<" , "<< poly_vehicle[0].y() <<"  , "<< poly_vehicle[0].z() <<")";
    }
    if (poly_abs.size() > 0)
    {
      std::cout << " polygen_points_abs[0]: (" << poly_abs[0].x() <<" , "<< poly_abs[0].y() <<"  , "<< poly_abs[0].z() <<")";
    }
    std::cout << "\n";
  }

  
  // 1. 时间同步：取最相近的两帧数据
  if (lcd_obstacle_list_input_.obstacle_size() == 0 || obstacle_list_output_.obstacle_size() == 0)
  {
    // 如果任一列表为空，直接使用非空的列表
    if (lcd_obstacle_list_input_.obstacle_size() > 0)
    {
      fusion_obstacle_list_ = lcd_obstacle_list_input_;
      for (auto &ob : fusion_obstacle_list_.obstacle())
      {
        ob.set_id(ob.id() + lcd_id_offset);
        ob.set_fusion_type(legion::interface::Obstacle::FusionType::LIDAR);
      }
    }
    else if (obstacle_list_output_.obstacle_size() > 0)
    {
      fusion_obstacle_list_ = obstacle_list_output_;
      for (auto &ob : fusion_obstacle_list_.obstacle())
      {
        ob.set_fusion_type(legion::interface::Obstacle::FusionType::CAMERA);
      }
    }
    else
    {
      fusion_obstacle_list_.clear_obstacle();
    }
    return;
  }
  
  // ========== 2. 时间同步检查 ==========
  // 检查两个障碍物列表的时间戳，如果时间差太大则跳过时间同步但仍进行融合
  bool has_valid_headers = lcd_obstacle_list_input_.has_header() && obstacle_list_output_.has_header();
  bool skip_time_sync = false;
  double time_diff = 0.0;
  
  if (has_valid_headers)
  {
    // 计算两个列表的时间差
    double lcd_time = lcd_obstacle_list_input_.header().stamp().sec() + 
                      lcd_obstacle_list_input_.header().stamp().nsec() * 1e-9;
    double output_time = obstacle_list_output_.header().stamp().sec() + 
                         obstacle_list_output_.header().stamp().nsec() * 1e-9;
    time_diff = std::abs(lcd_time - output_time);
    
    // 如果时间差太大（超过配置的阈值），跳过时间同步，但仍进行IOU融合
    if (time_diff > time_sync_max_diff)
    {
      AINFO << "Time difference too large: " << time_diff << "s, skip time sync but still do IOU fusion";
      skip_time_sync = true;
    }
  }
  else
  {
    AINFO << "Missing header in obstacle lists, skip time sync but still do IOU fusion";
    skip_time_sync = true;
  }
  
  // ========== 3. IOU匹配和融合 ==========
  // 使用配置文件中的IOU阈值进行匹配
  
  std::vector<legion::interface::Obstacle> fused_obstacles;
  std::vector<bool> lcd_matched(lcd_obstacle_list_input_.obstacle_size(), false);
  std::vector<bool> output_matched(obstacle_list_output_.obstacle_size(), false);
  
  // 遍历 obstacle_list_output_ 中的每个障碍物
  for (size_t i = 0; i < obstacle_list_output_.obstacle_size(); ++i)
  {
    const auto &output_ob = obstacle_list_output_.obstacle(i);
    double max_iou = 0.0;
    double max_bbox_iou = 0.0;  // 矩形框IOU
    int best_match_idx = -1;
    bool use_bbox_match = false;  // 是否使用矩形框匹配
    
    // 在 lcd_obstacle_list_input_ 中寻找最佳匹配
    for (size_t j = 0; j < lcd_obstacle_list_input_.obstacle_size(); ++j)
    {
      if (lcd_matched[j]) continue;
      
      const auto &lcd_ob = lcd_obstacle_list_input_.obstacle(j);
      
      // 先尝试多边形IOU匹配
      double iou = CalculatePolygonIoU(output_ob, lcd_ob);
      
      // 如果多边形IOU不够，尝试矩形框IOU（对形状变化更鲁棒）
      double bbox_iou = CalculateBoundingBoxIoU(output_ob, lcd_ob);
      
      // 检查位置距离（用于形状变化时的匹配）
      double dist = std::sqrt(
        std::pow(output_ob.center_pos_abs().x() - lcd_ob.center_pos_abs().x(), 2) +
        std::pow(output_ob.center_pos_abs().y() - lcd_ob.center_pos_abs().y(), 2)
      );
      
      // 优先使用多边形IOU，但如果位置很近且矩形框IOU较高，也考虑使用
      bool is_close_and_bbox_match = (dist < close_match_distance && bbox_iou > iou_threshold);
      
      if (iou > max_iou)
      {
        max_iou = iou;
        best_match_idx = j;
        use_bbox_match = false;
      }
      
      if (bbox_iou > max_bbox_iou && (bbox_iou > iou || is_close_and_bbox_match))
      {
        max_bbox_iou = bbox_iou;
        if (best_match_idx < 0 || (iou <= iou_threshold && bbox_iou > iou_threshold))
        {
          best_match_idx = j;
          use_bbox_match = true;
        }
      }
    }
    
    // 使用多边形IOU或矩形框IOU进行匹配（对形状变化更鲁棒）
    bool match_success = false;
    if (best_match_idx >= 0)
    {
      if (use_bbox_match)
      {
        // 使用矩形框IOU匹配（对形状变化更鲁棒）
        match_success = (max_bbox_iou > iou_threshold);
      }
      else
      {
        // 使用多边形IOU匹配
        match_success = (max_iou > iou_threshold);
      }
      
      // 如果多边形IOU匹配失败，但矩形框IOU足够且位置很近，也认为匹配成功
      if (!match_success && max_bbox_iou > iou_threshold)
      {
        const auto &lcd_ob = lcd_obstacle_list_input_.obstacle(best_match_idx);
        double dist = std::sqrt(
          std::pow(output_ob.center_pos_abs().x() - lcd_ob.center_pos_abs().x(), 2) +
          std::pow(output_ob.center_pos_abs().y() - lcd_ob.center_pos_abs().y(), 2)
        );
        if (dist < close_match_distance)
        {
          match_success = true;
          use_bbox_match = true;
        }
      }
    }
    
    // ========== 3.2 匹配成功，进行融合 ==========
    if (match_success && best_match_idx >= 0)
    {
      const auto &lcd_ob = lcd_obstacle_list_input_.obstacle(best_match_idx);
      double output_area = CalculateObstacleArea(output_ob);
      double lcd_area = CalculateObstacleArea(lcd_ob);
      
      legion::interface::Obstacle fused_ob;
      if (lcd_area > output_area)
      {
        // LCD障碍物面积更大，保留Lidar障碍物，但使用LCD障碍物的位置和多边形信息
        fused_ob = output_ob;
        
        // 复制LCD障碍物的位置和多边形字段
        fused_ob.set_center_pos_abs(lcd_ob.center_pos_abs());
        fused_ob.set_center_pos_vehicle(lcd_ob.center_pos_vehicle());
        
        // 复制多边形点
        std::vector<legion::interface::Point3D> lcd_polygon_abs, lcd_polygon_vehicle;
        lcd_ob.polygon_point_abs(lcd_polygon_abs);
        lcd_ob.polygon_point_vehicle(lcd_polygon_vehicle);
        fused_ob.clear_polygon_point_abs();
        fused_ob.set_polygon_point_abs(lcd_polygon_abs);
        fused_ob.clear_polygon_point_vehicle();
        fused_ob.set_polygon_point_vehicle(lcd_polygon_vehicle);
        
        // 确保速度、朝向、类别、子类别使用Lidar的值（因为LCD数据中这些值默认为0）
        fused_ob.set_velocity_abs(output_ob.velocity_abs());
        fused_ob.set_velocity_vehicle(output_ob.velocity_vehicle());
        fused_ob.set_theta_abs(output_ob.theta_abs());
        fused_ob.set_theta_vehicle(output_ob.theta_vehicle());
        fused_ob.set_type(output_ob.type());
        fused_ob.set_sub_type(output_ob.sub_type());
        
        fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::FUSED);
      }
      else
      {
        // Lidar障碍物面积更大或相等，保留Lidar障碍物（速度、朝向、类别、子类别已包含）
        fused_ob = output_ob;
        fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::FUSED);
      }
      fused_obstacles.push_back(fused_ob);
      
      output_matched[i] = true;
      lcd_matched[best_match_idx] = true;
    }
    else
    {
      // ========== 3.3 无匹配，保留Lidar障碍物 ==========
      // 无匹配，保留 obstacle_list_output_ 中的障碍物，fusion_type 置为 CAMERA
      legion::interface::Obstacle fused_ob = output_ob;
      fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::CAMERA);
      fused_obstacles.push_back(fused_ob);
      
      output_matched[i] = true;
    }
  }
  
  // ========== 4. 添加未匹配的LCD障碍物 ==========
  // 在添加之前，先尝试基于位置匹配历史Lidar ID，避免因形状变化导致的ID跳变
  for (size_t j = 0; j < lcd_obstacle_list_input_.obstacle_size(); ++j)
  {
    if (!lcd_matched[j])
    {
      legion::interface::Obstacle fused_ob = lcd_obstacle_list_input_.obstacle(j);
      
      // 首先尝试从历史帧中匹配Lidar ID，避免因形状变化导致的ID跳变
      int historical_id = MatchHistoricalObstacle(fused_ob);
      
      if (historical_id >= 0 && historical_id < lcd_id_offset)
      {
        // 找到历史Lidar ID，使用它作为稳定ID，不加上10000
        fused_ob.set_id(historical_id);
        fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::FUSED);
      }
      else
      {
        // 如果历史匹配失败，尝试基于位置的简单匹配（检查相邻网格中的Lidar ID）
        PositionKey key = GetPositionKey(fused_ob);
        int best_lidar_id = -1;
        double min_distance = 1e10;
        
        // 检查相邻网格（扩大搜索范围到5x5，以应对形状变化）
        for (int dx = -2; dx <= 2; ++dx)
        {
          for (int dy = -2; dy <= 2; ++dy)
          {
            PositionKey search_key;
            search_key.grid_x = key.grid_x + dx;
            search_key.grid_y = key.grid_y + dy;
            
            auto it = lidar_id_history_.find(search_key);
            if (it != lidar_id_history_.end())
            {
              // 计算实际距离
              double dist = std::sqrt(dx * dx + dy * dy) * grid_size;
              if (dist < min_distance && dist < close_match_distance)
              {
                min_distance = dist;
                best_lidar_id = it->second;
              }
            }
          }
        }
        
        if (best_lidar_id >= 0 && best_lidar_id < 10000)
        {
          // 找到附近的历史Lidar ID，使用它作为稳定ID
          fused_ob.set_id(best_lidar_id);
          fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::FUSED);
        }
        else
        {
          // 未找到历史ID，使用LCD的ID（加上10000）
          fused_ob.set_id(fused_ob.id() + 10000);
          fused_ob.set_fusion_type(legion::interface::Obstacle::FusionType::LIDAR);
        }
      }
      
      fused_obstacles.push_back(fused_ob);
    }
  }
  
  // 设置融合后的障碍物列表
  fusion_obstacle_list_.clear_obstacle();
  fusion_obstacle_list_.set_obstacle(fused_obstacles);
  // 设置 header（优先使用 obstacle_list_output_ 的 header）
  if (obstacle_list_output_.has_header())
  {
    fusion_obstacle_list_.set_header(obstacle_list_output_.header());
  }
  else if (lcd_obstacle_list_input_.has_header())
  {
    fusion_obstacle_list_.set_header(lcd_obstacle_list_input_.header());
  }
  fusion_obstacle_list_.set_sensor_id(obstacle_list_output_.sensor_id());
  fusion_obstacle_list_.set_is_valid(true);
  fusion_obstacle_list_.set_error_code(obstacle_list_output_.error_code());
  
  // 注意：ID稳定化逻辑已移到数据发布之前（在MotionManagerRun中），
  // 这样可以更好地处理数据源闪烁和形状差异带来的ID跳变问题
  
  // std::cout << "[MotionManager] FuseObstacleLists: fused " << fused_obstacles.size() 
  //           << " obstacles (LCD: " << lcd_obstacle_list_input_.obstacle_size() 
  //           << ", Output: " << obstacle_list_output_.obstacle_size() << ")" << "\n";
}

} // namespace fusion
} // namespace perception
} // namespace legion
