/**
 * @file    lidar_cluster_detect.cpp
 * @author  zdhy
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <time.h>
#include <fstream>
#include <sys/time.h>
#include <limits>
#include <algorithm>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "lidar_cluster_detect.h"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"
#include "modules/common/interface/point_cloud.hpp"
#include "modules/common/interface/obstacle_list.hpp"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {

void LidarClusterDetect::Init() {
  // step1 初始化状态设置为false
  {
    is_init_ = false;
    function_activation_ = false;
  }

  // step2 变量初始化
  { VariableInit(); }

  // step3 配置文件初始化
  {
    std::ifstream in(config_file_path_);
    in >> lidar_cluster_detect_json_;
    if (lidar_cluster_detect_json_.is_null()) {
      std::cout << "lidar_cluster_detect_json_ is null" << std::endl;
      return;
    }
  }
  // step4 日志初始化
  {LOGGING_INIT(lidar_cluster_detect_conf_, lidar_cluster_detect_json_)}

  // step4 IPC初始化
  {MESSAGE_INIT(lidar_cluster_detect_conf_, lidar_cluster_detect_json_)}

  // step5 读取配置文件
  {
    produce_lidar_cluster_detect_command_duration_ = lidar_cluster_detect_json_
        ["produce_lidar_cluster_detect_command_duration"];
    publish_lidar_cluster_detect_command_duration_ = lidar_cluster_detect_json_
        ["publish_lidar_cluster_detect_command_duration"];
    lidar_cluster_detect_conf_->set_use_system_timestamp(
        lidar_cluster_detect_json_["use_system_timestamp"]);
  }

  // step5.1 初始化聚类参数
  {
    json params_json;
    std::string params_file_path;
    
    // 获取参数文件路径
    // if (lidar_cluster_detect_json_.contains("params_file")) {
      params_file_path = lidar_cluster_detect_json_["params_file"];
      
      // 如果是相对路径，基于主配置文件所在目录解析
      // if (params_file_path[0] != '/') {
      //   std::filesystem::path config_path(config_file_path_);
      //   std::filesystem::path config_dir = config_path.parent_path();
      //   std::filesystem::path params_path(params_file_path);
      //   std::filesystem::path full_params_path = config_dir / params_path;
      //   params_file_path = full_params_path.string();
      // }
      
      // 加载参数文件
      std::ifstream params_in(params_file_path);
      if (params_in.is_open()) {
        params_in >> params_json;
        params_in.close();
        std::cout << "Loaded clustering parameters from: " << params_file_path << std::endl;
      } else {
        std::cout << "Warning: Cannot open params file: " << params_file_path 
                  << ", using default values" << std::endl;
        params_json = json::object(); // 空对象，使用默认值
      }
    // } else {
    //   std::cout << "Warning: params_file not found in config, using default values" << std::endl;
    //   params_json = json::object(); // 空对象，使用默认值
    // }
    
    // 从参数文件读取配置，如果不存在则使用默认值
    cell_map_min_x_ = params_json.value("cell_map_min_x", -10.0f);
    cell_map_max_x_ = params_json.value("cell_map_max_x", 10.0f);
    cell_map_min_y_ = params_json.value("cell_map_min_y", -20.0f);
    cell_map_max_y_ = params_json.value("cell_map_max_y", 40.0f);
    cell_map_min_z_ = params_json.value("cell_map_min_z", -0.9f);
    cell_map_max_z_ = params_json.value("cell_map_max_z", 0.5f);
    cell_width_ = params_json.value("cell_width", 0.4f);
    cell_length_ = params_json.value("cell_length", 0.8f);
    point_min_size_for_cell_ = params_json.value("point_min_size_for_cell", 5);
    point_min_size_for_box_ = params_json.value("point_min_size_for_box", 10);
    cell_max_size_for_big_box_ = params_json.value("cell_max_size_for_big_box", 100);
    obstacle_min_height_ = params_json.value("obstacle_min_height", 0.05f);
    delta_x_ = params_json.value("delta_x", 0.0f);
    delta_y_ = params_json.value("delta_y", 0.0f);
    in_estimate_pose_ = params_json.value("in_estimate_pose", false);
    max_polygon_points_ = params_json.value("max_polygon_points", 50);  // 默认最大50个点
    
    col_size_ = (int)((cell_map_max_x_ - cell_map_min_x_) / cell_width_) + 1;
    row_size_ = (int)((cell_map_max_y_ - cell_map_min_y_) / cell_length_) + 1;
  }

  // step6 故障码初始化
  FaultMonitorInit();

  // step7 算法初始化
  {}

  // step8 定时器和线程初始化
  {
    status_detect_duration_ = (uint32_t)(
        double)lidar_cluster_detect_json_["status"]["status_detect_duration"];

    ad_timer_manager_ =
        std::make_shared<ADTimerManager<LidarClusterDetect, void>>();
    task_100ms_ = std::make_shared<WheelTimer<LidarClusterDetect, void>>(
        ad_timer_manager_);
    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create task_thread_ thread.";
      return;
    }
  }
  std::cout << "lidar_cluster_detect init" << std::endl;

  // step9 初始化状态为true
  { is_init_ = true; }
  
  // 手动激活
  TaskActivate();
}

void LidarClusterDetect::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void LidarClusterDetect::VariableInit() {
  lidar_cluster_detect_conf_ = std::make_shared<LidarClusterDetectConf>();
}

void LidarClusterDetect::Print() {}

void LidarClusterDetect::Log() {}

void LidarClusterDetect::TaskActivate() {
  if (is_init_ == false) {
    return;
  }
  // IPC激活
  MessagesActivate();
  if (message_manager_.empty()) {
    AERROR << "No message manager configured. Check message.active_message in config.";
  }
  if (function_activation_) {
    return;
  }
  task_100ms_->AddTimer(100, &LidarClusterDetect::Task100ms, this);
  // 所有定时器都使用高级定时器，方便激活和去激活。
  std::cout << "===================function activate=================="
            << std::endl;
  function_activation_ = true;
  return;
}

void LidarClusterDetect::TaskStop() {
  if (is_init_ == false) {
    return;
  }
  // IPC去激活
  MessagesDeActivate();
  if (function_activation_ == false) {
    return;
  }
  task_100ms_->Stop();
  {
    // 清除所有内部计算的中间结果，保证回到刚init完的状态
  }
  std::cout << "******************function stop***************" << std::endl;
  function_activation_ = false;
  return;
}

void LidarClusterDetect::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<LidarClusterDetect>> message_manager) {
  message_manager_.insert(
      std::pair<std::string,
                std::shared_ptr<MessageManager<LidarClusterDetect>>>(
          name, message_manager));
}

void LidarClusterDetect::Task100ms(void* param) {
  if (function_activation_ == false) {
    return;
  }
  ComputeLidarClusterDetectCommandOnTimer();
}

void LidarClusterDetect::PublishObstacleList(
    legionclaw::interface::ObstacleList obstacle_list) {
#if LCM_ENABLE
  if (message_manager_.count("LCM") > 0)
    message_manager_["LCM"]->PublishObstacleList(obstacle_list);
#endif

#if ROS_ENABLE
  if (message_manager_.count("ROS") > 0)
    message_manager_["ROS"]->PublishObstacleList(obstacle_list);
#endif

#if DDS_ENABLE
  if (message_manager_.count("DDS") > 0)
    message_manager_["DDS"]->PublishObstacleList(obstacle_list);
#endif

#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0)
    message_manager_["ROS2"]->PublishObstacleList(obstacle_list);
#endif
}

void LidarClusterDetect::PublishClusterPointCloud(
    ClusterVectorPtr& filter_cluster_vector_ptr,
    const legionclaw::interface::Header& header) {
#if ROS2_ENABLE
  if (message_manager_.count("ROS2") > 0 && ros2_message_manager_ != nullptr) {
    // Convert clusters to common PointCloud interface for publishing
    legionclaw::interface::PointCloud cluster_cloud;
    cluster_cloud.set_header(header);
    cluster_cloud.set_is_dense(true);

    std::vector<legionclaw::interface::PointXYZIRT> points;
    for (size_t i = 0; i < filter_cluster_vector_ptr->size(); ++i) {
      for (const auto& pt : (*filter_cluster_vector_ptr)[i].cluster_points_) {
        legionclaw::interface::PointXYZIRT point_xyzirt;
        point_xyzirt.set_x(pt.x);
        point_xyzirt.set_y(pt.y);
        point_xyzirt.set_z(pt.z);
        // Use cluster index * 10 as intensity to distinguish clusters
        point_xyzirt.set_intensity(static_cast<uint32_t>(i * 10));
        points.emplace_back(point_xyzirt);
      }
    }

    cluster_cloud.set_point(points);
    cluster_cloud.set_width(static_cast<uint32_t>(cluster_cloud.point_size()));
    cluster_cloud.set_height(1);

    ros2_message_manager_->PublishClusterPointCloud(cluster_cloud);
  }
#endif
}
void LidarClusterDetect::PublishFaults() {
  if (is_init_ == false) {
    return;
  }
  legionclaw::interface::Faults faults;
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

std::shared_ptr<LidarClusterDetectConf> LidarClusterDetect::GetConf() const {
  return lidar_cluster_detect_conf_;
}

void LidarClusterDetect::HandleObuCmdMsg(legionclaw::interface::ObuCmdMsg obu_cmd_msg) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lidar_cluster_detect_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = obu_cmd_msg.header();
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

void LidarClusterDetect::HandlePointCloud(
    legionclaw::interface::PointCloud point_cloud) {
  if (is_init_ == false) {
    return;
  }
  if (function_activation_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lidar_cluster_detect_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = point_cloud.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      point_cloud.set_header(header);
    }
    point_cloud_ = point_cloud;

  }
}

LidarClusterDetect::CellMatrixPtr LidarClusterDetect::BuildCellMap(
    const legionclaw::interface::PointCloud& point_cloud,
    int col_size, int row_size) {
  CellMatrixPtr cell_matrix_ptr = std::make_shared<CellMatrix>(
      col_size, CellVector(row_size, PointsCell()));
  
  std::vector<legionclaw::interface::PointXYZIRT> point_vec;
  point_cloud.point(point_vec);
  
  for (const auto& pt : point_vec) {
    float x = pt.x();
    float y = pt.y();
    float z = pt.z();
    
    if (x > cell_map_min_x_ && x < cell_map_max_x_ &&
        y > cell_map_min_y_ && y < cell_map_max_y_ &&
        z > cell_map_min_z_ && z < cell_map_max_z_) {
      int col_id = (int)((x - cell_map_min_x_) / cell_width_);
      int row_id = (int)((y - cell_map_min_y_) / cell_length_);
      
      if (col_id >= 0 && col_id < col_size && row_id >= 0 && row_id < row_size) {
        PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = pt.intensity();
        (*cell_matrix_ptr)[col_id][row_id].cell_points_.push_back(point);
      }
    }
  }
  return cell_matrix_ptr;
}

void LidarClusterDetect::ClusterConnectedPoints(
    CellMatrixPtr& cell_matrix_ptr,
    ClusterVectorPtr& cluster_vector_ptr) {
  // Initialize find flags and filter cells by height
  for (size_t i = 0; i < cell_matrix_ptr->size(); i++) {
    for (size_t j = 0; j < (*cell_matrix_ptr)[0].size(); j++) {
      (*cell_matrix_ptr)[i][j].find_flag_ = 0;
      float min_z = std::numeric_limits<float>::max();
      float max_z = -std::numeric_limits<float>::max();
      
      for (const auto& pt : (*cell_matrix_ptr)[i][j].cell_points_) {
        if (pt.z < min_z) min_z = pt.z;
        if (pt.z > max_z) max_z = pt.z;
      }
      
      double cell_height = max_z - min_z;
      if (cell_height < obstacle_min_height_) {
        (*cell_matrix_ptr)[i][j].cell_points_.clear();
      }
    }
  }

  // Cluster connected cells
  for (size_t i = 0; i < cell_matrix_ptr->size(); i++) {
    for (size_t j = 0; j < (*cell_matrix_ptr)[0].size(); j++) {
      Cluster clusteri;
      if ((*cell_matrix_ptr)[i][j].find_flag_ == 0) {
        if ((*cell_matrix_ptr)[i][j].cell_points_.size() >
            (size_t)point_min_size_for_cell_) {
          (*cell_matrix_ptr)[i][j].find_flag_ = 1;
          Find8(i, j, cell_matrix_ptr, clusteri);
          cluster_vector_ptr->push_back(clusteri);
        }
      }
    }
  }
}

void LidarClusterDetect::Find8(int col_id, int row_id,
                               CellMatrixPtr& cell_matrix_ptr,
                               Cluster& clusteri) {
  // Add points from current cell
  for (const auto& pt : (*cell_matrix_ptr)[col_id][row_id].cell_points_) {
    clusteri.cluster_points_.push_back(pt);
  }
  clusteri.cell_col_.push_back(col_id);
  clusteri.cell_row_.push_back(row_id);
  
  // Check 8 neighbors
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) continue; // skip self
      
      int nexti = col_id + i;
      int nextj = row_id + j;
      
      if (nexti < 0 || nextj < 0 ||
          (size_t)nexti >= cell_matrix_ptr->size() ||
          (size_t)nextj >= (*cell_matrix_ptr)[0].size()) {
        continue;
      }
      
      if ((*cell_matrix_ptr)[nexti][nextj].find_flag_ == 1) {
        continue;
      }
      
      (*cell_matrix_ptr)[nexti][nextj].find_flag_ = 1;
      
      if ((*cell_matrix_ptr)[nexti][nextj].cell_points_.size() >
          (size_t)point_min_size_for_cell_) {
        for (const auto& pt : (*cell_matrix_ptr)[nexti][nextj].cell_points_) {
          clusteri.cluster_points_.push_back(pt);
        }
        clusteri.cell_col_.push_back(nexti);
        clusteri.cell_row_.push_back(nextj);
        
        if (clusteri.cell_col_.size() > (size_t)cell_max_size_for_big_box_) {
          continue;
        }
        
        Find8(nexti, nextj, cell_matrix_ptr, clusteri);
      }
    }
  }
}

void LidarClusterDetect::BuildCluster(
    ClusterVectorPtr& cluster_vector_ptr,
    ClusterVectorPtr& filter_cluster_vector_ptr,
    const legionclaw::interface::Header& header) {
  for (auto& cluster : *cluster_vector_ptr) {
    if (cluster.cluster_points_.size() < (size_t)point_min_size_for_box_) {
      continue;
    }
    
    // Calculate bounding box
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (const auto& pt : cluster.cluster_points_) {
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
      
      if (pt.x < min_x) min_x = pt.x;
      if (pt.x > max_x) max_x = pt.x;
      if (pt.y < min_y) min_y = pt.y;
      if (pt.y > max_y) max_y = pt.y;
      if (pt.z < min_z) min_z = pt.z;
      if (pt.z > max_z) max_z = pt.z;
    }
    
    // Calculate centroid
    size_t point_count = cluster.cluster_points_.size();
    if (point_count > 0) {
      cluster.center_x = sum_x / point_count;
      cluster.center_y = sum_y / point_count;
      cluster.center_z = sum_z / point_count;
    }
    
    cluster.min_x = min_x;
    cluster.max_x = max_x;
    cluster.min_y = min_y;
    cluster.max_y = max_y;
    cluster.min_z = min_z;
    cluster.max_z = max_z;
    
    // Calculate dimensions
    cluster.length = max_x - min_x;
    cluster.width = max_y - min_y;
    cluster.height = max_z - min_z;
    
    if (cluster.height < obstacle_min_height_) {
      continue;
    }
    
    filter_cluster_vector_ptr->push_back(cluster);
  }
}

void LidarClusterDetect::ConvertClustersToObstacleList(
    ClusterVectorPtr& filter_cluster_vector_ptr,
    const legionclaw::interface::Header& header,
    legionclaw::interface::ObstacleList& obstacle_list) {
  obstacle_list.clear_obstacle();
  obstacle_list.set_header(header);
  // obstacle_list.set_sensor_id(legionclaw::common::SensorID::LIDAR_TOP);
  obstacle_list.set_is_valid(true);
  // obstacle_list.set_error_code(legionclaw::common::ErrorCode::OK);
  
  std::vector<legionclaw::interface::Obstacle> obstacles;
  int obstacle_id = 0;
  
  for (const auto& cluster : *filter_cluster_vector_ptr) {
    legionclaw::interface::Obstacle obstacle;
    
    // Set basic properties
    obstacle.set_id(obstacle_id++);
    obstacle.set_existence_prob(1.0);
    obstacle.set_type(legionclaw::common::ObstacleType::OBSTACLE_UNKNOWN);
    obstacle.set_confidence(1.0);
    
    // Set center position
    legionclaw::interface::Point3D center_pos;
    center_pos.set_x(cluster.center_x + delta_x_);
    center_pos.set_y(cluster.center_y + delta_y_);
    center_pos.set_z(cluster.min_z + cluster.height / 2.0f);
    obstacle.set_center_pos_vehicle(center_pos);
    obstacle.set_center_pos_abs(center_pos);
    
    // Set dimensions
    obstacle.set_length((cluster.length < 0) ? -cluster.length : cluster.length);
    obstacle.set_width((cluster.width < 0) ? -cluster.width : cluster.width);
    obstacle.set_height((cluster.height < 0) ? -cluster.height : cluster.height);
    
    // Set polygon points (convex hull)
    std::vector<legionclaw::interface::Point3D> polygon_points;
    if (cluster.cluster_points_.size() >= 3) {
      // Convert cluster points to OpenCV Point2f for convex hull calculation
      std::vector<cv::Point2f> points;
      for (const auto& pt : cluster.cluster_points_) {
        points.push_back(cv::Point2f(pt.x + delta_x_, pt.y + delta_y_));
      }
      
      // Calculate convex hull using OpenCV
      std::vector<cv::Point2f> hull;
      cv::convexHull(points, hull);
      
      // 如果凸包点数超过限制，使用 approxPolyDP 简化
      std::vector<cv::Point2f> simplified_hull;
      if (hull.size() > static_cast<size_t>(max_polygon_points_)) {
        // 使用 Douglas-Peucker 算法简化多边形
        // epsilon 参数控制简化程度，这里根据多边形周长动态计算
        double perimeter = cv::arcLength(hull, true);
        double epsilon = perimeter * 0.01;  // 1% 的周长作为容差
        cv::approxPolyDP(hull, simplified_hull, epsilon, false);
        
        // 如果简化后仍然超过限制，使用均匀采样
        if (simplified_hull.size() > static_cast<size_t>(max_polygon_points_)) {
          simplified_hull.clear();
          size_t step = std::max(1UL, hull.size() / max_polygon_points_);
          for (size_t i = 0; i < hull.size(); i += step) {
            simplified_hull.push_back(hull[i]);
            if (simplified_hull.size() >= static_cast<size_t>(max_polygon_points_)) {
              break;
            }
          }
          // 确保包含最后一个点
          if (!simplified_hull.empty() && simplified_hull.back() != hull.back()) {
            simplified_hull.push_back(hull.back());
          }
        }
        hull = simplified_hull;
      }
      
      // Convert convex hull points back to Point3D
      for (const auto& hull_pt : hull) {
        legionclaw::interface::Point3D poly_pt;
        poly_pt.set_x(hull_pt.x);
        poly_pt.set_y(hull_pt.y);
        poly_pt.set_z(cluster.min_z);
        polygon_points.push_back(poly_pt);
      }
    } else {
      // If less than 3 points, use all points directly
      for (const auto& pt : cluster.cluster_points_) {
        legionclaw::interface::Point3D poly_pt;
        poly_pt.set_x(pt.x + delta_x_);
        poly_pt.set_y(pt.y + delta_y_);
        poly_pt.set_z(cluster.min_z);
        polygon_points.push_back(poly_pt);
      }
    }
    obstacle.set_polygon_point_vehicle(&polygon_points);
    obstacle.set_polygon_point_abs(&polygon_points);
    
    // Set timestamp
    legionclaw::interface::Time timestamp;
    timestamp.set_sec(header.stamp().sec());
    timestamp.set_nsec(header.stamp().nsec());
    obstacle.set_timestamp(timestamp);
    obstacle.set_create_time(timestamp);
    obstacle.set_last_updated_time(timestamp);
    
    obstacles.push_back(obstacle);
  }
  
  obstacle_list.set_obstacle(&obstacles);
}

void LidarClusterDetect::ComputeLidarClusterDetectCommandOnTimer() {

  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.point_cloud_ = point_cloud_;
  }

  Status status = CheckInput(&local_view_);
  // check data

  if (!status.ok()) {
    return;
  }

  // 算法计算 - 点云聚类
  std::vector<legionclaw::interface::PointXYZIRT> point_vec;
  local_view_.point_cloud_.point(point_vec);
  
  if (point_vec.empty()) {
    return;
  }
  
  ClusterVectorPtr cluster_vector_ptr = std::make_shared<ClusterVector>();
  ClusterVectorPtr filter_cluster_vector_ptr = std::make_shared<ClusterVector>();
  
  CellMatrixPtr cell_matrix_ptr = BuildCellMap(local_view_.point_cloud_, col_size_, row_size_);
  ClusterConnectedPoints(cell_matrix_ptr, cluster_vector_ptr);
  BuildCluster(cluster_vector_ptr, filter_cluster_vector_ptr, 
               local_view_.point_cloud_.header());
  
  // Convert clusters to obstacle list
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ConvertClustersToObstacleList(filter_cluster_vector_ptr,
                                  local_view_.point_cloud_.header(),
                                  obstacle_list_);
  }
  
  // Publish obstacle list
  PublishObstacleList(obstacle_list_);
  
  // Publish cluster point cloud for visualization
  PublishClusterPointCloud(filter_cluster_vector_ptr,
                           local_view_.point_cloud_.header());
                           
}
Status LidarClusterDetect::CheckInput(LocalView* local_view) {
  return Status::Ok();
}

void LidarClusterDetect::MessagesInit() {
  if (lidar_cluster_detect_conf_ == nullptr)
    return;

  if (lidar_cluster_detect_json_.find("message") ==
          lidar_cluster_detect_json_.end() ||
      !lidar_cluster_detect_json_["message"].is_object()) {
    AERROR << "config missing message section";
    return;
  }
  const auto &message_cfg = lidar_cluster_detect_json_["message"];
  if (message_cfg.find("active_message") == message_cfg.end() ||
      !message_cfg["active_message"].is_array() ||
      message_cfg.find("message_info") == message_cfg.end() ||
      !message_cfg["message_info"].is_array()) {
    AERROR << "config message.active_message or message.message_info invalid";
    return;
  }

  const auto &active_message = message_cfg["active_message"];
  const auto &message_info = message_cfg["message_info"];

  for (const auto &active_item : active_message) {
    if (!active_item.is_number_integer()) {
      continue;
    }
    int idx = active_item.get<int>();
    if (idx < 0 || idx >= static_cast<int>(message_info.size())) {
      AERROR << "active_message index out of range: " << idx;
      continue;
    }

    const auto &message = message_info[idx];
    if (!message.is_object()) {
      continue;
    }

    int type = message.value("type", -1);
    std::string name = message.value("name", "");
    if (name.empty()) {
      AERROR << "message_info[" << idx << "] name is empty";
      continue;
    }

    switch (type) {
#if LCM_ENABLE
    case legionclaw::common::MessageType::LCM: {
      AINFO << "message type:LCM";

      lcm_message_manager_ =
          std::make_shared<LcmMessageManager<LidarClusterDetect>>();
      ResigerMessageManager(name, lcm_message_manager_);

      lcm_message_manager_->Init(this);
    } break;
#endif
#if DDS_ENABLE
    case legionclaw::common::MessageType::DDS: {
      AINFO << "message type:DDS";

      dds_message_manager_ =
          std::make_shared<DdsMessageManager<LidarClusterDetect>>();
      ResigerMessageManager(name, dds_message_manager_);

      dds_message_manager_->Init(this);
    } break;
#endif
#if ROS_ENABLE
    case legionclaw::common::MessageType::ROS: {
      AINFO << "message type:ROS";

      ros_message_manager_ =
          std::make_shared<RosMessageManager<LidarClusterDetect>>();
      ResigerMessageManager(name, ros_message_manager_);
      ros_message_manager_->Init(this);
    } break;
#endif
#if ROS2_ENABLE
    case legionclaw::common::MessageType::ROS2: {
      AINFO << "message type:ROS2";

      ros2_message_manager_ =
          std::make_shared<Ros2MessageManager<LidarClusterDetect>>();
      ResigerMessageManager(name, ros2_message_manager_);

      ros2_message_manager_->Init(this);
    } break;
#endif

#if ADSFI_ENABLE
    case legionclaw::common::MessageType::ADSFI: {
      AINFO << "message type:ADSFI";

      adsfi_message_manager_ =
          std::make_shared<AdsfiMessageManager<LidarClusterDetect>>();
      ResigerMessageManager(name, adsfi_message_manager_);

      adsfi_message_manager_->Init(this);
    } break;
#endif
    default: {
      AERROR << "unknown message type";
    } break;
    }
  }
}

void LidarClusterDetect::FaultMonitorInit() {
  legionclaw::interface::FaultCodeCallback sendheart_callback_func =
      std::bind(&LidarClusterDetect::PublishFaults, this);
  legionclaw::interface::FaultCodeCallback fault_callback_func = nullptr;
  FAULTCODE_INIT("../../../../common/data/faults/faults.json",
                 "lidar_cluster_detect", faultcodeset_, sendheart_callback_func,
                 fault_callback_func)
}

void LidarClusterDetect::MessagesActivate() {
  if (lidar_cluster_detect_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->Activate();
  }
  return;
}

void LidarClusterDetect::MessagesDeActivate() {
  if (lidar_cluster_detect_conf_ == nullptr) {
    return;
  }
  for (auto message_manager : message_manager_) {
    message_manager.second->DeActivate();
  }
  return;
}

void LidarClusterDetect::StatusDetectOnTimer() {}

void LidarClusterDetect::Spin() {
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
} // namespace legionclaw
