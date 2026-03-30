/**
 * @file    lidar_detect.cpp
 * @author  hyzx
 * @date    2022-05-06
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#include <fstream>
#include <sys/time.h>
#include <time.h>
#include <chrono>

#include "lidar_detect.h"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/point_cloud.hpp"
#include "modules/common/macros/macros.h"
#include "modules/common/time/time_tool.h"

/**
 * @namespace legionclaw::perception::lidar
 * @brief legionclaw::perception::lidar
 */

namespace legionclaw {
namespace perception {
namespace lidar {

void LidarDetect::Init() {
  // step1 初始化状态设置为false
  { is_init_ = false; }

  // step2 变量初始化
  { VariableInit(); }

  // step3 配置文件初始化
  {
    std::ifstream in(config_file_path_);
    in >> lidar_detect_json_;
    if (lidar_detect_json_.is_null()) {
      std::cout << "lidar_detect_json_ is null" << std::endl;
      return;
    }
  }
  // step4 日志初始化
  {LOGGING_INIT(lidar_detect_conf_, lidar_detect_json_)}

  // step4 IPC初始化
  {MESSAGE_INIT(lidar_detect_conf_, lidar_detect_json_)}

  // step5 读取配置文件
  {
    produce_lidar_detect_command_duration_ =
        lidar_detect_json_["produce_lidar_detect_command_duration"];
    publish_lidar_detect_command_duration_ =
        lidar_detect_json_["publish_lidar_detect_command_duration"];
    lidar_detect_conf_->set_use_system_timestamp(
        lidar_detect_json_["use_system_timestamp"]);
    scn_engine_file_path = lidar_detect_json_["scn_engine_file_path"];
    rpn_engine_file_path = lidar_detect_json_["rpn_engine_file_path"];
    general_threshold = lidar_detect_json_["general_threshold"];
    big_vehicle_threshold = lidar_detect_json_["big_vehicle_threshold"];
    unknow_threshold = lidar_detect_json_["unknow_threshold"];
  }

  // step6 故障码初始化
  FaultMonitorInit();

  // step7 算法初始化
  {
    Params params;
    checkCudaErrors(cudaStreamCreate(&stream));
    bool verbose = false;
    static CenterPoint model(rpn_engine_file_path, scn_engine_file_path, verbose);
    centerpoint = &model;
    (*centerpoint).prepare();
    checkCudaErrors(cudaMalloc((void **)&d_points, MAX_POINTS_NUM * params.feature_num * sizeof(float)));
    cout << "初始化成功！！" << endl;
    // int dev = 0;
    // cudaDeviceProp devProp;
    // CHECK(cudaGetDeviceProperties(&devProp, dev));
    // std::cout << "使用GPU device " << dev << ": " << devProp.name << std::endl;
    // std::cout << "SM的数量: " << devProp.multiProcessorCount << std::endl;
    // std::cout << "每个线程块的共享内存大小: " << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
    // std::cout << "每个线程块的最大线程数： " << devProp.maxThreadsPerBlock << std::endl;
    // std::cout << "每个EM的最大线程数: " << devProp.maxThreadsPerMultiProcessor << std::endl;
    // std::cout << "每个SM的最大线程数: " << devProp.maxThreadsPerMultiProcessor / 32 << std::endl; 
  }

  // step8 定时器和线程初始化
  {
    status_detect_duration_ = (uint32_t)(
        double)lidar_detect_json_["status"]["status_detect_duration"];

    TimerManager<LidarDetect>::AddTimer(
        produce_lidar_detect_command_duration_,
        &LidarDetect::ComputeLidarDetectCommandOnTimer, this);

    TimerManager<LidarDetect>::AddTimer(
        status_detect_duration_, &LidarDetect::StatusDetectOnTimer, this);

    ad_timer_manager_ = std::make_shared<ADTimerManager<LidarDetect, void>>();
    task_10ms_ =
        std::make_shared<WheelTimer<LidarDetect, void>>(ad_timer_manager_);
    task_10ms_->AddTimer(10, &LidarDetect::Task10ms, this);
    task_thread_.reset(new std::thread([this] { Spin(); }));
    if (task_thread_ == nullptr) {
      AERROR << "Unable to create can task_thread_ thread.";
      return;
    }
  }

  // step9 初始化状态为true
  { is_init_ = true; }
}

void LidarDetect::Join() {
  if (task_thread_ != nullptr && task_thread_->joinable()) {
    task_thread_->join();
    task_thread_.reset();
    AINFO << "task_thread_ stopped [ok].";
  }
}

void LidarDetect::VariableInit() {
  lidar_detect_conf_ = std::make_shared<LidarDetectConf>();
}

void LidarDetect::Print() {}

void LidarDetect::Log() {}

void LidarDetect::ResigerMessageManager(
    std::string name,
    std::shared_ptr<MessageManager<LidarDetect>> message_manager) {
  message_manager_.insert(
      std::pair<std::string, std::shared_ptr<MessageManager<LidarDetect>>>(
          name, message_manager));
}

void LidarDetect::Task10ms(void *param) {}

void LidarDetect::PublishObstacleList(
    legionclaw::interface::ObstacleList obstacle_list) {
#if LCM_ENABLE
  message_manager_["LCM"]->PublishObstacleList(obstacle_list);
#endif

#if ROS_ENABLE
  message_manager_["ROS"]->PublishObstacleList(obstacle_list);
#endif

#if DDS_ENABLE
  message_manager_["DDS"]->PublishObstacleList(obstacle_list);
#endif

#if ROS2_ENABLE
  message_manager_["ROS2"]->PublishObstacleList(obstacle_list);
#endif
}

std::shared_ptr<LidarDetectConf> LidarDetect::GetConf() const {
  return lidar_detect_conf_;
}

void LidarDetect::HandlePointCloud(legionclaw::interface::PointCloud point_cloud) {
  if (is_init_ == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lidar_detect_conf_->use_system_timestamp() == true) {
      legionclaw::interface::Header header = point_cloud.header();
      header.set_stamp(TimeTool::Now2TmeStruct());
      point_cloud.set_header(header);
    }
    point_cloud_ = point_cloud;
  }
}

void LidarDetect::ComputeLidarDetectCommandOnTimer() {

  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.point_cloud_ = point_cloud_;
  }

  Status status = CheckInput(&local_view_);
  // check data

  if (!status.ok()) {
  } 
  else 
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int point_size = local_view_.point_cloud_.point_size();
    std::cout << "points size:"<< point_size << endl;

    if(point_size > 0)
    {
      float* points = new float[5 * point_size];
      PointToBuffer(points, local_view_.point_cloud_);

      checkCudaErrors(cudaMemcpy(d_points, points, 5 * point_size * sizeof(float), cudaMemcpyHostToDevice));
      (*centerpoint).doinfer((void *)d_points, point_size, stream);
      (*centerpoint).perf_report();

      delete [] points;
      std::cout<<"Num of objects: " << (*centerpoint).nms_pred_.size() << std::endl;
    }

    legionclaw::interface::ObstacleList result_obstacle_list;
    result_obstacle_list.header().set_stamp(local_view_.point_cloud_.header().stamp());

    BoxesToObstacleList((*centerpoint).nms_pred_, result_obstacle_list);
    GetPolygon(result_obstacle_list);

    // vector<legionclaw::interface::Obstacle> temp_obl;
    // for(auto &ob:result_obstacle_list.obstacle())
    // {
    //   if((ob.sub_type() == 5 || ob.sub_type() == 6) &&
    //       ob.confidence() >= big_vehicle_threshold)
    //   {
    //     temp_obl.push_back(ob);
    //   }else if(ob.sub_type() == 0 && ob.confidence() >= unknow_threshold)
    //   {
    //     temp_obl.push_back(ob);
    //   }else
    //   {
    //     if(ob.confidence() >= general_threshold)
    //     {
    //       temp_obl.push_back(ob);
    //     }
    //   }
    // }
    // result_obstacle_list.set_obstacle(temp_obl);

    double sensor_time = result_obstacle_list.header().stamp().sec() + result_obstacle_list.header().stamp().nsec() * 1e-9;
    double now_time = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() * 1e-3;
    double perception_time = now_time - sensor_time;
    cout << "perception_time: " << perception_time << endl;
    for(auto &ob:result_obstacle_list.obstacle())
    {
      ob.set_tracking_time(perception_time);
      legionclaw::interface::Point3D temp_pos;
    }
    PublishObstacleList(result_obstacle_list);
  }

  //算法计算
}
Status LidarDetect::CheckInput(LocalView *local_view) { return Status::Ok(); }

void LidarDetect::MessagesInit() {
  if (lidar_detect_conf_ == nullptr)
    return;

  std::map<std::string, legionclaw::common::Message>::iterator iter;
  for (auto &iter : lidar_detect_conf_->messages()) {
    auto message = iter.second;

    switch (message.type) {
#if LCM_ENABLE
    case legionclaw::common::MessageType::LCM: {
      AINFO << "message type:LCM";

      lcm_message_manager_ = std::make_shared<LcmMessageManager<LidarDetect>>();
      ResigerMessageManager(message.name, lcm_message_manager_);

      lcm_message_manager_->Init(this);
    } break;
#endif
#if DDS_ENABLE
    case legionclaw::common::MessageType::DDS: {
      AINFO << "message type:DDS";

      dds_message_manager_ = std::make_shared<DdsMessageManager<LidarDetect>>();
      ResigerMessageManager(message.name, dds_message_manager_);

      dds_message_manager_->Init(this);
    } break;
#endif
#if ROS_ENABLE
    case legionclaw::common::MessageType::ROS: {
      AINFO << "message type:ROS";

      ros_message_manager_ = std::make_shared<RosMessageManager<LidarDetect>>();
      ResigerMessageManager(message.name, ros_message_manager_);
      ros_message_manager_->Init(this);
    } break;
#endif
#if ROS2_ENABLE
    case legionclaw::common::MessageType::ROS2: {
      AINFO << "message type:ROS2";

      ros2_message_manager_ =
          std::make_shared<Ros2MessageManager<LidarDetect>>();
      ResigerMessageManager(message.name, ros2_message_manager_);

      ros2_message_manager_->Init(this);
    } break;
#endif

#if ADSFI_ENABLE
    case legionclaw::common::MessageType::ADSFI: {
      AINFO << "message type:ADSFI";

      adsfi_message_manager_ =
          std::make_shared<AdsfiMessageManager<LidarDetect>>();
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

void LidarDetect::FaultMonitorInit() {}

void LidarDetect::StatusDetectOnTimer() {}

void LidarDetect::Spin() {
  while (1) {
    ad_timer_manager_->DetectTimers(NULL);
    usleep(1000);
  }
}
void LidarDetect::PointToBuffer(float* points, legionclaw::interface::PointCloud pointcloud)
{
  float* p = points;
  for(legionclaw::interface::PointXYZIRT it_point : pointcloud.point())
  {
    float x = it_point.x();
    float y = it_point.y();
    float z = it_point.z();
    (*p)=x;
    p++;
    (*p)=y;
    p++;
    (*p)=z;
    p++;
    (*p)=static_cast<float>(it_point.intensity());
   // cout << it_point.intensity() << endl;
    p++;
    (*p)=static_cast<float>(0);
    p++;
  }
  p=nullptr;
}

void LidarDetect::BoxesToObstacleList(std::vector<Bndbox> preboxes, legionclaw::interface::ObstacleList &result_obstacle_list)
{
  std::vector<legionclaw::interface::Obstacle> obstacle_list;
  for(auto box : preboxes)
  {
    legionclaw::interface::Obstacle obstacle;
    obstacle.set_id(-1);
    obstacle.set_timestamp(point_cloud_.header().stamp());
    legionclaw::interface::Point3D obstacle_center;
    obstacle_center.set_x(box.x);
    obstacle_center.set_y(box.y);
    obstacle_center.set_z(box.z);
    obstacle.set_center_pos_vehicle(obstacle_center);
    obstacle.set_length(box.l);
    obstacle.set_width(box.w);
    obstacle.set_height(box.h);
    legionclaw::interface::Point3D obstacle_vel;
    obstacle_vel.set_x(box.vx);
    obstacle_vel.set_y(box.vy);
    obstacle_vel.set_z(0);
    obstacle.set_velocity_vehicle(obstacle_vel);
    obstacle.set_confidence(box.score);
    obstacle.set_confidence_type(legionclaw::interface::Obstacle::ConfidenceType::CONFIDENCE_CNN);
    obstacle.set_fusion_type(legionclaw::interface::Obstacle::FusionType::LIDAR);
    if(box.id == 5 || box.id == 9)
    {
      obstacle.set_theta_vehicle(0);
    }
    else
    {
      obstacle.set_theta_vehicle(-box.rt-3.1415926535*0.5);
    }

    vector<legionclaw::common::ObstacleType> Type{OBSTACLE_UNKNOWN, OBSTACLE_UNKNOWN_MOVABLE, OBSTACLE_UNKNOWN_UNMOVABLE, OBSTACLE_PEDESTRIAN, OBSTACLE_BICYCLE, OBSTACLE_VEHICLE};
    vector<legionclaw::common::ObstacleSubType> Sub_Type{ST_CAR,ST_TRUCK,ST_TRUCK,ST_BUS,ST_TRUCK,ST_UNKNOWN_UNMOVABLE,ST_MOTORCYCLIST,ST_CYCLIST,ST_PEDESTRIAN,ST_TRAFFICCONE};
    switch(box.id)
    {
      case 0:obstacle.set_type(Type[5]);
      case 1:obstacle.set_type(Type[5]);
      case 2:obstacle.set_type(Type[5]);
      case 3:obstacle.set_type(Type[5]);
      case 4:obstacle.set_type(Type[5]);
      case 5:obstacle.set_type(Type[2]);
      case 6:obstacle.set_type(Type[4]);
      case 7:obstacle.set_type(Type[4]);
      case 8:obstacle.set_type(Type[3]);
      case 9:obstacle.set_type(Type[2]);
    }
    obstacle.set_sub_type(Sub_Type[box.id]);
    obstacle.set_existence_prob(box.score);
    obstacle_list.push_back(obstacle);
  }
  result_obstacle_list.set_obstacle(obstacle_list);
}

void LidarDetect::GetPolygon(legionclaw::interface::ObstacleList &result_obstacle_list)
{
  for(auto &ob:result_obstacle_list.obstacle())
  {
    legionclaw::interface::Point3D center_pos = ob.center_pos_vehicle();
    double theta = ob.theta_vehicle();
    double wid = ob.width();
    double len = ob.length();
    double hei = ob.height();
    std::vector<legionclaw::interface::Point3D> poly_list;
    legionclaw::interface::Point3D point1,point2,point3,point4;
    point1.set_x(center_pos.x() + len/2);
    point1.set_y(center_pos.y() - wid/2);
    point1.set_z(center_pos.z() - hei/2);
    point2.set_x(center_pos.x() + len/2);
    point2.set_y(center_pos.y() + wid/2);
    point2.set_z(center_pos.z() - hei/2);
    point3.set_x(center_pos.x() - len/2);
    point3.set_y(center_pos.y() + wid/2);
    point3.set_z(center_pos.z() - hei/2);
    point4.set_x(center_pos.x() - len/2);
    point4.set_y(center_pos.y() - wid/2);
    point4.set_z(center_pos.z() - hei/2); 
    poly_list.push_back(point1);
    poly_list.push_back(point2);
    poly_list.push_back(point3);
    poly_list.push_back(point4);
    ob.set_polygon_point_vehicle(poly_list);
  
    std::vector<legionclaw::interface::Point3D> new_poly_list;
    for(auto poly : ob.polygon_point_vehicle())
    {
      double xx = (poly.x() - center_pos.x())*cos(theta)-(poly.y() - center_pos.y())*sin(theta) + center_pos.x();
      double yy = (poly.x() - center_pos.x())*sin(theta)+(poly.y() - center_pos.y())*cos(theta) + center_pos.y();
      double zz = poly.z() - ob.height()/2;
      poly.set_x(xx);
      poly.set_y(yy);
      new_poly_list.push_back(poly);
    }
    ob.set_polygon_point_vehicle(new_poly_list);
  }
}

} // namespace lidar
} // namespace perception
} // namespace legionclaw
