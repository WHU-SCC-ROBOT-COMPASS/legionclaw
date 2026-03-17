/**
 * @file    control.h
 * @author  jiangchengjie
 * @date    2021-09-27
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iomanip>
#include <mutex>
#include <thread>

#include "modules/common/interface/path.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/perception_obstacle.hpp"
#include "modules/common/interface/obstacle_list.hpp"
#include "modules/common/interface/planning_cmd.hpp"
#include "modules/common/interface/point_2d.hpp"
#include "modules/common/interface/point_3d.hpp"
#include "modules/common/interface/point_cloud.hpp"
#include "modules/common/interface/point_enu.hpp"
#include "modules/common/interface/point_llh.hpp"
#include "modules/common/interface/point_xyzirt.hpp"
#include "modules/common/interface/polygon.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/interface/prediction_out_array.hpp"
#include "modules/common/interface/prediction_trajectory_point.hpp"
#include "modules/common/interface/quaternion.hpp"
#include "modules/common/interface/radar_obstacle.hpp"
#include "modules/common/interface/radar_obstacle_list_msg.hpp"
#include "modules/common/interface/radar_state_error.hpp"
#include "modules/common/interface/radar_state.hpp"
#include "modules/common/interface/radar_state_mode.hpp"
#include "modules/common/interface/road_mark.hpp"
#include "modules/common/interface/routing_request.hpp"
#include "modules/common/interface/routing_response.hpp"
#include "modules/common/interface/rss_info.hpp"
#include "modules/common/interface/sensor_calibrator.hpp"
#include "modules/common/interface/sl_point.hpp"
#include "modules/common/interface/speed_point.hpp"
#include "modules/common/interface/status.hpp"
#include "modules/common/interface/stop_info.hpp"
#include "modules/common/interface/stop_point.hpp"
#include "modules/common/interface/stories.hpp"
#include "modules/common/interface/time.hpp"
#include "modules/common/interface/traffic_light_box.hpp"
#include "modules/common/interface/traffic_light_debug.hpp"
#include "modules/common/interface/traffic_light.hpp"
#include "modules/common/interface/traffic_light_msg.hpp"
#include "modules/common/interface/trajectory.hpp"
#include "modules/common/interface/trajectory_in_prediction.hpp"
#include "modules/common/interface/trajectory_point.hpp"
#include "modules/common/interface/trajectory_point_in_prediction.hpp"
#include "modules/common/interface/ultrasonic.hpp"
#include "modules/common/interface/vehicle_motion.hpp"
#include "modules/common/interface/vehicle_motion_point.hpp"
#include "modules/common/interface/vehicle_signal.hpp"
#include "modules/common/fault/fault_client.hpp"
#include "controller/controller.h"
#include "message_manager/message_manager.h"
#include "modules/common/interface/location.hpp"
#include "modules/common/base_message/message.pb.h"
#include "modules/common/json/json.hpp"
#include "modules/common/logging/logging.h"
#include "modules/common/status/status.h"
#include "modules/common/timer/ad_timer_manager.h"
#include "modules/common/timer/timer_manager.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_tools.h"
#include "modules/common/math/vec3d.h"
#include "modules/control/src/common/local_view.h"
#include "modules/control/src/common/interpolation_1d.h"
#include "modules/control/src/common/control_gflags.h"
#include "modules/common/configs/config_gflags.h"

#if LCM_ENABLE
#include "message_manager/lcm/lcm_message_manager.h"
#endif
#if DDS_ENABLE
#include "message_manager/dds/dds_message_manager.h"
#endif
#if ROS_ENABLE
#include "message_manager/ros/ros_message_manager.h"
#endif
#if ROS2_ENABLE
#include "message_manager/ros2/ros2_message_manager.h"
#endif

#include "controller/controller_agent.h"
#include "modules/control/src/proto/control_conf.pb.h"
/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {
using namespace legionclaw::common;
using namespace legionclaw::common::math;
using json = nlohmann::json;
using Matrix = Eigen::MatrixXd;
/**
 * @class Control
 * @brief 控制类.
 */
class Control {
 public:
  /**
   * @brief 构造函数
   * @param[in]  file_path        配置参数路径
   */
  Control(std::string file_path) : config_file_path_(file_path){};
  /**
   * @brief 析构函数
   */
  ~Control() = default;
  /**
   * @brief 初始化
   */
  void Init();

  /**
   * @brief 绑定主线程
   */
  void Join();

  /**
   * @brief 获取Conf对象
   * @return std::shared_ptr<ControlConf> Conf对象指针
   */
  std::shared_ptr<ControlConf> GetConf() const;

 protected:
  /// @brief 初始化状态
  bool is_init_;
  /// @brief 配置文件路径
  std::string config_file_path_;
  /// @brief 控制逻辑设置
  std::shared_ptr<ControlConf> control_conf_;
  /// @brief 控制器代理
  ControllerAgent controller_agent_;
  /// @brief 消息控制器
  std::map<std::string, std::shared_ptr<MessageManager<Control>>>
      message_manager_;
#if LCM_ENABLE
  std::shared_ptr<LcmMessageManager<Control>> lcm_message_manager_;
#endif
#if DDS_ENABLE
  /// @brief  DDS消息处理器
  std::shared_ptr<DdsMessageManager<Control>> dds_message_manager_;
#endif
#if ROS_ENABLE
  /// @brief  ROS消息处理器
  std::shared_ptr<RosMessageManager<Control>> ros_message_manager_;
#endif
#if ROS2_ENABLE
  std::shared_ptr<Ros2MessageManager<Control>> ros2_message_manager_;
#endif
#if ADSFI_ENABLE
  /// @brief  adsfi消息处理器
  std::shared_ptr<AdsfiMessageManager<Control>> adsfi_message_manager_;
#endif

 protected:
  /**
   * @brief     注册消息控制器.
   * @param[in] message_manager　消息控制器对象指针.
   * @return    void.
   */
  void ResigerMessageManager(
      std::string name,
      std::shared_ptr<MessageManager<Control>> message_manager);

  /**
   * @brief     变量初始化
   *
   */
  void VariableInit();

  /**
   * @brief     消息初始化.
   * @return    void.
   */
  void MessagesInit();

  /**
   * @brief 周期发送控制输出到底盘,以及广播control状态
   * @return void.
   */
  void Task10ms(void *param);

  /**
   * @brief     发送控制命令
   * @param[in]  control_cmd      控制命令
   */
  void PublishControlCommand(legionclaw::interface::ControlCommand control_cmd);

  /**
   * @brief     发送控制诊断分析
   * @param  control_analysis      控制诊断分析
   */
  void PublishControlAnalysis(
      legionclaw::interface::ControlAnalysis control_analysis);

  /**
   * @brief     发送故障码
   * @param  faults      故障
   */
  void PublishFaults();
  /**
   * @brief     设置控制BCM命令.
   * @param    control_cmd 控制命令　planning_cmd　BCM.
   */
  void SetBcmControlCmd(legionclaw::interface::ControlCommand *control_cmd,
                        legionclaw::interface::PlanningCmd planning_cmd);

  /**
   * @brief     遇到障碍物停障
   * @param
  //  */
  // bool StopIfObstacleClose();
  bool StopIfObstacleClose(legionclaw::interface::ObstacleList obstacle_list);

  /**
   * @brief
   * @param  events
   */
  void PublishEvents(legionclaw::interface::Events events);

  /**
   * @brief     打印调试.
   * @return    void.
   */
  void Print();

  /**
   * @brief     日志调试.
   * @return    void.
   */
  void Log();

  void Estop();

 protected:
  /// @brief 车辆类型名
  std::string vehicle_name_;
  /// @brief 控制命令生产周期
  uint32_t produce_control_command_duration_;
  /// @brief 控制命令发送周期
  uint32_t publish_control_command_duration_;

  std::shared_ptr<DependencyInjector> injector_;

  /// @brief 控制命令生产周期
  double ts_;

 public:
  /**
   * @brief     轨迹消息接收,复制接收到的轨迹数据到本地.
   * @param[in] trajectory 规划轨迹.
   * @return    void.
   */
  void HandleADCTrajectory(legionclaw::interface::ADCTrajectory trajectory_msg);

  /**
   * @brief     定位信息接收,复制接收到的定位数据到本地.
   * @param[in] location 定位信息.
   * @return    void.
   */
  void HandleLocation(legionclaw::interface::Location location);

  /**
   * @brief     底盘信息接收,复制接收到的底盘数据到本地
   * @param[in] chassis 车辆底盘信息.
   * @return    void.
   */
  void HandleChassis(legionclaw::interface::Chassis chassis);

  // void HandleVehicle(legionclaw::interface::Vehicle vehicle);

  /**
   * @brief     ObstacleList消息接收.
   * @param[in] obstacle_list .
   * @return    void.
   */
  void HandleObstacleList(legionclaw::interface::ObstacleList obstacle_list);

  /**
   * @brief     Planning cmd信息接收.
   * @param[in] planning_cmd 车辆bcm命令 .
   * @return    void.
   */
  void HandlePlanningCmd(legionclaw::interface::PlanningCmd planning_cmd);

  /**
   * @brief     计算控制算法输出.
   * @return    void.
   */
  void ComputeControlCommandOnTimer();

  /**
   * @brief     检查输入参数
   *
   * @param local_view
   * @return Status
   */
  Status CheckInput(LocalView *local_view);

  /**
   * @brief     检查时间戳
   *
   * @param local_view
   * @return Status
   */
  Status CheckTimestamp(const LocalView &local_view);

  bool SharedAutonomyTransition(const double &steer_target, double &steer_cmd);

 protected:
  // //控制器
  // Controller controller_;

  legionclaw::interface::Events events_;
  /// @brief 轨迹初始化
  bool trajectory_initialized_;
  /// @brief 轨迹接收锁
  std::mutex trajectory_rx_mutex_;
  /// @brief 轨迹索引
  volatile uint8_t trajectory_switch_index_;
  /// @brief 轨迹
  legionclaw::interface::ADCTrajectory adc_trajectory_;
  /// @brief 定位
  legionclaw::interface::LocalizationEstimate localization_;
  /// @brief 底盘信息
  legionclaw::interface::Chassis chassis_;
  // /// @brief 车辆信息
  // legionclaw::interface::Vehicle vehicle_;
  /// @brief 障碍物信息
  legionclaw::interface::ObstacleList obstacle_list_;
  /// @brief 控制命令
  legionclaw::interface::ControlCommand control_cmd_;
  /// @brief  BCM控制命令
  legionclaw::interface::PlanningCmd planning_cmd_;
  /// @brief 控制分析数据
  legionclaw::interface::ControlAnalysis control_analysis_;
  /// @brief 故障码
  legionclaw::interface::Faults faults_;
  legionclaw::interface::FaultCodeSet *faultcodeset_;
  /// @brief 驾驶模式
  int32_t driving_mode_;
  /// @brief  task线程
  std::unique_ptr<std::thread> task_thread_;
  /// @brief debug调试定位
  legionclaw::interface::Location localization_debug_;

  bool estop_ = false;
  std::string estop_reason_;
  std::mutex mutex_;
  LocalView local_view_;
  /// @brief 人工切自动过渡状态标志
  bool shared_autonomy_;
  bool enable_shared_autonomy_;
  double steer_angle_rate_;
  double steer_angle_tolerance_;
  /// @brief 人工进入自动瞬间转角反馈
  double pre_steer_cmd_;

  double stop_obs_false_timestamp_;

 protected:
  /// @brief 定时器
  std::shared_ptr<ADTimerManager<Control, void>> ad_timer_manager_;
  /// @brief 定时器
  std::shared_ptr<WheelTimer<Control, void>> task_10ms_;
  std::unique_ptr<Interpolation1D> steer_rate_speed_interpolation_;
  /**
   * @brief 循环
   */
  void Spin();
};
}  // namespace control
}  // namespace legionclaw
