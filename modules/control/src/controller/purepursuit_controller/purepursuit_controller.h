/**
 * @file              purepursuit_controller.h
 * @author            yanglanjiang (yanglanjiang@indrv.cn)
 * @brief
 * @version           1.0.0
 * @date              2023-04-15 04:27:43
 * @copyright         Copyright (c) 2023
 * @license           GNU General Public License (GPL)
 */

#ifndef CONTROLLER_PURSUIT_LAT_LATCONTROLLER_H_
#define CONTROLLER_PURSUIT_LAT_LATCONTROLLER_H_

#include <Eigen/Core>
#include <fstream>
#include <memory>

#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/trajectory_analyzer/trajectory_analyzer.h"
#include "controller/controller.h"
#include "controller/pid/pid_controller.h"
#include "modules/control/src/proto/control_conf.pb.h"
#include "simple_lateral_debug.hpp"

using namespace legionclaw::common;
using namespace legionclaw::control;
using legionclaw::interface::VehicleParam;

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {
/**
 * @class PurePursuitController
 *
 * @brief PurePursuitController.
 */
class PurePursuitController : public Controller {
 public:
  /**
   * @brief 构造函数
   */
  PurePursuitController();

  /**
   * @brief 析构函数
   */
  ~PurePursuitController();

  /**
   * @brief  初始化
   * @param[in]  injector         车辆状态接口
   * @param[in]  control_conf     配置文件
   * @return legionclaw::common::Status
   */
  legionclaw::common::Status Init(std::shared_ptr<DependencyInjector> injector,
                              const ControlConf *control_conf) override;

  /**
   * @brief  根据当前车辆状态计算控制命令
   * @param[in]  localization     定位信息
   * @param[in]  chassis          底盘信息
   * @param[in]  trajectory       轨迹
   * @param[out]  cmd              控制命令
   * @param[out]  analysis         控制数据分析
   * @return legionclaw::common::Status
   */
  legionclaw::common::Status ComputeControlCommand(
      const legionclaw::interface::LocalizationEstimate *localization,
      const legionclaw::interface::Chassis *chassis,
      const legionclaw::interface::ADCTrajectory *trajectory,
      legionclaw::interface::ControlCommand *cmd,
      legionclaw::interface::ControlAnalysis *analysis) override;

  /**
   * @brief 重置Pure Pursuit控制器
   * @return legionclaw::common::Status
   */
  legionclaw::common::Status Reset() override;

  /**
   * @brief 关闭Pure Pursuit控制器
   */
  void Stop() override;

  /**
   * @brief Pure Pursuit控制器名字
   * @return std::string 控制器名字
   */
  std::string Name() const override;

 protected:
  /// @brief 配置文件
  const ControlConf *control_conf_ = nullptr;
  /// @brief 转向模式（STEER_MODE_2WS、STEER_MODE_4WS）
  VehicleParam::SteerMode steer_mode_;
  /// @brief 日志记录开关
  bool enable_log_debug_ = false;
  /// @brief 转向日志文件
  std::ofstream steer_log_file_;
  /// @brief 车辆参数
  VehicleParam vehicle_param_;
  /// @brief 名字
  const std::string name_;
  /// @brief 车辆状态接口
  std::shared_ptr<DependencyInjector> injector_;
  /// @brief 计算周期
  double ts_ = 0.01;
  /// @brief 前轴距
  double lf_ = 0.0;
  /// @brief 后轴距
  double lr_ = 0.0;
  /// @brief 轴距
  double wheelbase_ = 0.0;
  /// @brief 转向比
  double steer_ratio_ = 0.0;
  /// @brief 前轮最大转向
  double front_steer_single_direction_max_degree_ = 0.0;
  /// @brief 前轮最小转向
  double front_steer_single_direction_min_degree_ = 0.0;
  /// @brief 后轮最大转向
  double rear_steer_single_direction_max_degree_ = 0.0;
  /// @brief 后轮最小转向
  double rear_steer_single_direction_min_degree_ = 0.0;
  /// @brief 最大横向加速度
  double max_lat_acc_ = 0.0;
  /// @brief 是否设置转向限制
  bool set_steer_limit_ = false;
  /// @brief 是否设置转向速率限制
  bool enable_maximum_steer_rate_limit_ = false;
  /// @brief 前轮最大转向速率限制下，单次最大转角变化
  double front_steer_diff_with_max_rate_ = 0.0;
  /// @brief 后轮最大转向速率限制下，单次最大转角变化
  double rear_steer_diff_with_max_rate_ = 0.0;
  /// @brief 最大转角变化率
  double max_front_steer_angle_rate_ = 400.0;
  /// @brief 前轮方向盘转角上次计算结果
  double pre_front_steer_angle_ = 0.0;
  /// @brief 后轮转角上次计算结果
  double pre_rear_steer_angle_ = 0.0;

  ///
  double lat_error_threshold_;
  /// @brief 航向角误差阈值（度），用于航向误差标准化和归一化
  double heading_error_threshold_ = 17.19;  // 默认值约 0.3 弧度 (≈17.19度)
  /// @brief 横向偏差最小值阈值（度），当横向偏差角度小于此值时，贡献值清零
  double lat_error_min_threshold_ = 0.0;
  /// @brief 航向角偏差最小值阈值（度），当航向角偏差小于此值时，贡献值清零
  double heading_error_min_threshold_ = 0.0;
  /// @brief 横向误差使能
  bool enable_lat_error_;
  /// @brief 横向误差pid控制器
  PIDController lat_error_pid_controller_;
  /// @brief 航向角误差使能
  bool enable_heading_error_;
  /// @brief 航向误差pid控制器
  PIDController heading_error_pid_controller_;
  /// @brief 航向角误差权重
  double heading_error_gain_;
  /// @brief 横向误差权重
  double lat_error_gain_;
  /// @brief 航向误差pid反馈
  double heading_pid_feedback_ = 0.0;
  /// @brief 前轮方向盘转角滤波器
  DigitalFilter digital_filter_f;
  /// @brief 后轮方向盘转角滤波器
  DigitalFilter digital_filter_r;

  /// @brief 当前定位x
  double current_x_;
  /// @brief 当前定位y
  double current_y_;

  /// @brief 日志索引
  int64_t log_index_ = 0;
  /// @brief 计算时间
  int64_t calculate_time_ = 0;
  /// @brief 最大计算时间
  int64_t calculate_time_max_ = 0;
  /// @brief 横向诊断调试信息
  std::shared_ptr<SimpleLateralDebug> simple_lateral_debug_;
  /// @brief 是否使用导航模式
  bool use_navigation_mode_ = true;
  /// @brief 查询相对时间
  double query_relative_time_;
  /// @brief 是否只查询时间最接近的点
  bool query_time_nearest_point_only_ = false;
  /// @brief 是否只查询未来时间的点
  bool query_forward_time_point_only_ = false;
  /// @brief 是否激活导航模式位置更新
  bool enable_navigation_mode_position_update_ = true;
  /// @brief 前向预测时间
  double lookahead_station_ = 0.0;
  /// @brief 后向预测时间
  double lookback_station_ = 0.0;
  /// @brief 前一次计算横向加速度
  double previous_lateral_acceleration_ = 0.0;
  /// @brief 前一次计算航向变化率
  double previous_heading_rate_ = 0.0;
  /// @brief 前一次计算参考航向变化率
  double previous_ref_heading_rate_ = 0.0;
  /// @brief 前一次计算航向加速度
  double previous_heading_acceleration_ = 0.0;
  /// @brief 前一次计算参考航向加速度
  double previous_ref_heading_acceleration_ = 0.0;

  // purepursuit配置参数

  /// @brief 是否使用延迟补偿
  bool use_delay_compensation_ = false;
  /// @brief 是否使用插值法查找目标点
  bool use_interpolate_lookahead_point_ = false;
  /// @brief 最小前视距离（用于目标点匹配）
  double minimum_lookahead_distance_ = 0.0;
  /// @brief 最大前视距离（用于目标点匹配）
  double maximum_lookahead_distance_ = 0.0;
  /// @brief 速度距离转化系数（用于目标点匹配）
  double speed_to_lookahead_ratio_ = 0.0;
  /// @brief Pure Pursuit算法最小预瞄距离（米）
  double purepursuit_minimum_lookahead_distance_ = 0.0;
  /// @brief Pure Pursuit算法最大预瞄距离（米）
  double purepursuit_maximum_lookahead_distance_ = 0.0;
  /// @brief Pure Pursuit算法预瞄距离计算的速度参数
  double purepursuit_speed_to_lookahead_ratio_ = 0.0;
  /// @brief 目标点预瞄距离（用于目标点匹配）
  double lookahead_distance_ = 0.0;
  /// @brief Pure Pursuit算法预瞄距离
  double purepursuit_lookahead_distance_ = 0.0;
  /// @brief 是否使用滤波器
  bool enable_digital_filter_ = false;

  /// @brief 规划轨迹分析器
  TrajectoryAnalyzer trajectory_analyzer_;
  // bool trajectory_transform_to_com_reverse_ = true;
  // bool trajectory_transform_to_com_drive_ = true;
  // bool enable_look_ahead_back_control_;

  /// @brief 方向盘回正步骤
  uint32_t steer_back_step_ = 0;

 private:
  /**
   * @brief 记录初始化参数
   */
  void LogInitParameters();
  /**
   * @brief 导入Pure Pursuit配置参数
   * @param[in]  control_conf     配置文件
   * @return true 成功
   * @return false 失败
   */
  bool LoadPurePursuitControlConf(const ControlConf *control_conf);
  /**
   * @brief 初始化滤波器
   * @param[in]  control_conf     配置文件
   */
  void InitializeFilters(const ControlConf *control_conf);

  /**
   * @brief 关闭日志文件
   */
  void CloseLogFile();
  /**
   * @brief 处理日志信息并写入文件
   * @param[in]  cmd              控制命令
   * @param[in]  debug            诊断调试信息
   * @param[in]  chassis          底盘信息
   */
  void ProcessLogs(const legionclaw::interface::ControlCommand *cmd,
                   const SimpleLateralDebug *debug,
                   const legionclaw::interface::Chassis *chassis);
  /**
   * @brief 控制分析消息赋值
   * @param[in]  cmd              控制命令
   * @param[in]  debug            诊断调试信息
   * @param[in]  chassis          底盘消息
   * @param[out]  analysis         控制分析消息
   */
  void SetAnalysis(const legionclaw::interface::ControlCommand *cmd,
                   const SimpleLateralDebug *debug,
                   const legionclaw::interface::Chassis *chassis,
                   legionclaw::interface::ControlAnalysis *analysis);
  /**
   * @brief 计算横向误差
   * @param[in]  x                定位x坐标
   * @param[in]  y                定位y坐标
   * @param[in]  theta            定位航向角
   * @param[in]  linear_v         线速度
   * @param[in]  angular_v        角速度
   * @param[in]  linear_a         线性加速度
   * @param[in]  target_point     目标点（来自ComputeTargetPoint）
   * @param[in]  trajectory_analyzer 规划轨迹分析器
   * @param[out]  debug            诊断调试信息
   * @param[in]  chassis          底盘消息
   */
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const legionclaw::interface::TrajectoryPoint &target_point,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug,
                            const legionclaw::interface::Chassis *chassis);
  /**
   * @brief 计算前视距离
   */
  void ComputeLookAheadDistance();
  /**
   * @brief 目标点信息结构体（包含坐标和曲率）
   */
  struct TargetPointInfo {
    legionclaw::common::math::Vec2d position;  // 目标点坐标
    double curvature;                      // 目标点曲率
  };

  /**
   * @brief 计算目标点
   * @param[in]  pos              定位x,y信息
   * @param[in]  theta            定位航向角
   * @param[in]  s                预瞄距离（米），用于计算目标点的前视距离
   * @param[in]  trajectory       轨迹信息
   * @return TrajectoryPoint 完整的目标点信息
   */
  legionclaw::interface::TrajectoryPoint ComputeTargetPoint(
      const legionclaw::common::math::Vec2d pos, const double theta, const double s,
      const legionclaw::interface::ADCTrajectory *trajectory);
  /**
   * @brief 计算两点距离
   * @param[in]  pos1             1点坐标x,y
   * @param[in]  pos2             2点坐标x,y
   * @return double 距离
   */
  double ComputePointsDistanceSquared(const legionclaw::common::math::Vec2d pos1,
                                      const legionclaw::common::math::Vec2d pos2);
  /**
   * @brief 计算2点相对1点，x、y位移距离
   * @param[in]  pos1             1点坐标
   * @param[in]  pos2             2点坐标
   * @return legionclaw::common::math::Vec2d x、y位移距离
   */
  legionclaw::common::math::Vec2d ComputeRelativeXYOffset(
      const legionclaw::common::math::Vec2d pos1,
      const legionclaw::common::math::Vec2d pos2);
  /**
   * @brief 获取日志文件名
   * @return std::string 文件名
   */
  std::string GetLogFileName();
  /**
   * @brief 写入表头
   * @param[in]  file_stream      日志文件
   */
  void WriteHeaders(std::ofstream &file_stream);
};
}  // namespace control
}  // namespace legionclaw

#endif
