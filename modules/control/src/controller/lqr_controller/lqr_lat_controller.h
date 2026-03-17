/**
 * @file lqr_lat_controller.h
 * @author jiang <jiangchengjie@indrv.cn>
 * @date  2018-07-07
 * @version 1.0.0
 * @par  Copyright(c)
 *        hy
 */

#ifndef CONTROLLER_LQR_LAT_LATCONTROLLER_H_
#define CONTROLLER_LQR_LAT_LATCONTROLLER_H_

#include <Eigen/Core>
#include <fstream>
#include <memory>

#include "modules/control/src/common/filters/digital_filter.h"
#include "modules/control/src/common/filters/digital_filter_coefficients.h"
#include "modules/control/src/common/filters/mean_filter.h"
#include "modules/control/src/common/interpolation_2d.h"
#include "modules/control/src/common/trajectory_analyzer/trajectory_analyzer.h"
#include "modules/control/src/controller/controller.h"
#include "modules/control/src/controller/leadlag_controller/leadlag_controller.h"
#include "modules/control/src/controller/mrac_controller/mrac_controller.h"
#include "modules/control/src/common/interpolation_1d.h"
#include "modules/control/src/proto/control_conf.pb.h"
#include "simple_lateral_debug.hpp"
using namespace std;

using namespace legionclaw::common;
using namespace legionclaw::control;
using legionclaw::interface::VehicleParam;
using Matrix = Eigen::MatrixXd;

/**
 * @namespace legionclaw::control
 * @brief legionclaw::control
 */
namespace legionclaw {
namespace control {
/**
 * @class LQRLatController
 *
 * @brief LQRLatController.
 */
class LQRLatController : public Controller {
 public:
  /**
   * @brief 构造函数
   */
  LQRLatController();

  /**
   * @brief 析构函数
   */
  ~LQRLatController();

  /**
   * @brief 初始化.
   * @param[in] control_conf control配置参数
   * @return Status 初始化状态
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
   * @brief 重置lqr controller
   * @return legionclaw::common::Status 
   */
  legionclaw::common::Status Reset() override;

  /**
   * @brief 停止lqr controller
   */
  void Stop() override;

  /**
   * @brief lqr controller名字
   * @return std::string 控制器名字
   */
  std::string Name() const override;

 protected:
  /// @brief 配置文件
  const ControlConf *control_conf_ = nullptr;
  VehicleParam::SteerMode steer_mode_;
  /// @brief 日志记录开关
  bool enable_log_debug_;
  /// @brief 打印开关
  bool lqr_print_enable_;
  /// @brief 转向日志文件
  std::ofstream steer_log_file_;
  /// @brief 车辆参数
  VehicleParam vehicle_param_;
  /// @brief  名字
  const std::string name_;

  // the following parameters are vehicle physics related.
  /// @brief  计算周期
  double ts_ = 0.01;
  /// @brief  前轮侧偏刚度
  double cf_ = 0.0;
  /// @brief  后轮侧偏刚度
  double cr_ = 0.0;
  /// @brief  轴距
  double wheelbase_ = 0.0;
  /// @brief  质量
  double mass_ = 0.0;
  /// @brief  前轴距
  double lf_ = 0.0;
  /// @brief  后轴距
  double lr_ = 0.0;
  /// @brief  转动惯量
  double iz_ = 0.0;
  /// @brief  方向盘转角与车轮转角系数
  double steer_ratio_ = 0.0;
  /// @brief  前轮转向最大值
  double front_steer_single_direction_max_degree_ = 0.0;
  /// @brief  前轮转向最小值
  double front_steer_single_direction_min_degree_ = 0.0;
  /// @brief  后轮转向最大值
  double rear_steer_single_direction_max_degree_ = 0.0;
  /// @brief  后轮转向最小值
  double rear_steer_single_direction_min_degree_ = 0.0;

  /// @brief  最大横向加速度
  double max_lat_acc_ = 0.0;

  /// @brief  预览窗口大小
  int preview_window_ = 0;
  /// @brief  没有预览的状态数，包括横向误差、横向错误率、航向误差、航向错误率
  int basic_state_size_ = 4;
  /// @brief  车辆状态矩阵
  Eigen::MatrixXd matrix_a_;
  /// @brief  车辆状态矩阵（离散时间）
  Eigen::MatrixXd matrix_ad_;
  /// @brief  车辆状态复合矩阵
  Eigen::MatrixXd matrix_adc_;
  /// @brief  控制矩阵
  Eigen::MatrixXd matrix_b_;
  /// @brief  控制矩阵（离散时间）
  Eigen::MatrixXd matrix_bd_;
  /// @brief  控制复合矩阵
  Eigen::MatrixXd matrix_bdc_;
  /// @brief  增益矩阵
  Eigen::MatrixXd matrix_k_;
  /// @brief  R矩阵
  Eigen::MatrixXd matrix_r_;
  /// @brief  q矩阵
  Eigen::MatrixXd matrix_q_;
  /// @brief  更新的q矩阵
  Eigen::MatrixXd matrix_q_updated_;
  /// @brief  车辆状态矩阵系数
  Eigen::MatrixXd matrix_a_coeff_;
  /// @brief  状态矩阵
  Eigen::MatrixXd matrix_state_;

  /// @brief  lqr迭代次数
  int lqr_max_iteration_ = 0;
  /// @brief  lqr求解的数值容差
  double lqr_eps_ = 0.0;

  /// @brief 滤波器
  DigitalFilter digital_filter_;
  /// @brief 横向误差插值器，对应速度
  std::unique_ptr<Interpolation1D> lat_err_interpolation_;
  /// @brief 航向误差插值器，对应速度
  std::unique_ptr<Interpolation1D> heading_err_interpolation_;
  /// @brief 横向误差插值器，对应误差大小
  std::unique_ptr<Interpolation1D> lat_err_apc_interpolation_;
  /// @brief 航向误差插值器，对应误差大小
  std::unique_ptr<Interpolation1D> heading_err_apc_interpolation_;
  /// @brief 横向速度插值器，对应速度
  std::unique_ptr<Interpolation1D> lat_speed_kp_interpolation_;
  /// @brief k矩阵插值器，对应速度
  std::vector<std::unique_ptr<Interpolation1D>> lqr_k_interpolation_vector;

  LeadlagConf reverse_leadlag_conf_;
  /// @brief 超前滞后控制器
  LeadlagController leadlag_controller_;

  /// @brief 横向误差滤波器;
  MeanFilter lateral_error_filter_;
  /// @brief 航向误差滤波器
  MeanFilter heading_error_filter_;

  /// @brief MRAC控制器
  MracController marc_controller_;

  /// @brief 上一次前轮转角
  double pre_front_steer_angle_ = 0.0;
  /// @brief 上一次后轮转角
  double pre_rear_steer_angle_ = 0.0;
  /// @brief 最小速度保护
  double minimum_speed_protection_ = 0.1;
  /// @brief 当前轨迹时间戳
  double current_trajectory_timestamp_ = -1.0;

  //double init_vehicle_x_ = 0.0;
  //double init_vehicle_y_ = 0.0;
  //double init_vehicle_heading_ = 0.0;

  // for compute the differential valute to estimate acceleration/lon_jerk
  /// @brief 上一次横向加速度
  double previous_lateral_acceleration_ = 0.0;
  /// @brief 前向预测时间
  double lookahead_station_ = 0.0;
  /// @brief 后向预测时间
  double lookback_station_ = 0.0;
  /// @brief 前一次计算航向变化率
  double previous_heading_rate_ = 0.0;
  /// @brief 前一次计算参考航向变化率
  double previous_ref_heading_rate_ = 0.0;
  /// @brief 前一次计算航向加速度
  double previous_heading_acceleration_ = 0.0;
  /// @brief 前一次计算参考航向加速度
  double previous_ref_heading_acceleration_ = 0.0;
  /// @brief 向前预览轨迹点个数
  int32_t preview_num_;

  double low_speed_bound_ = 0.0;
  /// @brief 查询相对时间
  double query_relative_time_;
  /// @brief 行驶方向
  double driving_orientation_ = 0.0;

  /// @brief 车辆状态接口
  std::shared_ptr<DependencyInjector> injector_;

  /// @brief 是否使用新规划模式
  bool use_new_planning_mode_ = false;
  /// @brief 仿状态机变量
  uint8_t state_machine = 0;
  /// @brief 后轴距添加值
  double add_lr_ = 0.0;

  // Mrac controller
  MracController mrac_controller_;
  /// @brief 是否使用导航模式
  bool use_navigation_mode_ = true;
  /// @brief 是否只查询时间最接近的点
  bool query_time_nearest_point_only_ = false;
  /// @brief 是否只查询未来时间的点
  bool query_forward_time_point_only_ = false;
  /// @brief 规划轨迹分析器
  TrajectoryAnalyzer trajectory_analyzer_;
  /// @brief 是否激活导航模式位置更新
  bool enable_navigation_mode_position_update_ = true;
  /// @brief 是否激活导航模式误差滤波器
  bool enable_navigation_mode_error_filter_ = false;
  /// @brief 倒车航向是否反向
  bool reverse_heading_control_ = false;
  /// @brief 倒车轨迹是否转换到质心
  bool trajectory_transform_to_com_reverse_ = true;
  /// @brief 前进轨迹是否转换到质心
  bool trajectory_transform_to_com_drive_ = true;
  /// @brief 是否激活增益表
  bool enable_gain_scheduler_ = true;
  bool enable_feedback_augment_on_high_speed_ = false;
  /// @brief 是否设置转向限制
  bool set_steer_limit_ = true;
  /// @brief 是否设置转向速率限制
  bool enable_maximum_steer_rate_limit_ = false;
  double lock_steer_speed_ = 0.081;
  double pre_front_steering_position_ = 0.0;
  /// @brief 前轮转向速率
  double front_steer_angle_rate_ = 200;
  double pre_rear_steering_position_ = 0.0;
  /// @brief 后轮转向速率
  double rear_steer_angle_rate_ = 200;
  bool enable_look_ahead_back_control_;
  /// @brief 日志索引
  int64_t log_index_ = 0;
  /// @brief lqr计算时间
  int64_t lqr_calculate_time_ = 0;
  /// @brief lqr计算时间最大值
  int64_t lqr_calculate_time_max_ = 0;
  /// @brief 当前定位x
  double current_x_;
  /// @brief 当前定位y
  double current_y_;
  bool enable_error_sigmoid_control_ = false;
  double lat_error_axis_;
  double lat_error_slope_;
  double heading_error_axis_;
  double heading_error_slope_;
  /// @brief 是否使用误差自适应调节
  bool enable_error_apc_scheduler_ = false;
  /// @brief 横向误差阈值
  double lateral_error_threshold_;
  /// @brief 航向误差阈值
  double heading_error_threshold_;
  /// @brief q矩阵横向误差调节最大系数
  double q_lat_err_max_;
  /// @brief q矩阵航向误差调节最大系数
  double q_heading_err_max_;
  /// @brief lqr matrix_k 标定表开关
  bool use_lqr_calibration_table_ = false;
  /// @brief 方向盘回正步骤
  uint32_t steer_back_step_ = 0;

 private:
  /**
   * @brief 初始化滤波器
   * @param[in]  control_conf     配置参数
   */
  void InitializeFilters(const ControlConf *control_conf);
  /**
   * @brief 加载横向参数增益表
   * @param[in]  lqr_controller_conf 配置参数
   */
  void LoadLatGainScheduler(const LQRControllerConf &lqr_controller_conf);
  /**
   * @brief 记录初始化参数
   */
  void LogInitParameters();
  /**
   * @brief 计算横向误差
   * @param[in]  x                定位x坐标
   * @param[in]  y                定位y坐标
   * @param[in]  theta            定位航向角
   * @param[in]  linear_v         线速度
   * @param[in]  angular_v        角速度
   * @param[in]  linear_a         线性加速度
   * @param[in]  trajectory_analyzer 规划轨迹分析器
   * @param[out]  debug            诊断调试信息
   * @param[in]  chassis          底盘消息
   */
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug,
                            const legionclaw::interface::Chassis *chassis);
  /**
   * @brief 更新驾驶方向
   * @param[in]  localization     定位消息
   * @param[in]  chassis          底盘消息
   */
  void UpdateDrivingOrientation(
      const legionclaw::interface::LocalizationEstimate *localization,
      const legionclaw::interface::Chassis *chassis);
  /**
   * @brief 导入lqr控制器配置参数
   * @param[in]  control_conf     配置文件
   * @return true 成功
   * @return false 失败
   */
  bool LoadLQRControlConf(const ControlConf *control_conf);
  /**
   * @brief 导入lqr标定表
   * @param[in]  lqr_calibration_table 标定表配置参数
   */
  void LoadLqrCalibrationTable(
      const LqrCalibrationTable &lqr_calibration_table);
  /**
   * @brief 更新adc、bdc矩阵
   */
  void UpdateMatrixCompound();
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
   * @brief 更新a、ad矩阵
   * @param[in]  chassis          底盘信息
   */
  void UpdateMatrix(const legionclaw::interface::Chassis *chassis);
  /**
   * @brief 计算前馈
   * @param[in]  ref_curvature    参考曲率
   * @param[in]  chassis          底盘信息
   * @return double 
   */
  double ComputeFeedForward(double ref_curvature,
                            const legionclaw::interface::Chassis *chassis) const;
  /**
   * @brief 更新车辆状态
   * @param[in]  trajectory       轨迹信息
   * @param[in]  localization     定位信息
   * @param[in]  chassis          底盘信息
   * @param[out]  debug            诊断调试信息
   */
  void UpdateState(const legionclaw::interface::ADCTrajectory *trajectory,
                   const legionclaw::interface::LocalizationEstimate *localization,
                   const legionclaw::interface::Chassis *chassis,
                   SimpleLateralDebug *debug);
  /**
   * @brief 从lqr标定表更新k矩阵
   */
  void UpdateMatrixKFromCalibration();
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
  /// @brief 横向诊断调试信息
  std::shared_ptr<SimpleLateralDebug> simple_lateral_debug_;
};
}  // namespace control
}  // namespace legionclaw

#endif
