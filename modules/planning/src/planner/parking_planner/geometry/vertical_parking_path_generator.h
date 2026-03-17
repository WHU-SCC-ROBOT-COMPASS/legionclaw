/**
 * @file
 *
 * @brief 垂直泊车路径生成
 */
#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include "modules/common/math/math_tools.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "parking_path_generator.h"

using namespace std;
namespace legionclaw {
namespace planning {
/**
 * @class VerticalParkingPathGenerator
 * @brief 生成泊车路径信息（基于泊车坐标系）
 */
class VerticalParkingPathGenerator : public ParkingPathGenerator {
 public:
  /**
   * @brief 构造函数
   */
  VerticalParkingPathGenerator() = default;
  /**
   * @brief 构造函数
   * @param parking_spot 输入量：泊车位信息。
   */
  VerticalParkingPathGenerator(const PlanningConf *planning_conf);
  /**
   * @brief 析构函数
   */
  ~VerticalParkingPathGenerator() = default;

  /**
   * @brief 计算进入车库的深度
   * @param
   * @return 1设置成功，0不成功。
   */
  void ComputeInDistance(const double &turning_radius, const double &safe_distance,
                         const double &vehicle_width, const double &parking_width);

  /**
   * @brief 泊车坐标系下，计算第一次前进的泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @return 1设置成功，0不成功。
   */
  Status ComputePathInStage1Move0_1(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move0_2(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage1Move0_4(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move1_1(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move1_2(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage1Move1_4(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move2_1(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move2_2(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage1Move2_4(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move3(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move4(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage1Move3_3(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage1Move4_3(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage1Move3_5(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputePathInStage3Move1(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage3Move2(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage3Move3(const legionclaw::interface::PathPoint &cur_pose, int director_pose);
  Status ComputePathInStage3Move4(const legionclaw::interface::PathPoint &cur_pose);
  Status ComputePathInStage3Move5(const legionclaw::interface::PathPoint &cur_pose);

  Status ComputeForwardPathInStage1(const legionclaw::interface::PathPoint &cur_pose) override;

  /**
   * @brief 泊车坐标系下，计算倒车泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @param spiral_order 输入量：螺旋曲线次数。
   * @return 1设置成功，0不成功。
   */
  Status ComputeBackwardPathInStage1(const legionclaw::interface::PathPoint &cur_pos) override;

  /**
   * @brief 确定停车位附近的可行驶区域（计算road_polygon_）
   */
  bool ConstructFreeSpace() override;

  Status ComputeForwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) override;

  Status ComputeBackwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) override;

  void SetParkingSpot(const legionclaw::interface::ParkingInfo &parking_spot) override;

  int get_parking_position();
  double steer_value_;

 protected:
  double theta_1, theta_2, theta_t, theta_4, theta_4_1, forward_distance, R, R1, l, d, d1, d2, d3,
      L, d4;
  double R3, theta_3, theta_t1;
  // theta_1,theta_2,theta_t,theta_4  move1 -> move4角度
  // forward_distance，move3前迁距离
  // R1，move1半径 ； R，move2计算半径
  // l move1结束，x方向与原点距离（车库中心线）
  // d 车道宽
  // d1 move1圆弧转动，y方向距离
  // d2 车圆弧转动，与后轴车道上沿距离
  // d3 车初始姿态，与车库上沿距离
  // L 车后轴到车头距离
  // d4 泊车方法2 move2圆弧转动，y方向距离
  double out_theta1, out_theta2, out_theta3, out_d, origin_to_up_d, backword_distance, t_theta;

  int parking_position_;  // 泊车位方位（在车辆的右侧（1）或左侧（-1））
  double angle_offset_;
  double upward_distance;
  legionclaw::interface::PathPoint local_vehicle_current_pose_;
  VerticalParkingConf vertical_parking_conf_;  // 泊车配置参数
  legionclaw::interface::VehicleParam vehicle_param_;
  // TODO 变量需要初始化
  legionclaw::interface::PathPoint stage1_f_a0_;
  legionclaw::interface::PathPoint stage1_f_a1_;
  legionclaw::interface::PathPoint stage1_b_a0_;
  legionclaw::interface::PathPoint stage1_b_a1_;
  legionclaw::interface::PathPoint stage1_b_a2_;
  legionclaw::interface::PathPoint stage1_b_a3_;
  legionclaw::interface::PathPoint stage2_f_a0_;
  legionclaw::interface::PathPoint stage2_f_a1_;
  legionclaw::interface::PathPoint stage2_b_a0_;
  legionclaw::interface::PathPoint stage2_b_a1_;
  legionclaw::interface::PathPoint stage1_m0_a0_;
  legionclaw::interface::PathPoint stage1_m0_a1_;

  legionclaw::interface::PathPoint stage1_m1_a0_;
  legionclaw::interface::PathPoint stage1_m1_a1_;
  legionclaw::interface::PathPoint stage1_m1_a2_;

  legionclaw::interface::PathPoint stage1_m2_a0_;
  legionclaw::interface::PathPoint stage1_m2_a1_;
  legionclaw::interface::PathPoint stage1_m2_a2_;
  legionclaw::interface::PathPoint stage1_m2_a3_;

  legionclaw::interface::PathPoint stage1_m3_a0_;
  legionclaw::interface::PathPoint stage1_m3_a1_;

  legionclaw::interface::PathPoint stage1_m4_a0_;
  legionclaw::interface::PathPoint stage1_m4_a1_;
  legionclaw::interface::PathPoint stage1_m4_a2_;
  legionclaw::interface::PathPoint stage1_m4_a3_;

  legionclaw::interface::PathPoint stage1_m3_3_a0_;
  legionclaw::interface::PathPoint stage1_m3_3_a1_;

  legionclaw::interface::PathPoint stage1_m4_3_a0_;
  legionclaw::interface::PathPoint stage1_m4_3_a1_;
  legionclaw::interface::PathPoint stage1_m4_3_a2_;
  legionclaw::interface::PathPoint stage1_m4_3_a3_;

  legionclaw::interface::PathPoint stage3_m1_a0_;
  legionclaw::interface::PathPoint stage3_m1_a1_;

  legionclaw::interface::PathPoint stage3_m2_a0_;
  legionclaw::interface::PathPoint stage3_m2_a1_;

  legionclaw::interface::PathPoint stage3_m3_a0_;
  legionclaw::interface::PathPoint stage3_m3_a1_;
  legionclaw::interface::PathPoint stage3_m3_a2_;

  legionclaw::interface::PathPoint stage3_m4_a0_;
  legionclaw::interface::PathPoint stage3_m4_a1_;

  legionclaw::interface::PathPoint stage3_m5_a0_;
  legionclaw::interface::PathPoint stage3_m5_a1_;
  legionclaw::interface::PathPoint stage3_m5_a2_;

  void UpdateStage1Move0_1ControlPoints(int parking_position);
  void UpdateStage1Move0_2ControlPoints(int parking_position);

  void UpdateStage1Move0_4ControlPoints(int parking_position);
  void UpdateStage1Move1_1ControlPoints(int parking_position);
  void UpdateStage1Move1_2ControlPoints(int parking_position);

  void UpdateStage1Move1_4ControlPoints(int parking_position);
  void UpdateStage1Move2_1ControlPoints(int parking_position);
  void UpdateStage1Move2_2ControlPoints(int parking_position);

  void UpdateStage1Move2_4ControlPoints(int parking_position);
  void UpdateStage1Move3ControlPoints(int parking_position);
  void UpdateStage1Move4ControlPoints(int parking_position);

  void UpdateStage1Move3_3ControlPoints(int parking_position);
  void UpdateStage1Move4_3ControlPoints(int parking_position);

  void UpdateStage1Move3_5ControlPoints(int parking_position);

  void UpdateStage1ForwardControlPoints(int parking_position);
  void UpdateStage1BackwardControlPoints(int parking_position);
  void UpdateStage2ForwardControlPoints(int parking_position);
  void UpdateStage2BackwardControlPoints(int parking_position);

  void UpdateStage3Move1ControlPoints(int parking_position);
  void UpdateStage3Move2ControlPoints(int parking_position);
  void UpdateStage3Move3ControlPoints(int parking_position);
  void UpdateStage3Move4ControlPoints(int parking_position);
  void UpdateStage3Move5ControlPoints(int parking_position);
};
}  // namespace planning
}  // namespace legionclaw