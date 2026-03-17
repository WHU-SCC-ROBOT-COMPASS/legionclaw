/**
 * @file
 *
 * @brief 平行泊车路径生成
 */

#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include "modules/common/data/vehicle_param/proto/vehicle_param.pb.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "parking_path_generator.h"

using namespace std;

namespace legionclaw {
namespace planning {
/**
 * @class ParallelParkingPathGenerator
 * @brief 生成泊车路径信息（基于泊车坐标系）
 */
class ParallelParkingPathGenerator : public ParkingPathGenerator {
 public:
  /**
   * @brief 构造函数
   */
  ParallelParkingPathGenerator() = default;
  /**
   * @brief 构造函数
   * @param parking_spot 输入量：泊车位信息。
   */
  ParallelParkingPathGenerator(const PlanningConf *planning_conf);
  /**
   * @brief 析构函数
   */
  ~ParallelParkingPathGenerator() = default;

  /**
   * @brief 泊车坐标系下，计算第一次前进的泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @return 1设置成功，0不成功。
   */
  Status ComputeForwardPathInStage1(const legionclaw::interface::PathPoint &cur_pose) override;

  /**
   * @brief 泊车坐标系下，计算倒车泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @param spiral_order 输入量：螺旋曲线次数。
   * @return 1设置成功，0不成功。
   */
  Status ComputeBackwardPathInStage1(const legionclaw::interface::PathPoint &cur_pose) override;

  Status ComputeForwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) override;

  Status ComputeBackwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) override;

  void SetParkingSpot(const legionclaw::interface::ParkingInfo &parking_spot) override;

  /**
   * @brief 确定停车位附近的可行驶区域（计算road_polygon_）
   */
  bool ConstructFreeSpace() override;

  /**
   * @brief 判断车库长宽是否满足平行泊车入库条件
   * @return true满足，false不满足。
   */
  bool IsParkingSpaceValid();

 protected:
  int parking_position_;  // 泊车位方位（在车辆的右侧（1）或左侧（-1））
  ParallelParkingConf parallel_parking_conf_;  // 泊车配置参数
  legionclaw::interface::VehicleParam vehicle_param_;
  legionclaw::interface::PathPoint local_vehicle_current_pose_;
  // 最小转弯半径
  double min_turning_radius_;
  double H_;  // 开始保持方向盘的点的纵向距离
  double S_;  // 开始保持方向盘的点的横向距离

  legionclaw::interface::PathPoint stage1_f_a0_;
  legionclaw::interface::PathPoint stage1_f_a1_;
  legionclaw::interface::PathPoint stage1_b_a0_;
  legionclaw::interface::PathPoint stage1_b_a1_;
  legionclaw::interface::PathPoint stage1_b_a2_;
  legionclaw::interface::PathPoint stage1_b_a3_;
  legionclaw::interface::PathPoint stage1_b_a4_;
  legionclaw::interface::PathPoint stage2_f_a0_;
  legionclaw::interface::PathPoint stage2_f_a1_;
  legionclaw::interface::PathPoint stage2_b_a0_;
  legionclaw::interface::PathPoint stage2_b_a1_;

  void UpdateStage1ForwardControlPoints(int parking_position);
  void UpdateStage1BackwardControlPoints(int parking_position);
  void UpdateStage2ForwardControlPoints(int parking_position);
  void UpdateStage2BackwardControlPoints(int parking_position);
};
}  // namespace planning
}  // namespace legionclaw