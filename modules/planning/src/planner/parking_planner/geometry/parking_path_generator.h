/**
 * @file
 *
 * @brief 泊车路径生成
 */
#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include "modules/common/enum/enum.h"
#include "modules/common/interface/parking_info.hpp"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/data/vehicle_param/proto/vehicle_param.pb.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/src/common/enum.h"
#include "modules/planning/src/common/math/include/spiral_curve/cubic_spiral_curve.h"
#include "modules/planning/src/common/math/include/spiral_curve/quintic_spiral_curve.h"
#include "modules/planning/src/proto/parking/parking_conf.pb.h"

using namespace std;
using namespace legionclaw::common;
namespace legionclaw {
namespace planning {
using legionclaw::common::Status;
using namespace legionclaw::common::math;
/**
 * @class ParkingPathGenerator
 * @brief 生成泊车路径信息（基于泊车坐标系）
 */
class ParkingPathGenerator {
 public:
  /**
   * @brief 构造函数
   */
  ParkingPathGenerator() = default;

  /**
   * @brief 析构函数
   */
  virtual ~ParkingPathGenerator() = default;

  /**
   * @brief 泊车坐标系下，计算第一次前进的泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @return 1设置成功，0不成功。
   */
  virtual Status ComputeForwardPathInStage1(const legionclaw::interface::PathPoint &cur_pose) = 0;

  /**
   * @brief 向后延长路径
   * @param start_pos 输入量：起始点。
   * @param extend_length 输入量：延长长度（单位：米）。
   * @param interval 输入量：延长路径的点间隔（单位：米）。
   * @param gear_flag 输入量：档位标志。
   * @param extend_path 输出量：延长的路径。
   * @return true：成功，false：失败
   */
  bool ExtendPath(const legionclaw::interface::PathPoint &start_pos, const double &extend_length,
                  const double &interval, const legionclaw::common::GearPosition &gear_position,
                  std::vector<legionclaw::interface::PathPoint> &extend_path);
  /**
   * @brief 计算圆弧上的点列（x,y,theta,kappa)
   * @param ox 输入量：圆心坐标x（单位：米）。
   * @param oy 输入量：圆心坐标y（单位：米）。
   * @param theta0
   * 输入量：圆弧起始角度（与东方向夹角，逆时针方向为正，单位：弧度）。
   * @param theta1
   * 输入量：圆弧结束角度（与东方向夹角，逆时针方向为正，单位：弧度）。
   * @param dl 输入量：点列采样弧长（单位：米）。
   * @param r 输入量：圆弧半径（单位：弧度）。
   * @param out_path 输出量：圆弧上的点列。
   * @return 1成功，0不成功。
   */
  bool CalculateCirclePath(const double &ox, const double &oy, const double &theta0,
                           const double &theta1, const double &dl, const double &r,
                           std::vector<legionclaw::interface::PathPoint> &out_path);

  /**
   * @brief 泊车坐标系下，计算倒车泊车路径
   * @param cur_pose 输入量：当前位置信息。
   * @param spiral_order 输入量：螺旋曲线次数。
   * @return 1设置成功，0不成功。
   */
  virtual Status ComputeBackwardPathInStage1(const legionclaw::interface::PathPoint &cur_pose) = 0;

  virtual Status ComputeForwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) = 0;

  virtual Status ComputeBackwardPathInStage2(const legionclaw::interface::PathPoint &cur_pose) = 0;

  /**
   * @brief 确定停车位附近的可行驶区域（计算road_polygon_）
   */
  virtual bool ConstructFreeSpace() = 0;

  virtual void SetParkingSpot(const legionclaw::interface::ParkingInfo &parking_spot) = 0;

  /**
   * @brief 判断pose与road_polygon_是否有碰撞
   */
  bool IsEnoughFreeSpace(const legionclaw::interface::PathPoint &pose);

  // 接口函数
  legionclaw::interface::ParkingInfo GetParkingSpot() { return parking_spot_; }
  // std::vector<legionclaw::interface::PathPoint> GetKeyPose() { return key_pose_;
  // }
  std::vector<legionclaw::interface::PathPoint> GetForwardPath() { return forward_path_; }
  std::vector<legionclaw::interface::PathPoint> GetBackwardPath() { return backward_path_; }

  void SetVehicleParam(const legionclaw::interface::VehicleParam &vehicle_param);

  void UpdateSpiralCurveConfig(SpiralCurveConf spiral_curve_conf);

 protected:
  legionclaw::interface::ParkingInfo parking_spot_;              // 泊车位信息
  std::vector<legionclaw::interface::PathPoint> forward_path_;   // 前进部分的路径
  std::vector<legionclaw::interface::PathPoint> backward_path_;  // 倒车部分的路径

  double in_distance_;  // 倒车过程中，方向盘开始回正时进入车库的深度

  std::vector<math::Polygon2d> road_polygon_;  // 用于描述停车位附近不可通行的区域

  legionclaw::interface::VehicleParam vehicle_param_;   // 车辆参数
  spiral::SpiralCurveConfig spiral_curve_config_;  // 螺旋曲线参数
};
}  // namespace planning
}  // namespace legionclaw