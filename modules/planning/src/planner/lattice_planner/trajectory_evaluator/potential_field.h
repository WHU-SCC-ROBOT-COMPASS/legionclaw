#pragma once

#include <omp.h>

#include <cstddef>
#include <map>
// #include <nav_msgs/OccupancyGrid.h>
// #include <visualization_msgs/MarkerArray.h>
#include "frame.h"
#include "map_matcher/map_matcher.h"
#include "modules/common/interface/path_point.hpp"
#include "modules/common/interface/prediction_obstacle.hpp"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/src/proto/driving/lattice_planner_conf.pb.h"
#include "modules/planning/src/proto/planning_conf.pb.h"
#include "reference_line/reference_line.h"

using namespace std;
using namespace legionclaw::common::math;

namespace legionclaw {
namespace planning {

class PotentialField {
 public:
  PotentialField() = default;
  ~PotentialField() = default;

 public:
  struct SingleGrid {
    double x, y;
    double s, d;
    double yaw;
    double potential_val;
    double obj_potential_val;
  };

  int GridsSize() const { return grids_.size(); }
  SingleGrid Grid(int index) const { return grids_.at(index); }
  std::vector<SingleGrid> Grids() const { return grids_; }
  vector<double>& GetVecLengthRemain() { return vec_length_remain_; }
  void ClearVecLengthRemain() { vec_length_remain_.clear(); }
  inline const std::vector<reference_line::ReferencePoint>& RefGlobalPath()
      const {
    return ref_global_path_;
  }
  reference_line::ReferencePoint RefGlobalPathPoint(int index) const {
    return ref_global_path_.at(index);
  }
  double length_front() const { return length_front_; }

  /**
   * @brief           创建基于道路的势场图
   * @param[in]   &global_paths 道路信息
   * @param[in]   &vehicle_state 车辆状态信息
   * @param[in]   &params 规划模块common配置
   * @param[in]   &evaluator_params 评估器配置
   * @param       &num_row 势场图尺寸/行
   * @param       &num_col 势场图尺寸/列
   * @param       &min_max_x_y 势场图边界
   */
  void InitPotentialFieldFromFrenet(
      const Frame* frame, const PlanningConf* params,
      const TrajectoryEvaluatorConf* evaluator_params, int& num_row,
      int& num_col, std::vector<double>& min_max_x_y);
  /**
   * @brief           计算势场值
   * @param[in]   &obj_list 预测障碍物信息
   * @param[in]   &vehicle_state 车辆状态信息
   * @param[in]   &evaluator_params 评估器配置
   * @param[in]   &threshold_static_speed 认为静止的速度边界
   */
  void CalculatePotentialField(
      const std::vector<PredictionObstacleMsg>& obj_list,
      const VehicleState& vehicle_state,
      const TrajectoryEvaluatorConf* evaluator_params,
      const double& threshold_static_speed);

  /**
   * @brief           依据边界对势场图进行重映射
   * @param[in]   &min_max_x_y 势场图边界
   * @param[in]   &num_row 势场图尺寸/行
   * @param[in]   &num_col 势场图尺寸/列
   * @param       &final_grids 最终势场结果
   */
  void CreateBoundingFiledandProjectPotentialField(
      std::vector<double>& min_max_x_y, int& num_row, int& num_col,
      std::vector<SingleGrid>& final_grids);

  /**
   * @brief
   * @param[in]
   * @param
   */
  void CalcRefLineRemainLen(const Frame *frame);

  /**
   * @brief           估计道路安全性
   * @param[in]   &global_paths 道路信息
   * @param[in]   &vehicle_state 车辆状态信息
   * @param[in]   &params 规划模块common配置
   * @param[in]   &evaluator_params 评估器配置
   * @param          &is_passages_safe
   * @param          &is_passages_safe_dclc
   * @param          &obstacles_lane_id
   */
  void EvaluateSidePassageSafety(
      const Frame *frame, const PlanningConf *params,
      const TrajectoryEvaluatorConf *evaluator_params,
      std::map<int, bool>& is_passages_safe_f,
      std::map<int, bool>& is_passages_safe_b,
      std::map<int, bool>& is_passages_safe_dclc,
      std::map<int, int>& obstacles_lane_id);

 private:
  // 栅格地图
  vector<SingleGrid> grids_;
  // vector<SingleGrid> final_grids_;
  // 道路参考线
  vector<reference_line::ReferencePoint> ref_global_path_;
  vector<double> vec_length_remain_;
  double length_front_;
};
class Mat3 {
 public:
  Mat3() {
    // initialize Identity by default
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) m[i][j] = 0;

    m[0][0] = m[1][1] = m[2][2] = 1;
  }

  Mat3(double transX, double transY, bool mirrorX, bool mirrorY) {
    m[0][0] = (mirrorX == true) ? -1 : 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = (mirrorY == true) ? -1 : 1;
    m[1][2] = transY;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double transX, double transY) {
    m[0][0] = 1;
    m[0][1] = 0;
    m[0][2] = transX;
    m[1][0] = 0;
    m[1][1] = 1;
    m[1][2] = transY;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(double rotation_angle) {
    double c = cos(rotation_angle);
    double s = sin(rotation_angle);
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = 0;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = 0;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  Mat3(PathPoint rotationCenter) {
    double c = cos(rotationCenter.theta());
    double s = sin(rotationCenter.theta());
    double u = rotationCenter.x();
    double v = rotationCenter.y();
    m[0][0] = c;
    m[0][1] = -s;
    m[0][2] = -u * c + v * s + u;
    m[1][0] = s;
    m[1][1] = c;
    m[1][2] = -u * s - v * c + v;
    m[2][0] = 0;
    m[2][1] = 0;
    m[2][2] = 1;
  }

  PathPoint operator*(PathPoint v) {
    PathPoint _v = v;
    double x = m[0][0] * _v.x() + m[0][1] * _v.y() + m[0][2] * 1;
    double y = m[1][0] * _v.x() + m[1][1] * _v.y() + m[1][2] * 1;
    v.set_x(x);
    v.set_y(y);
    return v;
  }

 private:
  double m[3][3];
};
}  // namespace planning
}  // namespace legionclaw
