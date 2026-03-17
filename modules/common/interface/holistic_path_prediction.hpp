/**
 * @file    holistic_path_prediction.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

#include "modules/common/interface/lane_line_cubic_curve.hpp"

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class HolisticPathPrediction {
public:
  HolisticPathPrediction() {
    planning_source_ =
        legionclaw::interface::HolisticPathPrediction::PlanningSource::INVALID;
    ego_lane_width_ = 0.0;
    confidence_ = 0.0;
  }
  ~HolisticPathPrediction() = default;

  enum PlanningSource {
    INVALID = 0,
    BOTH_LANE = 1,
    LEFT_LANE = 2,
    RIGHT_LANE = 3,
    LEFT_ROAD_EDGE = 4,
    RIGHT_ROAD_EDGE = 5,
    FRONT_VEHICLE_TRAJECTORY = 6,
    FREESPACE = 7,
    EGO_TRAJECTORY_PREDICTION = 8,
  };

  inline void set_hpp(const legionclaw::interface::LaneLineCubicCurve &hpp) {
    hpp_ = hpp;
    hpp_ptr_ = &hpp_;
  }

  inline const legionclaw::interface::LaneLineCubicCurve &hpp() const {
    return hpp_;
  }

  inline legionclaw::interface::LaneLineCubicCurve *mutable_hpp() { return &hpp_; }

  inline bool has_hpp() { return (hpp_ptr_ != nullptr); }

  inline void set_planning_source(
      const legionclaw::interface::HolisticPathPrediction::PlanningSource
          &planning_source) {
    planning_source_ = planning_source;
    planning_source_ptr_ = &planning_source_;
  }

  inline const legionclaw::interface::HolisticPathPrediction::PlanningSource &
  planning_source() const {
    return planning_source_;
  }

  inline legionclaw::interface::HolisticPathPrediction::PlanningSource *
  mutable_planning_source() {
    return &planning_source_;
  }

  inline bool has_planning_source() {
    return (planning_source_ptr_ != nullptr);
  }

  inline void set_ego_lane_width(const double &ego_lane_width) {
    ego_lane_width_ = ego_lane_width;
    ego_lane_width_ptr_ = &ego_lane_width_;
  }

  inline const double &ego_lane_width() const { return ego_lane_width_; }

  inline double *mutable_ego_lane_width() { return &ego_lane_width_; }

  inline bool has_ego_lane_width() { return (ego_lane_width_ptr_ != nullptr); }

  inline void set_confidence(const double &confidence) {
    confidence_ = confidence;
    confidence_ptr_ = &confidence_;
  }

  inline const double &confidence() const { return confidence_; }

  inline double *mutable_confidence() { return &confidence_; }

  inline bool has_confidence() { return (confidence_ptr_ != nullptr); }

  void operator=(const HolisticPathPrediction &holistic_path_prediction) {
    CopyFrom(holistic_path_prediction);
  }

  void CopyFrom(const HolisticPathPrediction &holistic_path_prediction) {
    hpp_ = holistic_path_prediction.hpp();
    planning_source_ = holistic_path_prediction.planning_source();
    ego_lane_width_ = holistic_path_prediction.ego_lane_width();
    confidence_ = holistic_path_prediction.confidence();
  }

protected:
  //行驶路径预测： 1、针对没有车道线等特殊路段如：收费站附近、进出匝道附近、
  //维修区域附近，可以虚拟出可行驶路径预测；
  //2、针对车辆遮挡的区域，可以虚拟出可行驶路径预测；
  legionclaw::interface::LaneLineCubicCurve hpp_;
  legionclaw::interface::LaneLineCubicCurve *hpp_ptr_ = nullptr;
  //规划路径来源:  0 - invalid,  1 - 双侧车道线,  2 - 左侧车道线,  3 -
  //右侧车道线, 4 - 左侧路沿，  5 - 右侧路沿， 6 - 前车轨迹，  7 - freespace, 8
  //- 本车轨迹预测
  legionclaw::interface::HolisticPathPrediction::PlanningSource planning_source_;
  legionclaw::interface::HolisticPathPrediction::PlanningSource
      *planning_source_ptr_ = nullptr;
  //自车道宽度
  double ego_lane_width_;
  double *ego_lane_width_ptr_ = nullptr;
  //置信度[0 - 1]
  double confidence_;
  double *confidence_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
