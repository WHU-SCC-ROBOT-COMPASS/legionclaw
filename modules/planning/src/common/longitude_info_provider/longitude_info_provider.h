/**
 * @file    planning.h
 * @author  zdhy
 * @date    2022
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#pragma once

#include <vector>

#include "modules/common/interface/trajectory_point.hpp"
#include "modules/planning/src/common/map_matcher/map_matcher.h"

namespace legionclaw {
namespace planning {

using namespace legionclaw::interface;

class LongitudeInfoProvider {
public:
    LongitudeInfoProvider(){};
    ~LongitudeInfoProvider(){};

public:
    /**
     * @brief . 计算轨迹速度、挡位信息，时间采样抽稀
     * @param .
     * @param .
     * @return .
     */
    bool ProvideTrajectoryLongitudinalInfo(
        const double &current_velocity, const double &accel,
        const double &limit_speed, std::vector<TrajectoryPoint> &trajectory);


};

}
}