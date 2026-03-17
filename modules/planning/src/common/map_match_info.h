/**
 * @file    map_match_info.h
 * @author  zdhy
 * @date    Mon August 8 19:00:03 2022
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#pragma once

namespace legionclaw {
namespace planning {

struct MapMatchInfo {
  int current_lane_id = -1;     //从左往右，从0开始计数，为自车根据一定规则判定所在global_id
  int current_lane_index = -1;  //自车所在的顺序，一般默认为1，共3车道可供行驶，即自车所在车道排在中间
  int lon_index = -1;           //自车在当前车道参考线的index
  double lat_offset = DBL_MAX;  //左+右-,自车距离当前车道的横向距离
  int match_index[3] = {-1};    //自车当前位置在对应参考线的index
  LaneInfoType ego_lane_type[3] = {LaneInfoType::LANE_TYPE_UNKNOWN};
  common::math::Vec2d greenbelt_occlusion_point = {DBL_MAX,DBL_MAX};
  //道路优先级：0（非推荐车道）1（推荐车道）
  int8_t priority[3] = {-1};
};

}  // namespace planning
}  // namespace legionclaw
