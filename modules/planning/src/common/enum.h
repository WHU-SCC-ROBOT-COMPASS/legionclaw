/**
 * @file    enum.h
 * @author  zdhy
 * @date    Wed Apr 28 19:00:03 2021
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */
#pragma once

namespace legionclaw {
namespace planning {
enum STOP_TYPE {
  STOP_TYPE_TERMINAL,  //目的地终点
  STOP_TYPE_JUNCTION   //路口停止线
};

enum REPLAN_FLAG {
  REPLAN_FLAG_NONE,  //不需要重规划
  REPLAN_FLAG_NAVI,  //网络层重规划
  REPLAN_FLAG_HUMAN  //人为重规划
};

enum ROUTE_REASON {
  OUTE_REASON_HMI,              // hmi上触发规划
  ROUTE_REASON_PLATOON_HEADER,  //车队头车下发规划
  ROUTE_REASON_PLATOON_OTHER    //车队其他车辆下发规划
};

enum DrivingFlag{
  DRIVING_INVALID,
  AUTO,
  HUMAN,
  ASSISTED
};

enum TaskStatus{
  TASK_INVALID,
  TASK_COMPLETE,
  TASK_INTERRUPT,
  TASK_FAILED
};

enum PlanningFlag{
  PLANNING_INVALID,//0
  PARKING_REQUEST,//1
  PARKING,//2
  PARKING_STOP,//3
  PARKING_OUT,//4
  PARKING_OUT_STOP,//5
  DRIVING//6
};

enum PlannerFlag{
  PLANNER_INVALID,
  LATTICE_PLANNER,
  EM_PLANNER,
  E_SLOW_STOP_PLAN,
  E_QUICK_STOP_PLAN,
  PARKING_PLANNER,
  PARKINGOUT_PLANNER
};

enum SelectorLatFlag{
  LAT_INVALID,
  FORWARD,
  STATION,
  CHANGE_LANE_LEFT,
  CHANGE_LANE_RIGHT,
  AVOIDANCE_PRE,
  AVOIDANCE_ACTU,
  AVOIDANCE_BACK_PRE
};

enum SelectorLonFlag{
  LON_INVALID,
  NORMAL
};

enum Lane_Changing_Mode {
  Invalid,             //无效
  Autonomous,          //自主换道
  Manual               //手动换道
};

enum Lane_Change_State {
  LANE_CHANGE_INVALID,
  LANE_CHANGING,
  LANE_CHANGE_COMPLETE
};

enum DynamicObjectsPose {
  D_OBJ_UNKNOWN,
  D_OBJ_FORWARD,
  D_OBJ_BACK,
  D_OBJ_LEFTUP,
  D_OBJ_LEFTDOWN,
  D_OBJ_RIGHTUP,
  D_OBJ_RIGHTDOWN
};

enum YieldAction {
  NO_ACTION,
  YIELD_TO_LEFT,
  YIELD_TO_RIGHT,
  KEEP_IN_MIDDLE
};

enum OutwardStatus {
  Outward_Invalid,
  Outward_Follow,
  Outward_Creep,
  Outward_Stop,
  Outward_Complete
};

enum InwardStatus {
  Inward_Invalid,
  Inward_Follow,
  Inward_Creep,
  Inward_Stop,
  Inward_Complete
};

enum LaneInfoType
{
  LANE_TYPE_UNKNOWN = 0,
  LEFT_TURN_NO_TURN_AROUND = 1,
  STRAIGHT = 2,
  RIGHT_TURN = 3,
  STRAIGHT_AND_LEFT_TURN = 4,
  STRAIGHT_AND_RIGHT_TURN = 5,
  LEFT_TURN_AND_TURN_AROUND = 6,
  NO_LEFT_TURN_ONLY_TURN_AROUND = 7
};

// enum RoadScenes{
//   UNKNOWN = 0,
//   ONE_TO_ONE = 1,
//   ONE_TO_MANY = 2,
//   MANY_TO_ONE = 3
// };

}  // namespace planning
}  // namespace legionclaw
