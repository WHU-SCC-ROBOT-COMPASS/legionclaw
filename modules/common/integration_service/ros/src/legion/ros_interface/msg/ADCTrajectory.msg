Headers   header     # timestamp is included in header
float64   total_path_length     # unit meters(m)
float64   total_path_time     # unit seconds(s)
TrajectoryPoint[] trajectory_points # 规划轨迹点列
int32   car_action     # 车辆行为： CAR_ACTION_NONE=0  //无效值 CAR_ACTION_SINGLE=1 //单车 CAR_ACTION_PLATOON_HEAD=2 //车队头车 CAR_ACTION_PLATOON_OTHER=3 //车队其他车辆
int32   behaviour_lat_state     # 自动驾驶状态（主要指横向状态）
int32   behaviour_lon_state     # 自动驾驶状态（主要指纵向状态）
int32   scenario     # 驾驶场景 0-UNKNOWN, 1-CRUISE, 2-CRUISE_URBAN 3-CRUISE_HIGHWAY, 2000-JUNCTION  2001-JUNCTION_TRAFFIC_LIGHT 2002-JUNCTION_STOP_SIGN
int32   driving_mode     # 驾驶模式： 1：人工驾驶， 2：辅助驾驶， 3：自动驾驶
int32   adc_trajectory_type     # 轨迹类型： TRAJECTORY_TYPE_ABSOLUTE=0 //绝对规划轨迹 TRAJECTORY_TYPE_RELATIVE=1 //相对规划轨迹 TRAJECTORY_TYPE_INVALID=2 //无效规划轨迹
EStop   estop     # emergency stop
uint8   is_replan     # is_replan == truemean replan triggered(default = false)
string   replan_reason     #  
int32   right_of_way_status     # UNPROTECTED = 0;PROTECTED = 1;
RSSInfo   rss_info     # output related to RSS
