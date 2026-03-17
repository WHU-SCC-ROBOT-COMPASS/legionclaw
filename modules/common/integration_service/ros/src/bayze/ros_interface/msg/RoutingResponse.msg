Headers   header     # timestamp is included in header
int32   plan_status     # 规划结果状态
int32   replan_flag     # REPLAN_FLAG_NONE=0  //不需要重规划 REPLAN_FLAG_NAVI=1 //网络层重规划 REPLAN_FLAG_HUMAN=2 //人为重规划
int32   route_reason     # 规划原因 ROUTE_REASON_HMI = 0 //hmi上触发规划 ROUTE_REASON_PLATOON_HEADER = 1 //车队头车下发规划 ROUTE_REASON_PLATOON_OTHER = 2 //车队其他车辆下发规划
LaneInfo[] lane_list # 分段路径规划结果
