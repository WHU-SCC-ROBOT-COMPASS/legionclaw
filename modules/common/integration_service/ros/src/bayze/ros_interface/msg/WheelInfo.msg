Headers   header     #  
int32   moving_status     # 0=UNKNOWN_VEHICLE_STATE  1=STATIONARY  2=MOVING_FORWARD  3=MOVING_BACKWARD
int32   steer_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE  4=EMERGENCY_MODE
float64   steering_value     # Real steering location
float64   steering_torque_nm     # Applied steering torque in [Nm]
float64   steering_rate_dps     # degree/s
float64   speed_mps     # Vehicle Speed in meters per second
uint8   veh_spd_vld     # 车速有效性
int32   gear     # 档位
uint8   gear_vld     # 档位有效性
int32   wheel_direction_rr     # 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
float64   wheel_spd_rr     # Actual speed of the right rear wheel
int32   wheel_direction_rl     # 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
float64   wheel_spd_rl     # Actual speed of the left rear wheel
int32   wheel_direction_fr     # 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
float64   wheel_spd_fr     # Actual speed of the right front wheel
int32   wheel_direction_fl     # 0=FORWARD 1=BACKWARD 2=STANDSTILL 3=INVALID
float64   wheel_spd_fl     # Actual speed of the left front wheel
float64   yaw_rate     # Yaw rate[degree/s] [-163.84|163.835]
int32   wss_fl_edges_sum     # 左前轮边缘和
int32   wss_fr_edges_sum     # 右前轮边缘和
int32   wss_rl_edges_sum     # 左后轮边缘和
int32   wss_rr_edges_sum     # 右后轮边缘和
uint8   wss_fl_edges_sum_vld     # 左前轮边缘和有效位（0：无效 1：有效）
uint8   wss_fr_edges_sum_vld     # 右前轮边缘和有效位（0：无效 1：有效）
uint8   wss_rl_edges_sum_vld     # 左后轮边缘和有效位（0：无效 1：有效）
uint8   wss_rr_edges_sum_vld     # 右后轮边缘和有效位（0：无效 1：有效）
float64   veh_lat_accel     # 车辆横向加速度
float64   veh_lgt_accel     # 车辆纵向加速度
uint8   veh_lat_accel_vld     # 车辆横向加速度有效性
uint8   veh_lgt_accel_vld     # 车辆纵向加速度有效性
