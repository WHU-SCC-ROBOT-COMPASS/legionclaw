Headers   header     #  
int32   steer_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
float64   front_steering_target     # target steering angle
float64   rear_steering_target     # target steering angle
float64   front_steering_rate     # target non-directional steering rate
float64   rear_steering_rate     # target non-directional steering rate
int32   accel_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
float64   accel_value     # target accel value
int32   brake_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
float64   brake_value     # target brake
int32   backup_brake_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
float64   backup_brake_value     # target brake
int32   epb_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
int32   epb_level     # 0=RELEASE 1=APPLIY 2=INVALID
int32   gear_driving_mode     # 0=COMPLETE_MANUAL  1=COMPLETE_AUTO_DRIVE 4=EMERGENCY_MODE
int8   emergency_brake_enable     # 0=disenable  1=enable
int32   gear_location     # 0=GEAR_NEUTRAL 1=GEAR_DRIVE 2=GEAR_REVERSE 3=GEAR_PARKING 4=GEAR_LOW 5=GEAR_INVALID 6=GEAR_NONE
float64   speed     # target speed, in m/s
float64   acceleration     # target acceleration in m`s^-2
int32   turn_lamp_ctrl     # 转向灯 0 关闭 1 开启 2 不响应
int32   high_beam_ctrl     # 0 关闭 1 开启 2 不响应
int32   low_beam_ctrl     # 0 关闭 1 开启 2 不响应
int32   horn_ctrl     # 0 关闭 1 开启 2 不响应
int32   front_wiper_ctrl     # 前雨刮器 0 关闭 1 开启
int32   rear_wiper_ctrl     # 后雨刮器 0 关闭 1 开启
int32   position_lamp_ctrl     # 位置灯 0 关闭 1 开启
int32   front_fog_lamp_ctrl     # 前雾灯 0 关闭 1 开启
int32   rear_fog_lamp_ctrl     # 后雾灯 0 关闭 1 开启
int32   brake_lamp_ctrl     # 刹车灯 一般情况自动控制 0 关闭 1 开启
int32   alarm_lamp_ctrl     # 警报灯 双闪 0 关闭 1 开启
int32   lf_door_ctrl     # 左前门控制 0 关闭 1 开启
int32   rf_door_ctrl     # 右前门控制 0 关闭 1 开启
int32   lr_door_ctrl     # 左后门控制 0 关闭 1 开启
int32   rr_door_ctrl     # 右后门控制 0 关闭 1 开启
