Header   header     #  
int32   driving_mode     # 驾驶状态
int32   gear_location     # 档位值
float64   steering_value     # 方向盘转角
float64   accel_value     # 线控驱动值（油门值）
float64   brake_value     # 线控制动值（刹车值）
float64   speed_mps     # 车速（m/s）
PointLLH   position     # 经纬度高程
PointENU   utm_position     # 相对参考点xyz坐标
float64   pitch     # 方向(单位弧度，-π～π,正东:0, 逆时针。都满足右手定则)
float64   roll     #  
float64   heading     #  
int32   rtk_flag     # RTK标志值
float64   origin_lat     # // 参考点纬度 GCJ-02格式
float64   origin_lon     # // 参考点经度 GCJ-02格式
float64   current_ins_yaw     #  
int32   auto_safe     # //  安全驾驶0:正常 1:左前车门没关 2:右前车门没关 3:左后车门没关 4:右后车门没关 5:未系安全带 6: 未打开反观镜 7:车辆未静止
float64   battery_power_percentage     # 电池电量值
int32   charging_gun_status     # 充电枪状态
int32[] chassis_error_code # 底盘错误码
int32   brake_lamp_status     # 刹车灯状态
int32   turn_lamp_status     # 转向灯状态
PointENU   mercator_position     # 转坐标系后的参考点
