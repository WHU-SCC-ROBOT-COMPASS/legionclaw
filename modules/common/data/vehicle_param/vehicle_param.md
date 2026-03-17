| 序号 | 参数 | 单位 | 说明 |
|:---|:---|:---|:---|
|1|brand|/|车辆型号：如红旗HQ_H9|
|2|length|m|车身长度|
|3|width|m|车身宽度|
|4|height|m|车身高度|
|5|mass_fl|kg|左前质量（随着载重变化）|
|6|mass_fr|kg|右前质量（随着载重变化）|
|7|mass_rl|kg|左后质量（随着载重变化）|
|8|mass_rr|kg|右后质量（随着载重变化）|
|9|wheel_radius|m| 车轮有效半径                                           |
|10|wheelbase|m|总轴距|
|11|front_edge_to_center|m|前边沿到后轴中心的距离|
|12|back_edge_to_center|m|后边沿到后轴中心的距离|
|13|lf|m|前轴长度（随着载重变化）|
|14|lr|m|后轴长度（随着载重变化）|
|15|cf|N/rad|前轮横向侧偏刚度|
|16|cr|N/rad|后轮横向侧偏刚度|
|17|steer_ratio|/|转向传动比（方向盘角度/轮子角度）|
|18|rolling_coefficient|/|车轮滚阻系数|
|19|air_density|Kg/m³|空气密度 1.3Kg/m³|
|20|air_damping_coefficient|/|空气阻尼系数|
|21|max_steer_angle|rad|最大方向盘转角值（左负右正，为右转最大值，如500）|
|22|min_steer_angle|rad|最小方向盘转角值（左负右正，为左转最大值，如-500）|
|23|speed_limit|m/s|最高速度限制|
|24|max_brake_value|/|最大制动命令 |
|25|min_brake_value|/|最小制动命令 |
|26|max_accel_value|/|最大驱动命令 |
|27|min_accel_value|/|最小驱动命令 |
|28|speed_deadzone|m/s|速度死区 |
|29|max_steer_angle_rate|deg/s|最大转角速率 |
|30|max_abs_speed_when_stopped|m/s|通过该速度判断车辆是否处于静止状态 |
|31|max_abs_speed_when_stopped_duration|s|换档时间 |
|32|brake_value_when_gear_transitioning|/|档位切换过程中下发的最小制动命令 |
|33|accel_deadzone|/|车辆驱动命令不产生加速度的无效区段 |
|34|brake_deadzone|/|车辆制动命令不产生减速度的无效区段 |
|35|acceleration_in_idle|m/s<sup>2</sup>|驱动为纯加速度控制的车辆，驱动命令介入的最小需求加速度 |
|36|deceleration_in_idle|m/s<sup>2</sup>|怠速减速度，无油门无刹车车辆自身产生的减速度 |
|37|standstill_acceleration|m/s<sup>2</sup>|最小停车减速度 |
|38|max_acceleration_jerk|m/s<sup>3</sup>|允许车辆加速过程每秒中加速度的最大变化率 |
|39|max_acceleration|m/s<sup>2</sup>|允许车辆能达到的最大加速度 |
|40|max_deceleration|m/s<sup>2</sup>|允许车辆能达到的最大减速度 |
|41|min_turning_radius|m|最小转弯半径 |

