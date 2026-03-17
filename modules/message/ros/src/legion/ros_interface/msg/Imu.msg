Header   header     # 头部信息
bool   is_valid     # 数据有效性标志位
Quaternion   orientation     # 角度（四元素）
float64[] orientation_covariance # 角度方差
Point3D   angular_velocity     # 角速度信息angularVelocity.x：x方向角速度信息angularVelocity.y: y方向角速度信息angularVelocity.z：z方向角速度信息
float64[] angular_velocity_covariance # 速度方差
Point3D   linear_acceleration     # 线加速度信息linearAcceleration.x：x方向线加速度信息linearAcceleration.y: y方向线加速度信息linearAcceleration.z：z方向线加速度信息
float64[] linear_acceleration_covariance # 线性加速度方差
