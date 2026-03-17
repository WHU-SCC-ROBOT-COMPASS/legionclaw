Header   header     # 消息头
PointLLH   position     # 经纬度高程
float64   pitch     # 方向(单位弧度，-π～π,正东:0, 逆时针。都满足右手定则)
float64   roll     #  
float64   heading     #  
Point3D   linear_velocity     # 线速度（m/s，北东地坐标系）
Point3D   linear_acceleration     # 线加速度（m/s2，前右下坐标系）
Point3D   angular_velocity     # 角速度（度/s，前右下，满足右手定则）
int32   rtk_flag     # RTK标志值
int32   odom_type     # 融合标志:0-(纯ins) 1-(纯轮速) 2-(泊车递推) 3-(无) 4-(orb匹配定位)
int32   auxiliary_type     # 辅助方式:0-NULL(无辅助方式) 1-laneliine_map(车道线/标识牌)2-feature_map(lidar)3-feature_map(lidar_camera)
int32   location_valid_flag     # Location有效性标志位0: invalid； 1: valid
float64   origin_lat     # 参考点纬度 GCJ-02格式
float64   origin_lon     # 参考点经度 GCJ-02格式
PointENU   utm_position     # 相对参考点xyz坐标
int32   change_origin_flag     # 0::无坐标系切换 1:坐标系切换成功2:坐标系切换中3:坐标系切换故障
PointENU   utm_position_next     # 相对参考点坐标(切换状态下，为新的原点坐标系下)
Point3D   position_std_dev     # 位置标准差
Point3D   orientation_std_dev     # 方向标准差
Point3D   linear_velocity_std_dev     # 速度标准差
Point3D   linear_acceleration_std_dev     # 加速度标准差
Point3D   angular_velocity_std_dev     # 角速度标准差
