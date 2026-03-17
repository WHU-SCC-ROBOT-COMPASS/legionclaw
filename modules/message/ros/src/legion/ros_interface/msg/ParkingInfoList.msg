Header   header     # 消息头
ParkingInfo[] parking_info # 检测出的停车位数组
int32   error_code     # 错误码（default = OK）
bool   is_valid     # 车位数据是否合法
bool   parking_out_enable     # 是否能泊出
int32   parking_out_direction     # 泊出方向
int32   parking_out_id     # 当前自车所在车位ID
