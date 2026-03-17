Headers   header     # 消息头
int32   sensor_id     # 安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机 1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机 5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
int8   error_code     # 错误码： ERROR_NONE = 0; ERROR_UNKNOWN = 1;
uint8   sensor_status     # 传感器状态： NORMAL=0； ABNORMAL=1；
int32   change_origin_flag     # 坐标切换状态（0-坐标系切换成功 1-坐标系切换中 2-坐标系切换故障）
uint8   is_valid     # 车道线数据是否合法
SensorCalibrator   sensor_calibrator     # 传感器标定参数
LaneLine[] camera_laneline # 车道线检测结果数组
HolisticPathPrediction   hpp     # 行驶预测线
RoadMark[] road_marks # 路面标识
