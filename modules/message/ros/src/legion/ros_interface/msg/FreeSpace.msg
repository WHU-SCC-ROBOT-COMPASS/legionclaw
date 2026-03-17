Header   header     # timestamp is included in header
int32   sensor_id     # 安装的传感器id(camera,lidar,radar) ' 相机id: 0-front_center相机 1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机 5-right_front相机 6-right_back相机 7-back相机'  8-相机融合
Polygon2D[] freespace_region # freespace轮廓图像坐标
int32   error_code     # 错误码（default = OK）
bool   is_valid     # 障碍物数据是否合法
