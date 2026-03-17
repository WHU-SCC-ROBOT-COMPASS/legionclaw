Header   header     # timestamp is included in header
int32   parking_space_id     # 停车位id
int32   parking_type     # PARKING_TYPE=0//无效泊车 PARKING_TYPE=1//水平泊车 PARKING_TYPE=2//垂直泊车 PARKING_TYPE=3//倾斜泊车
int32   parking_status     # PARKING_ENABLE=0//可泊 PARKING_DISENABLE=1//不可泊 PARKING_NONOPTIONAL=2//不可选
float64   confidence     # 车位置信度
Point3D   center_point_of_parking     # 世界坐标系下停车位中心点
float64   theta     # 世界坐标系下的夹角（单位rad），车位出口与正东方向夹角
float64   width     # 停车位宽（单位m）
float64   length     # 停车位长单位m）
float64   yaw_offset     # 停车位角度偏移量（倾斜车位），倾斜车位与道路边线夹角,(单位m）
Polygon3D   polygon     # 世界坐标系下的车位多边形
int32   sensor_id     # 车位数据来自相机id 0-front_center相机 1-front_left相机 2-front_right相机 3-left_front相机 4-left_back右后相机 5-right_front相机 6-right_back相机 7-back相机 8-相机融合
bool   is_lane_width_valid     # 车道宽度是否有效
float64   lane_width     # 泊车可用(双)车道宽度
ParkingStopper[] parking_stoppers # 车位内部的限位器数组
int32   parking_direction_type     #   DIR_INVALID = 0,   LEFT = 1,   UP = 2,   RIGHT = 3,   DOWN = 4,
int32   left_occupied_status     # 0-unknown 未知，1-empty 空闲，2-occupied 被占用
int32   right_occupied_status     # 0-unknown 未知，1-empty 空闲，2-occupied 被占用
int32   parking_source_type     # 0-线车位， 1-空间车位， 2-融合车位
