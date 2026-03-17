Header   header     # timestamp is  included in header
int32   parking_space_id     # 停车位id
int32   parking_type     # PARKING_TYPE=0//无效泊车 PARKING_TYPE=1//垂直泊车 PARKING_TYPE=2//水平泊车 PARKING_TYPE=3//倾斜泊车
bool   is_parking_enable     # PARKING_ENABLE=0//不可泊入 PARKING_ENABLE=1//可泊入
float64   confidence     # 停车位置信度
Point3D   center_point_of_parking     # 车辆坐标系的停车位中心点
float64   theta     # 车辆坐标系下停车位的朝向
float64   width     # 停车位宽
float64   length     # 停车位长
float64   yaw_offset     # 停车位角度偏移量（倾斜车位），倾斜车位与道路边线夹角
ImageKeyPoint[] parking_points_in_image # 图像坐标系下停车位关键点，顺时针方向
