Header   header     # timestamp is  included in header
int32   parking_space_id     # // 停车位id
int32   parking_type     # // PARKING_TYPE=0 //无效泊车 PARKING_TYPE=1 //水平泊车 PARKING_TYPE=2 //垂直泊车 PARKING_TYPE=3 //倾斜泊车
int32   parking_status     # PARKING_ENABLE=0//可泊 PARKING_DISENABLE=1//不可泊 PARKING_NONOPTIONAL=2//不可选
Point3D   center_point_of_parking     # // 世界坐标系下停车位中心点
float64   theta     # // 世界坐标系下的夹角（单位rad），车位出口与正东方向夹角
float64   width     # // 停车位宽（单位m）
float64   length     # // 停车位长单位m）
bool   is_custom_parking     # // 是否是自定义车位
Polygon3D   polygon     # 世界坐标系下的车位多边形
int32   parking_source_type     # 0-线车位， 1-空间车位， 2-融合车位
