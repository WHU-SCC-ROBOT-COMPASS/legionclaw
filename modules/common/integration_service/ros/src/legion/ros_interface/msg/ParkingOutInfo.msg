Headers   header     # timestamp is included in header
int32   parking_out_id     # 若车位有id，同停车位id
int32   parking_direction_type     #  
uint8   is_parking_out_enable     # true可泊出，false不可泊出
Point3D   parking_out_point     # 世界坐标系下提车点位置
float64   theta     # 世界坐标系下的夹角（单位rad），提车点与正东方向夹角
