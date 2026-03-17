int32   id     #  
int32   light_flag     # 是否有效，无效就可能是清除了，或者已经过路口了
int32   light_color     # 信号灯颜色，绿灯可通行，false红灯不能通行
float64   light_remain_time     # 信号灯剩余时间
float64   distance_to_stop     # 停车距离，停止线
int32   direction_flag     # 是否有效，无效就可能是清除了，或者已经过路口了
int32   direction     # 路口通行方向，0：无效、1：左、2：前、3：右
float64   distance_to_junction     # 距路口距离
Point3D[] stop_line # 路口停止线
