Headers   header     # timestamp is included in header
int32   seq     # Sequence number，lane_list数据有更新则加1，否则保持不变
Point3D   ego_point     # 生成该LocalMap局部地图消息时的自车位置
float64   range     # 生成该LocalMap局部地图消息的范围，以ego_point为中心，range为边长的正方形，单位：m
LaneInfoExtend[] lane_list # 自车周边LocalMap范围内的车道信息列表
