int64   road_id     # 道路的id
int8   road_type     # 道路的类型: 0：未知或一般道路  1：环岛 2：交叉路口 3：主路 4：辅路
int8   turn_type     # 规划路线上道路的转向信息 1：左转 2：直行 3：右转
float64   avg_curvature     # 道路的平均曲率
int32   curvature_size     # 道路中心线上点(曲率)数量
CurvatureInfo[] curvature # 道路曲率信息数组
