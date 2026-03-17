int32   name_region     # 前后左右区域
float64   score     # 车辆前后左右的区域打分获得的分值(0-100表示逐步危险程度)
int32   rank_risk     # 危险程度(0-3四级划分，不同分级对应不同驾驶处理策略)
Point3D[] region_polygon # 世界坐标系下的区域多边形
