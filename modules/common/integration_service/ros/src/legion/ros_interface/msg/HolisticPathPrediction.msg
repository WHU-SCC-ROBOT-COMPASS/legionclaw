LaneLineCubicCurve   hpp     # 行驶路径预测： 1、针对没有车道线等特殊路段如：收费站附近、进出匝道附近、 维修区域附近，可以虚拟出可行驶路径预测； 2、针对车辆遮挡的区域，可以虚拟出可行驶路径预测；
int32   planning_source     # 规划路径来源:  0 - invalid,  1 - 双侧车道线,  2 - 左侧车道线,  3 - 右侧车道线, 4 - 左侧路沿，  5 - 右侧路沿， 6 - 前车轨迹，  7 - freespace, 8 - 本车轨迹预测
float64   ego_lane_width     # 自车道宽度
float64   confidence     # 置信度[0 - 1]
