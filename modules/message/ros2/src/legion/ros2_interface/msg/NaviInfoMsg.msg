Header   header     # timestamp is included in header
uint32   lane_type     # 道路类型0: 常规道路; 1: 路口；（如果没有，给0）
uint32   lane_count     # 当前道路切面车道个数
uint32   lane_index     # 当前车所在道路索引，最左侧为0
uint32   lane_target     # 目标行驶车道索引，最左侧为0
float32   road_speed     # 地图限速，单位：米/秒
float32   turning_speed     # 转弯建议速度，根据曲率计算，单位：米/秒（如果没有，给极大值）
float32   turning_deriction     # 转弯完成后，道路朝向，单位：角度（如果没有，给0）
float32   distance_to_cross     # 距离路口的距离，单位：米
bool   traffic_light_stop     # 路口是否需要红绿灯停车
uint32   crossing_behavior     # 路口决策行为：0: 直行; 1: 左转; 2: 右转; 3: 掉头
float32   distance_to_stop     # 全局剩余里程，单位：米
