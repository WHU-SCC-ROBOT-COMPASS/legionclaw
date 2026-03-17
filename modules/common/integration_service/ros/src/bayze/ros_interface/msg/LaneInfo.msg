int8   priority     # 道路优先级：0（非推荐车道）1（推荐车道）
int8   global_id     # 道路全局id（从左至右，从0开始）
int8   predecessor_id     # 上一车道id
int8   successor_id     # 下一车道id
int8   left_neighbor_id     # 左边相邻车道id
int8   right_neighbor_id     # 右边相邻车道id
int8   type     # 0：未知 1：左转道不可掉头 2：直行道 3：右转道 4：直行且可左转道 5：直行且可右转道 6：左转道且可掉头 7：不可左转仅可掉头
LanePoint[] lane_points # 道路参考线（z）
