int8   priority     # 道路优先级
int64   global_id     # 道路全局id（从左至右，从0开始）
int64[] predecessor_ids # 所有的前继车道id数组
int64[] successor_ids # 所有的后续车道id数组
int64   left_neighbor_id     # 左边相邻车道id
int64   right_neighbor_id     # 右边相邻车道id
int8   type     # 类型（预留）
LanePoint[] lane_points # 道路参考线（z）
