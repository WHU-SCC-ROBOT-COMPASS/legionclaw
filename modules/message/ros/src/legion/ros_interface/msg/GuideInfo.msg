Header   header     # 头部信息
float64   next_dis     # 下一条规划路线上道路的距离
GuideRoad   current_road     # 当前道路的引导信息
GuideRoad   next_road     # 下一条规划道路的引导信息
int8   round_status     # 环岛状态: 0：未知或非环岛状态 1：当前为非环岛道路并且规划路线即将驶入环岛主路 2：当前道路类型是环岛并且规划路线还是在环岛主路中行驶 3：当前道路类型是环岛并且规划路线即将驶出环岛主路
int8   intersection_status     # 交叉路口状态: 0：未知 1：非交叉路口状态 2：交叉路口通行状态
int8   roads_status     # 当前行车推荐状态引导：(跨路口) 0：1km内（1km配置文件可配置）无需要左转、右转路口 1：需要左转路口（1km配置文件可配置） 2：需要右转路口  3：车道级规划失败
