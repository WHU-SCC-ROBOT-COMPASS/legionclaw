Header   header     # 消息头
LaneletInfo[] route # 一次导航的全程道路集合，从起点到终点排序
LaneletInfo   current_lanelet     # 当前所在车道的信息
float64   total_mileage     # 总里程
float64   cur_mileage     # 当前里程
LaneletInfo[] cur_slice # 当前地图切片,用来绘制局部地图
