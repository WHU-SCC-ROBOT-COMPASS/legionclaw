Headers   header     # 消息头
UssObstacle[] uss_obstacles # 检测出的超声波障碍物数组
float64[] uss_ranges # 超声波检测的距离数组，存放顺序按照FA,FB,FC,FD,FE,FF,RA,RB,RC,RD,RE,RF
int32   error_code     # 错误码（default = OK）
uint8   is_valid     # 超声波障碍物数据是否合法
