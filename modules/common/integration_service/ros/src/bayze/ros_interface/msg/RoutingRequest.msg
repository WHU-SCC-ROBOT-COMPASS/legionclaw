Headers   header     # timestamp is included in header
string   request_source     # 发起请求的来源名称
int8   request_type     # REQUEST_TYPE=0//无效 REQUEST_TYPE=1//正常路由 REQUEST_TYPE=2//泊车路由
int16   num_of_kp     # 关键点数量
KeyPoint[] key_point_list # 关键点列表，第一个为起点，最后一个为终点，中间为途经点
