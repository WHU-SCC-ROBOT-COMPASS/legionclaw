int32   id     # obstacle ID.
uint8   life_time     # 障碍物存在时间
Point2D   relative_position     # 雷达坐标系障碍物位置
Point2D   relative_position_rms     # 雷达坐标系障碍物位置标准差
Point2D   relative_velocity     # 雷达坐标系障碍物速度
Point2D   relative_velocity_rms     # 雷达坐标系障碍物速度标准差
Point2D   relative_acceleration     # 雷达坐标系障碍物加速度
Point2D   relative_acceleration_rms     # 雷达坐标系障碍物加速度标准差
float64   rcs     # 雷达散射面积
float64   snr     #  
int32   moving_status     # STATIONARY = 0; NEARING = 1; AWAYING = 2; NONE = 3;
float64   width     # 雷达障碍物宽度
float64   length     # 雷达障碍物长度
float64   height     # 雷达障碍物高度
float64   theta     #  
Point2D   absolute_position     # obstacle position in map coordinate system
Point2D   absolute_position_rms     #  
Point2D   absolute_velocity     # obstacle position in map coordinate system
Point2D   absolute_velocity_rms     #  
Point2D   absolute_acceleration     #  
Point2D   absolute_acceleration_rms     #  
float64   orientation     # 雷达障碍物方位角
float64   orient_rms     # 雷达障碍物方位角标准差
float64   yaw     #  
float64   yaw_rms     #  
int32   count     #  
int32   moving_frames_count     #  
Status   status     # 状态码
float64   underpass_probability     #  
float64   overpass_probability     #  
uint8   exist_probability     # 障碍物存在的概率
uint8   mov_property     # 障碍物移动属性：MOVING_PROPERTY_MOVING=0，MOVING_PROPERTY_STATIONARY=1，MOVING_PROPERTY_ONCOMING=2，MOVING_PROPERTY_STATIONARY_CANDIDATE=3，MOVING_PROPERTY_UNKOWN=4，MOVING_PROPERTY_CROSSING_STATIONARY=5，MOVING_PROPERTY_CROSSING_MOVING=6，MOVING_PROPERTY_STOPPED=7
uint8   track_state     # 跟踪状态：TRACK_STATE_DELETED=0；TRACK_STATE_INITED=1；TRACK_STATE_MEASURED=2；TRACK_STATE_PREDICTED=3；TRACK_STATE_DELETED_FOR_MERGE=4；TRACK_STATE_NEW_FOR_MERGE=5；
uint8   track_type     # 追踪的障碍物类型：TRACK_TYPE_POINT=0；TRACK_TYPE_CAR=1；TRACK_TYPE_TRUCK=2；TRACK_TYPE_PEDESTRIAN=3；TRACK_TYPE_MOTORCYCLE=4；TRACK_TYPE_BICYCLE=5；TRACK_TYPE_WIDE=6；TRACK_TYPE_RESERVED=7；TRACK_TYPE_BRIDGE=8；
