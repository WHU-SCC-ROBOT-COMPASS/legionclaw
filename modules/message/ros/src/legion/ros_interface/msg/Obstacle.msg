Time   timestamp     # 时间戳
int32   id     # 障碍物id
float64   existence_prob     # 障碍物存在的概率
Time   create_time     # 障碍物被识别的时间戳
Time   last_updated_time     # 障碍物上一次更新的时间
Point3D   center_pos_vehicle     # 车辆坐标系下障碍物的中心位置
Point3D   center_pos_abs     # 世界坐标系下障碍物的中心位置
float64   theta_vehicle     # 车辆坐标系下障碍物的朝向
float64   theta_abs     # 世界坐标系下障碍物的朝向
Point3D   velocity_vehicle     # 车辆坐标系下障碍物的速度
Point3D   velocity_abs     # 世界坐标系下障碍物的速度
float64   length     # 障碍物长度
float64   width     # 障碍物宽度
float64   height     # 障碍物高度
ImageKeyPoint[] image_key_points # 图像坐标系下障碍物多边形
Point3D[] polygon_point_abs # 世界坐标系下障碍物多边形
Point3D[] polygon_point_vehicle # 车辆坐标系下障碍物多边形
float64   tracking_time     # 障碍物被追踪的时间
int32   type     # 障碍物类别： UNKNOWN = 0; UNKNOWN_MOVABLE = 1; UNKNOWN_UNMOVABLE = 2; PEDESTRIAN = 3; BICYCLE = 4; VEHICLE = 5;
float64   confidence     # 障碍物类别置信度
int32   confidence_type     # 障碍物置信度类别： CONFIDENCE_UNKNOWN = 0; CONFIDENCE_CNN = 1; CONFIDENCE_RADAR = 2;
Point3D[] drops # 障碍物轨迹点
Point3D   acceleration_vehicle     # 车辆坐标系下障碍物的加速度
Point3D   acceleration_abs     # 世界坐标系下障碍物的加速度
Point2D   anchor_point_image     # 障碍物尾框中心点(图像坐标系)
Point3D   anchor_point_vehicle     # 障碍物尾框中心点(车辆坐标系)
Point3D   anchor_point_abs     # 障碍物尾框中心点(世界坐标系)
BBox2D   bbox2d     # 障碍物图像框
BBox2D   bbox2d_rear     # 障碍物图像尾框
int32   sub_type     # 障碍物类别： ST_UNKNOWN = 0; ST_UNKNOWN_MOVABLE = 1; ST_UNKNOWN_UNMOVABLE = 2; ST_CAR = 3; ST_VAN = 4; ST_TRUCK = 5; ST_BUS = 6; ST_CYCLIST = 7; ST_MOTORCYCLIST = 8; ST_TRICYCLIST = 9; ST_PEDESTRIAN = 10; ST_TRAFFICCONE = 11; ST_PILLAR = 12; ST_SPEED_BUMP = 13;
float64   height_above_ground     # 障碍物近地点到地面的高度
float64[] position_abs_covariance # 世界坐标系下障碍物中心位置的协方差矩阵
float64[] velocity_abs_covariance # 世界坐标系下障碍物速度的协方差矩阵
float64[] acceleration_abs_covariance # 世界坐标系下障碍物加速度的协方差矩阵
float64   theta_abs_covariance     # 世界坐标系下障碍物朝向的协方差矩阵
float64[] position_vehicle_covariance # 车辆坐标系下障碍物中心位置的协方差矩阵
float64[] velocity_vehicle_covariance # 车辆坐标系下障碍物速度的协方差矩阵
float64[] acceleration_vehicle_covariance # 车辆坐标系下障碍物加速度的协方差矩阵
float64   theta_vehicle_covariance     # 车辆坐标系下障碍物朝向的协方差矩阵
SensorCalibrator   sensor_calibrator     # 传感器标定参数
uint8   cipv_flag     # 障碍物状态标志（0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE）
int32   lane_position     # 车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE 2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN
float64   pihp_percentage     # 临车道车辆压线比例
int32   blinker_flag     # 障碍物车辆信号灯状态： 0-OFF 1-LEFT_TURN_VISIBLE 2-LEFT_TURN_ON 3-RIGHT_TURN_VISIBLE 4-RIGHT_TURN_ON 5-BRAKE_VISIBLE 6-BRAKE_ON 7-UNKNOWN
int32   fusion_type     # 融合障碍物类型 0-CAMERA 1-RADAR 2-LIDAR 3-ULTRASONIC 4-FUSED 5-UNKNOWN
