int32   id     # 障碍物id
Point3D   center_pos_vehicle     # 车辆坐标系下障碍物的中心位置
Point3D   center_pos_abs     # 世界坐标系下障碍物的中心位置
float64   theta_vehicle     # 车辆坐标系下障碍物的朝向
float64   theta_abs     # 世界坐标系下障碍物的朝向
float64   length     # 障碍物长度
float64   width     # 障碍物宽度
float64   height     # 障碍物高度
int32   type     # 障碍物类别： UNKNOWN = 0; UNKNOWN_MOVABLE = 1; UNKNOWN_UNMOVABLE = 2; PEDESTRIAN = 3; BICYCLE = 4; VEHICLE = 5;
float64   confidence     # 障碍物类别置信度
int32   confidence_type     # 障碍物置信度类别： CONFIDENCE_UNKNOWN = 0; CONFIDENCE_CNN = 1; CONFIDENCE_RADAR = 2;
int32   sub_type     # 障碍物类别： ST_UNKNOWN = 0; ST_UNKNOWN_MOVABLE = 1; ST_UNKNOWN_UNMOVABLE = 2; ST_CAR = 3; ST_VAN = 4; ST_TRUCK = 5; ST_BUS = 6; ST_CYCLIST = 7; ST_MOTORCYCLIST = 8; ST_TRICYCLIST = 9; ST_PEDESTRIAN = 10; ST_TRAFFICCONE = 11; ST_PILLAR = 12; ST_SPEED_BUMP = 13;
Point3D[] points #  
uint8   cipv_flag     # 障碍物状态标志（0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE）
int32   fusion_type     #  
